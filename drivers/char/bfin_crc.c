/*
 * Blackfin On-Chip hardware CRC Driver
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#include <asm/blackfin.h>
#include <asm/bfin_crc.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/portmux.h>

#define DRIVER_NAME "bfin-crc"

static LIST_HEAD(bfin_crc_list);

static void bfin_crc_config_dma(unsigned long dma_ch, unsigned long addr,
	unsigned long size, unsigned long dma_config, int mod_dir)
{
	unsigned int dma_count;
	unsigned int dma_shift;
	int dma_mod;

	dma_config |= PSIZE_32;

	if (addr % 4 == 0) {
		dma_config |= WDSIZE_32;
		dma_count = size >> 2;
		dma_shift = 2;
	} else if (addr % 2 == 0) {
		dma_config |= WDSIZE_16;
		dma_count = size >> 1;
		dma_shift = 1;
	} else {
		dma_config |= WDSIZE_8;
		dma_count = size;
		dma_shift = 0;
	}

	/* If the two memory regions have a chance of overlapping, make
	 * sure the memcpy still works as expected.  Do this by having the
	 * copy run backwards instead.
	 */
	dma_mod = (1 << dma_shift) * mod_dir;
	addr += size + dma_mod;

	set_dma_start_addr(dma_ch, addr);
	set_dma_x_count(dma_ch, dma_count);
	set_dma_x_modify(dma_ch, dma_mod);
	set_dma_config(dma_ch, dma_config);
	enable_dma(dma_ch);
}

static int bfin_crc_run(struct bfin_crc *crc, unsigned int opmode, struct crc_info *info)
{
	int ret = 0;
	unsigned long control;
	unsigned int timeout = 100000;
	int mod_dir;

	if (info->crc_datasize == 0)
		return 0;
	else if (info->crc_datasize % 4 != 0) {
		dev_info(crc->mdev.this_device, "CRC data size should be multiply of 4 bytes.\n");
		return -EFAULT;
	}

	/* config CRC */
	crc->regs->poly = info->crc_poly;
	SSYNC();

	if (opmode == MODE_DATA_FILL)
		crc->regs->fillval = info->crc_fillval;
	else
		crc->regs->compare = info->crc_compare;

	control = (opmode & OPMODE) << OPMODE_OFFSET;
	control |= info->bitmirr << BITMIRR_OFFSET;
	control |= info->bytmirr << BYTMIRR_OFFSET;
	control |= info->w16swp << W16SWP_OFFSET;
	control |= info->fdsel << FDSEL_OFFSET;
	control |= info->rsltmirr << RSLTMIRR_OFFSET;
	control |= info->polymirr << POLYMIRR_OFFSET;
	control |= info->cmpmirr << CMPMIRR_OFFSET;

	if (opmode == MODE_DMACPY_CRC)
		control |= OBRSTALL;

	crc->regs->control = control;
	crc->regs->datacnt = info->crc_datasize >> 2;

	while (!(crc->regs->status & LUTDONE) && (--timeout) > 0)
		cpu_relax();

	if (!timeout) {
		dev_dbg(crc->mdev.this_device, "Timeout when generating LUT.\n");
		return -EBUSY;
	}

	/* setup CRC interrupts */
	crc->regs->intrenset = CMPERRI | DCNTEXPI;
	SSYNC();

	/* setup CRC DMA */
	switch (opmode) {
	case MODE_CALC_CRC:
	case MODE_DATA_VERIFY:
		flush_dcache_range(info->in_addr, info->in_addr + info->crc_datasize);
		bfin_crc_config_dma(crc->dma_ch_src, info->in_addr,
			info->crc_datasize, 0, 1);
		break;
	case MODE_DMACPY_CRC:
		flush_dcache_range(info->in_addr, info->in_addr + info->crc_datasize);
		invalidate_dcache_range(info->out_addr, info->out_addr + info->crc_datasize);
		if (info->out_addr < info->in_addr + info->crc_datasize)
			mod_dir = -1;
		else
			mod_dir = 1;
		bfin_crc_config_dma(crc->dma_ch_src, info->in_addr,
			info->crc_datasize, 0, mod_dir);
		bfin_crc_config_dma(crc->dma_ch_dest, info->out_addr,
			info->crc_datasize, WNR, mod_dir);
		break;
	case MODE_DATA_FILL:
		invalidate_dcache_range(info->out_addr, info->out_addr + info->crc_datasize);
		bfin_crc_config_dma(crc->dma_ch_dest, info->out_addr,
			info->crc_datasize, WNR, 1);
		break;
	default:
		goto out;
	}

	/* enable CRC operation */
	crc->regs->control |= BLKEN;
	SSYNC();

	/* wait for completion */
	ret = wait_for_completion_interruptible(&crc->c);
	if (ret < 0) {
		dev_dbg(crc->mdev.this_device, "Completion waiting is interrupted.\n");
		goto out;
	}

	if (opmode == MODE_DMACPY_CRC || MODE_DATA_FILL)
		while (crc->regs->status & OBR)
			cpu_relax();

	/* prepare results */
	info->crc_result = crc->regs->result;
	crc->regs->control = 0;
	SSYNC();
	disable_dma(crc->dma_ch_src);
	disable_dma(crc->dma_ch_dest);
out:
	return ret;
}

/**
 *	bfin_crc_handler - CRC interrupt handler
 */
static irqreturn_t bfin_crc_handler(int irq, void *dev_id)
{
	struct bfin_crc *crc = dev_id;

	if (crc->regs->status & DCNTEXP) {
		crc->regs->status = DCNTEXP;
		SSYNC();
		complete(&crc->c);
		return IRQ_HANDLED;
	} else
		return IRQ_NONE;
}

/**
 *	bfin_crc_open - Open the Device
 *	@inode: inode of device
 *	@filp: file handle of device
 */
static int bfin_crc_open(struct inode *inode, struct file *filp)
{
	int minor;
	struct bfin_crc *crc;
	int ret;

	minor = MINOR(inode->i_rdev);
	filp->private_data = NULL;
	list_for_each_entry(crc, &bfin_crc_list, list)
		if (crc->mdev.minor == minor) {
			filp->private_data = crc;
			break;
		}

	if (!filp->private_data)
		return -ENODEV;

	if (mutex_lock_interruptible(&crc->mutex))
		return -ERESTARTSYS;

	init_completion(&crc->c);

	ret = request_irq(crc->irq, bfin_crc_handler, IRQF_SHARED, DRIVER_NAME, crc);
	if (ret) {
		pr_err("unable to request blackfin crc irq\n");
		goto out;
	}

	if (request_dma(crc->dma_ch_src, "BFIN_CRC_SRC") < 0) {
		printk(KERN_NOTICE "Unable to attach Blackfin CRC source DMA channel\n");
		ret = -EBUSY;
		goto out;
	}

	if (request_dma(crc->dma_ch_dest, "BFIN_CRC_DEST") < 0) {
		printk(KERN_NOTICE "Unable to attach Blackfin CRC destination DMA channel\n");
		free_dma(crc->dma_ch_src);
		ret = -EBUSY;
		goto out;
	}

out:
	mutex_unlock(&crc->mutex);

	return 0;
}

/**
 *	bfin_crc_close - Close the Device
 *	@inode: inode of device
 *	@filp: file handle of device
 */
static int bfin_crc_release(struct inode *inode, struct file *filp)
{
	int minor;
	struct bfin_crc *crc;

	minor = MINOR(inode->i_rdev);
	filp->private_data = NULL;
	list_for_each_entry(crc, &bfin_crc_list, list)
		if (crc->mdev.minor == minor) {
			filp->private_data = crc;
			break;
		}

	if (!filp->private_data)
		return -ENODEV;

	if (mutex_lock_interruptible(&crc->mutex))
		return -ERESTARTSYS;

	free_dma(crc->dma_ch_src);
	free_dma(crc->dma_ch_dest);
	free_irq(crc->irq, crc);
	mutex_unlock(&crc->mutex);

	return 0;
}

/**
 *	bfin_crc_ioctl - Operate CRC Device
 *	@filp: file handle of device
 *	@cmd: hardware crc command
 *	@arg: argument
 */
static long bfin_crc_ioctl(struct file *filp,
				unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct crc_info bfin_crc_info;
	struct bfin_crc *crc;
	int ret = 0;

	crc = filp->private_data;

	if (mutex_lock_interruptible(&crc->mutex))
		return -ERESTARTSYS;

	if (copy_from_user(&bfin_crc_info, argp, sizeof(bfin_crc_info))) {
		ret = -EFAULT;
		goto out;
	}

	switch (cmd) {
	case CRC_IOC_CALC_CRC:
		ret = bfin_crc_run(crc, MODE_CALC_CRC, &bfin_crc_info);
		break;
	case CRC_IOC_MEMCPY_CRC:
		ret = bfin_crc_run(crc, MODE_DMACPY_CRC, &bfin_crc_info);
		break;
	case CRC_IOC_VERIFY_VAL:
		ret = bfin_crc_run(crc, MODE_DATA_VERIFY, &bfin_crc_info);
		break;
	case CRC_IOC_FILL_VAL:
		ret = bfin_crc_run(crc, MODE_DATA_FILL, &bfin_crc_info);
		break;
	default:
		ret = -ENOTTY;
	}

	if (ret > 0 && copy_to_user(argp, &bfin_crc_info, sizeof(bfin_crc_info)))
		ret = -EFAULT;

out:
	mutex_unlock(&crc->mutex);
	return ret;
}

#ifdef CONFIG_PM
/**
 *	bfin_crc_suspend - suspend the watchdog
 *	@pdev: device being suspended
 *	@state: requested suspend state
 */
static int bfin_crc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct bfin_crc *crc = platform_get_drvdata(pdev);

	return 0;
}

/**
 *	bfin_crc_resume - resume the watchdog
 *	@pdev: device being resumed
 */
static int bfin_crc_resume(struct platform_device *pdev)
{
	struct bfin_crc *crc = platform_get_drvdata(pdev);

	return 0;
}
#else
# define bfin_crc_suspend NULL
# define bfin_crc_resume NULL
#endif

static const struct file_operations bfin_crc_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= bfin_crc_ioctl,
	.open		= bfin_crc_open,
	.release	= bfin_crc_release,
};

/**
 *	bfin_crc_probe - Initialize module
 *
 *	Registers the misc device.  Actual device
 *	initialization is handled by bfin_crc_open().
 */
static int __devinit bfin_crc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct bfin_crc *crc = NULL;
	int ret;

	crc = kzalloc(sizeof(*crc), GFP_KERNEL);
	if (!crc) {
		dev_err(&pdev->dev, "fail to malloc bfin_crc\n");
		return -ENOMEM;
	}

	mutex_init(&crc->mutex);
	crc->mdev.minor	= MISC_DYNAMIC_MINOR;
	snprintf(crc->name, 20, "%s%d", DRIVER_NAME, pdev->id);
	crc->mdev.name	= crc->name;
	crc->mdev.fops	= &bfin_crc_fops;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out_error_free_mem;
	}

	crc->regs = ioremap(res->start, resource_size(res));
	if (!crc->regs) {
		dev_err(&pdev->dev, "Cannot map CRC IO\n");
		ret = -ENXIO;
		goto out_error_free_mem;
	}

	crc->irq = platform_get_irq(pdev, 0);
	if (crc->irq < 0) {
		dev_err(&pdev->dev, "No CRC DCNTEXP IRQ specified\n");
		ret = -ENOENT;
		goto out_error_unmap;
	}

	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "No CRC DMA source channel specified\n");
		ret = -ENOENT;
		goto out_error_unmap;
	}
	crc->dma_ch_src = res->start;

	res = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	if (res == NULL) {
		dev_err(&pdev->dev, "No CRC DMA destination channel specified\n");
		ret = -ENOENT;
		goto out_error_unmap;
	}
	crc->dma_ch_dest = res->start;

	ret = misc_register(&crc->mdev);
	if (ret) {
		dev_err(&pdev->dev, "cannot register CRC miscdev\n");
		return ret;
	}

	list_add(&crc->list, &bfin_crc_list);
	dev_set_drvdata(&pdev->dev, crc);

	dev_info(&pdev->dev, "initialized\n");

	return 0;

out_error_unmap:
	iounmap((void *)crc->regs);
out_error_free_mem:
	kfree(crc);
	return ret;
}

/**
 *	bfin_crc_remove - Initialize module
 *
 *	Unregisters the misc device.  Actual device
 *	deinitialization is handled by bfin_crc_close().
 */
static int __devexit bfin_crc_remove(struct platform_device *pdev)
{
	struct bfin_crc *crc = platform_get_drvdata(pdev);

	dev_set_drvdata(&pdev->dev, NULL);

	if (crc) {
		misc_deregister(&crc->mdev);
		list_del(&crc->list);
		iounmap((void *)crc->regs);
		kfree(crc);
	}

	return 0;
}

static struct platform_driver bfin_crc_driver = {
	.probe     = bfin_crc_probe,
	.remove    = __devexit_p(bfin_crc_remove),
	.suspend   = bfin_crc_suspend,
	.resume    = bfin_crc_resume,
	.driver    = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

/**
 *	bfin_crc_init - Initialize module
 *
 *	Checks the module params and registers the platform driver.
 *	Real work is in the platform probe function.
 */
static int __init bfin_crc_init(void)
{
	int ret;

	pr_info("Blackfin hardware CRC driver\n");

	ret = platform_driver_register(&bfin_crc_driver);
	if (ret) {
		pr_info(KERN_ERR "unable to register driver\n");
		return ret;
	}

	return 0;
}

/**
 *	bfin_crc_exit - Deinitialize module
 */
static void __exit bfin_crc_exit(void)
{
	platform_driver_unregister(&bfin_crc_driver);
}

module_init(bfin_crc_init);
module_exit(bfin_crc_exit);

MODULE_AUTHOR("Sonic Zhang <sonic.zhang@analog.com>");
MODULE_DESCRIPTION("Blackfin hardware CRC Driver");
MODULE_LICENSE("GPL");

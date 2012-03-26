/*
 * Blackfin Linkport driver
 *
 * Copyright 2012-2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/completion.h>

#include <linux/workqueue.h>

#include <asm/irq.h>
#include <asm/blackfin.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>
#include <asm/portmux.h>

#include <asm/gptimers.h>

/* fifo size in elements (ints) */
#define FIFO_SIZE      1024

#define LINKPORT_DRVNAME "bfin-linkport"

#define BFIN_LP_DMA_MODE

#define LP_CTL_EN 0x1
#define LP_CTL_TRAN 0x8
#define LP_CTL_TRQMSK  0x100
#define LP_CTL_RRQMSK  0x200
#define LP_CTL_ITMSK  0x800

#define LP_STAT_DONE 0x1000
#define LP_STAT_LTRQ 0x1
#define LP_STAT_LRRQ 0x2
#define LP_STAT_LPIT 0x8
#define LP_STAT_FFST 0x70
#define LP_STAT_LERR 0x80
#define LP_STAT_LPBS 0x100

struct bfin_lp_register {
	uint32_t ctl;
	uint32_t stat;
	uint32_t div;
	uint32_t cnt;
	uint32_t tx;
	uint32_t rx;
	uint32_t tx_shadow;
	uint32_t rx_shadow;
};

static const unsigned short per_req_lp0[] = {
	P_LP0_CLK,
	P_LP0_ACK,
	P_LP0_D0,
	P_LP0_D1,
	P_LP0_D2,
	P_LP0_D3,
	P_LP0_D4,
	P_LP0_D5,
	P_LP0_D6,
	P_LP0_D7,
	0
};

static const unsigned short per_req_lp1[] = {
	P_LP1_CLK,
	P_LP1_ACK,
	P_LP1_D0,
	P_LP1_D1,
	P_LP1_D2,
	P_LP1_D3,
	P_LP1_D4,
	P_LP1_D5,
	P_LP1_D6,
	P_LP1_D7,
	0
};

static const unsigned short per_req_lp2[] = {
	P_LP2_CLK,
	P_LP2_ACK,
	P_LP2_D0,
	P_LP2_D1,
	P_LP2_D2,
	P_LP2_D3,
	P_LP2_D4,
	P_LP2_D5,
	P_LP2_D6,
	P_LP2_D7,
	0
};

static const unsigned short per_req_lp3[] = {
	P_LP3_CLK,
	P_LP3_ACK,
	P_LP3_D0,
	P_LP3_D1,
	P_LP3_D2,
	P_LP3_D3,
	P_LP3_D4,
	P_LP3_D5,
	P_LP3_D6,
	P_LP3_D7,
	0
};

struct bfin_linkport {
	struct list_head lp_dev;
	struct class *class;
	int major;
	spinlock_t lp_dev_lock;
};

struct bfin_lp_dev {
	struct list_head list;
	struct device *device;
	volatile struct bfin_lp_register *regs;
	wait_queue_head_t rx_waitq;
	spinlock_t lock;
	struct workqueue_struct *workqueue;
	struct work_struct transfer_work;
	int linkport_num;
	int dma_chan;
	int status;
	int irq;
	int status_irq;
	int count;
	DECLARE_KFIFO_PTR(lpfifo, unsigned int);
	struct completion complete;
	const unsigned short *per_linkport;
};

struct bfin_linkport *linkport_dev;

struct bfin_lp_dev lp_dev_info[4] = {
	{
		.regs = LP0_CTL,
		.irq = IRQ_LP0,
		.status_irq = IRQ_LP0_STAT,
		.per_linkport = per_req_lp0,
		.dma_chan = CH_LP0,
	},
	{
		.regs = LP1_CTL,
		.irq = IRQ_LP1,
		.status_irq = IRQ_LP1_STAT,
		.per_linkport = per_req_lp1,
		.dma_chan = CH_LP1,
	},
	{
		.regs = LP2_CTL,
		.irq = IRQ_LP2,
		.status_irq = IRQ_LP2_STAT,
		.per_linkport = per_req_lp2,
		.dma_chan = CH_LP2,
	},
	{
		.regs = LP3_CTL,
		.irq = IRQ_LP3,
		.status_irq = IRQ_LP3_STAT,
		.per_linkport = per_req_lp3,
		.dma_chan = CH_LP3,
	},
};


int bfin_lp_config_channel(struct bfin_lp_dev *lpdev, int direction)
{
	uint32_t reg;
	if (direction)
		reg = LP_CTL_TRAN | LP_CTL_TRQMSK;
	else
		reg =  LP_CTL_RRQMSK;

	lpdev->regs->ctl = reg;

	SSYNC();
	return 0;
}

int bfin_lp_tx_dma(struct bfin_lp_dev *lpdev, void *buf, size_t count)
{

	set_dma_config(lpdev->dma_chan,
			set_bfin_dma_config(DIR_READ, DMA_FLOW_STOP,
				INTR_ON_BUF,
				DIMENSION_LINEAR,
				DATA_SIZE_32,
				DMA_SYNC_RESTART));
	set_dma_start_addr(lpdev->dma_chan, buf);
	set_dma_x_count(lpdev->dma_chan, count);
	set_dma_x_modify(lpdev->dma_chan, 1);
	SSYNC();
	enable_dma(lpdev->dma_chan);

	return 0;
}

int bfin_lp_rx_dma(struct bfin_lp_dev *lpdev, void *buf, size_t count)
{

	set_dma_config(lpdev->dma_chan,
			set_bfin_dma_config(DIR_WRITE, DMA_FLOW_STOP,
				INTR_ON_BUF,
				DIMENSION_LINEAR,
				DATA_SIZE_32,
				DMA_SYNC_RESTART));
	set_dma_start_addr(lpdev->dma_chan, buf);
	set_dma_x_count(lpdev->dma_chan, count);
	set_dma_x_modify(lpdev->dma_chan, 1);
	SSYNC();
	enable_dma(lpdev->dma_chan);

	return 0;
}

int bfin_lp_get_rx_fifo(struct bfin_lp_dev *lpdev)
{
	int state = (lpdev->regs->stat & LP_STAT_FFST) >> 4;

	if (state <= 4)
		return state;
	else
		return 0;
}

static void lp_rx_fifo(struct bfin_lp_dev *dev)
{
	int cnt;

	cnt = bfin_lp_get_rx_fifo(dev);
	while (cnt) {
		unsigned int data;
		data = dev->regs->rx;
		if (!kfifo_put(&dev->lpfifo, &data))
			goto out;
		cnt--;
	}
out:
	enable_irq(dev->irq);
	/* wake up read/write block. */
	wake_up_interruptible(&dev->rx_waitq);
}

static void transfer_fn(struct work_struct *work)
{
	struct bfin_lp_dev *dev = container_of(work,
			struct bfin_lp_dev, transfer_work);

	if (dev->status == LP_STAT_LTRQ) {
		unsigned int data = 0;
		while (kfifo_get(&dev->lpfifo, &data)) {
			if ((dev->regs->stat & LP_STAT_FFST) == 0x60)
				break;
			while (dev->regs->stat & LP_STAT_LPBS);
			dev->regs->tx = data;
			while (dev->regs->stat & LP_STAT_LPBS);
		}
		if (kfifo_len(&dev->lpfifo) == 0) {
			dev->status = LP_STAT_DONE;
			if (dev->regs->stat & LP_STAT_LPBS) {
				enable_irq(dev->irq);
				return;
			}
			complete(&dev->complete);
		}
	} else if (dev->status == LP_STAT_DONE) {
		kfifo_reset(&dev->lpfifo);
	} else {
		lp_rx_fifo(dev);
	}
}

static irqreturn_t bfin_lp_irq(int irq, void *dev_id)
{
	struct bfin_lp_dev *dev = (struct bfin_lp_dev *)dev_id;
	unsigned long flags;
	uint32_t stat = dev->regs->stat;

	pr_debug("bfin lp irq %d stat %x dev %p status %d\n", irq, stat, dev, dev->status);

	if (stat & LP_STAT_LTRQ) {
		if (kfifo_len(&dev->lpfifo)) {
			dev->status = LP_STAT_LTRQ;
			dev->regs->ctl |= LP_CTL_EN;
		}
		goto out;
	}

	if (stat & LP_STAT_LRRQ) {
		dev->status = LP_STAT_LRRQ;
		dev->regs->ctl |= LP_CTL_EN;
		goto out;
	}

	disable_irq_nosync(irq);
	queue_work(dev->workqueue, &dev->transfer_work);
out:
	dev->regs->stat = stat;
	dev->count++;
	return IRQ_HANDLED;
}


static int bfin_lp_open(struct inode *inode, struct file *filp)
{
	unsigned long flags;
	struct bfin_lp_dev *dev;
	unsigned int index = iminor(inode);
	int ret = -EBUSY;

	dev = &lp_dev_info[index];

	spin_lock_irqsave(&dev->lock, flags);

	filp->private_data = dev;

	spin_unlock_irqrestore(&dev->lock, flags);

	ret = kfifo_alloc(&dev->lpfifo, FIFO_SIZE, GFP_KERNEL);
	if (ret) {
		printk(KERN_ERR "error kfifo_alloc\n");
		return ret;
	}

	if (peripheral_request_list(dev->per_linkport, LINKPORT_DRVNAME)) {
		printk("Requesting Peripherals failed\n");

		return ret;
	}

	if (request_dma(dev->dma_chan, LINKPORT_DRVNAME) < 0) {
		printk(KERN_NOTICE "Unable to attach Blackfin LINKPORT DMA channel\n");
		goto free_per;
	}

	if (request_irq(dev->irq, bfin_lp_irq, 0, LINKPORT_DRVNAME, dev)) {
		printk(KERN_ERR "Requesting irq failed\n");
		goto free_dma;
	}

	if (request_irq(dev->status_irq, bfin_lp_irq, 0, LINKPORT_DRVNAME, dev)) {
		printk(KERN_ERR "Requesting status irq failed\n");
		goto free_dma;
	}

	dev->workqueue = create_singlethread_workqueue("linkport_work");
	if (!dev->workqueue) {
		printk(KERN_ERR "create workqueue failed\n");
		goto free_dma;
	}

	dev->regs->div = 1;
	dev->regs->stat = 0xFF;
	dev->regs->ctl = 0;
	SSYNC();

	return 0;

free_dma:
	free_dma(dev->dma_chan);
free_per:
	peripheral_free_list(dev->per_linkport);
	return ret;
}

static int bfin_lp_release(struct inode *inode, struct file *filp)
{
	struct bfin_lp_dev *dev = filp->private_data;

	wait_for_completion(&dev->complete);

	dev->regs->ctl = 0;

	destroy_workqueue(dev->workqueue);
	kfifo_free(&dev->lpfifo);
	peripheral_free_list(dev->per_linkport);
	free_dma(dev->dma_chan);
	free_irq(dev->irq, dev);
	free_irq(dev->status_irq, dev);
	return 0;
}

static ssize_t bfin_lp_read(struct file *filp, char *buf, size_t count, loff_t *pos)
{
	struct bfin_lp_dev *dev = filp->private_data;
	int fifo_cnt = 0;
	unsigned int copied;
	unsigned int n;
	int ret;

	n = count / 4;
	count = 4 * n;

	dev->regs->div = 1;

	bfin_lp_config_channel(dev, 0);

	dev->regs->ctl |= LP_CTL_EN;


	while (n) {
		fifo_cnt = kfifo_len(&dev->lpfifo);
		if (!fifo_cnt) {
			wait_event_interruptible(dev->rx_waitq, kfifo_len(&dev->lpfifo) != 0);
			continue;
		}

		ret = kfifo_to_user(&dev->lpfifo, buf, fifo_cnt * 4, &copied);
		n -= fifo_cnt;
	}

	complete(&dev->complete);

	return count;
}

static ssize_t bfin_lp_write(struct file *filp, const char *buf, size_t count, loff_t *pos)
{
	struct bfin_lp_dev *dev = filp->private_data;
	unsigned int copied;
	int ret;
	int i;

	ret = kfifo_from_user(&dev->lpfifo, buf, count, &copied);

	dev->regs->div = 1;

	bfin_lp_config_channel(dev, 1);

	return ret ? ret : copied;
}

static long bfin_lp_ioctl(struct file *filp, uint cmd, unsigned long arg)
{
	return 0;
}


static const struct file_operations linkport_fops = {
	.owner = THIS_MODULE,
	.read = bfin_lp_read,
	.write = bfin_lp_write,
	.unlocked_ioctl = bfin_lp_ioctl,
	.open = bfin_lp_open,
	.release = bfin_lp_release,
};

static ssize_t
linkport_status_show(struct class *class, struct class_attribute *attr, char *buf)
{
	char *p = buf;
	struct bfin_lp_dev *dev;

	p += sprintf(p, "linkport status\n");
	list_for_each_entry(dev, &linkport_dev->lp_dev, list) {
		p += sprintf(p, "linkport num %d\n", dev->linkport_num);
	}
	return p - buf;
}

static ssize_t
linkport_reg_show(struct class *class, struct class_attribute *attr, char *buf)
{
	char *p = buf;
	struct bfin_lp_dev *dev;

	p += sprintf(p, "linkport status\n");
	list_for_each_entry(dev, &linkport_dev->lp_dev, list) {
		p += sprintf(p, "linkport num %d\n", dev->linkport_num);
		p += sprintf(p, "\t clt %d\n", dev->regs->ctl);
		p += sprintf(p, "\t stat %d\n", dev->regs->stat);
	}
	return (p - buf);
}

static ssize_t
linkport_reg_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	char *p;
	int rw = 0;
	char buffer[64];
	uint32_t res;
	uint32_t addr;
	uint32_t value = 0;
	char *endp;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';

	p = buffer;

	while (*p == ' ')
		p++;

	if (p[0] == 'r')
		rw = 0;
	else if (p[0] == 'w')
		rw = 1;
	else
		printk(KERN_DEBUG "-EINVAL\n");

	if (p[1] < '0' && p[1] > '9')
		printk(KERN_DEBUG "-EINVAL2\n");

	res = simple_strtoul(&p[1], &endp, 10);

	if (res == 8)
		p += 2;
	else if (res == 16)
		p += 3;
	else if (res == 32)
		p += 3;
	else
		printk(KERN_DEBUG "-EINVAL3\n");


	while (*p == ' ')
		p++;

	addr = simple_strtoul(p, &endp, 16);

	if (rw) {
		p = endp;
		while (*p == ' ')
			p++;

		value = simple_strtoul(p, NULL, 16);
		switch (res) {
		case 8:
			bfin_write8(addr, (uint8_t)value);
			value = bfin_read8(addr);
			break;
		case 16:
			bfin_write16(addr, (uint16_t)value);
			value = bfin_read16(addr);
			break;
		case 32:
			bfin_write32(addr, (uint32_t)value);
			value = bfin_read32(addr);
			break;
		}
		printk(KERN_DEBUG "write addr %08x reg %08x\n", addr, value);
	} else {
		switch (res) {
		case 8:
			value = bfin_read8(addr);
			break;
		case 16:
			value = bfin_read16(addr);
			break;
		case 32:
			value = bfin_read32(addr);
			break;
		}
		printk(KERN_DEBUG "read addr %08x reg %08x\n", addr, value);
	}
	return count;
}

static struct class *linkport_class;
static CLASS_ATTR(status, S_IRWXUGO, &linkport_status_show, NULL);
static CLASS_ATTR(reg, S_IRWXUGO, &linkport_reg_show, &linkport_reg_store);

/*
 * bfin_linkport_init - Initialize module
 *
 *
 */

static int __init bfin_linkport_init(void)
{
	int err;
	dev_t lp_dev;
	int i;

	linkport_dev = kzalloc(sizeof(*linkport_dev), GFP_KERNEL);
	if (!linkport_dev) {
		return -ENOMEM;
	}

	linkport_dev->major = register_chrdev(0, "bfin-linkport", &linkport_fops);
	if (linkport_dev->major < 0) {
		err = linkport_dev->major;
		printk("Error %d registering chrdev for device\n", err);
		goto free;
	}

	lp_dev = MKDEV(linkport_dev->major, 0);

	linkport_dev->class = class_create(THIS_MODULE, "linkport");
	err = class_create_file(linkport_dev->class, &class_attr_status);
	if (err) {
		printk("Error %d registering class device\n", err);
		goto free_chrdev;
	}

	err = class_create_file(linkport_dev->class, &class_attr_reg);
	if (err) {
		printk("Error %d registering class device\n", err);
		class_remove_file(linkport_dev->class, &class_attr_status);
		goto free_chrdev;
	}

	INIT_LIST_HEAD(&linkport_dev->lp_dev);
	spin_lock_init(&linkport_dev->lp_dev_lock);

	for (i = 0; i < 4; i++) {
		struct device *dev;
		dev = device_create(linkport_dev->class, NULL, lp_dev + i, &lp_dev_info[i], "linkport%d", i);
		if (!dev)
			goto free_chrdev;
		lp_dev_info[i].device = dev;
		lp_dev_info[i].linkport_num = i;
		spin_lock_init(&lp_dev_info[i].lock);
		init_waitqueue_head(&lp_dev_info[i].rx_waitq);
		INIT_WORK(&lp_dev_info[i].transfer_work, transfer_fn);
		INIT_LIST_HEAD(&lp_dev_info[i].list);
		init_completion(&lp_dev_info[i].complete);
		list_add(&lp_dev_info[i].list, &linkport_dev->lp_dev);
	}

	return 0;

free_chrdev:
	unregister_chrdev(linkport_dev->major, "bfin-linkport");
free:
	kfree(linkport_dev);
	return err;
}
module_init(bfin_linkport_init);

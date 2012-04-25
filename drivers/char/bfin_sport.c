/*
 * simple char interface to Blackfin SPORT peripheral
 *
 * Copyright 2004-2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#define DRV_NAME "bfin_sport_raw"
#define pr_fmt(fmt) (DRV_NAME ": " fmt)

#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <asm/blackfin.h>
#include <asm/bfin_sport.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>
#include <asm/portmux.h>

static LIST_HEAD(sport_list);

static struct sport_dev *bfin_sport_dev;

struct sport_dev {
	struct list_head list;
	struct device *dev;
	struct miscdevice misc;
	char name[16]; /*bfin_sport#*/
	const unsigned short *pin_req;

	int dma_rx_chan;
	int dma_tx_chan;

	int rx_irq;
	unsigned char *rx_buf;	/* Buffer store the received data */
	int rx_len;		/* How many bytes will be received */
	int rx_received;	/* How many bytes has been received */

	int tx_irq;
	const unsigned char *tx_buf;
	int tx_len;
	int tx_sent;

	int err_irq;

	struct mutex mutex;
	unsigned int open_count;

	struct completion c;

	volatile struct sport_register __iomem *regs;
	resource_size_t reg_base;
	unsigned long reg_len;
	struct sport_config config;
};

static int maybe_request_irq(int irq, irq_handler_t handler,
	unsigned long flags, const char *name, struct sport_dev *dev)
{
	int ret = 0;

	if (irq == -1)
		return ret;

	ret = request_irq(irq, handler, flags, name, dev);
	if (ret)
		dev_err(dev->dev, "unable to request irq %s\n", name);

	return ret;
}

static void maybe_free_irq(int irq, void *dev)
{
	if (irq != -1)
		free_irq(irq, dev);
}

static irqreturn_t dma_rx_irq_handler(int irq, void *dev_id);
static irqreturn_t dma_tx_irq_handler(int irq, void *dev_id);

/* note: multichannel is in unit of 8 channels,
   tdm_count is # channels NOT / 8 !
*/
static int sport_set_multichannel(volatile struct sport_register *regs,
		int tdm_count, int packed, int frame_delay)
{
	if (tdm_count) {
		int shift = 32 - tdm_count;
		unsigned int mask = (0xffffffff >> shift);

		regs->mcmc1 = ((tdm_count >> 3) - 1) << 12;	/* WSIZE bits */
		regs->mcmc2 = (frame_delay << 12) | MCMEN |
			(packed ? (MCDTXPE | MCDRXPE) : 0);

		regs->mtcs0 = regs->mrcs0 = mask;
	} else {
		regs->mcmc1 = regs->mcmc2 = 0;
		regs->mtcs0 = regs->mrcs0 = 0;
	}

	regs->mtcs1 = regs->mtcs2 = regs->mtcs3 = 0;
	regs->mrcs1 = regs->mrcs2 = regs->mrcs3 = 0;

	SSYNC();

	return 0;
}

static uint16_t sport_wordsize(struct sport_dev *dev, int word_len)
{
	uint16_t wordsize = 0;

	if (word_len <= 8)
		wordsize = WDSIZE_8;
	else if (word_len <= 16)
		wordsize = WDSIZE_16;
	else if (word_len <= 32)
		wordsize = WDSIZE_32;
	else
		dev_err(dev->dev, "word_len of %d is invalid\n", word_len);

	return wordsize;
}

static irqreturn_t dma_rx_irq_handler(int irq, void *dev_id)
{
	struct sport_dev *dev = dev_id;
	int status;

	dev_dbg(dev->dev, "%s enter\n", __func__);
	status = dev->regs->mcmc2;
	if (status & MCMEN)
		dev->regs->rcr1 &= ~RSPEN;
	dev->regs->rcr1 &= ~RSPEN;
	SSYNC();
	disable_dma(dev->dma_rx_chan);

	complete(&dev->c);

	clear_dma_irqstat(dev->dma_rx_chan);
	return IRQ_HANDLED;
}

static irqreturn_t dma_tx_irq_handler(int irq, void *dev_id)
{
	struct sport_dev *dev = dev_id;
	unsigned int status;

	dev_dbg(dev->dev, "%s enter\n", __func__);
	if (dev->err_irq != -1)
		disable_irq(dev->err_irq);
	status = get_dma_curr_irqstat(dev->dma_tx_chan);
	while (status & DMA_RUN) {
		dev_dbg(dev->dev, "status:0x%04x\n", status);
		status = get_dma_curr_irqstat(dev->dma_tx_chan);
	}
	status = dev->regs->stat;
	while (!(status & TXHRE)) {
		dev_dbg(dev->dev, "status:%x\n", status);
		udelay(1);
		status = dev->regs->stat;
	}
	/* Wait for the last byte sent out */
	udelay(500);
	dev_dbg(dev->dev, "%s status:%x\n", __func__, status);

	dev->regs->tcr1 &= ~TSPEN;
	status = dev->regs->mcmc2;
	if (status & MCMEN)
		dev->regs->rcr1 &= ~RSPEN;
	SSYNC();
	if (dev->err_irq != -1)
		enable_irq(dev->err_irq);
	disable_dma(dev->dma_tx_chan);

	complete(&dev->c);

	/* Clear the interrupt status */
	clear_dma_irqstat(dev->dma_tx_chan);

	return IRQ_HANDLED;
}


static inline void sport_rx_read(struct sport_dev *dev)
{
	struct sport_config *cfg = &dev->config;

	if (cfg->word_len <= 8)
		while (dev->rx_received < dev->rx_len &&
				(dev->regs->stat & RXNE)) {
			u8 *buf = (void *)dev->rx_buf + dev->rx_received;
			*buf = dev->regs->rx16;
			dev->rx_received += 1;
		}
	else if (cfg->word_len <= 16)
		while (dev->rx_received < dev->rx_len &&
				(dev->regs->stat & RXNE)) {
			u16 *buf = (void *)dev->rx_buf + dev->rx_received;
			*buf = dev->regs->rx16;
			dev->rx_received += 2;
		}
	else
		while (dev->rx_received < dev->rx_len &&
				(dev->regs->stat & RXNE)) {
			u32 *buf = (void *)dev->rx_buf + dev->rx_received;
			*buf = bfin_read_sport_rx32(dev->regs);
			dev->rx_received += 4;
		}
}

static irqreturn_t sport_rx_handler(int irq, void *dev_id)
{
	struct sport_dev *dev = dev_id;
	sport_rx_read(dev);

	if (dev->rx_received >= dev->rx_len) {
		dev->regs->rcr1 &= ~RSPEN;
		complete(&dev->c);
	}

	return IRQ_HANDLED;
}

static inline void sport_tx_write(struct sport_dev *dev)
{
	struct sport_config *cfg = &dev->config;

	if (cfg->word_len <= 8)
		while (dev->tx_sent < dev->tx_len &&
				!(dev->regs->stat & TXF)) {
			u8 *buf = (void *)dev->tx_buf + dev->tx_sent;
			dev->regs->tx16 = *buf;
			dev->tx_sent += 1;
		}
	else if (cfg->word_len <= 16)
		while (dev->tx_sent < dev->tx_len &&
				!(dev->regs->stat & TXF)) {
			u16 *buf = (void *)dev->tx_buf + dev->tx_sent;
			dev->regs->tx16 = *buf;
			dev->tx_sent += 2;
		}
	else
		while (dev->tx_sent < dev->tx_len &&
				!(dev->regs->stat & TXF)) {
			u32 *buf = (void *)dev->tx_buf + dev->tx_sent;
			dev->regs->tx32 = *buf;
			dev->tx_sent += 4;
		}
}

static irqreturn_t sport_tx_handler(int irq, void *dev_id)
{
	struct sport_dev *dev = dev_id;

	if (dev->config.mode != NDSO_MODE) {
		if (dev->tx_sent < dev->tx_len)
			sport_tx_write(dev);
	}

	if (dev->tx_len != 0 && dev->tx_sent >= dev->tx_len
			&& dev->config.int_clk) {
		unsigned int stat;

		stat = dev->regs->stat;
		while (!(stat & TXHRE)) {
			dev_dbg(dev->dev, "%s: stat:%x\n", __func__, stat);
			udelay(1);
			stat = dev->regs->stat;
		}
		udelay(500);
		dev->regs->tcr1 &= ~TSPEN;
		SSYNC();
		dev_dbg(dev->dev, "%s: stat:%x\n", __func__, stat);
		complete(&dev->c);
	}

	return IRQ_HANDLED;
}


static irqreturn_t sport_err_handler(int irq, void *dev_id)
{
	struct sport_dev *dev = dev_id;
	uint16_t status;

	dev_dbg(dev->dev, "%s enter\n", __func__);
	status = dev->regs->stat;

	if (status & (TOVF | TUVF | ROVF | RUVF)) {
		dev->regs->stat = (status & (TOVF | TUVF | ROVF | RUVF));
		if (dev->config.dma_enabled) {
			disable_dma(dev->dma_rx_chan);
			disable_dma(dev->dma_tx_chan);
		}
		dev->regs->tcr1 &= ~TSPEN;
		dev->regs->rcr1 &= ~RSPEN;
		SSYNC();

		if (!dev->config.dma_enabled && !dev->config.int_clk) {
			if (status & TUVF)
				complete(&dev->c);
		} else
			dev_warn(dev->dev, "sport %p status error:%s%s%s%s\n",
					dev->regs,
					(status & TOVF) ? " TOVF" : "",
					(status & TUVF) ? " TUVF" : "",
					(status & ROVF) ? " ROVF" : "",
					(status & RUVF) ? " RUVF" : "");
	}

	/* XXX: should we always complete here and have read/write error ? */

	return IRQ_HANDLED;
}

static inline struct sport_dev *to_sport_dev(struct file *filp)
{
	return filp->private_data;
}

/*
 * Open and close
 */
static int sport_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	int minor;
	struct sport_dev *dev;

	dev_dbg(dev->dev, "%s enter\n", __func__);

	minor = MINOR(inode->i_rdev);
	list_for_each_entry(dev, &sport_list, list)
		if (dev->misc.minor == minor) {
			filp->private_data = dev;
			break;
		}

	mutex_init(&dev->mutex);

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	if (dev->open_count++)
		goto done;

	memset(&dev->config, 0, sizeof(dev->config));

	dev->rx_buf = NULL;
	dev->rx_len = 0;
	dev->rx_received = 0;
	dev->tx_buf = NULL;
	dev->tx_len = 0;
	dev->tx_sent = 0;
	init_completion(&dev->c);

	dev->regs = devm_ioremap(dev->dev, dev->reg_base, dev->reg_len);
	if (!dev->regs) {
		dev_err(dev->dev, "unable to map regs\n");
		goto fail;
	}

	ret = maybe_request_irq(dev->tx_irq, sport_tx_handler, IRQF_SHARED,
			KBUILD_MODNAME "-tx", dev);
	if (ret)
		goto fail;

	ret = maybe_request_irq(dev->rx_irq, sport_rx_handler, IRQF_SHARED,
			KBUILD_MODNAME "-rx", dev);
	if (ret)
		goto fail1;
	ret = maybe_request_irq(dev->err_irq, sport_err_handler, 0,
			KBUILD_MODNAME "-err", dev);
	if (ret)
		goto fail2;

	ret = peripheral_request_list(dev->pin_req, KBUILD_MODNAME);
	if (ret) {
		dev_err(dev->dev, "requesting peripherals failed\n");
		goto fail3;
	}

done:
	mutex_unlock(&dev->mutex);
	return 0;

fail3:
	free_irq(dev->err_irq, dev);
fail2:
	free_irq(dev->rx_irq, dev);
fail1:
	free_irq(dev->tx_irq, dev);
fail:
	mutex_unlock(&dev->mutex);
	return ret;
}

static int sport_release(struct inode *inode, struct file *filp)
{
	struct sport_dev *dev = to_sport_dev(filp);
	dev_dbg(dev->dev, "%s enter\n", __func__);

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	if (--dev->open_count)
		goto done;

	dev->regs->tcr1 &= ~TSPEN;
	dev->regs->rcr1 &= ~RSPEN;

	if (dev->config.dma_enabled) {
		free_dma(dev->dma_rx_chan);
		free_dma(dev->dma_tx_chan);
	} else {
		maybe_free_irq(dev->tx_irq, dev);
		maybe_free_irq(dev->rx_irq, dev);
	}
	maybe_free_irq(dev->err_irq, dev);

	peripheral_free_list(dev->pin_req);

	devm_iounmap(dev->dev, (void *)dev->regs);
done:
	mutex_unlock(&dev->mutex);
	return 0;
}

static void sport_ndso_tx_write(struct sport_dev *dev)
{
	struct sport_config *cfg = &dev->config;
	int dummy;

	dev->regs->tcr1 |= TSPEN;
	dev->regs->rcr1 |= RSPEN;
	if (cfg->word_len <= 8)
		while (dev->tx_sent < dev->tx_len) {
			u8 *buf = (void *)dev->tx_buf + dev->tx_sent;
			dev->regs->tx16 = *buf;
			dev->tx_sent += 1;
			while (!(dev->regs->stat & RXNE))
				cpu_relax();
			dummy = dev->regs->rx16;
		}
	else if (cfg->word_len <= 16)
		while (dev->tx_sent < dev->tx_len) {
			u16 *buf = (void *)dev->tx_buf + dev->tx_sent;
			dev->regs->tx16 = *buf;
			dev->tx_sent += 2;
			while (!(dev->regs->stat & RXNE))
				cpu_relax();
			dummy = dev->regs->rx16;
		}
	else
		while (dev->tx_sent < dev->tx_len) {
			u32 *buf = (void *)dev->tx_buf + dev->tx_sent;
			dev->regs->tx32 = *buf;
			dev->tx_sent += 4;
			while (!(dev->regs->stat & RXNE))
				cpu_relax();
			dummy = dev->regs->rx32;
		}
	dev->regs->tcr1 &= ~TSPEN;
	dev->regs->rcr1 &= ~RSPEN;
}


static void sport_ndso_rx_read(struct sport_dev *dev)
{
	struct sport_config *cfg = &dev->config;

	dev->regs->tcr1 |= TSPEN;
	dev->regs->rcr1 |= RSPEN;
	if (cfg->word_len <= 8)
		while (dev->rx_received < dev->rx_len) {
			u8 *buf = (void *)dev->rx_buf + dev->rx_received;
			dev->regs->tx16 = 0x00;
			while (!(dev->regs->stat & RXNE))
				cpu_relax();
			*buf = dev->regs->rx16;
			dev->rx_received += 1;
		}
	else if (cfg->word_len <= 16)
		while (dev->rx_received < dev->rx_len) {
			u16 *buf = (void *)dev->rx_buf + dev->rx_received;
			dev->regs->tx16 = 0x0000;
			while (!(dev->regs->stat & RXNE))
				cpu_relax();
			*buf = dev->regs->rx16;
			dev->rx_received += 2;
		}
	else
		while (dev->rx_received < dev->rx_len) {
			u32 *buf = (void *)dev->rx_buf + dev->rx_received;
			dev->regs->tx32 = 0x0000;
			while (!(dev->regs->stat & RXNE))
				cpu_relax();
			*buf = dev->regs->rx32;
			dev->rx_received += 4;
		}
	dev->regs->tcr1 &= ~TSPEN;
	dev->regs->rcr1 &= ~RSPEN;
}


static ssize_t sport_read(struct file *filp, char __user *buf, size_t count,
		loff_t *f_pos)
{
	struct sport_dev *dev = to_sport_dev(filp);
	struct sport_config *cfg = &dev->config;
	int status;

	dev_dbg(dev->dev, "%s count:%ld\n", __func__, count);

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	if (cfg->dma_enabled) {
		int word_bytes = (cfg->word_len + 7) / 8;
		uint16_t dma_config, xcount, ycount;

		if (word_bytes == 3)
			word_bytes = 4;

		/* Invalidate the buffer */
		invalidate_dcache_range((unsigned long)buf,
				(unsigned long)(buf + count));
		dev_dbg(dev->dev, "DMA mode read\n");
		dma_cache_sync(dev->dev, buf, count, DMA_FROM_DEVICE);
		/* Configure dma */
		dma_config = WNR | RESTART | DI_EN |
			sport_wordsize(dev, cfg->word_len);
		xcount = count / word_bytes;
		ycount = 0;
		if ((count / word_bytes) > 0x8000) {
			ycount = (count / word_bytes) >> 15;
			xcount = 0x8000;
			dma_config |= DMA2D;
		}
		set_dma_start_addr(dev->dma_rx_chan, (unsigned long)buf);
		set_dma_x_count(dev->dma_rx_chan, xcount);
		set_dma_x_modify(dev->dma_rx_chan, word_bytes);
		if (ycount > 0) {
			set_dma_y_count(dev->dma_rx_chan, ycount);
			set_dma_y_modify(dev->dma_rx_chan, word_bytes);
		}
		set_dma_config(dev->dma_rx_chan, dma_config);

		enable_dma(dev->dma_rx_chan);
	} else {
		dev->rx_buf = buf;
		dev->rx_len = count;
		dev->rx_received = 0;
	}

	if (dev->config.mode == NDSO_MODE) {
		sport_ndso_rx_read(dev);
		goto out;
	}

	dev->regs->rcr1 |= RSPEN;
	status = dev->regs->mcmc2;
	if (status & MCMEN)
		dev->regs->tcr1 |= TSPEN;
	SSYNC();

	if (wait_for_completion_interruptible(&dev->c)) {
		dev_dbg(dev->dev, "Receive a signal to interrupt\n");
		count = -ERESTARTSYS;
		/* fall through */
	}
out:
	dev_dbg(dev->dev, "Complete called in dma rx irq handler\n");
	mutex_unlock(&dev->mutex);

	return count;
}

static ssize_t sport_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	int status;
	struct sport_dev *dev = to_sport_dev(filp);
	struct sport_config *cfg = &dev->config;
	dev_dbg(dev->dev, "%s count:%ld  dma_tx_chan:%d\n",
			__func__, count, dev->dma_tx_chan);

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	/* Configure dma to start transfer */
	if (cfg->dma_enabled) {
		uint16_t dma_config, xcount, ycount;
		int word_bytes = (cfg->word_len + 7) / 8;

		if (word_bytes == 3)
			word_bytes = 4;

		dev_dbg(dev->dev, "DMA mode\n");
		dma_cache_sync(dev->dev, (void *)buf, count, DMA_TO_DEVICE);

		/* Configure dma */
		dma_config = RESTART | DI_EN |
			sport_wordsize(dev, cfg->word_len);
		xcount = count / word_bytes;
		ycount = 0;
		if ((count / word_bytes) > 0x8000) {
			ycount = (count / word_bytes) >> 15;
			xcount = 0x8000;
			dma_config |= DMA2D;
		}
		set_dma_start_addr(dev->dma_tx_chan, (unsigned long)buf);
		set_dma_x_count(dev->dma_tx_chan, xcount);
		set_dma_x_modify(dev->dma_tx_chan, word_bytes);
		if (ycount > 0) {
			set_dma_y_count(dev->dma_tx_chan, ycount);
			set_dma_y_modify(dev->dma_tx_chan, word_bytes);
		}
		set_dma_config(dev->dma_tx_chan, dma_config);

		/* dma irq should not be handled before sport is enabled */
		disable_irq(dev->tx_irq);
		enable_dma(dev->dma_tx_chan);
		dev->regs->tcr1 |= TSPEN;
		enable_irq(dev->tx_irq);
	} else {
		/* Configure parameters to start PIO transfer */
		dev->tx_buf = buf;
		dev->tx_len = count;
		dev->tx_sent = 0;
		if (dev->config.mode == NDSO_MODE) {
			sport_ndso_tx_write(dev);
			goto out;
		}
		sport_tx_write(dev);
		dev->regs->tcr1 |= TSPEN;
	}

	status = dev->regs->mcmc2;
	if (status & MCMEN)
		dev->regs->rcr1 |= RSPEN;
	dev->regs->tcr1 |= TSPEN;
	SSYNC();

	dev_dbg(dev->dev, "wait for transfer finished\n");
	if (wait_for_completion_interruptible(&dev->c)) {
		dev_dbg(dev->dev, "Receive a signal to interrupt\n");
		count = -ERESTARTSYS;
		/* fall through */
	}
	dev_dbg(dev->dev, "waiting over\n");
out:
	mutex_unlock(&dev->mutex);

	return count;
}

static int sport_configure(struct sport_dev *dev, struct sport_config *config)
{
	unsigned int tcr1, tcr2, rcr1, rcr2;
	unsigned int clkdiv, fsdiv;
	struct sport_config *old_cfg = &dev->config;

	tcr1 = tcr2 = rcr1 = rcr2 = 0;
	clkdiv = fsdiv = 0;

	if ((old_cfg->dma_enabled == 0) && (config->dma_enabled)) {
		int ret;
		free_irq(dev->tx_irq, dev);
		free_irq(dev->rx_irq, dev);

		/* Request rx dma and set irq handler */
		ret = request_dma(dev->dma_rx_chan, "sport_rx_dma_chan");
		if (ret) {
			pr_err("unable to request sport rx dma channel\n");
			return ret;
		}
		set_dma_callback(dev->dma_rx_chan, dma_rx_irq_handler, dev);

		/* Request tx dma and set irq handler */
		ret = request_dma(dev->dma_tx_chan, "sport_tx_dma_chan");
		if (ret) {
			pr_err("unable to request sport tx dma channel\n");
			return ret;
		}
		set_dma_callback(dev->dma_tx_chan, dma_tx_irq_handler, dev);
	}
	memcpy(old_cfg, config, sizeof(*config));

	if ((dev->regs->tcr1 & TSPEN) || (dev->regs->rcr1 & RSPEN))
		return -EBUSY;

	if (config->mode == TDM_MODE) {
		if (config->channels & 0x7 || config->channels > 32)
			return -EINVAL;

		sport_set_multichannel(dev->regs, config->channels, 1,
				config->frame_delay);
	} else {
		tcr1 |= (config->lsb_first << 4) | (config->fsync << 10) |
			(config->data_indep << 11) | (config->act_low << 12) |
			(config->late_fsync << 13) | (config->tckfe << 14);
		if (config->sec_en)
			tcr2 |= TXSE;

		rcr1 |= (config->lsb_first << 4) | (config->fsync << 10) |
			(config->data_indep << 11) | (config->act_low << 12) |
			(config->late_fsync << 13) | (config->tckfe << 14);
		if (config->sec_en)
			rcr2 |= RXSE;
	}

	/* Using internal clock */
	if (config->int_clk) {
		u_long sclk = get_sclk();

		if (config->serial_clk < 0 || config->serial_clk > sclk / 2)
			return -EINVAL;
		clkdiv = sclk / (2 * config->serial_clk) - 1;
		fsdiv = config->serial_clk / config->fsync_clk - 1;

		tcr1 |= (ITCLK | ITFS);
		rcr1 |= (IRCLK | IRFS);
	}

	/* Setting data format */
	tcr1 |= (config->data_format << 2);	/* Bit TDTYPE */
	rcr1 |= (config->data_format << 2);	/* Bit TDTYPE */
	if (config->word_len >= 3 && config->word_len <= 32) {
		tcr2 |= config->word_len - 1;
		rcr2 |= config->word_len - 1;
	} else
		return -EINVAL;

	dev->regs->rcr1 = rcr1;
	dev->regs->rcr2 = rcr2;
	dev->regs->rclkdiv = clkdiv;
	dev->regs->rfsdiv = fsdiv;
	dev->regs->tcr1 = tcr1;
	dev->regs->tcr2 = tcr2;
	dev->regs->tclkdiv = clkdiv;
	dev->regs->tfsdiv = fsdiv;
	SSYNC();

	dev_dbg(dev->dev, "tcr1:0x%x, tcr2:0x%x, rcr1:0x%x, rcr2:0x%x\n"
			"mcmc1:0x%x, mcmc2:0x%x\n",
			dev->regs->tcr1, dev->regs->tcr2,
			dev->regs->rcr1, dev->regs->rcr2,
			dev->regs->mcmc1, dev->regs->mcmc2);

	return 0;
}
static long sport_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct sport_dev *dev = to_sport_dev(filp);
	struct sport_config config;

	dev_dbg(dev->dev, "%s: enter, arg:0x%lx\n", __func__, arg);
	switch (cmd) {
	case SPORT_IOC_CONFIG:
		if (copy_from_user(&config, (void *)arg, sizeof(config)))
			return -EFAULT;
		if (sport_configure(dev, &config) < 0)
			return -EFAULT;
	break;

	default:
			return -EINVAL;
	}

	return 0;
}

static const struct file_operations bfin_sport_fops = {
	.owner = THIS_MODULE,
	.read = sport_read,
	.write = sport_write,
	.unlocked_ioctl = sport_ioctl,
	.open = sport_open,
	.release = sport_release,
};

#ifdef CONFIG_PM
static int bfin_sport_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int bfin_sport_resume(struct platform_device *dev)
{
	return 0;
}

#else
#define bfin_sport_suspend NULL
#define bfin_sport_resume  NULL
#endif

static int __devinit bfin_sport_probe(struct platform_device *pdev)
{
	struct sport_dev *dev;
	struct miscdevice *misc;
	int ret;
	struct resource *res;
	unsigned short *pdata;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->dev = &pdev->dev;
	snprintf(dev->name, sizeof(dev->name), "sport%i", pdev->id);
	ret = -ENOENT;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev->dev, "no regs found\n");
		goto err;
	}
	dev->reg_base = res->start;
	dev->reg_len = resource_size(res);

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(dev->dev, "no platform data found\n");
		goto err;
	}
	dev->pin_req = pdata;

	misc = &dev->misc;
	misc->minor = MISC_DYNAMIC_MINOR;
	misc->name = dev->name;
	misc->fops = &bfin_sport_fops;

	ret = misc_register(&dev->misc);
	if (ret) {
		dev_err(dev->dev, "unable to register a misc device\n");
		goto err;
	}
	printk(KERN_ERR "misc = 0x%x\n", misc);

	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	dev->dma_tx_chan = res ? res->start : -1;

	res = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	dev->dma_rx_chan = res ? res->start : -1;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	dev->tx_irq = res ? res->start : -1;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	dev->rx_irq = res ? res->start : -1;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 2);
	dev->err_irq = res ? res->start : -1;

	list_add(&dev->list, &sport_list);
	platform_set_drvdata(pdev, dev);

	return 0;

err:
	kfree(dev);
	return ret;
}

static int __devexit bfin_sport_remove(struct platform_device *pdev)
{
	struct sport_dev *dev = platform_get_drvdata(pdev);
	int ret;

	ret = misc_deregister(&dev->misc);
	if (ret)
		return ret;

	kfree(dev);

	return 0;
}

static struct platform_driver bfin_sport_driver = {
	.driver  = {
		.name  = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe   = bfin_sport_probe,
	.remove  = __devexit_p(bfin_sport_remove),
	.suspend = bfin_sport_suspend,
	.resume  = bfin_sport_resume,
};

static int __init bfin_sport_init(void)
{
	return platform_driver_register(&bfin_sport_driver);
}
module_init(bfin_sport_init);

static void __exit bfin_sport_exit(void)
{
	platform_driver_unregister(&bfin_sport_driver);
}
module_exit(bfin_sport_exit);

MODULE_AUTHOR("Roy Huang <roy.huang@analog.com>");
MODULE_DESCRIPTION("Common Blackfin SPORT driver");
MODULE_LICENSE("GPL");

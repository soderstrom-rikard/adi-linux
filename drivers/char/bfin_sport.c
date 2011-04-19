/*
 * simple char interface to Blackfin SPORT peripheral
 *
 * Copyright 2004-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <asm/atomic.h>
#include <asm/blackfin.h>
#include <asm/bfin_sport.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>
#include <asm/portmux.h>

struct sport_dev {
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

	int sport_mode;
	int sport_clkdiv;

	struct mutex mutex;
	unsigned int open_count;

	struct completion c;

	volatile struct sport_register *regs;
	struct sport_config config;
};

/* XXX: this should get pushed to platform device */
#define SPORT_REQ(x) \
	[x] = {P_SPORT##x##_TFS, P_SPORT##x##_DTPRI, P_SPORT##x##_TSCLK, P_SPORT##x##_DTSEC, \
	       P_SPORT##x##_RFS, P_SPORT##x##_DRPRI, P_SPORT##x##_RSCLK, P_SPORT##x##_DRSEC, 0}
static u16 sport_req[][9] = {
#ifdef SPORT0_TCR1
	SPORT_REQ(0),
#endif
#ifdef SPORT1_TCR1
	SPORT_REQ(1),
#endif
#ifdef SPORT2_TCR1
	SPORT_REQ(2),
#endif
#ifdef SPORT3_TCR1
	SPORT_REQ(3),
#endif
};

#define SPORT_PARAMS(x) \
	[x] = { \
		.dma_rx_chan = CH_SPORT##x##_RX, \
		.dma_tx_chan = CH_SPORT##x##_TX, \
		.rx_irq      = IRQ_SPORT##x##_RX, \
		.tx_irq      = IRQ_SPORT##x##_TX, \
		.err_irq     = IRQ_SPORT##x##_ERROR, \
		.regs        = (void *)SPORT##x##_TCR1, \
	}
static struct sport_dev sport_devs[] = {
#ifdef SPORT0_TCR1
	SPORT_PARAMS(0),
#endif
#ifdef SPORT1_TCR1
	SPORT_PARAMS(1),
#endif
#ifdef SPORT2_TCR1
	SPORT_PARAMS(2),
#endif
#ifdef SPORT3_TCR1
	SPORT_PARAMS(3),
#endif
};

static irqreturn_t dma_rx_irq_handler(int irq, void *dev_id);
static irqreturn_t dma_tx_irq_handler(int irq, void *dev_id);

/* note: multichannel is in units of 8 channels, tdm_count is # channels NOT / 8 ! */
static int sport_set_multichannel(volatile struct sport_register *regs,
				  int tdm_count, int packed, int frame_delay)
{
	if (tdm_count) {
		int shift = 32 - tdm_count;
		unsigned int mask = (0xffffffff >> shift);

		regs->mcmc1 = ((tdm_count >> 3) - 1) << 12;	/* set WSIZE bits */
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

static u16 hz_to_sport_clkdiv(u32 speed_hz)
{
	u_long clk, sclk = get_sclk();
	int div = (sclk / (2 * speed_hz)) - 1;

	if (div < 0)
		div = 0;

	clk = sclk / (2 * (div + 1));

	if (clk > speed_hz)
		div++;

	return div;
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
	} else if (config->mode == I2S_MODE) {
		tcr1 |= (TCKFE | TFSR);
		tcr2 |= TSFSE;

		rcr1 |= (RCKFE | RFSR);
		rcr2 |= RSFSE;
	} else if (config->mode == NDSO_MODE) {
		dev->sport_mode = NDSO_MODE;
		rcr1 = RFSR | LARFS | LRFS;
		tcr1 = ITCLK | ITFS | TFSR | LATFS | LTFS;
		clkdiv = dev->sport_clkdiv;
		fsdiv = config->word_len - 1;
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

	if (!config->dma_enabled) {
		if (config->mode == NDSO_MODE) {
			disable_irq(dev->tx_irq);
			disable_irq(dev->rx_irq);
		} else {
			enable_irq(dev->tx_irq);
			enable_irq(dev->rx_irq);
		}
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

	pr_debug("tcr1:0x%x, tcr2:0x%x, rcr1:0x%x, rcr2:0x%x\n"
		 "mcmc1:0x%x, mcmc2:0x%x, mtcs0:0x%x, mrcs0:0x%x\n",
		 dev->regs->tcr1, dev->regs->tcr2,
		 dev->regs->rcr1, dev->regs->rcr2,
		 dev->regs->mcmc1, dev->regs->mcmc2,
		 dev->regs->mtcs0, dev->regs->mrcs0);

	return 0;
}

static inline uint16_t sport_wordsize(int word_len)
{
	uint16_t wordsize = 0;

	if (word_len <= 8)
		wordsize = WDSIZE_8;
	else if (word_len <= 16)
		wordsize = WDSIZE_16;
	else if (word_len <= 32)
		wordsize = WDSIZE_32;
	else
		pr_err("word_len of %d is invalid\n", word_len);

	return wordsize;
}

static irqreturn_t dma_rx_irq_handler(int irq, void *dev_id)
{
	struct sport_dev *dev = dev_id;
	int status;

	pr_debug("%s enter\n", __func__);
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

	pr_debug("%s enter\n", __func__);
	disable_irq(dev->err_irq);
	status = get_dma_curr_irqstat(dev->dma_tx_chan);
	while (status & DMA_RUN) {
		pr_debug("status:0x%04x\n", status);
		status = get_dma_curr_irqstat(dev->dma_tx_chan);
	}
	status = dev->regs->stat;
	while (!(status & TXHRE)) {
		pr_debug("status:%x\n", status);
		udelay(1);
		status = dev->regs->stat;
	}
	/* Wait for the last byte sent out */
	udelay(500);
	pr_debug("%s status:%x\n", __func__, status);

	dev->regs->tcr1 &= ~TSPEN;
	status = dev->regs->mcmc2;
	if (status & MCMEN)
		dev->regs->rcr1 &= ~RSPEN;
	SSYNC();
	enable_irq(dev->err_irq);
	disable_dma(dev->dma_tx_chan);

	complete(&dev->c);

	/* Clear the interrupt status */
	clear_dma_irqstat(dev->dma_tx_chan);

	return IRQ_HANDLED;
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

	if (dev->sport_mode != NDSO_MODE) {
		if (dev->tx_sent < dev->tx_len)
			sport_tx_write(dev);
	}

	if (dev->tx_len != 0 && dev->tx_sent >= dev->tx_len
	    && dev->config.int_clk) {
		unsigned int stat;

		stat = dev->regs->stat;
		while (!(stat & TXHRE)) {
			pr_debug("%s: stat:%x\n", __func__, stat);
			udelay(1);
			stat = dev->regs->stat;
		}
		udelay(500);
		dev->regs->tcr1 &= ~TSPEN;
		SSYNC();
		pr_debug("%s: stat:%x\n", __func__, stat);
		complete(&dev->c);
	}

	return IRQ_HANDLED;
}

static irqreturn_t sport_err_handler(int irq, void *dev_id)
{
	struct sport_dev *dev = dev_id;
	uint16_t status;

	pr_debug("%s enter\n", __func__);
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
			pr_warning("sport %p status error:%s%s%s%s\n",
			       dev->regs,
			       status & TOVF ? " TOVF" : "",
			       status & TUVF ? " TUVF" : "",
			       status & ROVF ? " ROVF" : "",
			       status & RUVF ? " RUVF" : "");
	}

	/* XXX: should we always complete here and have read/write error ? */

	return IRQ_HANDLED;
}

/*
 * Open and close
 */

static int sport_open(struct inode *inode, struct file *filp)
{
	int ret, minor;
	struct sport_dev *dev;

	pr_debug("%s enter\n", __func__);

	minor = MINOR(inode->i_rdev);
	if (minor >= ARRAY_SIZE(sport_devs))
		return -ENODEV;

	dev = &sport_devs[minor];
	if (!dev->regs)
		return -ENODEV;

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	filp->private_data = dev;	/* for other methods */

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

	ret = request_irq(dev->tx_irq, sport_tx_handler, IRQF_SHARED, KBUILD_MODNAME "-tx", dev);
	if (ret) {
		pr_err("unable to request sport tx irq\n");
		goto fail;
	}

	ret = request_irq(dev->rx_irq, sport_rx_handler, IRQF_SHARED, KBUILD_MODNAME "-rx", dev);
	if (ret) {
		pr_err("unable to request sport rx irq\n");
		goto fail1;
	}

	ret = request_irq(dev->err_irq, sport_err_handler, 0, KBUILD_MODNAME "-err", dev);
	if (ret) {
		pr_err("unable to request sport err irq\n");
		goto fail2;
	}

	ret = peripheral_request_list(sport_req[minor], KBUILD_MODNAME);
	if (ret) {
		pr_err("requesting peripherals failed\n");
		goto fail3;
	}

	dev->sport_clkdiv = 0x24;
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
	int minor;
	struct sport_dev *dev;

	pr_debug("%s enter\n", __func__);

	minor = MINOR(inode->i_rdev);
	dev = &sport_devs[minor];

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
		free_irq(dev->tx_irq, dev);
		free_irq(dev->rx_irq, dev);
	}
	free_irq(dev->err_irq, dev);

	peripheral_free_list(sport_req[minor]);

 done:
	mutex_unlock(&dev->mutex);
	return 0;
}

static ssize_t sport_read(struct file *filp, char __user *buf, size_t count,
			  loff_t *f_pos)
{
	struct sport_dev *dev = filp->private_data;
	struct sport_config *cfg = &dev->config;
	int status;

	pr_debug("%s count:%ld\n", __func__, count);

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
		pr_debug("DMA mode read\n");
		/* Configure dma */
		dma_config =
		    (WNR | RESTART | sport_wordsize(cfg->word_len) | DI_EN);
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

	if (dev->sport_mode == NDSO_MODE) {
		sport_ndso_rx_read(dev);
		goto out;
	}

	dev->regs->rcr1 |= RSPEN;
	status = dev->regs->mcmc2;
	if (status & MCMEN)
		dev->regs->tcr1 |= TSPEN;
	SSYNC();

	if (wait_for_completion_interruptible(&dev->c)) {
		pr_debug("Receive a signal to interrupt\n");
		count = -ERESTARTSYS;
		/* fall through */
	}
out:
	pr_debug("Complete called in dma rx irq handler\n");
	mutex_unlock(&dev->mutex);

	return count;
}

static ssize_t sport_write(struct file *filp, const char __user *buf,
			   size_t count, loff_t *f_pos)
{
	int status;
	struct sport_dev *dev = filp->private_data;
	struct sport_config *cfg = &dev->config;
	pr_debug("%s count:%ld  dma_tx_chan:%d\n",
		 __func__, count, dev->dma_tx_chan);

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	/* Configure dma to start transfer */
	if (cfg->dma_enabled) {
		uint16_t dma_config, xcount, ycount;
		int word_bytes = (cfg->word_len + 7) / 8;

		if (word_bytes == 3)
			word_bytes = 4;

		pr_debug("DMA mode\n");
		flush_dcache_range((unsigned long)buf,
				   (unsigned long)(buf + count));

		/* Configure dma */
		dma_config = (RESTART | sport_wordsize(cfg->word_len) | DI_EN);
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

		enable_dma(dev->dma_tx_chan);
	} else {
		/* Configure parameters to start PIO transfer */
		dev->tx_buf = buf;
		dev->tx_len = count;
		dev->tx_sent = 0;
		if (dev->sport_mode == NDSO_MODE) {
			sport_ndso_tx_write(dev);
			goto out;
		}
		sport_tx_write(dev);
	}

	status = dev->regs->mcmc2;
	if (status & MCMEN)
		dev->regs->rcr1 |= RSPEN;
	dev->regs->tcr1 |= TSPEN;
	SSYNC();

	pr_debug("wait for transfer finished\n");
	if (wait_for_completion_interruptible(&dev->c)) {
		pr_debug("Receive a signal to interrupt\n");
		count = -ERESTARTSYS;
		/* fall through */
	}
	pr_debug("waiting over\n");
out:
	mutex_unlock(&dev->mutex);

	return count;
}

static long sport_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct sport_dev *dev = filp->private_data;
	struct sport_config config;
	unsigned long value;

	pr_debug("%s: enter, arg:0x%lx\n", __func__, arg);
	switch (cmd) {
	case SPORT_IOC_CONFIG:
		if (copy_from_user(&config, (void *)arg, sizeof(config)))
			return -EFAULT;
		if (sport_configure(dev, &config) < 0)
			return -EFAULT;
		break;

	case SPORT_IOC_GET_SYSTEMCLOCK:
		value = get_sclk();
		if (copy_to_user((void *)arg, &value, sizeof(value)))
			return -EFAULT;
		break;

	case SPORT_IOC_SET_BAUDRATE:
		if (arg > (133000000 / 4))
			return -EINVAL;
		dev->sport_clkdiv = hz_to_sport_clkdiv(arg);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t
sport_status_show(struct class *sport_class, struct class_attribute *attr,
                  char *buf)
{
	char *p;
	unsigned short i;
	p = buf;

	for (i = 0; i < ARRAY_SIZE(sport_devs); ++i) {
		if (!sport_devs[i].regs)
			continue;
		p += sprintf(p,
			"sport%d:\n"
			"\trx_irq=%d rx_received=%d tx_irq=%d tx_sent=%d\n"
			"\tmode=%d channels=%d data_format=%d word_len=%d\n",
			i, sport_devs[i].rx_irq,
			sport_devs[i].rx_received,
			sport_devs[i].tx_irq,
			sport_devs[i].tx_sent,
			sport_devs[i].config.mode,
			sport_devs[i].config.channels,
			sport_devs[i].config.data_format,
			sport_devs[i].config.word_len);
	}

	return p - buf;
}

static const struct file_operations sport_fops = {
	.owner = THIS_MODULE,
	.read = sport_read,
	.write = sport_write,
	.unlocked_ioctl = sport_ioctl,
	.open = sport_open,
	.release = sport_release,
};

static struct class *sport_class;

static CLASS_ATTR(status, S_IRUGO, &sport_status_show, NULL);

static struct cdev sport_cdev;
static dev_t sport_dev;

static void __exit sport_cleanup_module(void)
{
	int i;

	cdev_del(&sport_cdev);
	unregister_chrdev_region(sport_dev, ARRAY_SIZE(sport_devs));

	for (i = 0; i < ARRAY_SIZE(sport_devs); ++i)
		if (sport_devs[i].regs)
			device_destroy(sport_class, sport_dev + i);
	class_destroy(sport_class);
}
module_exit(sport_cleanup_module);

static int __init sport_init_module(void)
{
	int ret, i, nr_devs;
	nr_devs = ARRAY_SIZE(sport_devs);

	ret = alloc_chrdev_region(&sport_dev, 0, nr_devs, "sport");
	if (ret) {
		pr_err("can't alloc %i minors\n", nr_devs);
		goto err;
	}

	cdev_init(&sport_cdev, &sport_fops);
	sport_cdev.owner = THIS_MODULE;
	ret = cdev_add(&sport_cdev, sport_dev, nr_devs);
	if (ret) {
		pr_err("cdev_add() failed\n");
		goto err_chrdev;
	}

	sport_class = class_create(THIS_MODULE, "sport");
	ret = class_create_file(sport_class, &class_attr_status);
	if (ret) {
		pr_err("class_create(sport) failed\n");
		goto err_cdev;
	}

	for (i = 0; i < nr_devs; ++i) {
		struct device *dev;
		if (!sport_devs[i].regs)
			continue;

		dev = device_create(sport_class, NULL, sport_dev + i,
		                    &sport_devs[i], "sport%d", i);
		if (!dev)
			goto err_dev;

		mutex_init(&sport_devs[i].mutex);

		pr_info("registered sport%i\n", i);
	}

	return 0;

 err_dev:
	while (--i > 0)
		if (sport_devs[i].regs)
			device_destroy(sport_class, sport_dev + i);
	class_destroy(sport_class);
 err_cdev:
	cdev_del(&sport_cdev);
 err_chrdev:
	unregister_chrdev_region(sport_dev, nr_devs);
 err:
	return ret;
}
module_init(sport_init_module);

MODULE_AUTHOR("Roy Huang <roy.huang@analog.com>");
MODULE_DESCRIPTION("Common Blackfin SPORT driver");
MODULE_LICENSE("GPL");

/*
 * bfin_eppi.c generic blackfin eppi driver
 *
 * Copyright (c) 2012 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>

#include <asm/bfin_ppi.h>
#include <asm/blackfin.h>
#include <asm/cacheflush.h>
#include <asm/dma.h>
#include <asm/portmux.h>

#include "bfin_eppi.h"

#define EPPI_DEVICE_NUM 3

struct eppi_params {
	u32 hdly;
	u32 vdly;
	u32 line;
	u32 frame;
	u32 hcnt;
	u32 vcnt;
	u32 clkdiv;
	u32 control;
};

struct eppi_dev {
	struct cdev cdev;
	struct eppi_params param;
	int dma_ch;
	int irq_err;
	struct bfin_eppi3_regs *reg;
	const unsigned short *per_fs;
	const unsigned short *per_data;
	int dlen;
	int dim;
	wait_queue_head_t waitq;
	spinlock_t lock;
	int opened;
	int busy;
};

static int eppi_major;
static struct eppi_dev *devices;
static struct class *eppi_class;

static const unsigned short eppi0_per_fs[] = {
	P_PPI0_CLK, P_PPI0_FS1, P_PPI0_FS2, 0,
};
static const unsigned short eppi0_per_data[] = {
	P_PPI0_D0, P_PPI0_D1, P_PPI0_D2, P_PPI0_D3,
	P_PPI0_D4, P_PPI0_D5, P_PPI0_D6, P_PPI0_D7,
	P_PPI0_D8, P_PPI0_D9, P_PPI0_D10, P_PPI0_D11,
	P_PPI0_D12, P_PPI0_D13, P_PPI0_D14, P_PPI0_D15,
	P_PPI0_D16, P_PPI0_D17, P_PPI0_D18, P_PPI0_D19,
	P_PPI0_D20, P_PPI0_D21, P_PPI0_D22, P_PPI0_D23,
	0,
};

static const unsigned short eppi1_per_fs[] = {
	P_PPI1_CLK, P_PPI1_FS1, P_PPI1_FS2, 0,
};
static const unsigned short eppi1_per_data[] = {
	P_PPI1_D0, P_PPI1_D1, P_PPI1_D2, P_PPI1_D3,
	P_PPI1_D4, P_PPI1_D5, P_PPI1_D6, P_PPI1_D7,
	P_PPI1_D8, P_PPI1_D9, P_PPI1_D10, P_PPI1_D11,
	P_PPI1_D12, P_PPI1_D13, P_PPI1_D14, P_PPI1_D15,
	P_PPI1_D16, P_PPI1_D17, 0,
};

static const unsigned short eppi2_per_fs[] = {
	P_PPI2_CLK, P_PPI2_FS1, P_PPI2_FS2, 0,
};
static const unsigned short eppi2_per_data[] = {
	P_PPI2_D0, P_PPI2_D1, P_PPI2_D2, P_PPI2_D3,
	P_PPI2_D4, P_PPI2_D5, P_PPI2_D6, P_PPI2_D7,
	P_PPI2_D8, P_PPI2_D9, P_PPI2_D10, P_PPI2_D11,
	P_PPI2_D12, P_PPI2_D13, P_PPI2_D14, P_PPI2_D15,
	P_PPI2_D16, P_PPI2_D17, 0,
};

static int compute_data_len(int dlen)
{
	int bits;
	switch (dlen) {
	case CFG_PPI_DATALEN_8:
		bits = 8;
		break;
	case CFG_PPI_DATALEN_10:
		bits = 10;
		break;
	case CFG_PPI_DATALEN_12:
		bits = 12;
		break;
	case CFG_PPI_DATALEN_14:
		bits = 14;
		break;
	case CFG_PPI_DATALEN_16:
		bits = 16;
		break;
	case CFG_PPI_DATALEN_18:
		bits = 18;
		break;
	case CFG_PPI_DATALEN_20:
		bits = 20;
		break;
	case CFG_PPI_DATALEN_24:
		bits = 24;
		break;
	default:
		bits = -1;
		break;
	}
	return bits;
}

static void eppi_reset(struct eppi_dev *dev)
{
	struct bfin_eppi3_regs *reg = dev->reg;

	bfin_write32(&reg->ctl, 0x0);
	bfin_write32(&reg->line, 0x0);
	bfin_write32(&reg->frame, 0x0);
	bfin_write32(&reg->hdly, 0x0);
	bfin_write32(&reg->vdly, 0x0);
	bfin_write32(&reg->hcnt, 0x0);
	bfin_write32(&reg->vcnt, 0x0);
	bfin_write32(&reg->fs1_wlhb, 0x0);
	bfin_write32(&reg->fs1_paspl, 0x0);
	bfin_write32(&reg->fs2_wlvb, 0x0);
	bfin_write32(&reg->fs2_palpf, 0x0);
	bfin_write32(&reg->clkdiv, 0x0);
	bfin_write32(&reg->stat, 0xc0ff);
	SSYNC();
}

static irqreturn_t eppi_irq(int irq, void *dev_id)
{
	struct eppi_dev *dev = (struct eppi_dev *)dev_id;
	unsigned long flags;

	clear_dma_irqstat(dev->dma_ch);
	if (!(dev->param.control & EPPI_CTL_DIR)) {
		spin_lock_irqsave(&dev->lock, flags);
		dev->busy = 0;
		spin_unlock_irqrestore(&dev->lock, flags);
		wake_up_interruptible(&dev->waitq);
	}

	return IRQ_HANDLED;
}

static irqreturn_t eppi_irq_err(int irq, void *dev_id)
{
	struct eppi_dev *dev = (struct eppi_dev *)dev_id;
	struct bfin_eppi3_regs *reg = dev->reg;
	unsigned long flags;

	bfin_write32(&reg->stat, 0xc0ff);

	if (dev->param.control & EPPI_CTL_DIR) {
		spin_lock_irqsave(&dev->lock, flags);
		dev->busy = 0;
		spin_unlock_irqrestore(&dev->lock, flags);
		wake_up_interruptible(&dev->waitq);
		dev->param.control &= ~EPPI_CTL_EN;
		bfin_write32(&reg->ctl, dev->param.control);
		disable_dma(dev->dma_ch);
	}

	return IRQ_HANDLED;
}

static long eppi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct eppi_dev *dev = filp->private_data;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	switch (cmd) {
	case CMD_PPI_XFR_TYPE:
		if (arg > CFG_PPI_XFR_TYPE_GP)
			goto err;
		dev->param.control &= ~EPPI_CTL_XFRTYPE;
		dev->param.control |= arg << 2;
		break;
	case CMD_PPI_FLD_SEL:
		if (arg > CFG_PPI_FLD_MODE1)
			goto err;
		if (arg == CFG_PPI_FLD_MODE1)
			dev->param.control |= EPPI_CTL_FLDSEL;
		else
			dev->param.control &= ~EPPI_CTL_FLDSEL;
		break;
	case CMD_PPI_CLKGEN:
		if (arg > CFG_PPI_INT_CLK)
			goto err;
		if (arg == CFG_PPI_INT_CLK)
			dev->param.control |= EPPI_CTL_ICLKGEN;
		else
			dev->param.control &= ~EPPI_CTL_ICLKGEN;
		break;
	case CMD_PPI_FSGEN:
		if (arg > CFG_PPI_INT_FS)
			goto err;
		if (arg == CFG_PPI_INT_FS)
			dev->param.control |= EPPI_CTL_IFSGEN;
		else
			dev->param.control &= ~EPPI_CTL_IFSGEN;
		break;
	case CMD_PPI_SIGNEXT:
		if (arg > CFG_PPI_SIGN_EXT)
			goto err;
		if (arg == CFG_PPI_SIGN_EXT)
			dev->param.control |= EPPI_CTL_SIGNEXT;
		else
			dev->param.control &= ~EPPI_CTL_SIGNEXT;
		break;
	case CMD_PPI_POLC:
		if (arg > CFG_PPI_DRISE_SRISE)
			goto err;
		dev->param.control &= ~EPPI_CTL_POLC;
		dev->param.control |= arg << 12;
		break;
	case CMD_PPI_POLS:
		if (arg > CFG_PPI_FS1LO_FS2LO)
			goto err;
		dev->param.control &= ~EPPI_CTL_POLS;
		dev->param.control |= arg << 14;
		break;
	case CMD_PPI_DLEN: {
		int cnt, ret;
		dev->dlen = compute_data_len(arg);
		if (dev->dlen < 0)
			goto err;
		for (cnt = 0; cnt < dev->dlen; cnt++) {
			ret = pinmux_request(dev->per_data[cnt],
					KBUILD_MODNAME);
			if (ret < 0) {
				while (cnt > 0)
					pinmux_free(dev->per_data[--cnt]);
				dev->dlen = 0;
				goto err;
			}
		}
		dev->param.control &= ~EPPI_CTL_DLEN;
		dev->param.control |= arg << 16;
		break;
	}
	case CMD_PPI_DMIRR:
		if (arg > CFG_PPI_MIRR)
			goto err;
		if (arg == CFG_PPI_MIRR)
			dev->param.control |= EPPI_CTL_DMIRR;
		else
			dev->param.control &= ~EPPI_CTL_DMIRR;
		break;
	case CMD_PPI_SKIPEN:
		if (arg > CFG_PPI_SKIP)
			goto err;
		if (arg == CFG_PPI_SKIP)
			dev->param.control |= EPPI_CTL_SKIPEN;
		else
			dev->param.control &= ~EPPI_CTL_SKIPEN;
		break;
	case CMD_PPI_SKIPEO:
		if (arg > CFG_PPI_SKIP_EVEN)
			goto err;
		if (arg == CFG_PPI_SKIP_EVEN)
			dev->param.control |= EPPI_CTL_SKIPEO;
		else
			dev->param.control &= ~EPPI_CTL_SKIPEO;
		break;
	case CMD_PPI_PACKEN:
		if (arg > CFG_PPI_PACK)
			goto err;
		if (arg == CFG_PPI_PACK)
			dev->param.control |= EPPI_CTL_PACKEN;
		else
			dev->param.control &= ~EPPI_CTL_PACKEN;
		break;
	case CMD_PPI_SWAPEN:
		if (arg > CFG_PPI_SWAP)
			goto err;
		if (arg == CFG_PPI_SWAP)
			dev->param.control |= EPPI_CTL_SWAPEN;
		else
			dev->param.control &= ~EPPI_CTL_SWAPEN;
		break;
	case CMD_PPI_SPLTEO:
		if (arg > CFG_PPI_SPLIT_EO)
			goto err;
		if (arg == CFG_PPI_SPLIT_EO)
			dev->param.control |= EPPI_CTL_SPLTEO;
		else
			dev->param.control &= ~EPPI_CTL_SPLTEO;
		break;
	case CMD_PPI_SUBSPLTODD:
		if (arg > CFG_PPI_SUBSPLIT_ODD)
			goto err;
		if (arg == CFG_PPI_SUBSPLIT_ODD)
			dev->param.control |= EPPI_CTL_SUBSPLTODD;
		else
			dev->param.control &= ~EPPI_CTL_SUBSPLTODD;
		break;
	case CMD_PPI_SET_DIMS: {
		int cnt, ret;
		if (arg < CFG_PPI_DIMS_1D || arg > CFG_PPI_DIMS_2D)
			goto err;
		for (cnt = 0; cnt < arg + 1; cnt++) {
			ret = pinmux_request(dev->per_fs[cnt],
					KBUILD_MODNAME);
			if (ret < 0) {
				while (cnt > 0)
					pinmux_free(dev->per_fs[--cnt]);
				goto err;
			}
		}

		dev->dim = arg;
		dev->param.control &= ~EPPI_CTL_FSCFG;
		if (dev->dim == CFG_PPI_DIMS_2D)
			dev->param.control |= EPPI_CTL_SYNC2;
		else
			dev->param.control |= EPPI_CTL_SYNC1;
		break;
	}
	case CMD_PPI_LINE:
		dev->param.line = arg;
		break;
	case CMD_PPI_FRAME:
		dev->param.frame = arg;
		break;
	case CMD_PPI_HDELAY:
		dev->param.hdly = arg;
		break;
	case CMD_PPI_HCOUNT:
		dev->param.hcnt = arg;
		break;
	case CMD_PPI_VDELAY:
		dev->param.vdly = arg;
		break;
	case CMD_PPI_VCOUNT:
		dev->param.vcnt = arg;
		break;
	case CMD_PPI_CLKDIV:
		dev->param.clkdiv = arg;
		break;
	default:
		goto err;
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;
err:
	spin_unlock_irqrestore(&dev->lock, flags);
	return -EINVAL;
}

static ssize_t eppi_read(struct file *filp, char *buf,
		size_t count, loff_t *f_pos)
{
	struct eppi_dev *dev = filp->private_data;
	struct bfin_eppi3_regs *reg = dev->reg;
	int step, dma_config;
	unsigned long flags;

	if (dev->dlen > 16 || (dev->param.control & EPPI_CTL_PACKEN))
		step = 4;
	else
		step = 2;

	if ((count & (step - 1)) || ((unsigned long)buf & (step - 1))) {
		pr_err("eppi_read: DMA buffer \
				address or length is not aligned\n");
		return -EINVAL;
	}

	if (dev->param.hcnt == 0) {
		if (dev->dim == CFG_PPI_DIMS_2D)
			return -EINVAL;
		if (dev->param.control & EPPI_CTL_PACKEN) {
			if (dev->dlen == 8)
				dev->param.hcnt = count;
			else if (dev->dlen > 8 && dev->dlen <= 16)
				dev->param.hcnt = count / 2;
			else
				dev->param.hcnt = count / 3;
		} else {
			if (dev->dlen <= 16)
				dev->param.hcnt = count / 2;
			else
				dev->param.hcnt = count / 4;
		}
		dev->param.line = dev->param.hcnt + dev->param.hdly + 2;
	}

	spin_lock_irqsave(&dev->lock, flags);
	if (dev->busy) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return -EBUSY;
	}
	dev->busy = 1;
	spin_unlock_irqrestore(&dev->lock, flags);

	blackfin_dcache_invalidate_range((unsigned long)buf,
			(unsigned long)buf + count);

	dma_config = DMA_FLOW_STOP | RESTART | WNR;
	if (step == 4)
		dma_config |= WDSIZE_32 | PSIZE_32;
	else
		dma_config |= WDSIZE_16 | PSIZE_16;

	if (dev->dim == CFG_PPI_DIMS_2D) {
		int x_count;
		if (dev->param.control & EPPI_CTL_PACKEN) {
			if (dev->dlen == 8)
				x_count = dev->param.hcnt / 4;
			else if (dev->dlen > 8 && dev->dlen <= 16)
				x_count = dev->param.hcnt / 2;
			else
				x_count = dev->param.hcnt * 3 / 4;
		} else {
			x_count = dev->param.hcnt;
		}
		dma_config |= DMA2D | DI_EN_Y;
		set_dma_x_count(dev->dma_ch, x_count);
		set_dma_x_modify(dev->dma_ch, step);
		set_dma_y_count(dev->dma_ch, dev->param.vcnt);
		set_dma_y_modify(dev->dma_ch, step);
	} else {
		dma_config |= DI_EN;
		set_dma_x_count(dev->dma_ch, count / step);
		set_dma_x_modify(dev->dma_ch, step);
	}
	set_dma_start_addr(dev->dma_ch, (unsigned long)buf);
	set_dma_config(dev->dma_ch, dma_config);

	dev->param.control &= ~EPPI_CTL_DIR;
	dev->param.control |= EPPI_CTL_EN;

	bfin_write32(&reg->line, dev->param.line);
	bfin_write32(&reg->frame, dev->param.frame);
	bfin_write32(&reg->hdly, dev->param.hdly);
	bfin_write32(&reg->vdly, dev->param.vdly);
	bfin_write32(&reg->hcnt, dev->param.hcnt);
	bfin_write32(&reg->vcnt, dev->param.vcnt);
	bfin_write32(&reg->clkdiv, dev->param.clkdiv);
	if (dev->param.control & EPPI_CTL_IFSGEN) {
		bfin_write32(&reg->fs1_wlhb,
				dev->param.hcnt + dev->param.hdly);
		bfin_write32(&reg->fs1_paspl, dev->param.line);
		if (dev->dim == CFG_PPI_DIMS_2D) {
			u32 vsync_width, vsync_period;
			vsync_width = dev->param.line
				* (dev->param.vcnt + dev->param.vdly);
			vsync_period = dev->param.line * dev->param.frame;
			bfin_write32(&reg->fs2_wlvb, vsync_width);
			bfin_write32(&reg->fs2_palpf, vsync_period);
		}
	}

	enable_dma(dev->dma_ch);
	SSYNC();
	bfin_write32(&reg->ctl, dev->param.control);

	if (wait_event_interruptible(dev->waitq, !dev->busy)) {
		dev->param.control &= ~EPPI_CTL_EN;
		bfin_write32(&reg->ctl, dev->param.control);
		clear_dma_irqstat(dev->dma_ch);
		disable_dma(dev->dma_ch);
		dev->busy = 0;
		return -ERESTARTSYS;
	}
	dev->param.control &= ~EPPI_CTL_EN;
	bfin_write32(&reg->ctl, dev->param.control);
	disable_dma(dev->dma_ch);
	return count;
}

static ssize_t eppi_write(struct file *filp, const char *buf,
		size_t count, loff_t *f_pos)
{
	struct eppi_dev *dev = filp->private_data;
	struct bfin_eppi3_regs *reg = dev->reg;
	int step, dma_config;
	unsigned long flags;

	if (dev->dlen > 16 || (dev->param.control & EPPI_CTL_PACKEN))
		step = 4;
	else
		step = 2;

	if ((count & (step - 1)) || ((unsigned long)buf & (step - 1))) {
		pr_err("eppi_write: DMA buffer \
				address or length is not aligned\n");
		return -EINVAL;
	}

	if (dev->param.hcnt == 0) {
		if (dev->dim == CFG_PPI_DIMS_2D)
			return -EINVAL;
		if (dev->param.control & EPPI_CTL_PACKEN) {
			if (dev->dlen == 8)
				dev->param.hcnt = count;
			else if (dev->dlen > 8 && dev->dlen <= 16)
				dev->param.hcnt = count / 2;
			else
				dev->param.hcnt = count / 3;
		} else {
			if (dev->dlen <= 16)
				dev->param.hcnt = count / 2;
			else
				dev->param.hcnt = count / 4;
		}
		dev->param.line = dev->param.hcnt + dev->param.hdly + 2;
	}

	spin_lock_irqsave(&dev->lock, flags);
	if (dev->busy) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return -EBUSY;
	}
	dev->busy = 1;
	spin_unlock_irqrestore(&dev->lock, flags);

	blackfin_dcache_invalidate_range((unsigned long)buf,
			(unsigned long)buf + count);

	dma_config = DMA_FLOW_STOP | RESTART;
	if (step == 4)
		dma_config |= WDSIZE_32 | PSIZE_32;
	else
		dma_config |= WDSIZE_16 | PSIZE_16;

	if (dev->dim == CFG_PPI_DIMS_2D) {
		int x_count;
		if (dev->param.control & EPPI_CTL_PACKEN) {
			if (dev->dlen == 8)
				x_count = dev->param.hcnt / 4;
			else if (dev->dlen > 8 && dev->dlen <= 16)
				x_count = dev->param.hcnt / 2;
			else
				x_count = dev->param.hcnt * 3 / 4;
		} else {
			x_count = dev->param.hcnt;
		}
		dma_config |= DMA2D | DI_EN_Y;
		set_dma_x_count(dev->dma_ch, x_count);
		set_dma_x_modify(dev->dma_ch, step);
		set_dma_y_count(dev->dma_ch, dev->param.vcnt);
		set_dma_y_modify(dev->dma_ch, step);
	} else {
		dma_config |= DI_EN;
		set_dma_x_count(dev->dma_ch, count / step);
		set_dma_x_modify(dev->dma_ch, step);
	}
	set_dma_start_addr(dev->dma_ch, (unsigned long)buf);
	set_dma_config(dev->dma_ch, dma_config);

	dev->param.control |= EPPI_CTL_DIR | EPPI_CTL_EN;

	bfin_write32(&reg->line, dev->param.line);
	bfin_write32(&reg->frame, dev->param.frame);
	bfin_write32(&reg->hdly, dev->param.hdly);
	bfin_write32(&reg->vdly, dev->param.vdly);
	bfin_write32(&reg->hcnt, dev->param.hcnt);
	bfin_write32(&reg->vcnt, dev->param.vcnt);
	bfin_write32(&reg->clkdiv, dev->param.clkdiv);
	if (dev->param.control & EPPI_CTL_IFSGEN) {
		bfin_write32(&reg->fs1_wlhb,
				dev->param.hcnt + dev->param.hdly);
		bfin_write32(&reg->fs1_paspl, dev->param.line);
		if (dev->dim == CFG_PPI_DIMS_2D) {
			u32 vsync_width, vsync_period;
			vsync_width = dev->param.line
				* (dev->param.vcnt + dev->param.vdly);
			vsync_period = dev->param.line * dev->param.frame;
			bfin_write32(&reg->fs2_wlvb, vsync_width);
			bfin_write32(&reg->fs2_palpf, vsync_period);
		}
	}

	enable_dma(dev->dma_ch);
	SSYNC();
	bfin_write32(&reg->ctl, dev->param.control);

	if (wait_event_interruptible(dev->waitq, !dev->busy)) {
		dev->param.control &= ~EPPI_CTL_EN;
		bfin_write32(&reg->ctl, dev->param.control);
		clear_dma_irqstat(dev->dma_ch);
		disable_dma(dev->dma_ch);
		dev->busy = 0;
		return -ERESTARTSYS;
	}
	return count;
}

static int eppi_open(struct inode *inode, struct file *filp)
{
	struct eppi_dev *dev;
	unsigned long flags;
	int ret;

	if (filp->f_flags & O_NONBLOCK)
		return -EINVAL;
	dev = container_of(inode->i_cdev, struct eppi_dev, cdev);

	spin_lock_irqsave(&dev->lock, flags);
	if (dev->opened) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return -EBUSY;
	}
	dev->opened = 1;
	spin_unlock_irqrestore(&dev->lock, flags);
	filp->private_data = dev;

	ret = request_dma(dev->dma_ch, "EPPI_DMA");
	if (ret) {
		pr_err("Unable to allocate DMA channel for EPPI\n");
		dev->opened = 0;
		return ret;
	}
	set_dma_callback(dev->dma_ch, eppi_irq, dev);

	ret = request_irq(dev->irq_err, eppi_irq_err, 0, "EPPI ERROR", dev);
	if (ret) {
		pr_err("Unable to allocate IRQ for EPPI\n");
		free_dma(dev->dma_ch);
		dev->opened = 0;
		return ret;
	}
	eppi_reset(dev);
	return 0;
}

static int eppi_release(struct inode *inode, struct file *filp)
{
	struct eppi_dev *dev = filp->private_data;
	unsigned long flags;
	int cnt;

	eppi_reset(dev);
	clear_dma_irqstat(dev->dma_ch);
	disable_dma(dev->dma_ch);
	free_dma(dev->dma_ch);
	free_irq(dev->irq_err, dev);

	if (dev->dim == CFG_PPI_DIMS_1D)
		for (cnt = 0; cnt < 2; cnt++)
			pinmux_free(dev->per_fs[cnt]);
	else if (dev->dim == CFG_PPI_DIMS_2D)
		for (cnt = 0; cnt < 3; cnt++)
			pinmux_free(dev->per_fs[cnt]);
	for (cnt = 0; cnt < dev->dlen; cnt++)
		pinmux_free(dev->per_data[cnt]);
	dev->dim = CFG_PPI_DIMS_UNDEF;
	dev->dlen = 0;

	spin_lock_irqsave(&dev->lock, flags);
	dev->opened = 0;
	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

static const struct file_operations eppi_fops = {
	.owner = THIS_MODULE,
	.read = eppi_read,
	.write = eppi_write,
	.unlocked_ioctl = eppi_ioctl,
	.open = eppi_open,
	.release = eppi_release,
};

static int eppi_setup_cdev(struct eppi_dev *dev, int index)
{
	int ret;
	dev_t devno = MKDEV(eppi_major, index);

	cdev_init(&dev->cdev, &eppi_fops);
	dev->cdev.owner = THIS_MODULE;
	ret = cdev_add(&dev->cdev, devno, 1);
	if (ret < 0)
		pr_err("Can't add eppi%d\n", index);
	return ret;
}

static int __init bfin_eppi_init(void)
{
	dev_t dev;
	int ret, i;

	ret = alloc_chrdev_region(&dev, 0, EPPI_DEVICE_NUM, "eppi");
	if (ret < 0) {
		pr_err("Can't get major\n");
		return ret;
	}
	eppi_major = MAJOR(dev);

	devices = kzalloc(EPPI_DEVICE_NUM * sizeof(*devices), GFP_KERNEL);
	if (!devices) {
		pr_err("No memory\n");
		ret = -ENOMEM;
		goto err;
	}
	eppi_class = class_create(THIS_MODULE, "eppi");
	if (IS_ERR(eppi_class)) {
		pr_err("Error creating eppi class");
		ret = -ENOMEM;
		goto err1;
	}

	for (i = 0; i < EPPI_DEVICE_NUM; i++) {
		ret = eppi_setup_cdev(&devices[i], i);
		if (ret < 0)
			goto err2;
		spin_lock_init(&devices[i].lock);
		init_waitqueue_head(&devices[i].waitq);
		devices[i].opened = 0;
		devices[i].busy = 0;
		devices[i].dim = CFG_PPI_DIMS_UNDEF;
		devices[i].dlen = 0;
		device_create(eppi_class, NULL, MKDEV(eppi_major, i),
					NULL, "eppi%d", i);
	}

	devices[0].dma_ch = CH_EPPI0_CH0;
	devices[0].irq_err = IRQ_EPPI0_STAT;
	devices[0].reg = (struct bfin_eppi3_regs *)EPPI0_STAT;
	devices[0].per_fs = eppi0_per_fs;
	devices[0].per_data = eppi0_per_data;

	devices[1].dma_ch = CH_EPPI1_CH0;
	devices[1].irq_err = IRQ_EPPI1_STAT;
	devices[1].reg = (struct bfin_eppi3_regs *)EPPI1_STAT;
	devices[1].per_fs = eppi1_per_fs;
	devices[1].per_data = eppi1_per_data;

	devices[2].dma_ch = CH_EPPI2_CH0;
	devices[2].irq_err = IRQ_EPPI2_STAT;
	devices[2].reg = (struct bfin_eppi3_regs *)EPPI2_STAT;
	devices[2].per_fs = eppi2_per_fs;
	devices[2].per_data = eppi2_per_data;

	return 0;
err2:
	while (i > 0) {
		i--;
		device_destroy(eppi_class, MKDEV(eppi_major, i));
		cdev_del(&devices[i].cdev);
	}
	class_destroy(eppi_class);
err1:
	kfree(devices);
err:
	unregister_chrdev_region(dev, EPPI_DEVICE_NUM);
	return ret;
}

static void __exit bfin_eppi_exit(void)
{
	dev_t dev = MKDEV(eppi_major, 0);
	int i;

	for (i = 0; i < EPPI_DEVICE_NUM; i++) {
		device_destroy(eppi_class, MKDEV(eppi_major, i));
		cdev_del(&devices[i].cdev);
	}
	class_destroy(eppi_class);
	kfree(devices);
	unregister_chrdev_region(dev, EPPI_DEVICE_NUM);
}

module_init(bfin_eppi_init);
module_exit(bfin_eppi_exit);

MODULE_DESCRIPTION("blackfin eppi driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");

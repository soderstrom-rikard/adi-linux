/*
 * File:         bf5xx_ac97_sport.c
 * Based on:
 * Author:       Roy Huang <roy.huang@analog.com>
 *
 * Created:      Tue Sep 21 10:52:42 CEST 2004
 * Description:
 *
 * Rev:          $Id: $
 *
 *               Copyright 2004-2007 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */


#define DEBUG
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <asm/bug.h>
#include <asm/dma.h>
#include <asm/blackfin.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>

#include "bf5xx-sport.h"

/* delay between frame sync pulse and first data bit in multichannel mode */
#define FRAME_DELAY (1<<12)

/* note: multichannel is in units of 8 channels,
 * tdm_count is # channels NOT / 8 ! */
int sport_set_multichannel(struct sport_device *sport,
		int tdm_count, int packed)
{
	pr_debug("%s( tdm_count=%d packed=%d )\n", __FUNCTION__, tdm_count, \
			packed);

	if ((sport->regs->tcr1 & TSPEN) || (sport->regs->rcr1 & RSPEN))
		return -EBUSY;

	if (tdm_count & 0x7)
		return -EINVAL;

	if (tdm_count > 32)
		return -EINVAL;  /* don't feel like overdoing it today :-) */

	SSYNC(); /* is this really neccesary? */

	if (tdm_count) {
		int shift = 32 - tdm_count;
		unsigned int mask = (0xffffffff >> shift);

		sport->regs->mcmc1 = ((tdm_count>>3)-1) << 12;
		sport->regs->mcmc2 = FRAME_DELAY | MCMEN | \
				(packed ? (MCDTXPE|MCDRXPE) : 0);

		sport->regs->mtcs0 = mask;
		sport->regs->mrcs0 = mask;
	} else {
		sport->regs->mcmc1 = 0;
		sport->regs->mcmc2 = 0;

		sport->regs->mtcs0 = 0;
		sport->regs->mrcs0 = 0;
	}

	sport->regs->mtcs1 = 0; sport->regs->mtcs2 = 0; sport->regs->mtcs3 = 0;
	sport->regs->mrcs1 = 0; sport->regs->mrcs2 = 0; sport->regs->mrcs3 = 0;

	SSYNC();

	return 0;
}

int sport_config_rx(struct sport_device *sport, unsigned int rcr1,
		unsigned int rcr2, unsigned int clkdiv, unsigned int fsdiv)
{
	if ((sport->regs->tcr1 & TSPEN) || (sport->regs->rcr1 & RSPEN))
		return -EBUSY;

	sport->regs->rcr1 = rcr1;
	sport->regs->rcr2 = rcr2;
	sport->regs->rclkdiv = clkdiv;
	sport->regs->rfsdiv = fsdiv;

	SSYNC();

	return 0;
}

int sport_config_tx(struct sport_device *sport, unsigned int tcr1,
		unsigned int tcr2, unsigned int clkdiv, unsigned int fsdiv)
{
	if ((sport->regs->tcr1 & TSPEN) || (sport->regs->rcr1 & RSPEN))
		return -EBUSY;

	sport->regs->tcr1 = tcr1;
	sport->regs->tcr2 = tcr2;
	sport->regs->tclkdiv = clkdiv;
	sport->regs->tfsdiv = fsdiv;

	SSYNC();

	return 0;
}

static void setup_desc(struct dmasg *desc, void *buf, int fragcount,
		size_t fragsize, unsigned int cfg,
		unsigned int x_count, unsigned int ycount, size_t size)
{

	int i;

	for (i = 0; i < fragcount; ++i) {
		desc[i].next_desc_addr  = (unsigned long)&(desc[i + 1]);
		desc[i].start_addr = (unsigned long)buf + i*fragsize;
		desc[i].cfg = cfg;
		desc[i].x_count = x_count;
		desc[i].x_modify = size;
		desc[i].y_count = ycount;
		desc[i].y_modify = size;
	}

	/* make circular */
	desc[fragcount-1].next_desc_addr = (unsigned long)desc;

	/* printk(KERN_ERR "setup desc: desc0=%p, next0=%lx, desc1=%p,"
		"next1=%lx\nx_count=%x,y_count=%x,addr=0x%lx,cfs=0x%x\n",
		&(desc[0]), desc[0].next_desc_addr,
		&(desc[1]), desc[1].next_desc_addr,
		desc[0].x_count, desc[0].y_count,
		desc[0].start_addr,desc[0].cfg);
	*/
}

static int sport_start(struct sport_device *sport)
{
	enable_dma(sport->dma_rx_chan);
	enable_dma(sport->dma_tx_chan);
	sport->regs->tcr1 |= TSPEN;
	sport->regs->rcr1 |= RSPEN;
	SSYNC();

	return 0;
}

static int sport_stop(struct sport_device *sport)
{
	sport->regs->tcr1 &= ~TSPEN;
	sport->regs->rcr1 &= ~RSPEN;
	SSYNC();

	disable_dma(sport->dma_rx_chan);
	disable_dma(sport->dma_tx_chan);

	return 0;
}

static inline int sport_hook_rx_dummy(struct sport_device *sport)
{
	struct dmasg *desc, temp_desc;
	unsigned long flags;
	struct dma_register *dma = sport->dma_rx;

	BUG_ON(sport->dummy_rx_desc == NULL);
	BUG_ON(sport->curr_rx_desc == sport->dummy_rx_desc);

	/* Maybe the dummy buffer descriptor ring is damaged */
	sport->dummy_rx_desc->next_desc_addr = \
			(unsigned long)sport->dummy_rx_desc;

	local_irq_save(flags);
	desc = (struct dmasg*)dma->next_desc_ptr;
	/* Copy the descriptor which will be damaged to backup */
	temp_desc = *desc;
	desc->x_count = 0x10;
	desc->y_count = 0;
	desc->next_desc_addr = (unsigned long)(sport->dummy_rx_desc);
	local_irq_restore(flags);
	/* Waiting for dummy buffer descriptor is already hooked*/
	while ((*(volatile unsigned long*)&dma->curr_desc_ptr - \
			sizeof(struct dmasg)) != \
			(unsigned long)sport->dummy_rx_desc) {}
	sport->curr_rx_desc = sport->dummy_rx_desc;
	/* Restore the damaged descriptor */
	*desc = temp_desc;

	return 0;
}

static inline int sport_rx_dma_start(struct sport_device *sport, int dummy)
{
	struct dma_register *dma = sport->dma_rx;

	if (dummy) {
		sport->dummy_rx_desc->next_desc_addr = \
				(unsigned long) sport->dummy_rx_desc;
		sport->curr_rx_desc = sport->dummy_rx_desc;
	} else
		sport->curr_rx_desc = sport->dma_rx_desc;

	dma->next_desc_ptr = (unsigned long)(sport->curr_rx_desc);
	dma->cfg           = DMAFLOW | NDSIZE | WDSIZE_32 | WNR;
	dma->x_count       = 0;
	dma->x_modify      = 0;
	dma->y_count       = 0;
	dma->y_modify      = 0;

	SSYNC();

	return 0;
}

static inline int sport_tx_dma_start(struct sport_device *sport, int dummy)
{
	struct dma_register *dma = sport->dma_tx;

	if (dummy) {
		sport->dummy_tx_desc->next_desc_addr = \
				(unsigned long) sport->dummy_tx_desc;
		sport->curr_tx_desc = sport->dummy_tx_desc;
	} else
		sport->curr_tx_desc = sport->dma_tx_desc;

	dma->next_desc_ptr = (unsigned long)(sport->curr_tx_desc);
	dma->cfg           = DMAFLOW | NDSIZE | WDSIZE_32 ;
	dma->x_count       = 0;
	dma->x_modify      = 0;
	dma->y_count       = 0;
	dma->y_modify      = 0;

	SSYNC();

	return 0;
}

int sport_rx_start(struct sport_device *sport)
{
	unsigned long flags;

	if (sport->rx_run)
		return -EBUSY;

	if (sport->tx_run) {
		/* tx is running, rx is not running */
		BUG_ON(sport->dma_rx_desc == NULL);
		BUG_ON(sport->curr_rx_desc != sport->dummy_rx_desc);
		local_irq_save(flags);
		sport->dummy_rx_desc->next_desc_addr = \
				(unsigned long)(sport->dma_rx_desc);
		local_irq_restore(flags);
		sport->curr_rx_desc = sport->dma_rx_desc;
	} else {
		sport_tx_dma_start(sport, 1);
		sport_rx_dma_start(sport, 0);
		sport_start(sport);
	}

	sport->rx_run = 1;

	return 0;
}

int sport_rx_stop(struct sport_device *sport)
{
	if (!sport->rx_run)
		return 0;

	if (sport->tx_run) {
		/* TX dma is still running, hook the dummy buffer */
		sport_hook_rx_dummy(sport);
	} else {
		/* Both rx and tx dma will be stopped */
		sport_stop(sport);
		sport->curr_rx_desc = NULL;
		sport->curr_tx_desc = NULL;
	}

	sport->rx_run = 0;

	return 0;
}

static inline int sport_hook_tx_dummy(struct sport_device *sport)
{
	struct dmasg *desc, temp_desc;
	unsigned long flags;
	struct dma_register *dma = sport->dma_tx;

	BUG_ON(sport->dummy_tx_desc == NULL);
	BUG_ON(sport->curr_tx_desc == sport->dummy_tx_desc);

	sport->dummy_tx_desc->next_desc_addr = \
			(unsigned long)sport->dummy_tx_desc;

	/* Shorten the time on last normal descriptor */
	local_irq_save(flags);
	desc = (struct dmasg*)dma->next_desc_ptr;
	/* Store the descriptor which will be damaged */
	temp_desc = *desc;
	desc->x_count = 0x10;
	desc->y_count = 0;
	desc->next_desc_addr = (unsigned long)(sport->dummy_tx_desc);
	local_irq_restore(flags);
	/* Waiting for dummy buffer descriptor is already hooked*/
	while ((*(volatile unsigned long*)&dma->curr_desc_ptr - \
			sizeof(struct dmasg)) != \
			(unsigned long)sport->dummy_tx_desc) {}
	sport->curr_tx_desc = sport->dummy_tx_desc;
	/* Restore the damaged descriptor */
	*desc = temp_desc;

	return 0;
}

int sport_tx_start(struct sport_device *sport)
{
	unsigned flags;

	pr_debug("%s: tx_run:%d, rx_run:%d\n", __FUNCTION__,
			sport->tx_run, sport->rx_run);
	if (sport->tx_run)
		return -EBUSY;

	if (sport->rx_run) {
		BUG_ON(sport->dma_tx_desc == NULL);
		BUG_ON(sport->curr_tx_desc != sport->dummy_tx_desc);
		/* Hook the normal buffer descriptor */
		local_irq_save(flags);
		sport->dummy_tx_desc->next_desc_addr = \
				(unsigned long)(sport->dma_tx_desc);
		local_irq_restore(flags);
		sport->curr_tx_desc = sport->dma_tx_desc;
	} else {
		sport_tx_dma_start(sport, 0);
		/* Let rx dma run the dummy buffer */
		sport_rx_dma_start(sport, 1);
		sport_start(sport);
	}
	sport->tx_run = 1;

	return 0;
}

int sport_tx_stop(struct sport_device *sport)
{
	if (!sport->tx_run)
		return 0;

	if (sport->rx_run) {
		/* RX is still running, hook the dummy buffer */
		sport_hook_tx_dummy(sport);
	} else {
		/* Both rx and tx dma stopped */
		sport_stop(sport);
		sport->curr_rx_desc = NULL;
		sport->curr_tx_desc = NULL;
	}

	sport->tx_run = 0;

	return 0;
}

static int inline compute_wdsize(size_t size)
{
	switch (size) {
	case 1:
		return WDSIZE_8;
	case 2:
		return WDSIZE_16;
	case 4:
	default:
		return WDSIZE_32;
	}
}

int sport_config_rx_dma(struct sport_device *sport, void *buf,
		int fragcount, size_t fragsize, size_t size)
{
	unsigned int x_count;
	unsigned int y_count;
	unsigned int cfg;
	dma_addr_t addr;

	pr_debug("%s( %p, %d, 0x%lx %ld )\n", __FUNCTION__, buf,
			fragcount, fragsize, size);

	x_count = fragsize / size;
	y_count = 0;

	/* for fragments larger than 64k words we use 2d dma,
	 * denote fragecount as two numbers' mutliply and both of them
	 * are less than 64k.*/
	if (x_count >= 0x10000) {
		int i, count = x_count;

		for (i = 16; i > 0; i--) {
			x_count = 1 << i;
			if ((count & (x_count - 1)) == 0) {
				y_count = count >> i;
				if (y_count < 0x10000)
					break;
			}
		}
		if (i == 0)
			return -EINVAL;
	}
	pr_debug("%s(x_count:0x%x, y_count:0x%x)\n", __FUNCTION__,
			x_count, y_count);

	if (sport->dma_rx_desc) {
		dma_free_coherent(NULL, sport->rx_desc_bytes, \
				sport->dma_rx_desc, 0);
	}

	/* Allocate a new descritor ring as current one. */
	sport->dma_rx_desc = dma_alloc_coherent(NULL, \
			fragcount * sizeof(struct dmasg),	&addr, 0);
	sport->rx_desc_bytes = fragcount * sizeof(struct dmasg);

	if (!sport->dma_rx_desc) {
		return -ENOMEM;
	}

	sport->rx_buf = buf;

	cfg     = 0x7000 | DI_EN | compute_wdsize(size) | WNR | \
		  (DESC_ELEMENT_COUNT << 8); /* large descriptor mode */

	if (y_count != 0)
		cfg |= DMA2D;

	setup_desc(sport->dma_rx_desc, buf, fragcount, fragsize,
			cfg|DMAEN, x_count, y_count, size);

	return 0;
}

int sport_config_tx_dma(struct sport_device *sport, void *buf,
		int fragcount, size_t fragsize, size_t size)
{
	unsigned int x_count;
	unsigned int y_count;
	unsigned int cfg;
	dma_addr_t addr;

	pr_debug("%s( %p, %d, %lx, %lx )\n", __FUNCTION__, buf,
			fragcount, fragsize, size);

	x_count = fragsize/size;
	y_count = 0;

	/* for fragments larger than 64k words we use 2d dma,
	 * denote fragecount as two numbers' mutliply and both of them
	 * are less than 64k.*/
	if (x_count >= 0x10000) {
		int i, count = x_count;

		for (i = 16; i > 0; i--) {
			x_count = 1 << i;
			if ((count & (x_count - 1)) == 0) {
				y_count = count >> i;
				if (y_count < 0x10000)
					break;
			}
		}
		if (i == 0)
			return -EINVAL;
	}
	pr_debug("%s(x_count:0x%x, y_count:0x%x)\n", __FUNCTION__,
			x_count, y_count);


	if (sport->dma_tx_desc) {
		dma_free_coherent(NULL, sport->tx_desc_bytes, \
				sport->dma_tx_desc, 0);
	}

	sport->dma_tx_desc = dma_alloc_coherent(NULL, \
			fragcount * sizeof(struct dmasg), &addr, 0);
	sport->tx_desc_bytes = fragcount * sizeof(struct dmasg);
	if (!sport->dma_tx_desc) {
		return -ENOMEM;
	}

	sport->tx_buf = buf;

	cfg     = 0x7000 | DI_EN | compute_wdsize(size) | \
		  (DESC_ELEMENT_COUNT << 8); /* large descriptor mode */

	if (y_count != 0)
		cfg |= DMA2D;

	setup_desc(sport->dma_tx_desc, buf, fragcount, fragsize,
			cfg|DMAEN, x_count, y_count, size);

	return 0;
}

/* setup dummy dma descriptor ring, which don't generate interrupts,
 * the x_modify is set to 0 */
static int sport_config_rx_dummy(struct sport_device *sport, size_t size)
{
	struct dma_register *dma;
	struct dmasg *desc;
	unsigned config;

	pr_debug("%s entered\n", __FUNCTION__);
	dma = sport->dma_rx;
#if L1_DATA_A_LENGTH != 0
	desc = (struct dmasg*)l1_data_sram_alloc(2 * sizeof(*desc));
#else
	{
		dma_addr_t addr;
		desc = dma_alloc_coherent(NULL, 2 * sizeof(*desc), &addr, 0);
	}
#endif
	if (desc == NULL)
		return -ENOMEM;

	memset(desc, 0, 2 * sizeof(*desc));
	sport->dummy_rx_desc = desc;

	desc->next_desc_addr = (unsigned long)desc;
	desc->start_addr = (unsigned long)sport->dummy_buf;
	config = DMAFLOW | NDSIZE | compute_wdsize(size) | WNR | DMAEN;
	desc->cfg = config;
	desc->x_count = 0x80;
	desc->x_modify = 0;
	desc->y_count = 0;
	desc->y_modify = 0;

	return 0;
}

static int sport_config_tx_dummy(struct sport_device *sport, size_t size)
{
	struct dma_register *dma;
	struct dmasg *desc;
	unsigned int config;

	pr_debug("%s entered\n", __FUNCTION__);
	dma = sport->dma_tx;

#if L1_DATA_A_LENGTH != 0
	desc = (struct dmasg*)l1_data_sram_alloc(2 * sizeof(*desc));
#else
	{
		dma_addr_t addr;
		desc = dma_alloc_coherent(NULL, 2*sizeof(*desc), &addr, 0);
	}
#endif
	if (!desc)
		return -ENOMEM;

	memset(desc, 0, 2 * sizeof(*desc));
	sport->dummy_tx_desc = desc;

	desc->next_desc_addr = (unsigned long)desc;
	desc->start_addr = (unsigned long)sport->dummy_buf + size;
	config = DMAFLOW | NDSIZE | compute_wdsize(size) | DMAEN;
	desc->cfg = config;
	desc->x_count = 0x80;
	desc->x_modify = 0;
	desc->y_count = 0;
	desc->y_modify = 0;

	return 0;
}

unsigned long sport_curr_offset_rx(struct sport_device *sport)
{
	struct dma_register *dma = sport->dma_rx;
	unsigned char *curr = *(unsigned char**) &(dma->curr_addr_ptr);
	return (curr - sport->rx_buf);
}

unsigned long sport_curr_offset_tx(struct sport_device *sport)
{
	struct dma_register *dma = sport->dma_tx;
	unsigned char *curr = *(unsigned char**) &(dma->curr_addr_ptr);
	return (curr - sport->tx_buf);
}

void incfrag(struct sport_device *sport, int *frag, int tx)
{
	++(*frag);
	if (tx == 1 && *frag == sport->tx_frags)
		*frag = 0;

	if (tx == 0 && *frag == sport->rx_frags)
		*frag = 0;
}

void decfrag(struct sport_device *sport, int *frag, int tx)
{
	--(*frag);
	if (tx == 1 && *frag == 0)
		*frag = sport->tx_frags;

	if (tx == 0 && *frag == 0)
		*frag = sport->rx_frags;
}

static int sport_check_status(struct sport_device *sport,
		unsigned int *sport_stat,
		unsigned int *rx_stat,
		unsigned int *tx_stat)
{
	int status = 0;

	if (sport_stat) {
		SSYNC();
		status = sport->regs->stat;
		if (status & (TOVF|TUVF|ROVF|RUVF))
			sport->regs->stat = (status & (TOVF|TUVF|ROVF|RUVF));
		SSYNC();
		*sport_stat = status;
	}

	if (rx_stat) {
		SSYNC();
		status = sport->dma_rx->irq_status;
		if (status & (DMA_DONE|DMA_ERR))
			sport->dma_rx->irq_status = status & (DMA_DONE|DMA_ERR);
		SSYNC();
		*rx_stat = status;
	}

	if (tx_stat) {
		SSYNC();
		status = sport->dma_tx->irq_status;
		if (status & (DMA_DONE|DMA_ERR))
			sport->dma_tx->irq_status = status & (DMA_DONE|DMA_ERR);
		SSYNC();
		*tx_stat = status;
	}

	return 0;
}

int  sport_dump_stat(struct sport_device *sport, char *buf, size_t len)
{
	int ret;

	ret = snprintf(buf, len,
			"sts: 0x%04x\n"
			"rx dma %d cfg: 0x%04x sts: 0x%04x\n"
			"tx dma %d cfg: 0x%04x sts: 0x%04x\n",
			sport->regs->stat,
			sport->dma_rx_chan, sport->dma_rx->cfg,
			sport->dma_rx->irq_status,
			sport->dma_tx_chan, sport->dma_tx->cfg,
			sport->dma_tx->irq_status);
	buf += ret;
	len -= ret;

	ret += snprintf(buf, len,
			"curr_rx_desc:0x%p, curr_tx_desc:0x%p\n"
			"dma_rx_desc:0x%p, dma_tx_desc:0x%p\n"
			"dummy_rx_desc:0x%p, dummy_tx_desc:0x%p\n",
			sport->curr_rx_desc, sport->curr_tx_desc,
			sport->dma_rx_desc, sport->dma_tx_desc,
			sport->dummy_rx_desc, sport->dummy_tx_desc);

	return ret;
}

static irqreturn_t rx_handler(int irq, void *dev_id)
{
	unsigned int rx_stat;
	struct sport_device *sport = dev_id;

	sport_check_status(sport, NULL, &rx_stat, NULL);
	if (!(rx_stat & DMA_DONE)) {
		printk(KERN_ERR "rx dma is already stopped\n");
	}
	if (sport->rx_callback) {
		sport->rx_callback(sport->private_data);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static irqreturn_t tx_handler(int irq, void *dev_id)
{
	unsigned int tx_stat;
	struct sport_device *sport = dev_id;

	sport_check_status(sport, NULL, NULL, &tx_stat);
	if (!(tx_stat & DMA_DONE)) {
		printk(KERN_ERR "tx dma is already stopped\n");
		return IRQ_HANDLED;
	}

	if (sport->tx_callback) {
		sport->tx_callback(sport->private_data);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static irqreturn_t err_handler(int irq, void *dev_id)
{
	unsigned int status;
	struct sport_device *sport = dev_id;

	pr_debug("%s\n", __FUNCTION__);
	if (sport_check_status(sport, &status, NULL, NULL)) {
		printk(KERN_ERR "error checking status ??");
		return IRQ_NONE;
	}

	if (status & (TOVF|TUVF|ROVF|RUVF)) {
		printk(KERN_WARNING  "sport status error:%s%s%s%s\n",
				status & TOVF ? " TOVF" : "",
				status & TUVF ? " TUVF" : "",
				status & ROVF ? " ROVF" : "",
				status & RUVF ? " RUVF" : "");
		sport_stop(sport);
	}

	if (sport->err_callback)
		sport->err_callback(sport->private_data);

	return IRQ_HANDLED;
}

int sport_set_callback(void (*rx_callback)(void *),
		       void (*tx_callback)(void *),
		       void (*err_callback)(void *))
{
	BUG_ON(rx_callback == NULL);
	BUG_ON(tx_callback == NULL);
	BUG_ON(err_callback == NULL);

	sport_handle->rx_callback = rx_callback;
	sport_handle->tx_callback = tx_callback;
	sport_handle->err_callback = err_callback;

	return 0;
}

struct sport_device *sport_init(struct sport_param *param,
		void *private_data)
{
	struct sport_device *sport;

	BUG_ON(param == NULL);
	BUG_ON(sizeof(struct sport_register) != 0x60);
	sport = kmalloc(sizeof(struct sport_device), GFP_KERNEL);
	if (!sport) {
		printk(KERN_ERR "Failed to allocate for sport device\n");
		return NULL;
	}

	memset(sport, 0, sizeof(struct sport_device));
	sport->dma_rx_chan = param->dma_rx_chan;
	sport->dma_tx_chan = param->dma_tx_chan;
	sport->dma_rx = param->dma_rx;
	sport->dma_tx = param->dma_tx;
	sport->err_irq = param->err_irq;
	sport->regs = param->regs;
	sport->private_data = private_data;

	if (request_dma(sport->dma_rx_chan, "SPORT RX Data") == -EBUSY) {
		printk(KERN_ERR "Failed to request RX dma %d\n", \
				sport->dma_rx_chan);
		goto __init_err1;
	}

	if (set_dma_callback(sport->dma_rx_chan, rx_handler, sport) != 0) {
		printk(KERN_ERR "Failed to request RX irq %d\n", \
				sport->dma_rx_chan);
		goto __init_err2;
	}

	if (request_dma(sport->dma_tx_chan, "SPORT TX Data") == -EBUSY) {
		printk(KERN_ERR "Failed to request TX dma %d\n", \
				sport->dma_tx_chan);
		goto __init_err2;
	}

	if (set_dma_callback(sport->dma_tx_chan, tx_handler, sport) != 0) {
		printk(KERN_ERR "Failed to request TX irq %d\n", \
				sport->dma_tx_chan);
		goto __init_err3;
	}

	if (request_irq(sport->err_irq, err_handler, IRQF_SHARED, "SPORT err",
			sport) < 0) {
		printk(KERN_ERR "Failed to request err irq:%d\n", \
				sport->err_irq);
		goto __init_err3;
	}

	pr_debug("dma rx: %p/%d tx: %p/%d, err irq:%d, regs:%p\n",
			sport->dma_rx, sport->dma_rx_chan,
			sport->dma_tx, sport->dma_tx_chan,
			sport->err_irq, sport->regs);

#if L1_DATA_A_LENGTH != 0
	sport->dummy_buf = l1_data_sram_alloc(DUMMY_BUF_LEN);
#else
	sport->dummy_buf = kmalloc(DUMMY_BUF_LEN, GFP_KERNEL);
#endif
	if (sport->dummy_buf == NULL) {
		printk(KERN_ERR "Failed to allocate dummy buffer\n");
		goto __error;
	}

	memset(sport->dummy_buf, 0, DUMMY_BUF_LEN);
	sport_config_rx_dummy(sport, DUMMY_BUF_LEN/2);
	sport_config_tx_dummy(sport, DUMMY_BUF_LEN/2);

	return sport;
__error:
	free_irq(sport->err_irq, sport);
__init_err3:
	free_dma(sport->dma_tx_chan);
__init_err2:
	free_dma(sport->dma_rx_chan);
__init_err1:
	kfree(sport);
	return NULL;
}

void sport_done(struct sport_device *sport)
{
	if (sport == NULL)
		return;

	sport_stop(sport);
	if (sport->dma_rx_desc)
		dma_free_coherent(NULL, sport->rx_desc_bytes, \
				sport->dma_rx_desc, 0);
	if (sport->dma_tx_desc)
		dma_free_coherent(NULL, sport->tx_desc_bytes, \
				sport->dma_tx_desc, 0);

#if L1_DATA_A_LENGTH != 0
	l1_data_sram_free(sport->dummy_rx_desc);
	l1_data_sram_free(sport->dummy_tx_desc);
	l1_data_sram_free(sport->dummy_buf);
#else
	dma_free_coherent(NULL, 2*sizeof(struct dmasg), sport->dummy_rx_desc, \
			0);
	dma_free_coherent(NULL, 2*sizeof(struct dmasg), sport->dummy_tx_desc, \
			0);
	kfree(sport->dummy_buf);
#endif
	free_dma(sport->dma_rx_chan);
	free_dma(sport->dma_tx_chan);
	free_irq(sport->err_irq, sport);

	kfree(sport);
}

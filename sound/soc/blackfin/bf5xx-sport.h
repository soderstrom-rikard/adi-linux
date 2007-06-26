/*
 * File:         bf5xx_ac97_sport.h
 * Based on:
 * Author:       Roy Huang <roy.huang@analog.com>
 *
 * Created:
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


#ifndef __BF5XX_SPORT_H__
#define __BF5XX_SPORT_H__

#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <asm/dma.h>

struct sport_register {
	unsigned short tcr1;    unsigned short reserved0;
	unsigned short tcr2;    unsigned short reserved1;
	unsigned short tclkdiv; unsigned short reserved2;
	unsigned short tfsdiv;  unsigned short reserved3;
	unsigned long tx;
	unsigned long reserved_l0;
	unsigned long rx;
	unsigned long reserved_l1;
	unsigned short rcr1;    unsigned short reserved4;
	unsigned short rcr2;    unsigned short reserved5;
	unsigned short rclkdiv; unsigned short reserved6;
	unsigned short rfsdiv;  unsigned short reserved7;
	unsigned short stat;    unsigned short reserved8;
	unsigned short chnl;    unsigned short reserved9;
	unsigned short mcmc1;   unsigned short reserved10;
	unsigned short mcmc2;   unsigned short reserved11;
	unsigned long mtcs0;
	unsigned long mtcs1;
	unsigned long mtcs2;
	unsigned long mtcs3;
	unsigned long mrcs0;
	unsigned long mrcs1;
	unsigned long mrcs2;
	unsigned long mrcs3;
};

#define DESC_ELEMENT_COUNT 9

struct sport_device {
	int dma_rx_chan;
	int dma_tx_chan;
	int err_irq;
	struct sport_register *regs;

	struct dma_register *dma_rx;
	struct dma_register *dma_tx;
	unsigned char *rx_buf;
	unsigned char *tx_buf;
	unsigned int rx_fragsize;
	unsigned int tx_fragsize;
	unsigned int rx_frags;
	unsigned int tx_frags;

#define DUMMY_BUF_LEN 8
	/* for dummy dma transfer */
	void *dummy_buf;

	/* DMA descriptor ring head of current audio stream*/
	struct dmasg *dma_rx_desc;
	struct dmasg *dma_tx_desc;
	unsigned int rx_desc_bytes;
	unsigned int tx_desc_bytes;

	unsigned int rx_run:1, /* rx is running */
	unsigned int tx_run:1; /* tx is running */

	struct dmasg *dummy_rx_desc;
	struct dmasg *dummy_tx_desc;

	struct dmasg *curr_rx_desc;
	struct dmasg *curr_tx_desc;

	int rx_curr_frag;
	int tx_curr_frag;

	unsigned int rcr1;
	unsigned int rcr2;
	int rx_tdm_count;

	unsigned int tcr1;
	unsigned int tcr2;
	int tx_tdm_count;

	void (*rx_callback)(void *data);
	void (*tx_callback)(void *data);
	void (*err_callback)(void *data);

	void *private_data;
};

extern struct sport_device *sport_handle;

struct sport_param {
	int dma_rx_chan;
	int dma_tx_chan;
	int err_irq;
	struct sport_register *regs;
	struct dma_register *dma_rx;
	struct dma_register *dma_tx;
};

struct sport_device *sport_init(struct sport_param *param,
		void *private_data);

void sport_done(struct sport_device *sport);

/* first use these ...*/

/* note: multichannel is in units of 8 channels, tdm_count is number of channels
 *  NOT / 8 ! all channels are enabled by default */
int sport_set_multichannel(struct sport_device *sport, int tdm_count,
		int packed);

int sport_config_rx(struct sport_device *sport,
		unsigned int rcr1, unsigned int rcr2,
		unsigned int clkdiv, unsigned int fsdiv);

int sport_config_tx(struct sport_device *sport,
		unsigned int tcr1, unsigned int tcr2,
		unsigned int clkdiv, unsigned int fsdiv);

/* ... then these: */

/* buffer size (in bytes) == fragcount * fragsize_bytes */

/* this is not a very general api, it sets the dma to 2d autobuffer mode */

int sport_config_rx_dma(struct sport_device *sport, void *buf,
		int fragcount, size_t fragsize_bytes, size_t size);

int sport_config_tx_dma(struct sport_device *sport, void *buf,
		int fragcount, size_t fragsize_bytes, size_t size);

int sport_tx_start(struct sport_device *sport);
int sport_tx_stop(struct sport_device *sport);
int sport_rx_start(struct sport_device *sport);
int sport_rx_stop(struct sport_device *sport);

/* for use in interrupt handler */
unsigned long sport_curr_offset_rx(struct sport_device *sport);
unsigned long sport_curr_offset_tx(struct sport_device *sport);

void incfrag(struct sport_device *sport, int *frag, int tx);
void decfrag(struct sport_device *sport, int *frag, int tx);

#endif /* BF53X_SPORT_H */

/*
 * File:         sound/blackfin/ad73322.c
 * Based on:	 sound/blackfin/ad73311.c
 * Author:       Cliff Cai <Cliff.Cai@analog.com>
 *
 * Created:      Tue May 06 2008
 * Description:  Driver for AD73322 sound chip connected to bf53x sport
 *
 * Rev:          $Id: ad73322.c 4104 2008-05-06 06:51:48Z cliff $
 *
 * Modified:
 *               Copyright 2008 Analog Devices Inc.
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

#ifndef __AD73322_H__
#define __AD73322_H__

/*8 means 4 AD73322 is connected in cascade mode,since every AD73322 has 2 
 *DAC/ADC pairs
 */
#define NUM_DEVICES_CHAIN 8

#define RUN_TX_ALL 0xff
#define RUN_RX_ALL 0xff

#define AD_CONTROL	0x8000
#define AD_DATA		0x0000
#define AD_READ		0x4000
#define AD_WRITE	0x0000

/* Control register A */
#define CTRL_REG_A	(0 << 8)

#define MODE_PRO	0x00
#define MODE_DATA	0x01
#define MODE_MIXED	0x03
#define DLB		0x04
#define SLB		0x08
#define DEVC(x)		((x & 0x7) << 4)
#define RESET		0x80

/* Control register B */
#define CTRL_REG_B	(1 << 8)

#define DIRATE(x)	(x & 0x3)
#define SCDIV(x)	((x & 0x3) << 2)
#define MCDIV(x)	((x & 0x7) << 4)
#define CEE		(1 << 7)

/* Control register C */
#define CTRL_REG_C	(2 << 8)

#define PUDEV		( 1 << 0 )
#define PUADC		( 1 << 3 )
#define PUDAC		( 1 << 4 )
#define PUREF		( 1 << 5 )
#define REFUSE		( 1 << 6 )

/* Control register D */
#define CTRL_REG_D	(3 << 8)

#define IGS(x)		(x & 0x7)
#define RMOD		( 1 << 3 )
#define OGS(x)		((x & 0x7) << 4)
#define MUTE		(x << 7)

/* Control register E */
#define CTRL_REG_E	(4 << 8)

#define DA(x)		(x & 0x1f)
#define IBYP		( 1 << 5 )

/* Control register F */
#define CTRL_REG_F	(5 << 8)

#define SEEN		( 1 << 5 )
#define INV		( 1 << 6 )
#define ALB		( 1 << 7 )


typedef struct {
	struct snd_pcm_substream*	substream;
	snd_pcm_uframes_t	dma_offset;
	snd_pcm_uframes_t	buffer_frames;
	snd_pcm_uframes_t	period_frames;
	unsigned int		periods;
	unsigned int		frame_bytes;
	/* Information about DMA */
	snd_pcm_uframes_t	dma_inter_pos;
	snd_pcm_uframes_t	dma_last_pos;
	snd_pcm_uframes_t	dma_pos_base;
	/* Information on virtual buffer */
	snd_pcm_uframes_t	next_inter_pos;
	snd_pcm_uframes_t	data_count;
	snd_pcm_uframes_t	data_pos_base;
	snd_pcm_uframes_t	boundary;
} substream_info_t;

typedef struct snd_ad73322 {
	struct snd_card	*card;
	struct bf53x_sport	*sport;
	spinlock_t    ad73322_lock;
	struct snd_pcm	*pcm[NUM_DEVICES_CHAIN];
	int	tx_dma_started;
	int	tx_status;
	int	rx_dma_started;
	int	rx_status;

	snd_pcm_uframes_t	tx_dma_pos;
	snd_pcm_uframes_t	rx_dma_pos;
	substream_info_t	tx_substreams[8];
	substream_info_t	rx_substreams[8];
	unsigned char *tx_dma_buf;
	unsigned char *rx_dma_buf;
} ad73322_t;

#endif

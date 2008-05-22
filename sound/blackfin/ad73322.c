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

/* 
 *This driver supports up to 4 AD73322 connected in cascade mode. 
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <asm/blackfin.h>
#include <asm/cacheflush.h>
#include <asm/irq.h>
#include <asm/gpio.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/info.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#include "ad73311.h"
#include "bf53x_sport.h"

#ifndef CONFIG_BFIN_DMA_5XX
#error "The sound driver requires the Blackfin Simple DMA"
#endif

#ifdef CONFIG_SND_DEBUG
#define snd_printk_marker() snd_printk(KERN_INFO "%s\n", __FUNCTION__)
#else
#define snd_printk_marker()
#endif

#define GPIO_SE CONFIG_SND_BFIN_AD73322_SE
#define GPIO_RESET CONFIG_SND_BFIN_AD73322_RESET
/*8 means 4 AD73322 is connected in cascade mode,since every AD73322 has 2 
 *DAC/ADC pairs
 */
#define NUM_DEVICES_CHAIN 8

#undef CONFIG_SND_DEBUG_CURRPTR  /* causes output every frame! */
#define AD73322_BUF_SZ 0x40000
#define PCM_BUFFER_MAX	0x10000	/* 32KB */
#define FRAGMENT_SIZE_MIN	(2*1024)
#define FRAGMENTS_MIN	2
#define FRAGMENTS_MAX	16
#define WORD_LENGTH	2

#define DMA_BUFFER_BYTES	AD73322_BUF_SZ
#define DMA_PERIOD_BYTES	(FRAGMENT_SIZE_MIN*4)
#define DMA_PERIODS		(DMA_BUFFER_BYTES / DMA_PERIOD_BYTES)
#define DMA_FRAME_BYTES		16
#define DMA_BUFFER_FRAMES	(DMA_BUFFER_BYTES/DMA_FRAME_BYTES)
#define DMA_PERIOD_FRAMES	(DMA_PERIOD_BYTES/DMA_FRAME_BYTES)

#define DRIVER_NAME "AD73322"
#define CHIP_NAME "Analog Devices AD73322"
static char *pcm_name[8] = { "AD73322PCM0", "AD73322PCM1", "AD73322PCM2", "AD73322PCM3", "AD73322PCM4", "AD73322PCM5", "AD73322PCM6", "AD73322PCM7"};

static unsigned int input_gain = 0x2;
module_param(input_gain, uint, 0);
MODULE_PARM_DESC(input_gain, "Input gain setting (0 <= input_gain <= 7)");

static unsigned int output_gain = 0x2;
module_param(output_gain, uint, 0);
MODULE_PARM_DESC(output_gain, "Output gain setting (0 <= output_gain <= 7)");

static struct platform_device *device ;

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
#define RUN_TX0 0x1
#define RUN_TX1 0x2
#define RUN_TX2 0x4
#define RUN_TX3 0x8
#define RUN_TX4 0x10
#define RUN_TX5 0x20
#define RUN_TX6 0x40
#define RUN_TX7 0x80
#define RUN_TX_ALL (RUN_TX0 | RUN_TX1 | RUN_TX2 | RUN_TX3| RUN_TX4| RUN_TX5| RUN_TX6| RUN_TX7)

#define RUN_RX0 0x1
#define RUN_RX1 0x2
#define RUN_RX2 0x4
#define RUN_RX3 0x8
#define RUN_RX4 0x10
#define RUN_RX5 0x20
#define RUN_RX6 0x40
#define RUN_RX7 0x80
#define RUN_RX_ALL (RUN_RX0 | RUN_RX1 | RUN_RX2 | RUN_RX3| RUN_RX4| RUN_RX5| RUN_RX6| RUN_RX7)
	snd_pcm_uframes_t	tx_dma_pos;
	snd_pcm_uframes_t	rx_dma_pos;
	substream_info_t	tx_substreams[8];
	substream_info_t	rx_substreams[8];
	unsigned char *tx_dma_buf;
	unsigned char *rx_dma_buf;
} ad73322_t;

static int snd_ad73322_startup(void);
static void snd_ad73322_stop(void);

static int get_cap_slotindex(int index)
{
	int slot_index = 6;
	switch(index) {
	case 0:slot_index = 6; 
		break;
	case 1:slot_index = 7; 
		break;
	case 2:slot_index = 4; 
		break;
	case 3:slot_index = 5; 
		break;
	case 4:slot_index = 2; 
		break;
	case 5:slot_index = 3; 
		break;
	case 6:slot_index = 0; 
		break;
	case 7:slot_index = 1; 
		break;
	}
	return slot_index;
}

static inline int find_substream(ad73322_t *chip,
		struct snd_pcm_substream *substream,	substream_info_t **info)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (chip->tx_substreams[0].substream == substream) {
			*info = &chip->tx_substreams[0];
			return 0;
		} else if (chip->tx_substreams[1].substream == substream) {
			*info = &chip->tx_substreams[1];
			return 1;
		} else if (chip->tx_substreams[2].substream == substream) {
			*info = &chip->tx_substreams[2];
			return 2;
		} else if (chip->tx_substreams[3].substream == substream) {
			*info = &chip->tx_substreams[3];
			return 3;
		} else if (chip->tx_substreams[4].substream == substream) {
			*info = &chip->tx_substreams[4];
			return 4;
		} else if (chip->tx_substreams[5].substream == substream) {
			*info = &chip->tx_substreams[5];
			return 5;
		} else if (chip->tx_substreams[6].substream == substream) {
			*info = &chip->tx_substreams[6];
			return 6;
		} else if (chip->tx_substreams[7].substream == substream) {
			*info = &chip->tx_substreams[7];
			return 7;
		} 
		else {
			*info = NULL;
			return -1;
		}
	} else {
		if (chip->rx_substreams[0].substream == substream) {
			*info = &chip->rx_substreams[0];
			return 0;
		} else if (chip->rx_substreams[1].substream == substream) {
			*info = &chip->rx_substreams[1];
			return 1;
		} else if (chip->rx_substreams[2].substream == substream) {
			*info = &chip->rx_substreams[2];
			return 2;
		} else if (chip->rx_substreams[3].substream == substream) {
			*info = &chip->rx_substreams[3];
			return 3;
		} else if (chip->rx_substreams[4].substream == substream) {
			*info = &chip->rx_substreams[4];
			return 4;
		} else if (chip->rx_substreams[5].substream == substream) {
			*info = &chip->rx_substreams[5];
			return 5;
		} else if (chip->rx_substreams[6].substream == substream) {
			*info = &chip->rx_substreams[6];
			return 6;
		} else if (chip->rx_substreams[7].substream == substream) {
			*info = &chip->rx_substreams[7];
			return 7;
		} 
		else {
			*info = NULL;
			return -1;
		}
	}
}
/*************************************************************
 *                pcm methods
 *************************************************************/

static struct snd_pcm_hardware snd_ad73322_play_hw = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_RESUME),
	.formats =          SNDRV_PCM_FMTBIT_S16_LE,
	.rates =            SNDRV_PCM_RATE_8000,
	.rate_min =         8000,
	.rate_max =         8000,
	.channels_min =     1,
	.channels_max =     1,
	.buffer_bytes_max = PCM_BUFFER_MAX,
	.period_bytes_min = FRAGMENT_SIZE_MIN,
	.period_bytes_max = PCM_BUFFER_MAX/2,
	.periods_min =      FRAGMENTS_MIN,
	.periods_max =      FRAGMENTS_MAX,
};
static struct snd_pcm_hardware snd_ad73322_cap_hw = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_RESUME),
	.formats =          SNDRV_PCM_FMTBIT_S16_LE,
	.rates =            SNDRV_PCM_RATE_8000,
	.rate_min =         8000,
	.rate_max =         8000,
	.channels_min =     1,
	.channels_max =     1,
	.buffer_bytes_max = PCM_BUFFER_MAX,
	.period_bytes_min = FRAGMENT_SIZE_MIN,
	.period_bytes_max = PCM_BUFFER_MAX/2,
	.periods_min =      FRAGMENTS_MIN,
	.periods_max =      FRAGMENTS_MAX,
};

static int snd_ad73322_play_open(struct snd_pcm_substream *substream)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	snd_printk_marker();
	substream->runtime->hw = snd_ad73322_play_hw;
	chip->tx_substreams[substream->pcm->device].substream = substream;
	return 0;
}

static int snd_ad73322_cap_open(struct snd_pcm_substream *substream)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker();

	substream->runtime->hw = snd_ad73322_cap_hw;
	chip->rx_substreams[substream->pcm->device].substream = substream;

	return 0;
}

static int snd_ad73322_play_close(struct snd_pcm_substream *substream)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	substream_info_t *sub_info = NULL;
	int index, i; 
	int slot_index;
	 
	index = find_substream(chip, substream, &sub_info);
	slot_index = NUM_DEVICES_CHAIN-(index+1);
	snd_printk_marker();
	if (index >= 0 && index <= 7) {
		sub_info->substream = NULL;
		for (i=0; i < DMA_BUFFER_FRAMES; i++)
			*((unsigned short *)chip->tx_dma_buf+i*8 + slot_index) = 0;
	}
	return 0;
}

static int snd_ad73322_cap_close(struct snd_pcm_substream *substream)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	substream_info_t *sub_info = NULL;
	int index, slot_index, i; 
	 
	index = find_substream(chip, substream, &sub_info);
	slot_index  = get_cap_slotindex(index);
	snd_printk_marker();
	if (index >= 0 && index <= 7) {
		sub_info->substream = NULL;
		for (i=0; i < DMA_BUFFER_FRAMES; i++)
			*((unsigned short *)chip->rx_dma_buf+i*8 + slot_index) = 0;
	}
	return 0;
}

static int snd_ad73322_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params* hwparams)
{	ad73322_t *chip = snd_pcm_substream_chip(substream);
	snd_printk_marker();
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		substream->runtime->dma_area = chip->tx_dma_buf;
		substream->runtime->dma_addr = (unsigned int)chip->tx_dma_buf;
		substream->runtime->dma_bytes = AD73322_BUF_SZ;
	} else {
		substream->runtime->dma_area = chip->rx_dma_buf;
		substream->runtime->dma_addr = (unsigned int)chip->rx_dma_buf;
		substream->runtime->dma_bytes = AD73322_BUF_SZ;
	}
	return 0;
}

static int snd_ad73322_hw_free(struct snd_pcm_substream *substream)
{
	snd_printk_marker();
	substream->runtime->dma_area = NULL;
	substream->runtime->dma_addr = 0;
	substream->runtime->dma_bytes = 0;

	return 0;
}

static int snd_ad73322_play_pre(struct snd_pcm_substream *substream)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	substream_info_t *sub_info = NULL;
	int index = find_substream(chip, substream, &sub_info);
	printk(KERN_ERR "play index = %d\n",index);
	snd_assert((index >= 0 && index <=7 && sub_info), return -EINVAL);
	sub_info->period_frames = runtime->period_size;
	sub_info->periods = runtime->periods;
	sub_info->buffer_frames = runtime->buffer_size;
	sub_info->frame_bytes = runtime->frame_bits / 8;
	sub_info->dma_inter_pos = 0;
	sub_info->dma_last_pos = 0;
	sub_info->dma_pos_base = 0;
	sub_info->next_inter_pos = sub_info->period_frames;
	sub_info->data_count = 0;
	sub_info->data_pos_base = 0;
	sub_info->boundary = DMA_BUFFER_FRAMES * sub_info->buffer_frames;

	while (sub_info->boundary * 2 <= (LONG_MAX - DMA_BUFFER_FRAMES * \
			sub_info->buffer_frames)) {
		sub_info->boundary *= 2;
	}
	sub_info->dma_offset = 0;
	
	return 0;
}

static int snd_ad73322_cap_pre(struct snd_pcm_substream *substream)
{

	ad73322_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	substream_info_t *sub_info = NULL;
	int index = find_substream(chip, substream, &sub_info);
	printk(KERN_ERR "cap index = %d\n",index);
	snd_assert((index >= 0 && index <=7 && sub_info), return -EINVAL);
	sub_info->period_frames = runtime->period_size;
	sub_info->periods = runtime->periods;
	sub_info->buffer_frames = runtime->buffer_size;
	sub_info->frame_bytes = runtime->frame_bits / 8;
	sub_info->dma_inter_pos = 0;
	sub_info->dma_last_pos = 0;
	sub_info->dma_pos_base = 0;
	sub_info->next_inter_pos = sub_info->period_frames;
	sub_info->data_count = 0;
	sub_info->data_pos_base = 0;
	sub_info->boundary = DMA_BUFFER_FRAMES * sub_info->buffer_frames;

	while (sub_info->boundary * 2 <= (LONG_MAX - DMA_BUFFER_FRAMES * \
			sub_info->buffer_frames)) {
		sub_info->boundary *= 2;
	}
	sub_info->dma_offset = 0;
	return 0;
}



static int snd_ad73322_play_trigger(struct snd_pcm_substream *substream, int cmd)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	substream_info_t *sub_info = NULL;
	int index = find_substream(chip, substream, &sub_info);
	snd_assert((index >= 0 && index <= 7 && sub_info), return -EINVAL);
	spin_lock(&chip->ad73322_lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (!chip->tx_dma_started) {
			chip->tx_dma_pos = 0;
			bf53x_sport_tx_start(chip->sport);
			if (!(chip->rx_status & RUN_TX_ALL))
				snd_ad73322_startup();
			chip->tx_dma_started = 1;
		}
		sub_info->dma_offset = chip->tx_dma_pos;
		chip->tx_status |= (1 << index);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		chip->tx_status &= ~ (1 << index);
		if (!(chip->tx_status & RUN_TX_ALL)) {
			chip->tx_dma_started = 0;
			bf53x_sport_tx_stop(chip->sport);
			if (!(chip->rx_status & RUN_TX_ALL))
				snd_ad73322_stop();
		}
		break;
	default:
		spin_unlock(&chip->ad73322_lock);
		return -EINVAL;
	}
	spin_unlock(&chip->ad73322_lock);

	snd_printd(KERN_INFO"cmd:%s,runmode:0x%x\n", cmd?"start":"stop",
								chip->runmode);
	return 0;
}

static int snd_ad73322_cap_trigger(struct snd_pcm_substream *substream, int cmd)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	substream_info_t *sub_info = NULL;
	int index = find_substream(chip, substream, &sub_info);
	snd_assert((index >= 0 && index <= 7 && sub_info), return -EINVAL);
	spin_lock(&chip->ad73322_lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (!chip->rx_dma_started) {
			chip->rx_dma_pos = 0;
			bf53x_sport_rx_start(chip->sport);
			if (!(chip->tx_status & RUN_TX_ALL))
				snd_ad73322_startup();
			chip->rx_dma_started = 1;
		}
		sub_info->dma_offset = chip->rx_dma_pos;
		chip->rx_status |= (1 << index);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		chip->rx_status &= ~ (1 << index);
		if (!(chip->rx_status & RUN_TX_ALL)) {
			chip->rx_dma_started = 0;
			bf53x_sport_rx_stop(chip->sport);
			if (!(chip->tx_status & RUN_TX_ALL))
				snd_ad73322_stop();
		}
		break;
	default:
		spin_unlock(&chip->ad73322_lock);
		return -EINVAL;
	}
	spin_unlock(&chip->ad73322_lock);

	snd_printd(KERN_INFO"cmd:%s,runmode:0x%x\n", cmd?"start":"stop",
							chip->runmode);
	return 0;
}

static snd_pcm_uframes_t snd_ad73322_play_ptr(struct snd_pcm_substream *substream)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	substream_info_t *sub_info = NULL;
	unsigned long diff = bf53x_sport_curr_offset_tx(chip->sport);
	unsigned long bytes_per_frame = 8*2;
	size_t frames = diff / bytes_per_frame;
	find_substream(chip, substream, &sub_info);
	frames = (frames + DMA_BUFFER_FRAMES - sub_info->dma_offset) % \
						DMA_BUFFER_FRAMES;

	if (sub_info->dma_last_pos > frames) {
		sub_info->dma_pos_base += DMA_BUFFER_FRAMES;
		if (sub_info->dma_pos_base >= sub_info->boundary)
			sub_info->dma_pos_base -= sub_info->boundary;
	}
	sub_info->dma_last_pos = frames;
	frames = (frames + sub_info->dma_pos_base) % sub_info->buffer_frames;
	return frames;
}

static snd_pcm_uframes_t snd_ad73322_cap_ptr(struct snd_pcm_substream *substream)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	substream_info_t *sub_info = NULL;
	unsigned long diff = bf53x_sport_curr_offset_rx(chip->sport);
	unsigned long bytes_per_frame = 8*2;
	size_t frames = diff / bytes_per_frame;
	find_substream(chip, substream, &sub_info);
	frames = (frames + DMA_BUFFER_FRAMES - sub_info->dma_offset) % \
						DMA_BUFFER_FRAMES;

	if (sub_info->dma_last_pos > frames) {
		sub_info->dma_pos_base += DMA_BUFFER_FRAMES;
		if (sub_info->dma_pos_base >= sub_info->boundary)
			sub_info->dma_pos_base -= sub_info->boundary;
	}
	sub_info->dma_last_pos = frames;
	frames = (frames + sub_info->dma_pos_base) % sub_info->buffer_frames;
	return frames;
}

static int snd_ad73322_play_copy(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos, void *src, snd_pcm_uframes_t count)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	unsigned short *isrc = (unsigned short *)src;
	unsigned short *dst = (unsigned short *)chip->tx_dma_buf;
	substream_info_t *sub_info = NULL;
	int index = find_substream(chip, substream, &sub_info);
	snd_pcm_uframes_t start, temp_count, temp2_count;
	int slot_index = NUM_DEVICES_CHAIN-(index+1);
	snd_assert((index >= 0 && index <=7 && sub_info), return -EINVAL);

	if (index > 0 && index <=7 && !(chip->tx_status & (1<<index))) {
		sub_info->data_count += count;
		return 0;
	}
	start = (sub_info->data_pos_base + pos + sub_info->dma_offset) % \
							DMA_BUFFER_FRAMES;
	if (start + count > DMA_BUFFER_FRAMES) {
		temp_count = DMA_BUFFER_FRAMES - start;
		temp2_count = start + count - DMA_BUFFER_FRAMES;
	} else {
		temp_count = count;
		temp2_count = 0;
	}
	dst += start * 8;
	while (temp_count--) {
		*(dst + slot_index) = *isrc++;
		dst += 8;
	}

	if (temp2_count) {
		dst = (unsigned short*)chip->tx_dma_buf;
		while (temp2_count--) {
			*(dst + slot_index) = *isrc++;
			dst += 8;
		}
	}
	sub_info->data_count += count;
	if (sub_info->data_count >= sub_info->buffer_frames) {
		sub_info->data_count -= sub_info->buffer_frames;
		sub_info->data_pos_base += sub_info->buffer_frames;
		if (sub_info->data_pos_base >= sub_info->boundary)
			sub_info->data_pos_base -= sub_info->boundary;
	}
	return 0;
}

static int snd_ad73322_cap_copy(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos, void *dst, snd_pcm_uframes_t count)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	unsigned short *idst = (unsigned short *)dst;
	unsigned short *src = (unsigned short *)chip->rx_dma_buf;
	substream_info_t *sub_info = NULL;
	int index = find_substream(chip, substream, &sub_info);
	snd_pcm_uframes_t start, temp_count, temp2_count;
	int slot_index = get_cap_slotindex(index);
	snd_assert((index >= 0 && index <=7 && sub_info), return -EINVAL);

	if (index > 0 && index <=7 && !(chip->tx_status & (1<<index))) {
		sub_info->data_count += count;
		return 0;
	}
	start = (sub_info->data_pos_base + pos + sub_info->dma_offset) % \
							DMA_BUFFER_FRAMES;
	if (start + count > DMA_BUFFER_FRAMES) {
		temp_count = DMA_BUFFER_FRAMES - start;
		temp2_count = start + count - DMA_BUFFER_FRAMES;
	} else {
		temp_count = count;
		temp2_count = 0;
	}
	src += start * 8;
	while (temp_count--) {
		*idst++ = *(src +slot_index); 
		src += 8;
	}

	if (temp2_count) {
		dst = (unsigned short*)chip->tx_dma_buf;
		while (temp2_count--) {
			*idst++ = *(src +slot_index); 
			src += 8;
		}
	}
	sub_info->data_count += count;
	if (sub_info->data_count >= sub_info->buffer_frames) {
		sub_info->data_count -= sub_info->buffer_frames;
		sub_info->data_pos_base += sub_info->buffer_frames;
		if (sub_info->data_pos_base >= sub_info->boundary)
			sub_info->data_pos_base -= sub_info->boundary;
	}
	return 0;
}

/* pcm method tables */

static struct snd_pcm_ops snd_ad73322_play_ops = {
	.open      = snd_ad73322_play_open,
	.close     = snd_ad73322_play_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.hw_params = snd_ad73322_hw_params,
	.hw_free   = snd_ad73322_hw_free,
	.prepare   = snd_ad73322_play_pre,
	.trigger   = snd_ad73322_play_trigger,
	.pointer   = snd_ad73322_play_ptr,
	.copy	   = snd_ad73322_play_copy,
};


static struct snd_pcm_ops snd_ad73322_cap_ops = {
	.open  = snd_ad73322_cap_open,
	.close = snd_ad73322_cap_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_ad73322_hw_params,
	.hw_free   = snd_ad73322_hw_free,
	.prepare   = snd_ad73322_cap_pre,
	.trigger   = snd_ad73322_cap_trigger,
	.pointer   = snd_ad73322_cap_ptr,
	.copy	   = snd_ad73322_cap_copy,
};

static inline void snd_ad73322_update(substream_info_t *sub_info)
{
	sub_info->dma_inter_pos += DMA_PERIOD_FRAMES;
	if (sub_info->dma_inter_pos >= sub_info->boundary)
		sub_info->dma_inter_pos -= sub_info->boundary;

	if (sub_info->dma_inter_pos >= sub_info->next_inter_pos) {
		snd_pcm_period_elapsed(sub_info->substream);
		sub_info->next_inter_pos += sub_info->period_frames;
		if (sub_info->next_inter_pos >= sub_info->boundary)
			sub_info->next_inter_pos -= sub_info->boundary;
	}
}


static void snd_ad73322_dma_rx(void *data)
{
	struct snd_ad73322 *ad73322 = data;
	int index;
	substream_info_t *sub_info = NULL;
	ad73322->rx_dma_pos = (ad73322->rx_dma_pos + DMA_PERIOD_FRAMES) % \
						DMA_BUFFER_FRAMES;
	for (index = 0; index < 8; index++) {
		sub_info = &ad73322->rx_substreams[index];
		if (sub_info->substream && ad73322->rx_status & (1<<index)) {
			snd_ad73322_update(sub_info);
		}
	}
}

static void snd_ad73322_dma_tx(void *data)
{
	struct snd_ad73322 *ad73322 = data;
	int index;
	substream_info_t *sub_info = NULL;
	ad73322->tx_dma_pos = (ad73322->tx_dma_pos + DMA_PERIOD_FRAMES) % \
						DMA_BUFFER_FRAMES;
	for (index = 0; index < 8; index++) {
		sub_info = &ad73322->tx_substreams[index];
		if (sub_info->substream && ad73322->tx_status & (1<<index)) {
			snd_ad73322_update(sub_info);
		}
	}

}

static void snd_ad73322_sport_err(void *data)
{
	printk(KERN_ERR "%s: error happened on sport\n", __FUNCTION__);
}

/*************************************************************
 *      card and device
 *************************************************************/
static int snd_ad73322_startup(void)
{
	snd_printd(KERN_INFO "%s is called\n", __FUNCTION__);
	gpio_direction_output(GPIO_SE, 1);
	return 0;
}

static void snd_ad73322_stop(void)
{
	snd_printd(KERN_INFO "%s is called\n", __FUNCTION__);
	/* Pull down SE pin on AD73322 */
	gpio_direction_output(GPIO_SE, 0);
}

static void snd_ad73322_reset(void)
{
	int i;
	snd_printd(KERN_INFO "%s is called\n", __FUNCTION__);
	
	/* Pull down GPIO_RESET pin on AD73322 */
	gpio_direction_output(GPIO_RESET, 0);
	for(i=0; i<50000; i++);
	gpio_direction_output(GPIO_RESET, 1);
	
}

/*************************************************************
 *                 ALSA Card Level
 *************************************************************/
static int snd_ad73322_configure(void)
{
	short ctrl_regs[8];
	short dev_addr,reg_addr;
	int i,j;	
	unsigned short status = 0;
	short ctrl_buffer[NUM_DEVICES_CHAIN*8];
	short *pctrl_buffer;

	/*regs configuration */
	ctrl_regs[7] = (NUM_DEVICES_CHAIN-1)<<4 | MODE_DATA;
	ctrl_regs[0] = MCDIV(0) | SCDIV(0) | DIRATE(0);
	ctrl_regs[1] = PUDEV | PUADC | PUDAC | PUREF | REFUSE ;
	ctrl_regs[2] = 0;
	ctrl_regs[3] = 0;
	ctrl_regs[4] = 0;
	ctrl_regs[5] = 0;
	ctrl_regs[6] = 0;
	
	pctrl_buffer = &ctrl_buffer[0];
	reg_addr = 1;
	for (i=0; i<8; i++)
	{
		dev_addr = NUM_DEVICES_CHAIN - 1;
		for (j=0; j<NUM_DEVICES_CHAIN; j++)
		{
			*pctrl_buffer++ = ctrl_regs[i] | (dev_addr<<11) | (reg_addr<<8) | AD_CONTROL;
			dev_addr--;
		}
	reg_addr++;
	if(reg_addr == 8) reg_addr = 0;

	}
	snd_ad73322_startup();
	snd_ad73322_reset();
	local_irq_disable();
	udelay(1);
	bfin_write_SPORT_TCR1(TFSR);
	bfin_write_SPORT_TCR2(0xF);
	SSYNC();	
	
	for (i=0; i<8; i++) {	
		for (j=0; j<NUM_DEVICES_CHAIN; j++)
			bfin_write_SPORT_TX16(ctrl_buffer[8*i+j]);
		bfin_write_SPORT_TCR1(bfin_read_SPORT_TCR1() | TSPEN);
		status = bfin_read_SPORT_STAT();
		while (!(status & TUVF)){
			udelay(1);
			status = bfin_read_SPORT_STAT();
			SSYNC();
		}
		bfin_write_SPORT_TCR1(bfin_read_SPORT_TCR1() & ~TSPEN);
	}	
	bfin_write_SPORT_TCR1(bfin_read_SPORT_TCR1() & ~TSPEN);
	SSYNC();
	local_irq_enable();
	snd_ad73322_stop();

	return 0;
}

static int __devinit snd_ad73322_pcm(struct snd_ad73322 *ad73322, int dev)
{
	int err = 0;
	struct snd_pcm *pcm;

	/* 1 playback and 1 capture substream */
	if ((err = snd_pcm_new(ad73322->card, pcm_name[dev], dev, 1, 1, &pcm))) {
		return err;
	}

	ad73322->pcm[dev] = pcm;
	strcpy(pcm->name, pcm_name[dev]);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_ad73322_play_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
			&snd_ad73322_cap_ops);
	pcm->private_data = ad73322;
	pcm->info_flags = 0;
	return 0;
}

static int __devinit snd_ad73322_probe(struct platform_device *pdev)
{
	int err;
	struct snd_card *card;
	struct snd_ad73322 *ad73322;
	struct bf53x_sport *sport;
	int i;
	dma_addr_t addr;
	if (device != NULL)
		return -ENOENT;

	if (gpio_request(GPIO_SE, "AD73322")) {
		printk(KERN_ERR "%s: Failed ro request GPIO_%d\n",__FUNCTION__, GPIO_SE);
		return -EBUSY;
	}

	if (gpio_request(GPIO_RESET, "AD73322RST")) {
		printk(KERN_ERR "%s: Failed ro request GPIO_12\n",__FUNCTION__);
		return -EBUSY;
	}
	gpio_direction_output(GPIO_SE, 1);
	gpio_direction_output(GPIO_RESET, 1);

	card = snd_card_new(-1, NULL, THIS_MODULE, sizeof(struct snd_ad73322));
	if (card == NULL)
		return -ENOMEM;
	if ((err = snd_ad73322_configure()) < 0)
		return -EFAULT;
	ad73322 = card->private_data;
	ad73322->card = card;
	ad73322->tx_dma_buf = dma_alloc_coherent(NULL, AD73322_BUF_SZ, &addr, GFP_KERNEL);
	if (!ad73322->tx_dma_buf) {
		printk(KERN_ERR "Failed to allocate dma memory\n");
		return -ENOMEM;
	}
	ad73322->rx_dma_buf = dma_alloc_coherent(NULL, AD73322_BUF_SZ, &addr, GFP_KERNEL);
	if (!ad73322->rx_dma_buf) {
		dma_free_coherent(NULL, AD73322_BUF_SZ, ad73322->tx_dma_buf, 0);
		printk(KERN_ERR "Failed to allocate dma memory\n");
		return -ENOMEM;
	}
	if ((sport = bf53x_sport_init(CONFIG_SND_BFIN_SPORT,
			SPORT_DMA_RX, snd_ad73322_dma_rx,
			SPORT_DMA_TX, snd_ad73322_dma_tx,
			SPORT_IRQ_ERR, snd_ad73322_sport_err, ad73322))
			== NULL) {
		err = -ENODEV;
		goto __sport_err;
	}

	ad73322->sport = sport;
	for (i=0; i<NUM_DEVICES_CHAIN; i++) {
		if ((err = snd_ad73322_pcm(ad73322, i)) < 0)
			goto __nodev;
	}
	bf53x_sport_config_rx(sport, RFSR, 0xF, 0, 0); 
	bf53x_sport_config_rx_dma(sport, ad73322->rx_dma_buf,
			DMA_PERIODS, DMA_PERIOD_BYTES, 2);
	bf53x_sport_config_tx(sport, TFSR, 0xF, 0, 0);
	bf53x_sport_config_tx_dma(sport, ad73322->tx_dma_buf,
			DMA_PERIODS, DMA_PERIOD_BYTES, 2);
	strcpy(card->driver, DRIVER_NAME);
	strcpy(card->shortname, CHIP_NAME);
	sprintf(card->longname, "%s at PF%d SPORT%d rx/tx dma %d/%d err irq %d",
	        card->shortname,
	        CONFIG_SND_BFIN_AD73322_SE,
	        CONFIG_SND_BFIN_SPORT,
	        SPORT_DMA_RX, SPORT_DMA_TX, SPORT_IRQ_ERR);

	snd_card_set_dev(card, (&pdev->dev));
	if ((err = snd_card_register(card)) < 0) {
		goto __nodev;
	}

	platform_set_drvdata(pdev, card);

	return 0;

__nodev:
	bf53x_sport_done(sport);
__sport_err:
	snd_card_free(card);
	return err;
}

static int __devexit snd_ad73322_remove(struct platform_device *pdev)
{
	struct snd_card *card;
	struct snd_ad73322 *ad73322;

	card = platform_get_drvdata(pdev);
	ad73322 = card->private_data;
	dma_free_coherent(NULL, AD73322_BUF_SZ, ad73322->tx_dma_buf, 0);
	dma_free_coherent(NULL, AD73322_BUF_SZ, ad73322->rx_dma_buf, 0);
	snd_ad73322_stop();
	bf53x_sport_done(ad73322->sport);
	snd_card_free(card);
	gpio_free(GPIO_RESET);
	gpio_free(GPIO_SE);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int snd_ad73322_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	struct snd_ad73322 *ad73322 = card->private_data;
	int i;

	snd_power_change_state(card, SNDRV_CTL_POWER_D3hot);
	for (i=0; i<NUM_DEVICES_CHAIN; i++)	
		snd_pcm_suspend_all(ad73322->pcm[i]);

	return 0;
}
static int snd_ad73322_resume(struct platform_device *pdev)
{
	int err = 0;
	struct snd_card *card = platform_get_drvdata(pdev);
	struct snd_ad73322 *ad73322 = card->private_data;
	err = bf53x_sport_config_rx(ad73322->sport, RFSR, 0xF, 0, 0);
	err = err || bf53x_sport_config_tx(ad73322->sport, TFSR, 0xF, 0, 0);
	if (err)
		snd_printk(KERN_ERR "Unable to set sport configuration\n");
	snd_power_change_state(card, SNDRV_CTL_POWER_D0);

	return 0;
}
#endif

static struct platform_driver snd_ad73322_driver = {
	.probe		= snd_ad73322_probe,
	.remove		= __devexit_p(snd_ad73322_remove),
#ifdef CONFIG_PM
	.suspend	= snd_ad73322_suspend,
	.resume		= snd_ad73322_resume,
#endif
	.driver		= {
			.name = DRIVER_NAME,
	},
};

static int __init snd_ad73322_init(void)
{
	int err;

	if (input_gain > 7) {
		printk(KERN_NOTICE DRIVER_NAME ": valid input_gain values are 0 to 7 inclusive\n");
		return -EINVAL;
	}

	if (output_gain > 7) {
		printk(KERN_NOTICE DRIVER_NAME ": valid output_gain values are 0 to 7 inclusive\n");
		return -EINVAL;
	}

	if ((err = platform_driver_register(&snd_ad73322_driver))<0)
		return err;

	device = platform_device_register_simple(DRIVER_NAME, 0, NULL, 0);
	if (IS_ERR(device)) {
		err = PTR_ERR(device);
		platform_driver_unregister(&snd_ad73322_driver);
		return err;
	}

	return err;
}

static void __exit snd_ad73322_exit(void)
{
	platform_device_unregister(device);
	platform_driver_unregister(&snd_ad73322_driver);
}

MODULE_AUTHOR("Cliff Cai <Cliff.Cai@analog.com>");
MODULE_DESCRIPTION("Blackfin/ADI AD73322");
MODULE_LICENSE("GPL");

module_init(snd_ad73322_init);
module_exit(snd_ad73322_exit);

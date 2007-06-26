/*
 * linux/sound/arm/bf5xx-pcm.c -- ALSA PCM interface for the Blackfin
 *
 * Copyright:	(C) 2007 Analog Device Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/dma.h>

#include "bf5xx-pcm.h"
#include "bf5xx-ac97.h"
#include "bf5xx-sport.h"

/* The total rx buffer is 256 * 1024 in byte,
 * total tx buffer is also 256 * 1024 in byte.
 * The total ac97 frames in tx buffer is 256 * 1024 / 32 = 8096.
 */
static const struct snd_pcm_hardware bf5xx_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= SNDRV_PCM_RATE_CONTINUOUS,
	.rate_min		= 8000,
	.rate_max		= 96000,
	.channels_min		= 1,
	.channels_max		= 2,
	.period_bytes_min	= 32,
	.period_bytes_max	= 8192 - 32,
	.periods_min		= 1,
	.periods_max		= PAGE_SIZE/32,
	.buffer_bytes_max	= 32 * 1024,
	.fifo_size		= 16,
};

static int bf5xx_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return 0;
}

static int bf5xx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return 0;
}

static int bf5xx_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sport_device *sport = runtime->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sport_config_tx_dma(sport, runtime->dma_area,
				runtime->periods,
				runtime->period_size, 2);
	} else {
		sport_config_rx_dma(sport, runtime->dma_area,
				runtime->periods,
				runtime->period_size, 2);
	}
	return 0;
}

static int bf5xx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct sport_device *sport = substream->runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			sport_tx_start(sport);
		else
			sport_rx_start(sport);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			sport_tx_stop(sport);
		else
			sport_rx_stop(sport);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t
bf5xx_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sport_device *sport = runtime->private_data;
	unsigned int curr;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		curr = sport_curr_offset_tx(sport);
	else
		curr = sport_curr_offset_rx(sport);

	return curr/32;
}

static	int bf5xx_pcm_copy(struct snd_pcm_substream *substream, int channel,
		    snd_pcm_uframes_t pos,
		    void __user *buf, snd_pcm_uframes_t count)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		bf5xx_ac97_pcm16_to_frame((struct ac97_frame*)runtime->dma_area,
				buf, count);
	} else
		bf5xx_ac97_frame_to_pcm16((struct ac97_frame*)runtime->dma_area,
				buf, count);

	return 0;
}

static int bf5xx_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	snd_soc_set_runtime_hwparams(substream, &bf5xx_pcm_hardware);

	ret = snd_pcm_hw_constraint_integer(runtime, \
			SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	if (sport_handle != NULL)
		runtime->private_data = sport_handle;
	else
		printk(KERN_ERR "sport_handle is NULL\n");
	return 0;

 out:
	return ret;
}

static int bf5xx_pcm_close(struct snd_pcm_substream *substream)
{
	return 0;
}

struct snd_pcm_ops bf5xx_pcm_ops = {
	.open		= bf5xx_pcm_open,
	.close		= bf5xx_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= bf5xx_pcm_hw_params,
	.hw_free	= bf5xx_pcm_hw_free,
	.prepare	= bf5xx_pcm_prepare,
	.trigger	= bf5xx_pcm_trigger,
	.pointer	= bf5xx_pcm_pointer,
	.copy		= bf5xx_pcm_copy,
};

static int bf5xx_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = bf5xx_pcm_hardware.buffer_bytes_max;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_coherent(pcm->card->dev, size*8,
					   &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static void bf5xx_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_coherent(buf->area);
		buf->area = NULL;
	}
}

static u64 bf5xx_pcm_dmamask = DMA_32BIT_MASK;

int bf5xx_pcm_new(struct snd_card *card, struct snd_soc_codec_dai *dai,
	struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &bf5xx_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_32BIT_MASK;

	if (dai->playback.channels_min) {
		ret = bf5xx_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		ret = bf5xx_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:
	return ret;
}

struct snd_soc_platform bf5xx_soc_platform = {
	.name		= "bf5xx-audio",
	.pcm_ops 	= &bf5xx_pcm_ops,
	.pcm_new	= bf5xx_pcm_new,
	.pcm_free	= bf5xx_pcm_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(bf5xx_soc_platform);

MODULE_AUTHOR("Roy Huang");
MODULE_DESCRIPTION("ADI Blackfin PCM DMA module");
MODULE_LICENSE("GPL");

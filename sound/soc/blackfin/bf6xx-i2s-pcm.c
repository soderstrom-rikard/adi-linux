/*
 * bf6xx-i2s-pcm.c - Analog Devices BF6XX i2s dma driver
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

#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#include "bf6xx-sport.h"


#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <asm/dma.h>

static void bfin_dma_irq(void *data)
{
	struct snd_pcm_substream *pcm = data;
	snd_pcm_period_elapsed(pcm);
}

static const struct snd_pcm_hardware bfin_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				   SNDRV_PCM_INFO_MMAP |
				   SNDRV_PCM_INFO_MMAP_VALID |
				   SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 32,
	.period_bytes_max	= 0x10000,
	.periods_min		= 1,
	.periods_max		= PAGE_SIZE/32,
	.buffer_bytes_max	= 0x20000, /* 128 kbytes */
	.fifo_size		= 16,
};

static int bfin_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	size_t size = bfin_pcm_hardware.buffer_bytes_max;
	snd_pcm_lib_malloc_pages(substream, size);

	return 0;
}

static int bfin_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_lib_free_pages(substream);

	return 0;
}

static int bfin_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sport_device *sport = runtime->private_data;
	int period_bytes = frames_to_bytes(runtime, runtime->period_size);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sport_set_tx_callback(sport, bfin_dma_irq, substream);
		sport_config_tx_dma(sport, runtime->dma_area,
			runtime->periods, period_bytes);
	} else {
		sport_set_rx_callback(sport, bfin_dma_irq, substream);
		sport_config_rx_dma(sport, runtime->dma_area,
			runtime->periods, period_bytes);
	}

	return 0;
}

static int bfin_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sport_device *sport = runtime->private_data;
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

static snd_pcm_uframes_t bfin_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sport_device *sport = runtime->private_data;
	unsigned int diff;
	snd_pcm_uframes_t frames;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		diff = sport_curr_offset_tx(sport);
	} else {
		diff = sport_curr_offset_rx(sport);
	}

	/*
	 * TX at least can report one frame beyond the end of the
	 * buffer if we hit the wraparound case - clamp to within the
	 * buffer as the ALSA APIs require.
	 */
	if (diff == snd_pcm_lib_buffer_bytes(substream))
		diff = 0;

	frames = bytes_to_frames(substream->runtime, diff);

	return frames;
}

static int bfin_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct sport_device *sport = snd_soc_dai_get_drvdata(cpu_dai);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	snd_soc_set_runtime_hwparams(substream, &bfin_pcm_hardware);

	ret = snd_pcm_hw_constraint_integer(runtime,
			SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	runtime->private_data = sport;
	return 0;
}

static int bfin_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	size_t size = vma->vm_end - vma->vm_start;
	vma->vm_start = (unsigned long)runtime->dma_area;
	vma->vm_end = vma->vm_start + size;
	vma->vm_flags |=  VM_SHARED;

	return 0 ;
}

static struct snd_pcm_ops bfin_pcm_i2s_ops = {
	.open		= bfin_pcm_open,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= bfin_pcm_hw_params,
	.hw_free	= bfin_pcm_hw_free,
	.prepare	= bfin_pcm_prepare,
	.trigger	= bfin_pcm_trigger,
	.pointer	= bfin_pcm_pointer,
	.mmap		= bfin_pcm_mmap,
};

static int bfin_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = bfin_pcm_hardware.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_coherent(pcm->card->dev, size,
			&buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;

	pr_info("%s, area:%p, size:0x%08lx\n", __func__,
		buf->area, buf->bytes);

	return 0;
}

static void bfin_pcm_free_dma_buffers(struct snd_pcm *pcm)
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
		dma_free_coherent(NULL, buf->bytes, buf->area, 0);
		buf->area = NULL;
	}
}

static u64 bfin_pcm_dmamask = DMA_BIT_MASK(32);

int bfin_pcm_i2s_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &bfin_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (dai->driver->playback.channels_min) {
		ret = bfin_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			return ret;
	}

	if (dai->driver->capture.channels_min)
		ret = bfin_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
	return ret;
}

static struct snd_soc_platform_driver bfin_i2s_soc_platform = {
	.ops		= &bfin_pcm_i2s_ops,
	.pcm_new	= bfin_pcm_i2s_new,
	.pcm_free	= bfin_pcm_free_dma_buffers,
};

static int __devinit bfin_i2s_soc_platform_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &bfin_i2s_soc_platform);
}

static int __devexit bfin_i2s_soc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver bfin_i2s_pcm_driver = {
	.driver = {
		.name = "bfin-i2s-pcm-audio",
		.owner = THIS_MODULE,
	},
	.probe = bfin_i2s_soc_platform_probe,
	.remove = __devexit_p(bfin_i2s_soc_platform_remove),
};

static int __init snd_bfin_i2s_pcm_init(void)
{
	return platform_driver_register(&bfin_i2s_pcm_driver);
}
module_init(snd_bfin_i2s_pcm_init);

static void __exit snd_bfin_i2s_pcm_exit(void)
{
	platform_driver_unregister(&bfin_i2s_pcm_driver);
}
module_exit(snd_bfin_i2s_pcm_exit);

MODULE_DESCRIPTION("Analog Devices BF6XX i2s dma driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");

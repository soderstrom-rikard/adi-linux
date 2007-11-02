/*
 * bf5xx-i2s.c  --  ALSA Soc Audio Layer
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    27th Aug 2007   Initial version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <asm/irq.h>
#include <asm/gpio.h>
#include <asm/portmux.h>
#include <linux/mutex.h>

#include "bf5xx-sport.h"
#include "bf5xx-i2s.h"

#define BF53X_I2S_DEBUG

#ifdef BF53X_I2S_DEBUG
#define i2s_printd(format, arg...) printk(KERN_INFO"sport-i2s: " format, ## arg)
#endif

#ifndef SLEN_T
#define                    SLEN_T  0x1f       /* SPORT Word Length */
#endif

#ifndef SLEN_R
#define                    SLEN_R  0x1f       /* SPORT Word Length */
#endif

/*
 * This is setup by the audio & dai ops and written to sport during prepare ()
 */
struct bf5xx_i2s_port {
	u16 tcr1;
	u16 rcr1;
	u16 tcr2;
	u16 rcr2;

	/* todo - sport master mode */
	unsigned int tclkdiv;
	unsigned int tfsdiv;
	unsigned int rclkdiv;
	unsigned int rfsdiv;
};
static struct bf5xx_i2s_port bf5xx_i2s[NUM_SPORT_I2S];

/* playback I2S setup */
static int bf5xx_i2s_set_tx_dai_fmt(struct snd_soc_cpu_dai *cpu_dai,
		unsigned int fmt)
{
	i2s_printd("%s : fmt %x\n", __func__, fmt);

	bf5xx_i2s[cpu_dai->id].tcr1 = 0;
	bf5xx_i2s[cpu_dai->id].tcr2 = 0;
	bf5xx_i2s[cpu_dai->id].tclkdiv = 0;
	bf5xx_i2s[cpu_dai->id].tfsdiv = 0;

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		bf5xx_i2s[cpu_dai->id].tcr1 |= TFSR | TCKFE;
		bf5xx_i2s[cpu_dai->id].tcr2 |= TSFSE;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		bf5xx_i2s[cpu_dai->id].tcr1 |= TFSR | LATFS | LTFS;
		bf5xx_i2s[cpu_dai->id].tcr2 |= TSFSE;
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		bf5xx_i2s[cpu_dai->id].tcr1 |= ITCLK | ITFS;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		bf5xx_i2s[cpu_dai->id].tcr1 |= ITFS;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		bf5xx_i2s[cpu_dai->id].tcr1 |= ITCLK;
		break;
	default:
		break;
	}
	return 0;
}

/* capture I2S setup */
static int bf5xx_i2s_set_rx_dai_fmt(struct snd_soc_cpu_dai *cpu_dai,
		unsigned int fmt)
{
	i2s_printd("%s : fmt %x\n", __func__, fmt);

	bf5xx_i2s[cpu_dai->id].rcr1 = 0;
	bf5xx_i2s[cpu_dai->id].rcr2 = 0;
	bf5xx_i2s[cpu_dai->id].rclkdiv = 0;
	bf5xx_i2s[cpu_dai->id].rfsdiv = 0;

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		bf5xx_i2s[cpu_dai->id].rcr1 |= RFSR | RCKFE;
		bf5xx_i2s[cpu_dai->id].rcr2 |= RSFSE;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		bf5xx_i2s[cpu_dai->id].rcr1 |= RFSR | LARFS | LRFS;
		bf5xx_i2s[cpu_dai->id].rcr2 |= RSFSE;
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		bf5xx_i2s[cpu_dai->id].rcr1 |= IRCLK | IRFS;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		bf5xx_i2s[cpu_dai->id].rcr1 |= IRFS;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		bf5xx_i2s[cpu_dai->id].rcr1 |= IRCLK;
		break;
	default:
		break;
	}
	return 0;
}

static int bf5xx_i2s_set_dai_fmt(struct snd_soc_cpu_dai *cpu_dai,
		unsigned int fmt)
{
	int ret;

	ret = bf5xx_i2s_set_tx_dai_fmt(cpu_dai, fmt);
	if (ret == 0)
		return bf5xx_i2s_set_rx_dai_fmt(cpu_dai, fmt);
	return ret;
}

static int bf5xx_i2s_tx_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;

	i2s_printd("%s : sport %d\n", __func__, cpu_dai->id);

	bf5xx_i2s[cpu_dai->id].tcr2 &= ~SLEN_T;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		bf5xx_i2s[cpu_dai->id].tcr2 |= 15;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		bf5xx_i2s[cpu_dai->id].tcr2 |= 23;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		bf5xx_i2s[cpu_dai->id].tcr2 |= 31;
		break;
	}

	return 0;
}

static int bf5xx_i2s_rx_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;

	i2s_printd("%s : sport %d\n", __func__, cpu_dai->id);

	bf5xx_i2s[cpu_dai->id].rcr2 &= ~SLEN_R;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		bf5xx_i2s[cpu_dai->id].rcr2 |= 15;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		bf5xx_i2s[cpu_dai->id].rcr2 |= 23;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		bf5xx_i2s[cpu_dai->id].rcr2 |= 31;
		break;
	}

	return 0;
}

static int bf5xx_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return bf5xx_i2s_tx_hw_params(substream, params);
	else
		return bf5xx_i2s_rx_hw_params(substream, params);
}

static int bf5xx_i2s_tx_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	struct sport_device *sport =
		(struct sport_device *)cpu_dai->private_data;

	i2s_printd("%s : sport %d\n", __func__, cpu_dai->id);

	return sport_config_tx(sport, bf5xx_i2s[cpu_dai->id].tcr1,
		bf5xx_i2s[cpu_dai->id].tcr2,
		bf5xx_i2s[cpu_dai->id].tclkdiv,
		bf5xx_i2s[cpu_dai->id].tfsdiv);
}

static int bf5xx_i2s_rx_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	struct sport_device *sport =
		(struct sport_device *)cpu_dai->private_data;

	i2s_printd("%s : sport %d\n", __func__, cpu_dai->id);

	return sport_config_tx(sport, bf5xx_i2s[cpu_dai->id].rcr1,
		bf5xx_i2s[cpu_dai->id].rcr2,
		bf5xx_i2s[cpu_dai->id].rclkdiv,
		bf5xx_i2s[cpu_dai->id].rfsdiv);
}

static int bf5xx_i2s_prepare(struct snd_pcm_substream *substream)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return bf5xx_i2s_tx_prepare(substream);
	else
		return bf5xx_i2s_rx_prepare(substream);
}

static int bf5xx_i2s_tx_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	struct sport_device *sport =
		(struct sport_device *)cpu_dai->private_data;
	int ret = 0;

	i2s_printd("%s : sport %d cmd %x\n", __func__, cpu_dai->id, cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_START:
		ret = sport_tx_start(sport);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = sport_tx_stop(sport);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int bf5xx_i2s_rx_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	struct sport_device *sport =
		(struct sport_device *)cpu_dai->private_data;
	int ret = 0;

	i2s_printd("%s : sport %d cmd %x\n", __func__, cpu_dai->id, cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_START:
		ret = sport_rx_start(sport);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = sport_rx_stop(sport);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int bf5xx_i2s_trigger(struct snd_pcm_substream *substream, int cmd)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return bf5xx_i2s_tx_trigger(substream, cmd);
	else
		return bf5xx_i2s_rx_trigger(substream, cmd);
}

static void bf5xx_i2s_shutdown(struct snd_pcm_substream *substream)
{
	i2s_printd("%s\n", __func__);
}

static int bf5xx_i2s_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	struct sport_device *sport =
		(struct sport_device *)cpu_dai->private_data;

	i2s_printd("%s : sport %d\n", __func__, cpu_dai->id);

	/* FIXME: This function is called prior to snd_soc_ops startup,
	 * so the sport handle is not yet passed to cpu_dai->private_data.

	if (sport == NULL) {
		printk(KERN_ERR "%s : invalid sport device\n", __func__);
		return -EINVAL;
	} else
	 *
	 */
		return 0;
}

#ifdef CONFIG_PM
static int bf5xx_i2s_suspend(struct platform_device *dev,
	struct snd_soc_cpu_dai *dai)
{
	struct sport_device *sport =
		(struct sport_device *)dai->private_data;

	i2s_printd("%s : sport %d\n", __func__, dai->id);
	if (!dai->active)
		return 0;

	if (dai->capture.active)
		sport_rx_stop(sport);
	if (dai->playback.active)
		sport_tx_stop(sport);
	return 0;
}

static int bf5xx_i2s_resume(struct platform_device *pdev,
	struct snd_soc_cpu_dai *dai)
{
	struct sport_device *sport =
		(struct sport_device *)dai->private_data;

	i2s_printd("%s : sport %d\n", __func__, cpu_dai->id);
	if (!dai->active)
		return 0;

	if (dai->capture.active) {
		sport_config_tx(sport, bf5xx_i2s[dai->id].rcr1,
			bf5xx_i2s[dai->id].rcr2,
			bf5xx_i2s[dai->id].rclkdiv,
			bf5xx_i2s[dai->id].rfsdiv);
		sport_rx_start(sport);
	}
	if (dai->playback.active) {
		sport_config_tx(sport, bf5xx_i2s[dai->id].tcr1,
			bf5xx_i2s[dai->id].tcr2,
			bf5xx_i2s[dai->id].tclkdiv,
			bf5xx_i2s[dai->id].tfsdiv);
		sport_tx_start(sport);
	}
	return 0;
}

#else
#define bf5xx_i2s_suspend	NULL
#define bf5xx_i2s_resume	NULL
#endif

#define BF5XX_I2S_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | \
		SNDRV_PCM_RATE_96000)

#define BF5XX_I2S_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |\
	SNDRV_PCM_FMTBIT_S32_LE)

struct snd_soc_cpu_dai bf5xx_i2s_dai[NUM_SPORT_I2S] = {
{
	.name = "bf5xx-i2s-0",
	.id = 0,
	.type = SND_SOC_DAI_I2S,
	.suspend = bf5xx_i2s_suspend,
	.resume = bf5xx_i2s_resume,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = BF5XX_I2S_RATES,
		.formats = BF5XX_I2S_FORMATS,},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = BF5XX_I2S_RATES,
		.formats = BF5XX_I2S_FORMATS,},
	.ops = {
		.startup = bf5xx_i2s_startup,
		.shutdown = bf5xx_i2s_shutdown,
		.trigger = bf5xx_i2s_trigger,
		.hw_params = bf5xx_i2s_hw_params,
		.prepare = bf5xx_i2s_prepare,},
	.dai_ops = {
		.set_fmt = bf5xx_i2s_set_dai_fmt,
	},
},
{
	.name = "bf5xx-i2s-1",
	.id = 1,
	.type = SND_SOC_DAI_I2S,
	.suspend = bf5xx_i2s_suspend,
	.resume = bf5xx_i2s_resume,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = BF5XX_I2S_RATES,
		.formats = BF5XX_I2S_FORMATS,},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = BF5XX_I2S_RATES,
		.formats = BF5XX_I2S_FORMATS,},
	.ops = {
		.startup = bf5xx_i2s_startup,
		.shutdown = bf5xx_i2s_shutdown,
		.trigger = bf5xx_i2s_trigger,
		.hw_params = bf5xx_i2s_hw_params,
		.prepare = bf5xx_i2s_prepare,},
	.dai_ops = {
		.set_fmt = bf5xx_i2s_set_dai_fmt,
	},
},
{
	.name = "bf5xx-i2s-2",
	.id = 2,
	.type = SND_SOC_DAI_I2S,
	.suspend = bf5xx_i2s_suspend,
	.resume = bf5xx_i2s_resume,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = BF5XX_I2S_RATES,
		.formats = BF5XX_I2S_FORMATS,},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = BF5XX_I2S_RATES,
		.formats = BF5XX_I2S_FORMATS,},
	.ops = {
		.startup = bf5xx_i2s_startup,
		.shutdown = bf5xx_i2s_shutdown,
		.trigger = bf5xx_i2s_trigger,
		.hw_params = bf5xx_i2s_hw_params,
		.prepare = bf5xx_i2s_prepare,},
	.dai_ops = {
		.set_fmt = bf5xx_i2s_set_dai_fmt,
	},
},
{
	.name = "bf5xx-i2s-3",
	.id = 3,
	.type = SND_SOC_DAI_I2S,
	.suspend = bf5xx_i2s_suspend,
	.resume = bf5xx_i2s_resume,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = BF5XX_I2S_RATES,
		.formats = BF5XX_I2S_FORMATS,},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = BF5XX_I2S_RATES,
		.formats = BF5XX_I2S_FORMATS,},
	.ops = {
		.startup = bf5xx_i2s_startup,
		.shutdown = bf5xx_i2s_shutdown,
		.trigger = bf5xx_i2s_trigger,
		.hw_params = bf5xx_i2s_hw_params,
		.prepare = bf5xx_i2s_prepare,},
	.dai_ops = {
		.set_fmt = bf5xx_i2s_set_dai_fmt,
	},
},};

EXPORT_SYMBOL_GPL(bf5xx_i2s_dai);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("Blackfin I2S SoC Interface");
MODULE_LICENSE("GPL");

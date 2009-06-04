/*
 * ad1938.c  --  ALSA Soc AD1938 codec support
 *
 * Copyright:	Analog Device Inc.
 * Author:	Barry Song <Barry.Song@analog.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    4 June 2009   Initial version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include "ad1938.h"
#include "ad1938_spi.h"

struct snd_soc_dai ad1938_dai = {
	.name = "AD1938",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE, },
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE, },
};
EXPORT_SYMBOL_GPL(ad1938_dai);

static int ad1938_soc_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	ret = ad1938_spi_init();
	if (ret < 0) {
		printk(KERN_ERR "ad1938: failed to init spi interface\n");
		goto spi_err;
	}

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;
	mutex_init(&codec->mutex);
	codec->name = "AD1938";
	codec->owner = THIS_MODULE;
	codec->dai = &ad1938_dai;
	codec->num_dai = 1;
	socdev->codec = codec;
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "ad1938: failed to create pcms\n");
		goto pcm_err;
	}

	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "ad1938: failed to register card\n");
		goto register_err;
	}

	/* default setting for ad1938: 8 channel AUX ADC mode, 16bit, 48000Hz */
	ad1938_spi_write(AD1938_DAC_CTRL0, 0x40);
	ad1938_spi_write(AD1938_DAC_CTRL1, 0x84);
	ad1938_spi_write(AD1938_DAC_CTRL2, 0x1A);
	ad1938_spi_write(AD1938_ADC_CTRL0, 0x32);
	ad1938_spi_write(AD1938_ADC_CTRL1, 0x43);
	ad1938_spi_write(AD1938_ADC_CTRL2, 0x6f);
	ad1938_spi_write(AD1938_PLL_CLK_CTRL0, 0x9C);
	ad1938_spi_write(AD1938_PLL_CLK_CTRL1, 0x04);

	return ret;

register_err:
	snd_soc_free_pcms(socdev);
pcm_err:
	kfree(socdev->codec);
	socdev->codec = NULL;
spi_err:
	return ret;
}

static int ad1938_soc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec == NULL)
		return 0;
	snd_soc_free_pcms(socdev);
	kfree(codec);
	ad1938_spi_done();

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_ad1938 = {
	.probe = 	ad1938_soc_probe,
	.remove = 	ad1938_soc_remove,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ad1938);

MODULE_DESCRIPTION("ASoC ad1938 driver");
MODULE_AUTHOR("Barry Song ");
MODULE_LICENSE("GPL");

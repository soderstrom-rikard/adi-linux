/*
 * ad73311.c  --  ALSA Soc AD73311/AD74111 codec support
 *
 * Copyright:	Analog Device Inc.
 * Author:	Cliff Cai <cliff.cai@analog.com>
 *
 * Modified:    Copyright 2010 Nanakos Chrysostomos <nanakos@wired-net.gr>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include "ad73311.h"

#if CONFIG_SND_AD7XXXX_SELECT == 0
#define DRV_NAME "AD73311"
#elif CONFIG_SND_AD7XXXX_SELECT == 1
#define DRV_NAME "AD74111"
#endif

#define AD73311_RATES (SNDRV_PCM_RATE_8000  | \
					SNDRV_PCM_RATE_16000 | \
					SNDRV_PCM_RATE_32000 | \
					SNDRV_PCM_RATE_64000)

static int ad73311_ogain_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *uvalue)
{
	if (ad73311.ogain != (7 - uvalue->value.integer.value[0])) {
		ad73311.ogain = 7 - uvalue->value.integer.value[0];
		return ad73311_reg_config();
	}
	return 0;
}

static int ad73311_ogain_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *uvalue)
{
	uvalue->value.integer.value[0] =  7 - ad73311.ogain;
	return 0;
}

static int ad73311_igain_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *uvalue)
{
	if (ad73311.igain != uvalue->value.integer.value[0]) {
		ad73311.igain =  uvalue->value.integer.value[0];
		return ad73311_reg_config();
	}
	return 0;
}

static int ad73311_igain_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *uvalue)
{
	uvalue->value.integer.value[0] = ad73311.igain;
	return 0;
}

static int ad73311_rfseen_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *uvalue)
{
	if (ad73311.rfseen != uvalue->value.integer.value[0]) {
		ad73311.rfseen = uvalue->value.integer.value[0];
		return ad73311_reg_config();
	}
	return 0;

}

static int ad73311_rfseen_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *uvalue)
{
	uvalue->value.integer.value[0] = ad73311.rfseen;
	return 0;
}

static int ad73311_srate_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *uvalue)
{
	int tmpvar = 0;
	if (ad73311.srate != uvalue->value.integer.value[0]) {
		ad73311.srate = uvalue->value.integer.value[0];
		switch (ad73311.srate) {
		case 0:
			tmpvar = SNDRV_PCM_RATE_8000;
			break;
		case 1:
			tmpvar = SNDRV_PCM_RATE_16000;
			break;
		case 2:
			tmpvar = SNDRV_PCM_RATE_32000;
			break;
		case 3:
			tmpvar = SNDRV_PCM_RATE_64000;
			break;
		}
		ad73311_dai.playback.rates = tmpvar;
		ad73311_dai.capture.rates = tmpvar;
		return ad73311_reg_config();
	}
	return 0;

}

static int ad73311_srate_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *uvalue)
{
	uvalue->value.integer.value[0] = ad73311.srate;
	return 0;
}

static const struct snd_kcontrol_new ad73311_snd_controls[] = {
	SOC_SINGLE_EXT("IGAIN", 0, 1, 7, 0, ad73311_igain_get, ad73311_igain_put),
	SOC_SINGLE_EXT("OGAIN", 0, 1, 7, 0, ad73311_ogain_get, ad73311_ogain_put),
	SOC_SINGLE_BOOL_EXT("SEEN", 0, ad73311_rfseen_get, ad73311_rfseen_put),
	SOC_SINGLE_EXT("SRATE", 0, 1, 3, 0, ad73311_srate_get, ad73311_srate_put),
};

struct snd_soc_dai ad73311_dai = {
	.name = DRV_NAME,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE, },
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE, },
};
EXPORT_SYMBOL_GPL(ad73311_dai);

static int ad73311_soc_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;
	mutex_init(&codec->mutex);
	codec->name = DRV_NAME;
	codec->dev = &pdev->dev;
	codec->owner = THIS_MODULE;
	codec->dai = &ad73311_dai;
	codec->num_dai = 1;
	socdev->card->codec = codec;
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to create pcms\n", DRV_NAME);
		goto pcm_err;
	}
	if (CONFIG_SND_AD7XXXX_SELECT == 0)
		/* Register alsa controls */
		snd_soc_add_controls(codec, ad73311_snd_controls,
					ARRAY_SIZE(ad73311_snd_controls));

	return ret;

pcm_err:
	kfree(socdev->card->codec);
	socdev->card->codec = NULL;
	return ret;
}

static int ad73311_soc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec == NULL)
		return 0;
	snd_soc_free_pcms(socdev);
	kfree(codec);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_ad73311 = {
	.probe = 	ad73311_soc_probe,
	.remove = 	ad73311_soc_remove,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ad73311);

static int __init ad73311_init(void)
{
	return snd_soc_register_dai(&ad73311_dai);
}
module_init(ad73311_init);

static void __exit ad73311_exit(void)
{
	snd_soc_unregister_dai(&ad73311_dai);
}
module_exit(ad73311_exit);

MODULE_DESCRIPTION("ASoC ad73311/ad74111 driver");
MODULE_AUTHOR("Cliff Cai, Nanakos Chrysostomos ");
MODULE_LICENSE("GPL");

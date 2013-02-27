/*
 * ALSA Soc AD74111 codec support
 *
 * Copyright 2009-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include "ad74111.h"

static struct snd_soc_dai_driver ad74111_dai = {
	.name = "ad74111-hifi",
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

static struct snd_soc_codec_driver soc_codec_dev_ad74111;

static int ad74111_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_ad74111, &ad74111_dai, 1);
}

static int ad74111_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver ad74111_codec_driver = {
	.driver = {
		.name = "ad74111",
		.owner = THIS_MODULE,
	},

	.probe = ad74111_probe,
	.remove = ad74111_remove,
};

static int __init ad74111_init(void)
{
	return platform_driver_register(&ad74111_codec_driver);
}
module_init(ad74111_init);

static void __exit ad74111_exit(void)
{
	platform_driver_unregister(&ad74111_codec_driver);
}
module_exit(ad74111_exit);

MODULE_DESCRIPTION("ASoC AD74111 driver");
MODULE_AUTHOR("Cliff Cai");
MODULE_LICENSE("GPL");

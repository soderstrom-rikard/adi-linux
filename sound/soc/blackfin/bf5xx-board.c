/*
 * bf5xx_board.c  --  SoC audio for Blackfin Processors
 *
 * Copyright 2007 Analog Device Inc.
 *
 * Authors: Roy Huang <roy.huang@analog.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    8th June 2007   Initial version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <asm/dma.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include <asm/gpio.h>
#include <asm/portmux.h>
#include "../codecs/ad1980.h"
#include "bf5xx-sport.h"
#include "bf5xx-pcm.h"
#include "bf5xx-ac97.h"

static struct snd_soc_machine bf5xx_board;

static int bf5xx_board_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_soc_ops bf5xx_board_ops = {
	.startup = bf5xx_board_startup,
};

static int bf5xx_board_ac97_init(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_dai_link bf5xx_board_dai = {
	.name = "AC97",
	.stream_name = "AC97 HiFi",
	.cpu_dai = &bfin_ac97_dai,
	.codec_dai = &ad1980_dai,
	.init = bf5xx_board_ac97_init,
	.ops = &bf5xx_board_ops,
};

static int bf5xx_probe(struct platform_device *pdev)
{
	return 0;
}

static struct snd_soc_machine bf5xx_board = {
	.name = "bf5xx-board",
	.probe = bf5xx_probe,
	.dai_link = &bf5xx_board_dai,
	.num_links = 1,
};

static struct snd_soc_device bf5xx_board_snd_devdata = {
	.machine = &bf5xx_board,
	.platform = &bf5xx_soc_platform,
	.codec_dev = &soc_codec_dev_ad1980,
};

static struct platform_device *bf5xx_board_snd_device;

static int __init bf5xx_board_init(void)
{
	int ret;

	bf5xx_board_snd_device = platform_device_alloc("soc-audio", -1);
	if (!bf5xx_board_snd_device)
		return -ENOMEM;

	platform_set_drvdata(bf5xx_board_snd_device, &bf5xx_board_snd_devdata);
	bf5xx_board_snd_devdata.dev = &bf5xx_board_snd_device->dev;
	ret = platform_device_add(bf5xx_board_snd_device);

	if (ret)
		platform_device_put(bf5xx_board_snd_device);

	return ret;
}

static void __exit bf5xx_board_exit(void)
{
	platform_device_unregister(bf5xx_board_snd_device);
}

module_init(bf5xx_board_init);
module_exit(bf5xx_board_exit);

/* Module information */
MODULE_AUTHOR("Roy Huang");
MODULE_DESCRIPTION("ALSA SoC BF5xx Board");
MODULE_LICENSE("GPL");

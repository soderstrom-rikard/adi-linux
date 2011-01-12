/*
 * board driver for ADAV80X sound chip
 *
 * Copyright 2010-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>

#include <asm/dma.h>
#include <asm/portmux.h>
#include <linux/gpio.h>
#include "../codecs/adav80x.h"
#include "bf5xx-sport.h"
#include "bf5xx-i2s-pcm.h"

static struct snd_soc_card bf5xx_adav80x;

static int bf5xx_adav80x_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int clk = 0;
	int ret = 0;

	pr_debug("%s rate %d format %x\n", __func__, params_rate(params),
		params_format(params));

	switch (params_rate(params)) {
	case 32000:
	case 44100:
	case 48000:
	case 96000:
		clk = params_rate(params);
		break;
	}

	/*
	 * CODEC is master for BCLK and LRC in this configuration.
	 */

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;
	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* For the ADAV80X evaluation board, use the on board crystal
	   as XIN, and use XIN as source for PLL1 */
	ret = snd_soc_dai_set_pll(codec_dai, ADAV80X_CLK_PLL1,
			ADAV80X_CLK_XIN, 27000000, clk);

	/* If you want to use XIN as clock source */
	/* ret = snd_soc_dai_set_pll(codec_dai, 0, ADAV80X_CLK_XIN,
		27000000, 0); */
	if (ret < 0)
		return ret;
	return 0;
}

static struct snd_soc_ops bf5xx_adav80x_ops = {
	.hw_params = bf5xx_adav80x_hw_params,
};

static struct snd_soc_dai_link bf5xx_adav80x_dai = {
	.name = "adav80x",
	.stream_name = "ADAV80X",
	.cpu_dai = &bf5xx_i2s_dai,
	.codec_dai = &adav80x_dai,
	.ops = &bf5xx_adav80x_ops,
};

static struct snd_soc_card bf5xx_adav80x = {
	.name = "bf5xx_adav80x",
	.platform = &bf5xx_i2s_soc_platform,
	.dai_link = &bf5xx_adav80x_dai,
	.num_links = 1,
};

static struct snd_soc_device bf5xx_adav80x_snd_devdata = {
	.card = &bf5xx_adav80x,
	.codec_dev = &soc_codec_dev_adav80x,
};

static struct platform_device *bf5xx_adav80x_snd_device;

static int __init bf5xx_adav80x_init(void)
{
	int ret;

	bf5xx_adav80x_snd_device = platform_device_alloc("soc-audio", -1);
	if (!bf5xx_adav80x_snd_device)
		return -ENOMEM;

	platform_set_drvdata(bf5xx_adav80x_snd_device,
				&bf5xx_adav80x_snd_devdata);
	bf5xx_adav80x_snd_devdata.dev = &bf5xx_adav80x_snd_device->dev;
	ret = platform_device_add(bf5xx_adav80x_snd_device);

	if (ret)
		platform_device_put(bf5xx_adav80x_snd_device);

	return ret;
}

static void __exit bf5xx_adav80x_exit(void)
{
	platform_device_unregister(bf5xx_adav80x_snd_device);
}

module_init(bf5xx_adav80x_init);
module_exit(bf5xx_adav80x_exit);

/* Module information */
MODULE_AUTHOR("Yi Li");
MODULE_DESCRIPTION("ALSA SoC ADAV80X Blackfin Board");
MODULE_LICENSE("GPL");

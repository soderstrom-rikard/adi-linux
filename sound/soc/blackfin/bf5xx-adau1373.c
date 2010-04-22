/*
 * board driver for adau1373 sound chip
 *
 * Copyright 2010 Analog Devices Inc.
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
#include "../codecs/adau1373.h"
#include "bf5xx-sport.h"
#include "bf5xx-i2s-pcm.h"
#include "bf5xx-i2s.h"

/* PLL settings coefficients, Crystal here is 12.288MHz one.
 * Change things here, if you want to support more rates or change
 * the crystal. Currently the most convenient way is to make PLL
 * generate 48k * 1024 or 44.1k * 1024 clock, then use source
 * clock divider and MCLK divider to get the required rate.
 */
static const struct _pll_settings adau1373_pll_settings[] = {
	/* 96k */
	{ 12288000, 96000, 0, 0, 0x0, 0x4, 0x0 },
	/* 48k */
	{ 12288000, 48000, 0, 0, 0x0, 0x4, 0x0 },
	/* 44.1k */
	{ 12288000, 44100, 40, 27, 0x0, 0x3, 0x1 },
	/* 22.05k */
	{ 12288000, 22050, 40, 27, 0x0, 0x3, 0x1 },
	/* 16k */
	{ 12288000, 16000, 0, 0, 0x0, 0x4, 0x0 },
	/* 8k */
	{ 12288000, 8000, 0, 0, 0x0, 0x4, 0x0 },
};

static struct adau1373_platform_data adau1373_pdata = {
	.pll_settings_num       = ARRAY_SIZE(adau1373_pll_settings),
	.pll_settings   = adau1373_pll_settings,
	.drc_settings	= { 0x07, 0x77, 0x33, 0x88, 0x5d, 0x77, 0x77, 0x13 },
};

static int bf5xx_adau1373_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;
	/* Change input crystal source here */
	unsigned int clk = 12288000;
	/* unsigned int clk = 11289600; */
	pr_debug("%s rate %d format %x\n", __func__, params_rate(params),
		params_format(params));
	/*
	 * If you are using the SPORT to generate clocking then this is
	 * where to do it.
	 */

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
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, clk, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops bf5xx_adau1373_ops = {
	.hw_params = bf5xx_adau1373_hw_params,
};

static struct snd_soc_dai_link bf5xx_adau1373_dai = {
	.name = "adau1373",
	.stream_name = "adau1373",
	.cpu_dai = &bf5xx_i2s_dai,
	.codec_dai = &adau1373_dai,
	.ops = &bf5xx_adau1373_ops,
};

static struct snd_soc_card bf5xx_adau1373 = {
	.name = "bf5xx_adau1373",
	.platform = &bf5xx_i2s_soc_platform,
	.dai_link = &bf5xx_adau1373_dai,
	.num_links = 1,
};

static struct snd_soc_device bf5xx_adau1373_snd_devdata = {
	.card = &bf5xx_adau1373,
	.codec_dev = &soc_codec_dev_adau1373,
};

static struct platform_device *bf5xx_adau1373_snd_device;

static int __init bf5xx_adau1373_init(void)
{
	int ret;

	pr_debug("%s enter\n", __func__);
	bf5xx_adau1373_snd_device = platform_device_alloc("soc-audio", -1);
	if (!bf5xx_adau1373_snd_device)
		return -ENOMEM;

	platform_set_drvdata(bf5xx_adau1373_snd_device,
				&bf5xx_adau1373_snd_devdata);
	platform_device_add_data(bf5xx_adau1373_snd_device, &adau1373_pdata,
				 sizeof(adau1373_pdata));
	bf5xx_adau1373_snd_devdata.dev = &bf5xx_adau1373_snd_device->dev;
	ret = platform_device_add(bf5xx_adau1373_snd_device);

	if (ret)
		platform_device_put(bf5xx_adau1373_snd_device);

	return ret;
}
module_init(bf5xx_adau1373_init);

static void __exit bf5xx_adau1373_exit(void)
{
	pr_debug("%s enter\n", __func__);
	platform_device_unregister(bf5xx_adau1373_snd_device);
}
module_exit(bf5xx_adau1373_exit);

/* Module information */
MODULE_AUTHOR("Cliff Cai");
MODULE_DESCRIPTION("ALSA SoC adau1373");
MODULE_LICENSE("GPL");

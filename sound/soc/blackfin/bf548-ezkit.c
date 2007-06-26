/*
 * bf548_ezkit.c  --  SoC audio for bf548 ezkit
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
#include <sound/soc-dapm.h>


#include "../codecs/ad1980.h"
#include "bf5xx-sport.h"
#include "bf5xx-pcm.h"
#include "bf5xx-ac97.h"

static int	sport_num = CONFIG_SND_BF5XX_SPORT_NUM;

static struct sport_param sport_params[4] = {
	{
		.dma_rx_chan	= CH_SPORT0_RX,
		.dma_tx_chan	= CH_SPORT0_TX,
		.err_irq	= IRQ_SPORT0_ERR,
		.regs		= (struct sport_register*)SPORT0_TCR1,
	},
	{
		.dma_rx_chan	= CH_SPORT1_RX,
		.dma_tx_chan	= CH_SPORT1_TX,
		.err_irq	= IRQ_SPORT1_ERR,
		.regs		= (struct sport_register*)SPORT1_TCR1,
	},
	{
		.dma_rx_chan	= CH_SPORT2_RX,
		.dma_tx_chan	= CH_SPORT2_TX,
		.err_irq	= IRQ_SPORT2_ERR,
		.regs		= (struct sport_register*)SPORT2_TCR1,
	},
	{
		.dma_rx_chan	= CH_SPORT3_RX,
		.dma_tx_chan	= CH_SPORT3_TX,
		.err_irq	= IRQ_SPORT3_ERR,
		.regs		= (struct sport_register*)SPORT3_TCR1,
	}
};

struct sport_device *sport_handle;

static struct snd_soc_machine bf548_ezkit;

#define TOSA_HP        0
#define TOSA_MIC_INT   1
#define TOSA_HEADSET   2
#define TOSA_HP_OFF    3
#define TOSA_SPK_ON    0
#define TOSA_SPK_OFF   1

static int bf548_ezkit_jack_func;
static int bf548_ezkit_spk_func;

static void bf548_ezkit_ext_control(struct snd_soc_codec *codec)
{
	int spk = 0, mic_int = 0, hp = 0, hs = 0;

	/* set up jack connection */
	switch (bf548_ezkit_jack_func) {
	case TOSA_HP:
		hp = 1;
		break;
	case TOSA_MIC_INT:
		mic_int = 1;
		break;
	case TOSA_HEADSET:
		hs = 1;
		break;
	}

	if (bf548_ezkit_spk_func == TOSA_SPK_ON)
		spk = 1;

	snd_soc_dapm_set_endpoint(codec, "Speaker", spk);
	snd_soc_dapm_set_endpoint(codec, "Mic (Internal)", mic_int);
	snd_soc_dapm_set_endpoint(codec, "Headphone Jack", hp);
	snd_soc_dapm_set_endpoint(codec, "Headset Jack", hs);
	snd_soc_dapm_sync_endpoints(codec);
}

static int bf548_ezkit_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->codec;

	/* check the jack status at stream startup */
	bf548_ezkit_ext_control(codec);
	return 0;
}

static struct snd_soc_ops bf548_ezkit_ops = {
	.startup = bf548_ezkit_startup,
};

static int bf548_ezkit_get_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = bf548_ezkit_jack_func;
	return 0;
}

static int bf548_ezkit_set_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (bf548_ezkit_jack_func == ucontrol->value.integer.value[0])
		return 0;

	bf548_ezkit_jack_func = ucontrol->value.integer.value[0];
	bf548_ezkit_ext_control(codec);
	return 1;
}

static int bf548_ezkit_get_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = bf548_ezkit_spk_func;
	return 0;
}

static int bf548_ezkit_set_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (bf548_ezkit_spk_func == ucontrol->value.integer.value[0])
		return 0;

	bf548_ezkit_spk_func = ucontrol->value.integer.value[0];
	bf548_ezkit_ext_control(codec);
	return 1;
}

/* bf548_ezkit dapm event handlers */
static int bf548_ezkit_hp_event(struct snd_soc_dapm_widget *w, int event)
{
	return 0;
}

/* bf548_ezkit machine dapm widgets */
static const struct snd_soc_dapm_widget bf548_ezkit_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", bf548_ezkit_hp_event),
SND_SOC_DAPM_HP("Headset Jack", NULL),
SND_SOC_DAPM_MIC("Mic (Internal)", NULL),
SND_SOC_DAPM_SPK("Speaker", NULL),
};

/* bf548_ezkit audio map */
static const char *audio_map[][3] = {

	/* headphone connected to HPOUTL, HPOUTR */
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Headphone Jack", NULL, "HPOUTR"},

	/* ext speaker connected to LOUT2, ROUT2 */
	{"Speaker", NULL, "LOUT2"},
	{"Speaker", NULL, "ROUT2"},

	/* internal mic is connected to mic1, mic2 differential - with bias */
	{"MIC1", NULL, "Mic Bias"},
	{"MIC2", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic (Internal)"},

	/* headset is connected to HPOUTR, and LINEINR with bias */
	{"Headset Jack", NULL, "HPOUTR"},
	{"LINEINR", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Headset Jack"},

	{NULL, NULL, NULL},
};

static const char *jack_function[] = {"Headphone", "Mic", "Line", "Headset",
	"Off"};
static const char *spk_function[] = {"On", "Off"};
static const struct soc_enum bf548_ezkit_enum[] = {
	SOC_ENUM_SINGLE_EXT(5, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static const struct snd_kcontrol_new bf548_ezkit_controls[] = {
	SOC_ENUM_EXT("Jack Function", bf548_ezkit_enum[0], bf548_ezkit_get_jack,
		bf548_ezkit_set_jack),
	SOC_ENUM_EXT("Speaker Function", bf548_ezkit_enum[1], \
			bf548_ezkit_get_spk, bf548_ezkit_set_spk),
};

static int bf548_ezkit_ac97_init(struct snd_soc_codec *codec)
{
	int i, err;

	/* Set pin 2, 3, 4, 6, 7 of PORT C to function 00 */
	if (sport_num == 0) {
		bfin_write_PORTC_FER(bfin_read_PORTC_FER() | 0xDC);
		bfin_write_PORTC_MUX(bfin_read_PORTC_MUX() & ~(0xF3F0));
		sport_params[0].dma_rx = base_addr[CH_SPORT0_RX];
		sport_params[0].dma_tx = base_addr[CH_SPORT0_TX];
	} else {
		printk(KERN_ERR "SPORT %x not support currently\n", sport_num);
	}
	sport_handle = sport_init(&sport_params[sport_num], codec->ac97);
	if (!sport_handle)
		return -ENODEV;

	sport_set_multichannel(sport_handle, 16, 1);
	sport_config_rx(sport_handle, IRFS, 0xF, 0, (16*16-1));
	sport_config_tx(sport_handle, ITFS, 0xF, 0, (16*16-1));

	snd_soc_dapm_set_endpoint(codec, "OUT3", 0);
	snd_soc_dapm_set_endpoint(codec, "MONOOUT", 0);

	/* add bf548_ezkit specific controls */
	for (i = 0; i < ARRAY_SIZE(bf548_ezkit_controls); i++) {
		err = snd_ctl_add(codec->card, snd_soc_cnew( \
				&bf548_ezkit_controls[i], codec, NULL));
		if (err < 0)
			return err;
	}

	/* add bf548_ezkit specific widgets */
	for (i = 0; i < ARRAY_SIZE(bf548_ezkit_dapm_widgets); i++) {
		snd_soc_dapm_new_control(codec, &bf548_ezkit_dapm_widgets[i]);
	}

	/* set up bf548_ezkit specific audio path audio_map */
	for (i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(codec, audio_map[i][0],
			audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_sync_endpoints(codec);
	return 0;
}

static struct snd_soc_dai_link bf548_ezkit_dai = {
	.name = "AC97",
	.stream_name = "AC97 HiFi",
	.cpu_dai = &bfin_ac97_dai,
	.codec_dai = &ad1980_dai,
	.init = bf548_ezkit_ac97_init,
	.ops = &bf548_ezkit_ops,
};

static struct snd_soc_machine bf548_ezkit = {
	.name = "bf548-ezkit",
	.dai_link = &bf548_ezkit_dai,
	.num_links = 1,
};

static struct snd_soc_device bf548_ezkit_snd_devdata = {
	.machine = &bf548_ezkit,
	.platform = &bf5xx_soc_platform,
	.codec_dev = &soc_codec_dev_ad1980,
};

static struct platform_device *bf548_ezkit_snd_device;

static int __init bf548_ezkit_init(void)
{
	int ret;

	bf548_ezkit_snd_device = platform_device_alloc("soc-audio", -1);
	if (!bf548_ezkit_snd_device)
		return -ENOMEM;

	platform_set_drvdata(bf548_ezkit_snd_device, &bf548_ezkit_snd_devdata);
	bf548_ezkit_snd_devdata.dev = &bf548_ezkit_snd_device->dev;
	ret = platform_device_add(bf548_ezkit_snd_device);

	if (ret)
		platform_device_put(bf548_ezkit_snd_device);

	return ret;
}

static void __exit bf548_ezkit_exit(void)
{
	platform_device_unregister(bf548_ezkit_snd_device);
}

module_init(bf548_ezkit_init);
module_exit(bf548_ezkit_exit);

/* Module information */
MODULE_AUTHOR("Roy Huang");
MODULE_DESCRIPTION("ALSA SoC BF548-EZKIT");
MODULE_LICENSE("GPL");

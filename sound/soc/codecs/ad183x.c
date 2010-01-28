/*
 * AD183X Audio Codec driver supporting AD1835A, AD1836, AD1837A, AD1838A, AD1839A
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/soc-dapm.h>
#include <linux/spi/spi.h>
#include "ad183x.h"

/* codec private data */
struct ad183x_chl_ctrls {
	struct snd_kcontrol_new *snd_ctrls;
	struct snd_soc_dapm_widget *dapm_widgets;
	struct snd_soc_dapm_route *audio_paths;
	int ctrl_num;
	int widget_num;
	int path_num;
};

struct ad183x_priv {
	struct snd_soc_codec codec;
	u16 reg_cache[AD183X_NUM_REGS];
	struct ad183x_chl_ctrls chl_ctrl;
};

static struct snd_soc_codec *ad183x_codec;
struct snd_soc_codec_device soc_codec_dev_ad183x;
static int ad183x_register(struct ad183x_priv *ad183x);
static void ad183x_unregister(struct ad183x_priv *ad183x);

/*
 * AD183X volume/mute/de-emphasis etc. controls
 */
static const char *ad183x_deemp[] = {"None", "44.1kHz", "32kHz", "48kHz"};

static const struct soc_enum ad183x_deemp_enum =
SOC_ENUM_SINGLE(AD183X_DAC_CTRL1, 8, 4, ad183x_deemp);

/* AD1835A/AD1837A: 4 stereo DAC, 1 stereo ADC; */
static const struct snd_kcontrol_new ad1835a_ad1837a_snd_controls[] = {
	/* DAC volume control */
	SOC_DOUBLE_R("DAC1 Volume", AD183X_DAC_L1_VOL,
			AD183X_DAC_R1_VOL, 0, 0x3FF, 0),
	SOC_DOUBLE_R("DAC2 Volume", AD183X_DAC_L2_VOL,
			AD183X_DAC_R2_VOL, 0, 0x3FF, 0),
	SOC_DOUBLE_R("DAC3 Volume", AD183X_DAC_L3_VOL,
			AD183X_DAC_R3_VOL, 0, 0x3FF, 0),
	SOC_DOUBLE_R("DAC4 Volume", AD183X_DAC_L4_VOL,
			AD183X_DAC_R4_VOL, 0, 0x3FF, 0),

	/* ADC switch control */
	SOC_DOUBLE("ADC1 Switch", AD183X_ADC_CTRL2, AD183X_ADCL1_MUTE,
			AD183X_ADCR1_MUTE, 1, 1),

	/* DAC switch control */
	SOC_DOUBLE("DAC1 Switch", AD183X_DAC_CTRL2, AD183X_DACL1_MUTE,
			AD183X_DACR1_MUTE, 1, 1),
	SOC_DOUBLE("DAC2 Switch", AD183X_DAC_CTRL2, AD183X_DACL2_MUTE,
			AD183X_DACR2_MUTE, 1, 1),
	SOC_DOUBLE("DAC3 Switch", AD183X_DAC_CTRL2, AD183X_DACL3_MUTE,
			AD183X_DACR3_MUTE, 1, 1),
	SOC_DOUBLE("DAC4 Switch", AD183X_DAC_CTRL2, AD183X_DACL4_MUTE,
			AD183X_DACR4_MUTE, 1, 1),

	/* ADC high-pass filter */
	SOC_SINGLE("ADC High Pass Filter Switch", AD183X_ADC_CTRL1,
			AD183X_ADC_HIGHPASS_FILTER, 1, 0),

	/* DAC de-emphasis */
	SOC_ENUM("Playback Deemphasis", ad183x_deemp_enum),
};

static const struct snd_soc_dapm_widget ad1835a_ad1837a_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", AD183X_DAC_CTRL1,
			AD183X_DAC_POWERDOWN, 1),
	SND_SOC_DAPM_ADC("ADC", "Capture", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SUPPLY("ADC_PWR", AD183X_ADC_CTRL1,
			AD183X_ADC_POWERDOWN, 1, NULL, 0),
	SND_SOC_DAPM_OUTPUT("DAC1OUT"),
	SND_SOC_DAPM_OUTPUT("DAC2OUT"),
	SND_SOC_DAPM_OUTPUT("DAC3OUT"),
	SND_SOC_DAPM_OUTPUT("DAC4OUT"),
	SND_SOC_DAPM_INPUT("ADC1IN"),
};

static const struct snd_soc_dapm_route ad1835a_ad1837a_audio_paths[] = {
	{ "DAC", NULL, "ADC_PWR" },
	{ "ADC", NULL, "ADC_PWR" },
	{ "DAC1OUT", "DAC1 Switch", "DAC" },
	{ "DAC2OUT", "DAC2 Switch", "DAC" },
	{ "DAC3OUT", "DAC3 Switch", "DAC" },
	{ "DAC3OUT", "DAC4 Switch", "DAC" },
	{ "ADC", "ADC1 Switch", "ADC1IN" },
};

/* AD1836: 3 stereo DAC, 2 stereo ADC; */
static const struct snd_kcontrol_new ad1836_snd_controls[] = {
	/* DAC volume control */
	SOC_DOUBLE_R("DAC1 Volume", AD183X_DAC_L1_VOL,
			AD183X_DAC_R1_VOL, 0, 0x3FF, 0),
	SOC_DOUBLE_R("DAC2 Volume", AD183X_DAC_L2_VOL,
			AD183X_DAC_R2_VOL, 0, 0x3FF, 0),
	SOC_DOUBLE_R("DAC3 Volume", AD183X_DAC_L3_VOL,
			AD183X_DAC_R3_VOL, 0, 0x3FF, 0),

	/* ADC switch control */
	SOC_DOUBLE("ADC1 Switch", AD183X_ADC_CTRL2, AD183X_ADCL1_MUTE,
			AD183X_ADCR1_MUTE, 1, 1),
	SOC_DOUBLE("ADC2 Switch", AD183X_ADC_CTRL2, AD183X_ADCL2_MUTE,
			AD183X_ADCR2_MUTE, 1, 1),

	/* DAC switch control */
	SOC_DOUBLE("DAC1 Switch", AD183X_DAC_CTRL2, AD183X_DACL1_MUTE,
			AD183X_DACR1_MUTE, 1, 1),
	SOC_DOUBLE("DAC2 Switch", AD183X_DAC_CTRL2, AD183X_DACL2_MUTE,
			AD183X_DACR2_MUTE, 1, 1),
	SOC_DOUBLE("DAC3 Switch", AD183X_DAC_CTRL2, AD183X_DACL3_MUTE,
			AD183X_DACR3_MUTE, 1, 1),

	/* ADC high-pass filter */
	SOC_SINGLE("ADC High Pass Filter Switch", AD183X_ADC_CTRL1,
			AD183X_ADC_HIGHPASS_FILTER, 1, 0),

	/* DAC de-emphasis */
	SOC_ENUM("Playback Deemphasis", ad183x_deemp_enum),
};

static const struct snd_soc_dapm_widget ad1836_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", AD183X_DAC_CTRL1,
			AD183X_DAC_POWERDOWN, 1),
	SND_SOC_DAPM_ADC("ADC", "Capture", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SUPPLY("ADC_PWR", AD183X_ADC_CTRL1,
			AD183X_ADC_POWERDOWN, 1, NULL, 0),
	SND_SOC_DAPM_OUTPUT("DAC1OUT"),
	SND_SOC_DAPM_OUTPUT("DAC2OUT"),
	SND_SOC_DAPM_OUTPUT("DAC3OUT"),
	SND_SOC_DAPM_INPUT("ADC1IN"),
	SND_SOC_DAPM_INPUT("ADC2IN"),
};

static const struct snd_soc_dapm_route ad1836_audio_paths[] = {
	{ "DAC", NULL, "ADC_PWR" },
	{ "ADC", NULL, "ADC_PWR" },
	{ "DAC1OUT", "DAC1 Switch", "DAC" },
	{ "DAC2OUT", "DAC2 Switch", "DAC" },
	{ "DAC3OUT", "DAC3 Switch", "DAC" },
	{ "ADC", "ADC1 Switch", "ADC1IN" },
	{ "ADC", "ADC2 Switch", "ADC2IN" },
};

/* AD1838A/AD1939A: 3 stereo DAC, 1 stereo ADC; */
static const struct snd_kcontrol_new ad1838a_ad1839a_snd_controls[] = {
	/* DAC volume control */
	SOC_DOUBLE_R("DAC1 Volume", AD183X_DAC_L1_VOL,
			AD183X_DAC_R1_VOL, 0, 0x3FF, 0),
	SOC_DOUBLE_R("DAC2 Volume", AD183X_DAC_L2_VOL,
			AD183X_DAC_R2_VOL, 0, 0x3FF, 0),
	SOC_DOUBLE_R("DAC3 Volume", AD183X_DAC_L3_VOL,
			AD183X_DAC_R3_VOL, 0, 0x3FF, 0),

	/* ADC switch control */
	SOC_DOUBLE("ADC1 Switch", AD183X_ADC_CTRL2, AD183X_ADCL1_MUTE,
			AD183X_ADCR1_MUTE, 1, 1),

	/* DAC switch control */
	SOC_DOUBLE("DAC1 Switch", AD183X_DAC_CTRL2, AD183X_DACL1_MUTE,
			AD183X_DACR1_MUTE, 1, 1),
	SOC_DOUBLE("DAC2 Switch", AD183X_DAC_CTRL2, AD183X_DACL2_MUTE,
			AD183X_DACR2_MUTE, 1, 1),
	SOC_DOUBLE("DAC3 Switch", AD183X_DAC_CTRL2, AD183X_DACL3_MUTE,
			AD183X_DACR3_MUTE, 1, 1),

	/* ADC high-pass filter */
	SOC_SINGLE("ADC High Pass Filter Switch", AD183X_ADC_CTRL1,
			AD183X_ADC_HIGHPASS_FILTER, 1, 0),

	/* DAC de-emphasis */
	SOC_ENUM("Playback Deemphasis", ad183x_deemp_enum),
};

static const struct snd_soc_dapm_widget ad1838a_ad1839a_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", AD183X_DAC_CTRL1,
			AD183X_DAC_POWERDOWN, 1),
	SND_SOC_DAPM_ADC("ADC", "Capture", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SUPPLY("ADC_PWR", AD183X_ADC_CTRL1,
			AD183X_ADC_POWERDOWN, 1, NULL, 0),
	SND_SOC_DAPM_OUTPUT("DAC1OUT"),
	SND_SOC_DAPM_OUTPUT("DAC2OUT"),
	SND_SOC_DAPM_OUTPUT("DAC3OUT"),
	SND_SOC_DAPM_INPUT("ADC1IN"),
};

static const struct snd_soc_dapm_route ad1838a_ad1839a_audio_paths[] = {
	{ "DAC", NULL, "ADC_PWR" },
	{ "ADC", NULL, "ADC_PWR" },
	{ "DAC1OUT", "DAC1 Switch", "DAC" },
	{ "DAC2OUT", "DAC2 Switch", "DAC" },
	{ "DAC3OUT", "DAC3 Switch", "DAC" },
	{ "ADC", "ADC1 Switch", "ADC1IN" },
};

/*
 * DAI ops entries
 */

static int ad183x_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		/* at present, we support adc aux mode to interface with
		 * blackfin sport tdm mode
		 */
	case SND_SOC_DAIFMT_DSP_A:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		/* ALCLK,ABCLK are both output, AD183X can only be master */
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad183x_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	int word_len = 0;

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		word_len = 3;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		word_len = 1;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		word_len = 0;
		break;
	}

	snd_soc_update_bits(codec, AD183X_DAC_CTRL1,
			AD183X_DAC_WORD_LEN_MASK, word_len);

	snd_soc_update_bits(codec, AD183X_ADC_CTRL2,
			AD183X_ADC_WORD_LEN_MASK, word_len);

	return 0;
}

#ifdef CONFIG_PM
static int ad183x_soc_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	/* reset clock control mode */
	u16 adc_ctrl2 = snd_soc_read(codec, AD183X_ADC_CTRL2);
	adc_ctrl2 &= ~AD183X_ADC_SERFMT_MASK;

	return snd_soc_write(codec, AD183X_ADC_CTRL2, adc_ctrl2);
}

static int ad183x_soc_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	/* restore clock control mode */
	u16 adc_ctrl2 = snd_soc_read(codec, AD183X_ADC_CTRL2);
	adc_ctrl2 |= AD183X_ADC_AUX;

	return snd_soc_write(codec, AD183X_ADC_CTRL2, adc_ctrl2);
}
#else
#define ad183x_soc_suspend NULL
#define ad183x_soc_resume  NULL
#endif

static int __devinit ad183x_spi_probe(struct spi_device *spi)
{
	struct snd_soc_codec *codec;
	struct ad183x_priv *ad183x;
	char *chip_name = spi->dev.platform_data;

	if (!chip_name)
		return -ENODEV;

	ad183x = kzalloc(sizeof(struct ad183x_priv), GFP_KERNEL);
	if (ad183x == NULL)
		return -ENOMEM;

	if ((strcmp(chip_name, "ad1835a") == 0) || (strcmp(chip_name, "ad1837a") == 0)) {
		ad183x->chl_ctrl.snd_ctrls = (struct snd_kcontrol_new *)ad1835a_ad1837a_snd_controls;
		ad183x->chl_ctrl.dapm_widgets = (struct snd_soc_dapm_widget *)ad1835a_ad1837a_dapm_widgets;
		ad183x->chl_ctrl.audio_paths = (struct snd_soc_dapm_route *)ad1835a_ad1837a_audio_paths;
		ad183x->chl_ctrl.ctrl_num = ARRAY_SIZE(ad1835a_ad1837a_snd_controls);
		ad183x->chl_ctrl.widget_num = ARRAY_SIZE(ad1835a_ad1837a_dapm_widgets);
		ad183x->chl_ctrl.path_num = ARRAY_SIZE(ad1835a_ad1837a_audio_paths);
		ad183x_dai.playback.channels_max = 8;
		ad183x_dai.capture.channels_max = 2;
	} else if ((strcmp(chip_name, "ad1835a") == 0) || (strcmp(chip_name, "ad1837a") == 0)) {
		ad183x->chl_ctrl.snd_ctrls = (struct snd_kcontrol_new *)ad1838a_ad1839a_snd_controls;
		ad183x->chl_ctrl.dapm_widgets = (struct snd_soc_dapm_widget *)ad1838a_ad1839a_dapm_widgets;
		ad183x->chl_ctrl.audio_paths = (struct snd_soc_dapm_route *)ad1838a_ad1839a_audio_paths;
		ad183x->chl_ctrl.ctrl_num = ARRAY_SIZE(ad1838a_ad1839a_snd_controls);
		ad183x->chl_ctrl.widget_num = ARRAY_SIZE(ad1838a_ad1839a_dapm_widgets);
		ad183x->chl_ctrl.path_num = ARRAY_SIZE(ad1838a_ad1839a_audio_paths);
		ad183x_dai.playback.channels_max = 6;
		ad183x_dai.capture.channels_max = 2;
	} else if (strcmp(chip_name, "ad1836") == 0) {
		ad183x->chl_ctrl.snd_ctrls = (struct snd_kcontrol_new *)ad1836_snd_controls;
		ad183x->chl_ctrl.dapm_widgets = (struct snd_soc_dapm_widget *)ad1836_dapm_widgets;
		ad183x->chl_ctrl.audio_paths = (struct snd_soc_dapm_route *)ad1836_audio_paths;
		ad183x->chl_ctrl.ctrl_num = ARRAY_SIZE(ad1836_snd_controls);
		ad183x->chl_ctrl.widget_num = ARRAY_SIZE(ad1836_dapm_widgets);
		ad183x->chl_ctrl.path_num = ARRAY_SIZE(ad1836_audio_paths);
		ad183x_dai.playback.channels_max = 6;
		ad183x_dai.capture.channels_max = 4;
	} else {
		dev_err(&spi->dev, "not supported chip type\n");
		return -EINVAL;
	}

	codec = &ad183x->codec;
	codec->control_data = spi;
	codec->dev = &spi->dev;

	dev_set_drvdata(&spi->dev, ad183x);

	return ad183x_register(ad183x);
}

static int __devexit ad183x_spi_remove(struct spi_device *spi)
{
	struct ad183x_priv *ad183x = dev_get_drvdata(&spi->dev);

	ad183x_unregister(ad183x);
	return 0;
}

static struct spi_driver ad183x_spi_driver = {
	.driver = {
		.name	= "ad183x",
		.owner	= THIS_MODULE,
	},
	.probe		= ad183x_spi_probe,
	.remove		= __devexit_p(ad183x_spi_remove),
};

static struct snd_soc_dai_ops ad183x_dai_ops = {
	.hw_params = ad183x_hw_params,
	.set_fmt = ad183x_set_dai_fmt,
};

/* codec DAI instance */
struct snd_soc_dai ad183x_dai = {
	.name = "AD183X",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 6,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &ad183x_dai_ops,
};
EXPORT_SYMBOL_GPL(ad183x_dai);

static int ad183x_register(struct ad183x_priv *ad183x)
{
	int ret;
	struct snd_soc_codec *codec = &ad183x->codec;

	if (ad183x_codec) {
		dev_err(codec->dev, "Another ad183x is registered\n");
		return -EINVAL;
	}

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	codec->private_data = ad183x;
	codec->reg_cache = ad183x->reg_cache;
	codec->reg_cache_size = AD183X_NUM_REGS;
	codec->name = "AD183X";
	codec->owner = THIS_MODULE;
	codec->dai = &ad183x_dai;
	codec->num_dai = 1;
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	ad183x_dai.dev = codec->dev;
	ad183x_codec = codec;

	ret = snd_soc_codec_set_cache_io(codec, 4, 12, SND_SOC_SPI);
	if (ret < 0) {
		dev_err(codec->dev, "failed to set cache I/O: %d\n",
				ret);
		kfree(ad183x);
		return ret;
	}

	/* default setting for ad183x */
	/* de-emphasis: 48kHz, power-on dac */
	snd_soc_write(codec, AD183X_DAC_CTRL1, 0x300);
	/* unmute dac channels */
	snd_soc_write(codec, AD183X_DAC_CTRL2, 0x0);
	/* high-pass filter enable, power-on adc */
	snd_soc_write(codec, AD183X_ADC_CTRL1, 0x100);
	/* unmute adc channles, adc aux mode */
	snd_soc_write(codec, AD183X_ADC_CTRL2, 0x180);
	/* left/right diff:PGA/MUX */
	snd_soc_write(codec, AD183X_ADC_CTRL3, 0x3A);
	/* volume */
	snd_soc_write(codec, AD183X_DAC_L1_VOL, 0x3FF);
	snd_soc_write(codec, AD183X_DAC_R1_VOL, 0x3FF);
	snd_soc_write(codec, AD183X_DAC_L2_VOL, 0x3FF);
	snd_soc_write(codec, AD183X_DAC_R2_VOL, 0x3FF);
	snd_soc_write(codec, AD183X_DAC_L3_VOL, 0x3FF);
	snd_soc_write(codec, AD183X_DAC_R3_VOL, 0x3FF);
	snd_soc_write(codec, AD183X_DAC_L4_VOL, 0x3FF);
	snd_soc_write(codec, AD183X_DAC_R4_VOL, 0x3FF);

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		kfree(ad183x);
		return ret;
	}

	ret = snd_soc_register_dai(&ad183x_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
		snd_soc_unregister_codec(codec);
		kfree(ad183x);
		return ret;
	}

	return 0;
}

static void ad183x_unregister(struct ad183x_priv *ad183x)
{
	snd_soc_unregister_dai(&ad183x_dai);
	snd_soc_unregister_codec(&ad183x->codec);
	kfree(ad183x);
	ad183x_codec = NULL;
}

static int ad183x_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct ad183x_priv *ad183x = ad183x_codec->private_data;
	struct ad183x_chl_ctrls *chl_ctrl = &ad183x->chl_ctrl;
	int ret = 0;

	if (ad183x_codec == NULL) {
		dev_err(&pdev->dev, "Codec device not registered\n");
		return -ENODEV;
	}

	socdev->card->codec = ad183x_codec;
	codec = ad183x_codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms: %d\n", ret);
		goto pcm_err;
	}

	snd_soc_add_controls(codec, chl_ctrl->snd_ctrls, chl_ctrl->ctrl_num);
	snd_soc_dapm_new_controls(codec, chl_ctrl->dapm_widgets, chl_ctrl->widget_num);
	snd_soc_dapm_add_routes(codec, chl_ctrl->audio_paths, chl_ctrl->path_num);
	snd_soc_dapm_new_widgets(codec);

	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		dev_err(codec->dev, "failed to register card: %d\n", ret);
		goto card_err;
	}

	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	return ret;
}

/* power down chip */
static int ad183x_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_ad183x = {
	.probe = 	ad183x_probe,
	.remove = 	ad183x_remove,
	.suspend =      ad183x_soc_suspend,
	.resume =       ad183x_soc_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ad183x);

static int __init ad183x_init(void)
{
	int ret;

	ret = spi_register_driver(&ad183x_spi_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register ad183x SPI driver: %d\n",
				ret);
	}

	return ret;
}
module_init(ad183x_init);

static void __exit ad183x_exit(void)
{
	spi_unregister_driver(&ad183x_spi_driver);
}
module_exit(ad183x_exit);

MODULE_DESCRIPTION("ASoC ad183x driver");
MODULE_AUTHOR("Barry Song <21cnbao@gmail.com>");
MODULE_LICENSE("GPL");

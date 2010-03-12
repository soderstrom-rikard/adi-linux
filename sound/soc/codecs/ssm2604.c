/*
 * Driver for ssm2604 sound codec
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "ssm2604.h"

#define SSM2604_VERSION "0.1"

struct snd_soc_codec_device soc_codec_dev_ssm2604;
static struct snd_soc_codec *ssm2604_codec;
/* codec private data */
struct ssm2604_priv {
	unsigned int sysclk;
	struct snd_pcm_substream *master_substream;
	struct snd_pcm_substream *slave_substream;
	struct snd_soc_codec codec;
};

/*
 * ssm2604 register cache
 * We can't read the ssm2604 register space when we are
 * using 2 wire for device control, so we cache them instead.
 * There is no point in caching the reset register
 */
static const u16 ssm2604_reg[SSM2604_CACHEREGNUM] = {
	0x0017, 0x0017, 0x0000, 0x0000,
	0x0000, 0x000a,	0x0000, 0x0000
};

#define ssm2604_reset(c)	snd_soc_write(c, SSM2604_RESET, 0)

static const char *ssm2604_deemph[] = {"None", "32Khz", "44.1Khz", "48Khz"};

static const struct soc_enum ssm2604_enum[] = {
	SOC_ENUM_SINGLE(SSM2604_APDIGI, 1, 4, ssm2604_deemph),
};

static const struct snd_kcontrol_new ssm2604_snd_controls[] = {

SOC_DOUBLE_R("Capture Volume", SSM2604_LINVOL, SSM2604_RINVOL, 0, 31, 0),
SOC_DOUBLE_R("Capture Switch", SSM2604_LINVOL, SSM2604_RINVOL, 7, 1, 1),

SOC_SINGLE("ADC High Pass Filter Switch", SSM2604_APDIGI, 0, 1, 1),
SOC_SINGLE("Store DC Offset Switch", SSM2604_APDIGI, 4, 1, 0),

SOC_ENUM("Capture Source", ssm2604_enum[0]),

SOC_ENUM("Playback De-emphasis", ssm2604_enum[1]),
};

/* Output Mixer */
static const struct snd_kcontrol_new ssm2604_output_mixer_controls[] = {
SOC_DAPM_SINGLE("Line Bypass Switch", SSM2604_APANA, 3, 1, 0),
};

static const struct snd_soc_dapm_widget ssm2604_dapm_widgets[] = {
SND_SOC_DAPM_MIXER("Output Mixer", SSM2604_PWR, 4, 1,
	&ssm2604_output_mixer_controls[0],
	ARRAY_SIZE(ssm2604_output_mixer_controls)),
SND_SOC_DAPM_DAC("DAC", "HiFi Playback", SSM2604_PWR, 3, 1),
SND_SOC_DAPM_OUTPUT("LOUT"),
SND_SOC_DAPM_OUTPUT("ROUT"),
SND_SOC_DAPM_ADC("ADC", "HiFi Capture", SSM2604_PWR, 2, 1),
SND_SOC_DAPM_PGA("Line Input", SSM2604_PWR, 0, 1, NULL, 0),
SND_SOC_DAPM_INPUT("RLINEIN"),
SND_SOC_DAPM_INPUT("LLINEIN"),
};

static const struct snd_soc_dapm_route audio_conn[] = {
	/* output mixer */
	{"Output Mixer", "Line Bypass Switch", "Line Input"},
	{"Output Mixer", "HiFi Playback Switch", "DAC"},

	/* outputs */
	{"ROUT", NULL, "Output Mixer"},
	{"LOUT", NULL, "Output Mixer"},

	/* input mux */
	{"Input Mux", "Line", "Line Input"},
	{"ADC", NULL, "Input Mux"},

	/* inputs */
	{"Line Input", NULL, "LLINEIN"},
	{"Line Input", NULL, "RLINEIN"},
};

static int ssm2604_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, ssm2604_dapm_widgets,
				  ARRAY_SIZE(ssm2604_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_conn, ARRAY_SIZE(audio_conn));

	return 0;
}

struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr:4;
	u8 bosr:1;
	u8 usb:1;
};

/* codec mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 48k */
	{12288000, 48000, 256, 0x0, 0x0, 0x0},
	{18432000, 48000, 384, 0x0, 0x1, 0x0},
	{12000000, 48000, 250, 0x0, 0x0, 0x1},

	/* 32k */
	{12288000, 32000, 384, 0x6, 0x0, 0x0},
	{18432000, 32000, 576, 0x6, 0x1, 0x0},
	{12000000, 32000, 375, 0x6, 0x0, 0x1},

	/* 8k */
	{12288000, 8000, 1536, 0x3, 0x0, 0x0},
	{18432000, 8000, 2304, 0x3, 0x1, 0x0},
	{11289600, 8000, 1408, 0xb, 0x0, 0x0},
	{16934400, 8000, 2112, 0xb, 0x1, 0x0},
	{12000000, 8000, 1500, 0x3, 0x0, 0x1},

	/* 96k */
	{12288000, 96000, 128, 0x7, 0x0, 0x0},
	{18432000, 96000, 192, 0x7, 0x1, 0x0},
	{12000000, 96000, 125, 0x7, 0x0, 0x1},

	/* 44.1k */
	{11289600, 44100, 256, 0x8, 0x0, 0x0},
	{16934400, 44100, 384, 0x8, 0x1, 0x0},
	{12000000, 44100, 272, 0x8, 0x1, 0x1},

	/* 88.2k */
	{11289600, 88200, 128, 0xf, 0x0, 0x0},
	{16934400, 88200, 192, 0xf, 0x1, 0x0},
	{12000000, 88200, 136, 0xf, 0x1, 0x1},
};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}
	return i;
}

static int ssm2604_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	u16 srate;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct ssm2604_priv *ssm2604 = codec->private_data;
	struct i2c_client *i2c = codec->control_data;
	u16 iface = snd_soc_read(codec, SSM2604_IFACE) & 0xfff3;
	int i = get_coeff(ssm2604->sysclk, params_rate(params));

	if (substream == ssm2604->slave_substream) {
		dev_dbg(&i2c->dev, "Ignoring hw_params for slave substream\n");
		return 0;
	}

	/*no match is found*/
	if (i == ARRAY_SIZE(coeff_div))
		return -EINVAL;

	srate = (coeff_div[i].sr << 2) |
		(coeff_div[i].bosr << 1) | coeff_div[i].usb;

	snd_soc_write(codec, SSM2604_ACTIVE, 0);
	snd_soc_write(codec, SSM2604_SRATE, srate);

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0004;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0008;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface |= 0x000c;
		break;
	}
	snd_soc_write(codec, SSM2604_IFACE, iface);
	snd_soc_write(codec, SSM2604_ACTIVE, ACTIVE_ACTIVATE_CODEC);
	return 0;
}

static int ssm2604_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct ssm2604_priv *ssm2604 = codec->private_data;
	struct i2c_client *i2c = codec->control_data;
	struct snd_pcm_runtime *master_runtime;

	/* The DAI has shared clocks so if we already have a playback or
	 * capture going then constrain this substream to match it.
	 * TODO: the ssm2604 allows pairs of non-matching PB/REC rates
	 */
	if (ssm2604->master_substream) {
		master_runtime = ssm2604->master_substream->runtime;
		dev_dbg(&i2c->dev, "Constraining to %d bits at %dHz\n",
			master_runtime->sample_bits,
			master_runtime->rate);

		if (master_runtime->rate != 0)
			snd_pcm_hw_constraint_minmax(substream->runtime,
						     SNDRV_PCM_HW_PARAM_RATE,
						     master_runtime->rate,
						     master_runtime->rate);

		if (master_runtime->sample_bits != 0)
			snd_pcm_hw_constraint_minmax(substream->runtime,
						     SNDRV_PCM_HW_PARAM_SAMPLE_BITS,
						     master_runtime->sample_bits,
						     master_runtime->sample_bits);

		ssm2604->slave_substream = substream;
	} else
		ssm2604->master_substream = substream;

	return 0;
}

static int ssm2604_pcm_prepare(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	/* set active */
	snd_soc_write(codec, SSM2604_ACTIVE, ACTIVE_ACTIVATE_CODEC);

	return 0;
}

static void ssm2604_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct ssm2604_priv *ssm2604 = codec->private_data;

	/* deactivate */
	if (!codec->active)
		snd_soc_write(codec, SSM2604_ACTIVE, 0);

	if (ssm2604->master_substream == substream)
		ssm2604->master_substream = ssm2604->slave_substream;

	ssm2604->slave_substream = NULL;
}

static int ssm2604_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = snd_soc_read(codec, SSM2604_APDIGI) & ~APDIGI_ENABLE_DAC_MUTE;
	if (mute)
		snd_soc_write(codec, SSM2604_APDIGI,
				mute_reg | APDIGI_ENABLE_DAC_MUTE);
	else
		snd_soc_write(codec, SSM2604_APDIGI, mute_reg);
	return 0;
}

static int ssm2604_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct ssm2604_priv *ssm2604 = codec->private_data;
	switch (freq) {
	case 11289600:
	case 12000000:
	case 12288000:
	case 16934400:
	case 18432000:
		ssm2604->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}

static int ssm2604_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface |= 0x0040;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0013;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0003;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
	default:
		return -EINVAL;
	}

	/* set iface */
	snd_soc_write(codec, SSM2604_IFACE, iface);
	return 0;
}

static int ssm2604_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	u16 reg = snd_soc_read(codec, SSM2604_PWR) & 0xff7f;

	switch (level) {
	case SND_SOC_BIAS_ON:
		/* vref/mid, osc on, dac unmute */
		snd_soc_write(codec, SSM2604_PWR, reg);
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		/* everything off except vref/vmid, */
		snd_soc_write(codec, SSM2604_PWR, reg | PWR_CLK_OUT_PDN);
		break;
	case SND_SOC_BIAS_OFF:
		/* everything off, dac mute, inactive */
		snd_soc_write(codec, SSM2604_ACTIVE, 0);
		snd_soc_write(codec, SSM2604_PWR, 0xffff);
		break;

	}
	codec->bias_level = level;
	return 0;
}

#define SSM2604_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_32000 |\
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |\
		SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define SSM2604_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops ssm2604_dai_ops = {
	.startup	= ssm2604_startup,
	.prepare	= ssm2604_pcm_prepare,
	.hw_params	= ssm2604_hw_params,
	.shutdown	= ssm2604_shutdown,
	.digital_mute	= ssm2604_mute,
	.set_sysclk	= ssm2604_set_dai_sysclk,
	.set_fmt	= ssm2604_set_dai_fmt,
};

struct snd_soc_dai ssm2604_dai = {
	.name = "SSM2604",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SSM2604_RATES,
		.formats = SSM2604_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SSM2604_RATES,
		.formats = SSM2604_FORMATS,},
	.ops = &ssm2604_dai_ops,
};
EXPORT_SYMBOL_GPL(ssm2604_dai);

static int ssm2604_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	ssm2604_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int ssm2604_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(ssm2604_reg); i++) {
		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		codec->hw_write(codec->control_data, data, 2);
	}
	ssm2604_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	ssm2604_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}

static int ssm2604_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int reg, ret = 0;

	socdev->card->codec = ssm2604_codec;
	codec = ssm2604_codec;

	ssm2604_reset(codec);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "ssm2604: failed to create pcms\n");
		goto pcm_err;
	}
	/*power on device*/
	snd_soc_write(codec, SSM2604_ACTIVE, 0);
	/* set the update bits */
	reg = snd_soc_read(codec, SSM2604_LINVOL);
	snd_soc_write(codec, SSM2604_LINVOL, reg | LINVOL_LRIN_BOTH);
	reg = snd_soc_read(codec, SSM2604_RINVOL);
	snd_soc_write(codec, SSM2604_RINVOL, reg | RINVOL_RLIN_BOTH);

	snd_soc_write(codec, SSM2604_APANA, APANA_SELECT_DAC);
	snd_soc_write(codec, SSM2604_PWR, 0);

	snd_soc_add_controls(codec, ssm2604_snd_controls,
				ARRAY_SIZE(ssm2604_snd_controls));
	ssm2604_add_widgets(codec);

	return ret;

pcm_err:
	return ret;
}

/* remove everything here */
static int ssm2604_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_ssm2604 = {
	.probe =	ssm2604_probe,
	.remove =	ssm2604_remove,
	.suspend =	ssm2604_suspend,
	.resume =	ssm2604_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ssm2604);

static int ssm2604_register(struct ssm2604_priv *ssm2604, enum snd_soc_control_type control)
{
	struct snd_soc_codec *codec = &ssm2604->codec;
	int ret = 0;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	codec->name = "SSM2604";
	codec->owner = THIS_MODULE;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->set_bias_level = ssm2604_set_bias_level;
	codec->dai = &ssm2604_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(ssm2604_reg);
	codec->reg_cache = kmemdup(ssm2604_reg, sizeof(ssm2604_reg),
					GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;

	ret = snd_soc_codec_set_cache_io(codec, 7, 9, control);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		return ret;
	}

	ret = snd_soc_register_dai(&ssm2604_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
		snd_soc_unregister_codec(codec);
		return ret;
	}

	return ret;
}

static void ssm2604_unregister(struct ssm2604_priv *ssm2604)
{
	struct snd_soc_codec *codec = &ssm2604->codec;

	ssm2604_set_bias_level(&ssm2604->codec, SND_SOC_BIAS_OFF);
	kfree(codec->reg_cache);
	snd_soc_unregister_dai(&ssm2604_dai);
	snd_soc_unregister_codec(&ssm2604->codec);
	kfree(ssm2604);
	ssm2604_codec = NULL;
}

static int ssm2604_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct ssm2604_priv *ssm2604;
	struct snd_soc_codec *codec;

	ssm2604 = kzalloc(sizeof(struct ssm2604_priv), GFP_KERNEL);
	if (ssm2604 == NULL)
		return -ENOMEM;
	codec = &ssm2604->codec;
	codec->private_data = ssm2604;

	i2c_set_clientdata(i2c, ssm2604);
	codec->control_data = i2c;

	codec->dev = &i2c->dev;
	ssm2604_codec = codec;

	return ssm2604_register(ssm2604, SND_SOC_I2C);
}

static __devexit int ssm2604_i2c_remove(struct i2c_client *client)
{
	struct ssm2604_priv *ssm2604 = i2c_get_clientdata(client);
	ssm2604_unregister(ssm2604);
	return 0;
}

static const struct i2c_device_id ssm2604_i2c_id[] = {
	{ "ssm2604", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ssm2604_i2c_id);

/* corgi i2c codec control layer */
static struct i2c_driver ssm2604_i2c_driver = {
	.driver = {
		.name = "ssm2604",
		.owner = THIS_MODULE,
	},
	.probe    = ssm2604_i2c_probe,
	.remove   = __devexit_p(ssm2604_i2c_remove),
	.id_table = ssm2604_i2c_id,
};

static int __init ssm2604_modinit(void)
{
	int ret;

	ret = i2c_add_driver(&ssm2604_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register ssm2604 I2C driver: %d\n",
		       ret);
	}

	return ret;
}
module_init(ssm2604_modinit);

static void __exit ssm2604_exit(void)
{
	i2c_del_driver(&ssm2604_i2c_driver);
}
module_exit(ssm2604_exit);

MODULE_DESCRIPTION("ASoC SSM2604 driver");
MODULE_AUTHOR("Cliff Cai");
MODULE_LICENSE("GPL");

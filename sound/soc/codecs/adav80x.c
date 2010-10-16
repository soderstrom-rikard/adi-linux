/*
 * ADAV80X Audio Codec driver supporting ADAV801, ADAV803
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/soc-dapm.h>
#include "adav80x.h"


struct adav80x_priv {
	struct snd_soc_codec codec;
	u16 reg_cache[ADAV80X_NUM_REGS];
	int clk_src; /* clock source for ADC, DAC and internal clock */
};

static struct snd_soc_codec *adav80x_codec;
struct snd_soc_codec_device soc_codec_dev_adav80x;
static int adav80x_register(struct adav80x_priv *adav80x, int bus_type);
static void adav80x_unregister(struct adav80x_priv *adav80x);

static const struct snd_soc_dapm_widget adav80x_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", ADAV80X_DAC_CTRL1, 7, 1),
	SND_SOC_DAPM_ADC("ADC", "Capture", ADAV80X_ADC_CTRL1, 5, 1),
	SND_SOC_DAPM_SUPPLY("PLL_PWR", ADAV80X_PLL_CTRL1, 1, 1, NULL, 0),
};

static const struct snd_soc_dapm_route audio_paths[] = {
	{ "DAC", NULL, "PLL_PWR" },
	{ "ADC", NULL, "PLL_PWR" },
};

/*
 * ADAV80X volume/mute/de-emphasis etc. controls
 */
static const char *adav80x_deemp[] = {"None", "44.1kHz", "32kHz", "48kHz"};

static const struct soc_enum adav80x_deemp_enum =
SOC_ENUM_SINGLE(ADAV80X_DAC_CTRL2, 0, 4, adav80x_deemp);

static const struct snd_kcontrol_new adav80x_snd_controls[] = {
	/* DAC volume control */
	SOC_DOUBLE_R("DAC Volume", ADAV80X_DAC_L_VOL,
			ADAV80X_DAC_R_VOL, 0, 0xFF, 0),
	/* DAC peak volume detect, read clears it */
	SOC_DOUBLE_R("DAC Peak Volume", ADAV80X_DAC_L_PEAK_VOL,
			ADAV80X_DAC_R_PEAK_VOL, 0, 0x3F, 0),

	/* ADC volume control */
	SOC_DOUBLE_R("ADC Volume", ADAV80X_ADC_L_VOL,
			ADAV80X_ADC_R_VOL, 0, 0xFF, 0),
	/* ADC peak volume detect */
	SOC_DOUBLE_R("ADC Peak Volume", ADAV80X_ADC_L_PEAK_VOL,
			ADAV80X_ADC_R_PEAK_VOL, 0, 0x3F, 0),

	/* ADC mute */
	SOC_DOUBLE("ADC Switch", ADAV80X_ADC_CTRL1, 2, 3, 1, 1),

	/* DAC mute */
	SOC_DOUBLE("DAC Switch", ADAV80X_DAC_CTRL1, 0, 1, 1, 1),

	/* ADC high-pass filter */
	SOC_SINGLE("ADC High Pass Filter Switch", ADAV80X_ADC_CTRL1, 6, 1, 0),

	/* DAC de-emphasis */
	SOC_ENUM("Playback Deemphasis", adav80x_deemp_enum),
};

/*
 * DAI ops entries
 */

static int adav80x_set_dai_fmt(struct snd_soc_dai *codec_dai,
				unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 rec_ctl = 0, playback_ctl = 0;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		/* use internal clock 1 */
		playback_ctl |= 0x10;
		rec_ctl |= 0x20;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	/* DAC word length depends on input, ADC need to set word length */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		playback_ctl |= 0x1;
		rec_ctl |= 0x1;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		playback_ctl |= 0x4;
		rec_ctl |= 0x3;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	/* Use default - seems no polarity setting */
	/* DAC control reg 1: CHSEL[1:0], POL[1:0] ? */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, ADAV80X_PLAYBACK_CTRL, playback_ctl);
	snd_soc_write(codec, ADAV80X_REC_CTRL, rec_ctl);

	return 0;
}

static int adav80x_set_adc_clock(struct snd_soc_codec *codec,
		int clk_id, unsigned int sample_rate)
{
	int reg;

	if (clk_id == ADAV80X_CLK_PLL1) {
		/* ADC assumes that the MCLK ratre is 256 times the sample rate.
		   We also assumes PLL1 clock rate to be (256 * Fs),
		   So set ADC MCLK divider to be 1 */

		reg = snd_soc_read(codec, ADAV80X_ADC_CTRL1);
		/* ADC Modulator clock is 6.144MHz Max,
		   need to set devidor properly */
		if (sample_rate == 96000)
			reg |= 0x80;
		else if (sample_rate == 48000)
			reg &= 0x7F;
		else
			/* Unsupported sample rate */
			return -1;

		snd_soc_write(codec, ADAV80X_ADC_CTRL1, reg);
	}

	return 0;
}

static int adav80x_set_dac_clock(struct snd_soc_codec *codec,
		int clk_id, unsigned int sample_rate)
{
	int reg;

	if (clk_id == ADAV80X_CLK_PLL1) {
		/* PLL1 clock rate is assumed to be 256 * Fs */

		reg = snd_soc_read(codec, ADAV80X_DAC_CTRL2);
		if (sample_rate == 96000)
			/* Set the MCLK divider to be MCLK/2,
			   and MCLK = 128 * Fs */
			reg |= 0x24;
		else
			reg &= 0x11;

		snd_soc_write(codec, ADAV80X_DAC_CTRL2, reg);
	}

	return 0;
}

static int adav80x_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	int word_len = 0;
	int rate = params_rate(params);

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct adav80x_priv *adav80x = snd_soc_codec_get_drvdata(codec);

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		word_len = 3;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		word_len = 2;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		word_len = 1;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		word_len = 0;
		break;
	}


	/* Playback port does not need to set word length? */

	/* Record Port Control */
	snd_soc_update_bits(codec, ADAV80X_REC_CTRL, 0x3<<2, word_len<<2);

	/* Set up clock */
	if (adav80x->clk_src == ADAV80X_CLK_PLL1) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			adav80x_set_dac_clock(codec, ADAV80X_CLK_PLL1, rate);
		else
			adav80x_set_adc_clock(codec, ADAV80X_CLK_PLL1, rate);
	}

	return 0;
}

#ifdef CONFIG_PM
static int adav80x_soc_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	return 0;
}

static int adav80x_soc_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define adav80x_soc_suspend NULL
#define adav80x_soc_resume  NULL
#endif

static int adav80x_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	int reg;

	reg = snd_soc_read(codec, ADAV80X_DAC_CTRL1);
	reg = (mute > 0) ? reg & ~0x3 : reg | 0x3;
	snd_soc_write(codec, ADAV80X_DAC_CTRL1, reg);

	return 0;
}


static int adav80x_set_dai_pll(struct snd_soc_dai *codec_dai, int pll_id,
		int source, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adav80x_priv *adav80x = snd_soc_codec_get_drvdata(codec);
	int reg = 0;

	/* For now, we only enable PLL1 with XIN as source */
	if (source != ADAV80X_CLK_XIN)
		return -1;

	if (freq_in && freq_out) {
		/* XIN - assumes 27MHz */
		if (freq_in != 27000000)
			return -1;

		if (pll_id != ADAV80X_CLK_PLL1)
			return -1;

		/* freq_out = sample_rate * 256 */
		switch (freq_out) {
		case 32000:
			reg = 0x8;
			break;
		case 44100:
			reg = 0xC;
			break;
		case 48000:
			reg = 0x0;
			break;
		case 64000:
			reg = 0x9;
			break;
		case 88200:
			reg = 0xD;
			break;
		case 96000:
			reg = 0x1;
			break;
		}

		/* Set PLL1 clock */
		snd_soc_write(codec, ADAV80X_PLL_CTRL2, reg);

		if (adav80x->clk_src == ADAV80X_CLK_PLL1)
			return 0;
		else
			adav80x->clk_src = ADAV80X_CLK_PLL1;

		/* select XIN as PLL1 clock source */
		snd_soc_write(codec, ADAV80X_PLL_CLK_SRC, 0x0);
		/* set PLL1 as clock source for internal clock, DAC, ADC */
		snd_soc_write(codec, ADAV80X_ICLK_CTRL1, 0x4A);
		snd_soc_write(codec, ADAV80X_ICLK_CTRL2, 0x10);
		/* Power on PLL1, power down PLL2, power on XTAL */
		snd_soc_write(codec, ADAV80X_PLL_CTRL1, 0x8);

	} else	{
		if (adav80x->clk_src == ADAV80X_CLK_XIN)
			return 0;
		else
			adav80x->clk_src = ADAV80X_CLK_XIN;

		/* Turn off PLL, power on XTAL */
		snd_soc_write(codec, ADAV80X_PLL_CTRL1, 0xC);

		/* DAC, ADC, ICLK clock source - XIN */
		snd_soc_write(codec, ADAV80X_ICLK_CTRL1, 0x0);
		snd_soc_write(codec, ADAV80X_ICLK_CTRL2, 0x0);
	}

	return 0;
}

static struct snd_soc_dai_ops adav80x_dai_ops = {
	.hw_params = adav80x_hw_params,
	.set_fmt = adav80x_set_dai_fmt,
	.digital_mute = adav80x_mute,
	.set_pll = adav80x_set_dai_pll,
};

/* codec DAI instance */
/* DAC sample rates: 32/44.1/48/96/192kHz, ADC sample reate: 48/96kHz - TBD */
struct snd_soc_dai adav80x_dai = {
	.name = "ADAV80X",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_96000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &adav80x_dai_ops,
};
EXPORT_SYMBOL_GPL(adav80x_dai);


static int adav80x_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	if (adav80x_codec == NULL) {
		dev_err(&pdev->dev, "Codec device not registered\n");
		return -ENODEV;
	}

	socdev->card->codec = adav80x_codec;
	codec = adav80x_codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms: %d\n", ret);
		goto pcm_err;
	}

	snd_soc_add_controls(codec, adav80x_snd_controls,
				ARRAY_SIZE(adav80x_snd_controls));
	snd_soc_dapm_new_controls(codec, adav80x_dapm_widgets,
				  ARRAY_SIZE(adav80x_dapm_widgets));
	snd_soc_dapm_add_routes(codec, audio_paths, ARRAY_SIZE(audio_paths));

pcm_err:
	return ret;
}

/* power down chip */
static int adav80x_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_adav80x = {
	.probe =	adav80x_probe,
	.remove =	adav80x_remove,
	.suspend =	adav80x_soc_suspend,
	.resume =	adav80x_soc_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_adav80x);

static int adav80x_register(struct adav80x_priv *adav80x, int bus_type)
{
	int ret;
	struct snd_soc_codec *codec = &adav80x->codec;

	if (adav80x_codec) {
		dev_err(codec->dev, "Another adav80x is registered\n");
		return -EINVAL;
	}

	mutex_init(&codec->mutex);
	snd_soc_codec_set_drvdata(codec, adav80x);
	codec->reg_cache = adav80x->reg_cache;
	codec->reg_cache_size = ADAV80X_NUM_REGS;
	codec->name = "ADAV80X";
	codec->owner = THIS_MODULE;
	codec->dai = &adav80x_dai;
	codec->num_dai = 1;

	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	adav80x_dai.dev = codec->dev;
	adav80x_codec = codec;


	if (bus_type == SND_SOC_I2C)
		/* addr(7-bit left shifted 1), data(8bit)*/
		ret = snd_soc_codec_set_cache_io(codec, 8, 8, bus_type);
	else
		/* register format: addr(7bit), data(9bit) */
		/* set spi bits_per_word to 8 instead of 16,
		   since addr need to be transfer before data */
		ret = snd_soc_codec_set_cache_io(codec, 7, 9, bus_type);
	if (ret < 0) {
		dev_err(codec->dev, "failed to set cache I/O: %d\n",
				ret);
		kfree(adav80x);
		return ret;
	}

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		kfree(adav80x);
		return ret;
	}

	ret = snd_soc_register_dai(&adav80x_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
		snd_soc_unregister_codec(codec);
		kfree(adav80x);
		return ret;
	}

	/* default setting for adav80x */

	/* Power down SYSCLK output, power down S/PDIF receiver */
	snd_soc_write(codec, ADAV80X_PLL_OUTE, 0x27);

	/* Disable S/PDIF transmitter */
	snd_soc_write(codec, ADAV80X_TX_CTRL, 0x0);

	/* Datapath: ADC->record, AUX_IN->SRC->AUX_OUT */
	snd_soc_write(codec, ADAV80X_DPATH_CTRL1, 0xC4);
	/* Datapath: playback->DAC, DIR->DIT */
	snd_soc_write(codec, ADAV80X_DPATH_CTRL2, 0x11);

	/* Soft-mute SRC output */
	snd_soc_write(codec, ADAV80X_GDELAY_MUTE, 0x80);

	/* DAC: de-emphasis: none, MCLCK divider: 1, MCLK=256xFs */
	/* snd_soc_write(codec, ADAV80X_DAC_CTRL2, 0x0); */
	/* Disable DAC zero flag */
	snd_soc_write(codec, ADAV80X_DAC_CTRL3, 0x6);
	/* DAC: volume */
	snd_soc_write(codec, ADAV80X_DAC_L_VOL, 0xFF);
	snd_soc_write(codec, ADAV80X_DAC_R_VOL, 0xFF);

	/* ADC: power up, unmute adc channles */
	/* snd_soc_write(codec, ADAV80X_ADC_CTRL1, 0x0); */
	/* MCLCK divider: 1 */
	/* snd_soc_write(codec, ADAV80X_ADC_CTRL2, 0x0); */
	/* ADC: volumn */
	snd_soc_write(codec, ADAV80X_ADC_L_VOL, 0xFF);
	snd_soc_write(codec, ADAV80X_ADC_R_VOL, 0xFF);

	/* Disable ALC */
	snd_soc_write(codec, ADAV80X_ALC_CTRL1, 0x0);

	return 0;
}

static void adav80x_unregister(struct adav80x_priv *adav80x)
{
	snd_soc_unregister_dai(&adav80x_dai);
	snd_soc_unregister_codec(&adav80x->codec);
	kfree(adav80x);
	adav80x_codec = NULL;
}

static int __devinit adav80x_bus_probe(struct device *dev, void *ctrl_data,
		int bus_type)
{
	struct snd_soc_codec *codec;
	struct adav80x_priv *adav80x;

	adav80x = kzalloc(sizeof(struct adav80x_priv), GFP_KERNEL);
	if (adav80x == NULL)
		return -ENOMEM;

	codec = &adav80x->codec;
	codec->control_data = ctrl_data;
	codec->dev = dev;

	dev_set_drvdata(dev, adav80x);

	return adav80x_register(adav80x, bus_type);
}

int adav80x_bus_remove(struct device *dev)
{
	struct adav80x_priv *adav80x = dev_get_drvdata(dev);

	adav80x_unregister(adav80x);
	return 0;
}

#if defined(CONFIG_SPI_MASTER)
static int __devinit adav80x_spi_probe(struct spi_device *spi)
{
	return adav80x_bus_probe(&spi->dev, spi, SND_SOC_SPI);
}

static int __devexit adav80x_spi_remove(struct spi_device *spi)
{
	return adav80x_bus_remove(&spi->dev);
}

static struct spi_driver adav80x_spi_driver = {
	.driver = {
		.name	= "adav80x",
		.owner	= THIS_MODULE,
	},
	.probe		= adav80x_spi_probe,
	.remove		= __devexit_p(adav80x_spi_remove),
};
#endif

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static const struct i2c_device_id adav80x_id[] = {
	{ "adav803", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adav80x_id);

static int __devinit adav80x_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	return adav80x_bus_probe(&client->dev, client, SND_SOC_I2C);
}

static int __devexit adav80x_i2c_remove(struct i2c_client *client)
{
	return adav80x_bus_remove(&client->dev);
}

static struct i2c_driver adav80x_i2c_driver = {
	.driver = {
		.name = "adav80x",
	},
	.probe    = adav80x_i2c_probe,
	.remove   = __devexit_p(adav80x_i2c_remove),
	.id_table = adav80x_id,
};
#endif

static int __init adav80x_init(void)
{
	int ret;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	ret =  i2c_add_driver(&adav80x_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register ADAV80X I2C driver: %d\n",
				ret);
	}
#elif defined(CONFIG_SPI_MASTER)
	ret = spi_register_driver(&adav80x_spi_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register ADAV80X SPI driver: %d\n",
				ret);
	}
#endif
	return ret;
}
module_init(adav80x_init);

static void __exit adav80x_exit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&adav80x_i2c_driver);
#elif defined(CONFIG_SPI_MASTER)
	spi_unregister_driver(&adav80x_spi_driver);
#endif
}
module_exit(adav80x_exit);

MODULE_DESCRIPTION("ASoC adav80x driver");
MODULE_AUTHOR("Yi Li");
MODULE_LICENSE("GPL");

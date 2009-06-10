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
#include <linux/spi/spi.h>
#include "ad1938.h"

struct snd_soc_device *ad1938_socdev;

/* struct to flag whether adc and dac need power to work */
struct ad1938_pwr_sta {
	int adc_pwr;
	int dac_pwr;
};

/* dac de-emphasis enum control */
static const char *ad1938_deemp[] = {"flat", "48kHz", "44.1kHz", "32kHz"};

static const struct soc_enum ad1938_enum[] = {
	SOC_ENUM_SINGLE(AD1938_DAC_CTRL2, 1, 4, ad1938_deemp),
};

/* AD1938 volume/mute/de-emphasis etc. controls */
static const struct snd_kcontrol_new ad1938_snd_controls[] = {
	/* DAC volume control */
	SOC_SINGLE("DAC L1 Volume", AD1938_DAC_L1_VOL, 0, 0xFF, 1),
	SOC_SINGLE("DAC R1 Volume", AD1938_DAC_R1_VOL, 0, 0xFF, 1),
	SOC_SINGLE("DAC L2 Volume", AD1938_DAC_L2_VOL, 0, 0xFF, 1),
	SOC_SINGLE("DAC R2 Volume", AD1938_DAC_R2_VOL, 0, 0xFF, 1),
	SOC_SINGLE("DAC L3 Volume", AD1938_DAC_L3_VOL, 0, 0xFF, 1),
	SOC_SINGLE("DAC R3 Volume", AD1938_DAC_R3_VOL, 0, 0xFF, 1),
	SOC_SINGLE("DAC L4 Volume", AD1938_DAC_L4_VOL, 0, 0xFF, 1),
	SOC_SINGLE("DAC R4 Volume", AD1938_DAC_R4_VOL, 0, 0xFF, 1),

	/* DAC mute control */
	SOC_SINGLE("DAC L1 Switch", AD1938_DAC_CHNL_MUTE, 0, 1, 1),
	SOC_SINGLE("DAC R1 Switch", AD1938_DAC_CHNL_MUTE, 1, 1, 1),
	SOC_SINGLE("DAC L2 Switch", AD1938_DAC_CHNL_MUTE, 2, 1, 1),
	SOC_SINGLE("DAC R2 Switch", AD1938_DAC_CHNL_MUTE, 3, 1, 1),
	SOC_SINGLE("DAC L3 Switch", AD1938_DAC_CHNL_MUTE, 4, 1, 1),
	SOC_SINGLE("DAC R3 Switch", AD1938_DAC_CHNL_MUTE, 5, 1, 1),
	SOC_SINGLE("DAC L4 Switch", AD1938_DAC_CHNL_MUTE, 6, 1, 1),
	SOC_SINGLE("DAC R4 Switch", AD1938_DAC_CHNL_MUTE, 7, 1, 1),

	/* ADC mute control */
	SOC_SINGLE("ADC L1 Switch", AD1938_ADC_CTRL0, ADC0_MUTE, 1, 1),
	SOC_SINGLE("ADC R1 Switch", AD1938_ADC_CTRL0, ADC1_MUTE, 1, 1),
	SOC_SINGLE("ADC L2 Switch", AD1938_ADC_CTRL0, ADC2_MUTE, 1, 1),
	SOC_SINGLE("ADC R2 Switch", AD1938_ADC_CTRL0, ADC3_MUTE, 1, 1),

	/* ADC high-pass filter */
	SOC_SINGLE("ADC High Pass Filter Switch", AD1938_ADC_CTRL0, ADC_HIGHPASS_FILTER, 1, 0),

	/* DAC de-emphasis */
	SOC_ENUM("Playback Deemphasis", ad1938_enum[0]),
};

/* add non dapm controls */
static int ad1938_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(ad1938_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&ad1938_snd_controls[i], codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

/* dai_ops.digital_mute entry */
static int ad1938_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	if (!mute)
		codec->write(codec, AD1938_DAC_CHNL_MUTE, 0);
	else
		codec->write(codec, AD1938_DAC_CHNL_MUTE, 0xff);

	return 0;
}

/* dai_ops.set_pll entry */
static int ad1938_set_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct ad1938_pwr_sta *pwr_sta = codec->private_data;
	int pll_reg;

	if (freq_out) {
		pll_reg = codec->read(codec, AD1938_PLL_CLK_CTRL0);
		pll_reg &= ~PLL_POWERDOWN;
		codec->write(codec, AD1938_PLL_CLK_CTRL0, pll_reg);
	} else {
		/* playing while recording, framework will poweroff-poweron pll redundantly */
		if ((pwr_sta->dac_pwr == 0) && (pwr_sta->adc_pwr == 0)) {
			pll_reg = codec->read(codec, AD1938_PLL_CLK_CTRL0);
			pll_reg |= PLL_POWERDOWN;
			codec->write(codec, AD1938_PLL_CLK_CTRL0, pll_reg);
		}
	}

	return 0;
}

/* ops.prepare entry */
static int ad1938_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct ad1938_pwr_sta *pwr_sta = codec->private_data;
	int pwr_reg;

	/* set active */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* If not poweron adc, dac can't work */
		pwr_reg = codec->read(codec, AD1938_ADC_CTRL0);
		if (pwr_reg & ADC_POWERDOWN)
			codec->write(codec, AD1938_ADC_CTRL0, pwr_reg & ~ADC_POWERDOWN);

		/* poweron dac */
		pwr_reg = codec->read(codec, AD1938_DAC_CTRL0);
		pwr_reg &= ~DAC_POWERDOWN;
		codec->write(codec, AD1938_DAC_CTRL0, pwr_reg);

		pwr_sta->dac_pwr = 1;
	} else {
		/* poweron adc */
		pwr_reg = codec->read(codec, AD1938_ADC_CTRL0);
		pwr_reg &= ~ADC_POWERDOWN;
		codec->write(codec, AD1938_ADC_CTRL0, pwr_reg);

		pwr_sta->adc_pwr = 1;
	}

	return 0;
}


/* ops.shutdown entry */
static void ad1938_pcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct ad1938_pwr_sta *pwr_sta = codec->private_data;
	int pwr_reg;

	/* deactivate */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* poweroff dac */
		pwr_reg = codec->read(codec, AD1938_DAC_CTRL0);
		pwr_reg |= DAC_POWERDOWN;
		codec->write(codec, AD1938_DAC_CTRL0, pwr_reg);

		pwr_sta->dac_pwr = 0;

		/* after poweroff dac, if adc is not opened, poweroff it too */
		if (pwr_sta->adc_pwr == 0) {
			pwr_reg = codec->read(codec, AD1938_ADC_CTRL0);
			pwr_reg |= ADC_POWERDOWN;
			codec->write(codec, AD1938_ADC_CTRL0, pwr_reg);
		}
	} else {
		/* if dac is still working, can't shutdown adc */
		if (pwr_sta->dac_pwr == 0) {
			pwr_reg = codec->read(codec, AD1938_ADC_CTRL0);
			pwr_reg |= ADC_POWERDOWN;
			codec->write(codec, AD1938_ADC_CTRL0, pwr_reg);
		}

		pwr_sta->adc_pwr = 0;
	}
}

/*
 * interface to read/write ad1938 register
 */

#define AD1938_SPI_ADDR    0x4
#define AD1938_SPI_READ    0x1
#define AD1938_SPI_BUFLEN  3

/*
 * write to the ad1938 register space
 */

static int ad1938_reg_write(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value)
{
	uint8_t buf[AD1938_SPI_BUFLEN];
	struct spi_transfer t = {
		.tx_buf = buf,
		.len = AD1938_SPI_BUFLEN,
	};
	struct spi_message m;

	buf[0] = AD1938_SPI_ADDR << 1;
	buf[1] = reg;
	buf[2] = value;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(codec->control_data, &m);
}

/*
 * read from the ad1938 register space
 */

static unsigned int ad1938_reg_read(struct snd_soc_codec *codec, unsigned int reg)
{
	char w_buf[AD1938_SPI_BUFLEN];
	char r_buf[AD1938_SPI_BUFLEN];
	int ret;

	struct spi_transfer t = {
		.tx_buf = w_buf,
		.rx_buf = r_buf,
		.len = AD1938_SPI_BUFLEN,
	};
	struct spi_message m;

	w_buf[0] = (AD1938_SPI_ADDR << 1) | AD1938_SPI_READ;
	w_buf[1] = reg;
	w_buf[2] = 0;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(codec->control_data, &m);
	if (ret == 0)
		return	r_buf[2];
	else
		return -EIO;
}

static int __devinit ad1938_spi_probe(struct spi_device *spi)
{
	spi->dev.power.power_state = PMSG_ON;
	ad1938_socdev->codec->control_data = spi;

	return 0;
}

static int __devexit ad1938_spi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver ad1938_spi_driver = {
	.driver = {
		.name	= "ad1938-spi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= ad1938_spi_probe,
	.remove		= __devexit_p(ad1938_spi_remove),
};

static int ad1938_spi_init(void)
{
	return spi_register_driver(&ad1938_spi_driver);
}

static void ad1938_spi_done(void)
{
	spi_unregister_driver(&ad1938_spi_driver);
}

/* codec DAI instance */
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
	.ops = {
		.prepare = ad1938_pcm_prepare,
		.shutdown = ad1938_pcm_shutdown,
	},
	.dai_ops = {
		.digital_mute = ad1938_mute,
		.set_pll = ad1938_set_pll,
	},
};
EXPORT_SYMBOL_GPL(ad1938_dai);

static int ad1938_soc_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	/* codec alloc and init */
	codec = kzalloc(sizeof(struct snd_soc_codec) + sizeof(struct ad1938_pwr_sta), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;
	mutex_init(&codec->mutex);
	codec->name = "AD1938";
	codec->owner = THIS_MODULE;
	codec->dai = &ad1938_dai;
	codec->num_dai = 1;
	codec->write = ad1938_reg_write;
	codec->read = ad1938_reg_read;
	codec->private_data = codec + 1;
	socdev->codec = codec;
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	ad1938_socdev = socdev;

	/* codec spi interface init */
	ret = ad1938_spi_init();
	if (ret < 0) {
		printk(KERN_ERR "ad1938: failed to init spi interface\n");
		goto spi_err;
	}

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

	/* default setting for ad1938, poweroff dac/adc/pll */
	codec->write(codec, AD1938_DAC_CTRL0, 0x41); /* sample rate:32/44.1/48kHz, sata delay=1, tdm mode */
	codec->write(codec, AD1938_DAC_CTRL1, 0x84); /* invert bclk, 256bclk/frame, latch in mid */
	codec->write(codec, AD1938_DAC_CTRL2, 0x1A); /* de-emphasis: 48kHz */
	codec->write(codec, AD1938_ADC_CTRL0, 0x33); /* high-pass filter enable */
	codec->write(codec, AD1938_ADC_CTRL1, 0x43); /* sata delay=1, adc aux mode */
	codec->write(codec, AD1938_ADC_CTRL2, 0x6F); /* left high, driver on rising edge */
	codec->write(codec, AD1938_DAC_CHNL_MUTE, 0xFF); /* mute all dac channels */
	codec->write(codec, AD1938_PLL_CLK_CTRL0, 0x9D); /* pll input:mclki/xi, master clock rate:512*fs */
	codec->write(codec, AD1938_PLL_CLK_CTRL1, 0x04);

	/* register controls for ad1938 */
	ad1938_add_controls(codec);

	printk(KERN_INFO "Analog Devices AD1938 codec registered\n");
	return ret;

register_err:
	snd_soc_free_pcms(socdev);
pcm_err:
	ad1938_spi_done();
spi_err:
	kfree(socdev->codec);
	socdev->codec = NULL;
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

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
};
EXPORT_SYMBOL_GPL(ad1938_dai);

struct snd_soc_device *ad1938_socdev;

/* DAC volume controls */
static const struct snd_kcontrol_new ad1938_snd_controls[] = {
	SOC_SINGLE("DAC L1", AD1938_DAC_L1_VOL, 0, 0xFF, 1),
	SOC_SINGLE("DAC R1", AD1938_DAC_R1_VOL, 0, 0xFF, 1),
	SOC_SINGLE("DAC L2", AD1938_DAC_L2_VOL, 0, 0xFF, 1),
	SOC_SINGLE("DAC R2", AD1938_DAC_R2_VOL, 0, 0xFF, 1),
	SOC_SINGLE("DAC L3", AD1938_DAC_L3_VOL, 0, 0xFF, 1),
	SOC_SINGLE("DAC R3", AD1938_DAC_R3_VOL, 0, 0xFF, 1),
	SOC_SINGLE("DAC L4", AD1938_DAC_L4_VOL, 0, 0xFF, 1),
	SOC_SINGLE("DAC R4", AD1938_DAC_R4_VOL, 0, 0xFF, 1),
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

static int ad1938_soc_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	/* codec alloc and init */
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;
	mutex_init(&codec->mutex);
	codec->name = "AD1938";
	codec->owner = THIS_MODULE;
	codec->dai = &ad1938_dai;
	codec->num_dai = 1;
	codec->write = ad1938_reg_write;
	codec->read = ad1938_reg_read;
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

	/* default setting for ad1938: 8 channel AUX ADC mode, 16bit, 48000Hz */
	codec->write(codec, AD1938_DAC_CTRL0, 0x40);
	codec->write(codec, AD1938_DAC_CTRL1, 0x84);
	codec->write(codec, AD1938_DAC_CTRL2, 0x1A);
	codec->write(codec, AD1938_ADC_CTRL0, 0x32);
	codec->write(codec, AD1938_ADC_CTRL1, 0x43);
	codec->write(codec, AD1938_ADC_CTRL2, 0x6f);
	codec->write(codec, AD1938_PLL_CLK_CTRL0, 0x9C);
	codec->write(codec, AD1938_PLL_CLK_CTRL1, 0x04);

	/* register controls for ad1938 */
	ad1938_add_controls(codec);

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

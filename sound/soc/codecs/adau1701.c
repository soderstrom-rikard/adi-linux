/*
 * Driver for ADAU1701 sound codec
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/sigma.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "adau1701.h"

#define AUDIO_NAME "adau1701"
#define ADAU1701_VERSION "0.10"
#define ADAU1701_FIRMWARE "SigmaDSP_fw.bin"

static const u16 adau1701_reg_defaults[] = {
	0x0000,     /* R0 */
	0x0000,     /* R1 */
	0x0000,     /* R2 */
	0x0000,     /* R3 */
	0x0000,     /* R4 */
	0x0000,     /* R5 */
	0x0000,     /* R6 */
	0x0000,     /* R7 */
	0x0000,     /* R8 */
	0x0000,     /* R9 */
	0x0000,     /* R10 */
	0x0000,     /* R11 */
	0x0000,     /* R12 */
	0x0000,     /* R13 */
	0x0000,     /* R14 */
	0x0000,     /* R15 */
	0x0000,     /* R16 */
	0x0000,     /* R17 */
	0x0000,     /* R18 */
	0x0000,     /* R19 */
	0x0000,     /* R20 */
	0x0000,     /* R21 */
	0x0000,     /* R22 */
	0x0000,     /* R23 */
	0x0000,     /* R24 */
	0x0000,     /* R25 */
	0x0000,     /* R26 */
	0x0000,     /* R27 */
	0x0000,     /* R28  - DSP Core Control */
	0x0000,     /* R29 */
	0x0000,     /* R30  - Serial Output Control */
	0x0000,     /* R31  - Serial Input Control*/
	0x0000,     /* R32 */
	0x0000,     /* R33 */
	0x0000,     /* R34  - Auxiliary ADC and Power Control */
	0x0000,     /* R35 */
	0x0000,     /* R36  - Auxiliary ADC Enable*/
	0x0000,     /* R37  - DAC Setup */
};

/* codec private data */
struct adau1701_priv {
	struct snd_soc_codec *codec;
};

/*
 * write register cache
 */
static inline int adau1701_write_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;

	if (reg < ADAU1701_FIRSTREG)
		reg = reg + ADAU1701_FIRSTREG;

	if ((reg < ADAU1701_FIRSTREG) || (reg > ADAU1701_LASTREG))
		return -1;

	cache[reg - ADAU1701_FIRSTREG] = value;

	return 0;
}

/*
 * write a multibyte ADAU1701 register
 */
static int adau1701_write_reg_block(struct snd_soc_codec *codec,
	unsigned int reg, u8 length, u8 *values)
{
	int count = length + 2; /*data plus 16bit register address*/
	u8 buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	buf[0] = (u8)(reg >> 8);
	buf[1] = (u8)(reg & 0xFF);

	if (length > 0)
		memcpy(&buf[2], values, length);

	if (codec->hw_write(codec->control_data, buf, count) == count)
		return 0;
	else {
		dev_err(codec->dev, "address block write failed.");
		return -EIO;
	}
}

/*
 * read ADAU1701 hw register and update cache
 */
static int adau1701_read_reg_byte(struct snd_soc_codec *codec,
	u16 reg)
{
	u8 addr[2];
	u8 buf[1] = {0};

	if (reg < ADAU1701_FIRSTREG)
		reg = reg + ADAU1701_FIRSTREG;

	if ((reg < ADAU1701_FIRSTREG) || (reg > ADAU1701_LASTREG))
		return -EIO;

	addr[0] = (u8)(reg >> 8);
	addr[1] = (u8)(reg & 0xFF);

	/* write the 2byte read address */
	if (codec->hw_write(codec->control_data, addr, 2) != 2) {
		printk(KERN_ERR "read_reg_byte:address write failed.");
		return -EIO;
	}

	if (i2c_master_recv(codec->control_data, buf, 1) != 1)
		return -EIO;

	return buf[0];
}

static int adau1701_setprogram(struct snd_soc_codec *codec)
{
	int ret = 0;

	ret = process_sigma_firmware(codec->control_data, ADAU1701_FIRMWARE);

	return ret;
}

static int adau1701_pcm_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	snd_soc_write(codec, ADAU1701_OSCIPOW, 0x0);

	return ret;
}

static void adau1701_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	snd_soc_write(codec, ADAU1701_OSCIPOW, 0x02);

}

static int adau1701_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 reg = 0;
	if (mute) {
		/* mute inputs/outputs */
		reg = snd_soc_read(codec, ADAU1701_AUXNPOW);
		reg |= AUXNPOW_AAPD | AUXNPOW_D1PD | AUXNPOW_D0PD;
		snd_soc_write(codec, ADAU1701_AUXNPOW, reg);
	} else {
		/* unmute inputs/outputs */
		reg = snd_soc_read(codec, ADAU1701_AUXNPOW);
		reg &= ~(AUXNPOW_AAPD | AUXNPOW_D1PD | AUXNPOW_D0PD);
		snd_soc_write(codec, ADAU1701_AUXNPOW, reg);
	}

	return 0;
}

static int adau1701_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 reg = 0;

	reg = adau1701_read_reg_byte(codec, (u16)ADAU1701_SERITL1);
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		reg |= SERITL1_LEFTJ;
		break;
	/* TODO: support TDM */
	default:
		return 0;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	/* TODO: support signal inversions */
	default:
		return 0;
	}

	/* set I2S iface format*/
	adau1701_write_reg_block(codec, ADAU1701_SERITL1, 1, &reg);
	return 0;
}

static int adau1701_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	u16 reg;
	switch (level) {
	case SND_SOC_BIAS_ON:
		reg = snd_soc_read(codec, ADAU1701_AUXNPOW);
		reg &= ~(AUXNPOW_AAPD | AUXNPOW_D1PD | AUXNPOW_D0PD |\
			AUXNPOW_VBPD | AUXNPOW_VRPD);
		snd_soc_write(codec, ADAU1701_AUXNPOW, reg);
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		snd_soc_write(codec, ADAU1701_OSCIPOW, 0x02);
		break;
	case SND_SOC_BIAS_OFF:
		/* everything off, dac mute, inactive */
		snd_soc_write(codec, ADAU1701_OSCIPOW, 0x02);
		reg = snd_soc_read(codec, ADAU1701_AUXNPOW);
		reg |= AUXNPOW_AAPD | AUXNPOW_D1PD | AUXNPOW_D0PD |\
			AUXNPOW_VBPD | AUXNPOW_VRPD;
		snd_soc_write(codec, ADAU1701_AUXNPOW, reg);
		break;

	}
	codec->bias_level = level;
	return 0;
}

#define ADAU1701_RATES SNDRV_PCM_RATE_48000

#define ADAU1701_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops adau1701_dai_ops = {
	.prepare	= adau1701_pcm_prepare,
	.shutdown	= adau1701_shutdown,
	.digital_mute	= adau1701_mute,
	.set_fmt	= adau1701_set_dai_fmt,
};

struct snd_soc_dai_driver adau1701_dai = {
	.name = "ADAU1701",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ADAU1701_RATES,
		.formats = ADAU1701_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ADAU1701_RATES,
		.formats = ADAU1701_FORMATS,
	},
	.ops = &adau1701_dai_ops,
};
EXPORT_SYMBOL_GPL(adau1701_dai);

static int adau1701_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	adau1701_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int adau1701_resume(struct snd_soc_codec *codec)
{
	adau1701_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

static ssize_t adau1371_dsp_load(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int ret = 0;
	struct adau1701_priv *adau1701 = dev_get_drvdata(dev);
	struct snd_soc_codec *codec = adau1701->codec;
	ret = adau1701_setprogram(codec);
	if (ret)
		return ret;
	else
		return count;
}
static DEVICE_ATTR(dsp, 0644, NULL, adau1371_dsp_load);

static int adau1701_reg_init(struct snd_soc_codec *codec)
{
	u16 reg16;
	u8 reg[3];

	reg16 = DSPCTRL_DAM | DSPCTRL_ADM;
	snd_soc_write(codec, ADAU1701_DSPCTRL, reg16);
	/* Load default program */
	adau1701_setprogram(codec);
	reg16 = DSPCTRL_DAM | DSPCTRL_ADM;
	snd_soc_write(codec, ADAU1701_DSPCTRL, reg16);
	reg[0] = 0x80;
	adau1701_write_reg_block(codec, ADAU1701_DSPRES, 1, &reg[0]);
	snd_soc_write(codec, ADAU1701_SEROCTL, 0);
	/* PLLMODE1 = 1, PLLMODE0 = 0, 256*fs */
	snd_soc_write(codec, ADAU1701_SEROCTL, 0x0d00);
	reg[0] = 0;
	adau1701_write_reg_block(codec, ADAU1701_SERITL1, 1, &reg[0]);
	reg[0] = MPCONF_SDATAP | MPCONF_SDATAP << 4;
	reg[1] = 0;
	reg[2] = MPCONF_SDATAP;
	adau1701_write_reg_block(codec, ADAU1701_MPCONF0, 3, &reg[0]);
	adau1701_write_reg_block(codec, ADAU1701_MPCONF1, 3, &reg[0]);
	snd_soc_write(codec, ADAU1701_AUXNPOW, 0);
	reg16 = AUXADCE_AAEN;
	snd_soc_write(codec, ADAU1701_AUXADCE, reg16);
	reg16 = DACSET_DACEN;
	snd_soc_write(codec, ADAU1701_DACSET, reg16);
	reg16 = DSPCTRL_DAM | DSPCTRL_ADM | DSPCTRL_CR;
	snd_soc_write(codec, ADAU1701_DSPCTRL, reg16);
	/* power-down oscillator */
	snd_soc_write(codec, ADAU1701_OSCIPOW, 0x02);

	return 0;
}

static int adau1701_probe(struct snd_soc_codec *codec)
{
	int ret = 0;

	struct adau1701_priv *adau1701 = snd_soc_codec_get_drvdata(codec);

	adau1701->codec = codec;
	ret = adau1701_reg_init(codec);
	if (ret < 0) {
		dev_err(codec->dev, "failed to initialize\n");
		return ret;
	}
	ret = snd_soc_codec_set_cache_io(codec, 16, 16, SND_SOC_I2C);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}
	ret = device_create_file(codec->dev, &dev_attr_dsp);
	if (ret)
		dev_err(codec->dev, "device_create_file() failed\n");

	return ret;
}

/* remove everything here */
static int adau1701_remove(struct snd_soc_codec *codec)
{
	adau1701_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

struct snd_soc_codec_driver soc_codec_dev_adau1701 = {
	.probe =	adau1701_probe,
	.remove =	adau1701_remove,
	.suspend =	adau1701_suspend,
	.resume =	adau1701_resume,
	.set_bias_level = adau1701_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(adau1701_reg_defaults),
	.reg_word_size = sizeof(u16),
	.reg_cache_default = adau1701_reg_defaults,
};

static __devinit int adau1701_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct adau1701_priv *adau1701;
	int ret = 0;

	adau1701 = kzalloc(sizeof(struct adau1701_priv), GFP_KERNEL);
	if (adau1701 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, adau1701);
	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_adau1701, &adau1701_dai, 1);
	if (ret < 0)
		kfree(adau1701);

	return ret;
}

static __devexit int adau1701_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id adau1701_i2c_id[] = {
	{ "adau1701", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adau1701_i2c_id);

/* corgi i2c codec control layer */
static struct i2c_driver adau1701_i2c_driver = {
	.driver = {
		.name = "adau1701",
		.owner = THIS_MODULE,
	},
	.probe    = adau1701_i2c_probe,
	.remove   = __devexit_p(adau1701_i2c_remove),
	.id_table = adau1701_i2c_id,
};

static int __init adau1701_modinit(void)
{
	int ret;

	ret = i2c_add_driver(&adau1701_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register adau1701 I2C driver: %d\n",
		       ret);
	}

	return ret;
}
module_init(adau1701_modinit);

static void __exit adau1701_exit(void)
{
	i2c_del_driver(&adau1701_i2c_driver);
}
module_exit(adau1701_exit);

MODULE_DESCRIPTION("ASoC ADAU1701 driver");
MODULE_AUTHOR("Cliff Cai");
MODULE_LICENSE("GPL");

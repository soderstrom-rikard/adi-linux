/*
 * Driver for ADAU1701 SigmaDSP processor
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

/* codec private data */
struct adau1701_priv {
	struct snd_soc_codec *codec;
	enum snd_soc_control_type control_type;
};

/*
 * Write a ADAU1701 register,since the register length is from 1 to 5,
 * So, use our own read/write functions instead of snd_soc_read/write.
 */
static int adau1701_write_register(struct snd_soc_codec *codec,
	u16 reg_address, u8 length, u32 value)
{
	int ret;
	int count = length + 2; /*data plus 16bit register address*/
	u8 buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	if (length == 0)
		return -1;
	buf[0] = (reg_address >> 8) & 0xFF;
	buf[1] = reg_address & 0xFF;
	if (length == 1)
		buf[2] = value & 0xFF;
	else if (length == 2) {
		buf[2] = (value >> 8) & 0xFF;
		buf[3] = value & 0xFF;
	} else if (length == 3) {
		buf[2] = (value >> 16) & 0xFF;
		buf[3] = (value >> 8) & 0xFF;
		buf[4] = value & 0xFF;
	}
	ret = i2c_master_send(codec->control_data, buf, count);

	return ret;

}

/*
 * read ADAU1701 hw register
 */
static u32 adau1701_read_register(struct snd_soc_codec *codec,
	u16 reg_address, u8 length)
{
	u8 addr[2];
	u8 buf[2];
	u32 value = 0;
	int ret;

	if (reg_address < ADAU1701_FIRSTREG)
		reg_address = reg_address + ADAU1701_FIRSTREG;

	if ((reg_address < ADAU1701_FIRSTREG) || (reg_address > ADAU1701_LASTREG))
		return -EIO;

	addr[0] = (reg_address >> 8) & 0xFF;
	addr[1] = reg_address & 0xFF;

	/* write the 2byte read address */
	ret = i2c_master_send(codec->control_data, addr, 2);
	if (ret)
		return ret;

	if (length == 1) {
		if (i2c_master_recv(codec->control_data, buf, 1) != 1)
			return -EIO;
		value = buf[0];
	} else if (length == 2) {
		if (i2c_master_recv(codec->control_data, buf, 2) != 2)
			return -EIO;
		value = (buf[0] << 8) | buf[1];
	}
	return value;
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
	int reg = 0;

	reg = SEROCTL_MASTER | SEROCTL_OBF16 | SEROCTL_OLF1024;
	adau1701_write_register(codec, ADAU1701_SEROCTL, 2, reg);

	return 0;
}

static void adau1701_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	adau1701_write_register(codec, ADAU1701_SEROCTL, 2, 0);
}

static int adau1701_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 reg = 0;

	if (mute) {
		/* mute inputs/outputs */
		reg = adau1701_read_register(codec, ADAU1701_AUXNPOW, 2);
		reg |= AUXNPOW_AAPD | AUXNPOW_D0PD | AUXNPOW_D1PD | AUXNPOW_D2PD | AUXNPOW_D3PD;
		adau1701_write_register(codec, ADAU1701_AUXNPOW, 2, reg);
	} else {
		/* unmute inputs/outputs */
		reg = adau1701_read_register(codec, ADAU1701_AUXNPOW, 2);
		reg &= ~(AUXNPOW_AAPD | AUXNPOW_D0PD | AUXNPOW_D1PD | AUXNPOW_D2PD | AUXNPOW_D3PD);
		adau1701_write_register(codec, ADAU1701_AUXNPOW, 2, reg);
	}

	return 0;
}

static int adau1701_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u32 reg = 0;

	reg = adau1701_read_register(codec, ADAU1701_SERITL1, 1);
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

	/* set iface format*/
	adau1701_write_register(codec, ADAU1701_SERITL1, 1, reg);
	return 0;
}

static int adau1701_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	u16 reg;
	switch (level) {
	case SND_SOC_BIAS_ON:
		reg = adau1701_read_register(codec, ADAU1701_AUXNPOW, 2);
		reg &= ~(AUXNPOW_AAPD | AUXNPOW_D0PD | AUXNPOW_D1PD |  AUXNPOW_D2PD |
			 AUXNPOW_D3PD | AUXNPOW_VBPD | AUXNPOW_VRPD);
		adau1701_write_register(codec, ADAU1701_AUXNPOW, 2, reg);
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		/* everything off, dac mute, inactive */
		reg = adau1701_read_register(codec, ADAU1701_AUXNPOW, 2);
		reg |= AUXNPOW_AAPD | AUXNPOW_D0PD | AUXNPOW_D1PD |  AUXNPOW_D2PD |
			 AUXNPOW_D3PD | AUXNPOW_VBPD | AUXNPOW_VRPD;
		adau1701_write_register(codec, ADAU1701_AUXNPOW, 2, reg);
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
	u32 reg;
	int ret = 0;

	reg = DSPCTRL_DAM | DSPCTRL_ADM;
	adau1701_write_register(codec, ADAU1701_DSPCTRL, 2, reg);
	/* Load default program */
	ret = adau1701_setprogram(codec);
	if (ret < 0) {
		printk(KERN_ERR "Loading program data failed\n");
		goto error;
	}
	reg = DSPCTRL_DAM | DSPCTRL_ADM;
	adau1701_write_register(codec, ADAU1701_DSPCTRL, 2, reg);
	reg = 0x08;
	adau1701_write_register(codec, ADAU1701_DSPRES, 1, reg);
	adau1701_write_register(codec, ADAU1701_SEROCTL, 2, 0);
	adau1701_write_register(codec, ADAU1701_SERITL1, 1, 0);
	/* Configure the multipurpose pins as serial in/out pins */
	reg = MPCONF_SDATAP | MPCONF_SDATAP << 16 | MPCONF_SDATAP << 20;
	adau1701_write_register(codec, ADAU1701_MPCONF0, 3, reg);
	reg = MPCONF_AUXADC << 8 | MPCONF_SDATAP << 12 | MPCONF_SDATAP << 16 |
		MPCONF_SDATAP << 20;
	adau1701_write_register(codec, ADAU1701_MPCONF1, 3, reg);
	adau1701_write_register(codec, ADAU1701_AUXNPOW, 2, 0);
	reg = AUXADCE_AAEN;
	adau1701_write_register(codec, ADAU1701_AUXADCE, 2, reg);
	reg = DACSET_DACEN;
	adau1701_write_register(codec, ADAU1701_DACSET, 2, reg);
	reg = DSPCTRL_DAM | DSPCTRL_ADM | DSPCTRL_CR;
	adau1701_write_register(codec, ADAU1701_DSPCTRL, 2, reg);
	/* Power-up the oscillator */
	adau1701_write_register(codec, ADAU1701_OSCIPOW, 2, 0);
error:
	return ret;
}

static int adau1701_probe(struct snd_soc_codec *codec)
{
	int ret = 0;

	struct adau1701_priv *adau1701 = snd_soc_codec_get_drvdata(codec);

	adau1701->codec = codec;
	ret = snd_soc_codec_set_cache_io(codec, 16, 16, adau1701->control_type);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}
	ret = adau1701_reg_init(codec);
	if (ret < 0) {
		dev_err(codec->dev, "failed to initialize\n");
		return ret;
	}
	ret = device_create_file(codec->dev, &dev_attr_dsp);
	if (ret)
		dev_err(codec->dev, "device_create_file() failed\n");

	return ret;
}

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
};

static __devinit int adau1701_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct adau1701_priv *adau1701;
	int ret = 0;

	adau1701 = kzalloc(sizeof(struct adau1701_priv), GFP_KERNEL);
	if (adau1701 == NULL)
		return -ENOMEM;

	adau1701->control_type = SND_SOC_I2C;
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
		.name = "adau1701-codec",
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

MODULE_DESCRIPTION("ASoC ADAU1701 SigmaDSP driver");
MODULE_AUTHOR("Cliff Cai");
MODULE_LICENSE("GPL");

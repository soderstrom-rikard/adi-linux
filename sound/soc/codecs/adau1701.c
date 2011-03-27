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

#define ADAU1701_FIRMWARE "adau1701.bin"
#define USE_OSCILLATOR	1
/* codec private data */
struct adau1701_priv {
	struct snd_soc_codec *codec;
	enum snd_soc_control_type control_type;
};

struct register_lut {
	u16 address;
	int length;
};

static const struct register_lut register_table[] = {

	{ADAU1701_IFACE0, 4},
	{ADAU1701_IFACE1, 4},
	{ADAU1701_IFACE2, 4},
	{ADAU1701_IFACE3, 4},
	{ADAU1701_IFACE4, 4},
	{ADAU1701_IFACE5, 4},
	{ADAU1701_IFACE6, 4},
	{ADAU1701_IFACE7, 4},

	{ADAU1701_GPIOSET, 2},

	{ADAU1701_AUXADC0, 2},
	{ADAU1701_AUXADC1, 2},
	{ADAU1701_AUXADC2, 2},
	{ADAU1701_AUXADC3, 2},

	{ADAU1701_DATCAP0, 2},
	{ADAU1701_DATCAP1, 2},

	{ADAU1701_DSPCTRL, 2},
	{ADAU1701_DSPRES, 1},
	{ADAU1701_SEROCTL, 1},
	{ADAU1701_SERITL1, 1},

	{ADAU1701_MPCONF0, 3},
	{ADAU1701_MPCONF1, 3},

	{ADAU1701_AUXNPOW, 2},
	{ADAU1701_AUXADCE, 2},

	{ADAU1701_OSCIPOW, 2},
	{ADAU1701_DACSET, 2},
};
/*
 * Write a ADAU1701 register,since the register length is from 1 to 5,
 * So, use our own read/write functions instead of snd_soc_read/write.
 */
static int adau1701_write_register(struct snd_soc_codec *codec,
	u16 reg_address, u32 value)
{
	int i;
	int length, count;
	u8 buf[8];

	for (i = 0; i < ARRAY_SIZE(register_table); i++) {
		if (register_table[i].address == reg_address)
			break;
	}
	if (i == ARRAY_SIZE(register_table)) {
		dev_err(codec->dev, "Wrong Register Address\n");
		return -EINVAL;
	}
	length = register_table[i].length;
	count = length + 2; /*data plus 16bit register address*/

	buf[0] = reg_address >> 8;
	buf[1] = reg_address;
	if (length == 1)
		buf[2] = value;
	else if (length == 2) {
		buf[2] = value >> 8;
		buf[3] = value;
	} else if (length == 3) {
		buf[2] = value >> 16;
		buf[3] = value >> 8;
		buf[4] = value;
	} else if (length == 4) {
		buf[2] = value >> 24;
		buf[3] = value >> 16;
		buf[4] = value >> 8;
		buf[5] = value;
	}
	return i2c_master_send(codec->control_data, buf, count);
}

/*
 * read ADAU1701 hw register
 */
static u32 adau1701_read_register(struct snd_soc_codec *codec,
	u16 reg_address)
{
	u8 addr[2];
	u8 buf[3];
	int ret, i, length;
	u32 value = 0;

	for (i = 0; i < ARRAY_SIZE(register_table); i++) {
		if (register_table[i].address == reg_address)
			break;
	}
	if (i == ARRAY_SIZE(register_table)) {
		dev_err(codec->dev, "Wrong Register Address\n");
		return -EINVAL;
	}
	length = register_table[i].length;

	if (reg_address < ADAU1701_FIRSTREG)
		reg_address = reg_address + ADAU1701_FIRSTREG;

	if ((reg_address < ADAU1701_FIRSTREG) || (reg_address > ADAU1701_LASTREG))
		return -EIO;

	addr[0] = reg_address >> 8;
	addr[1] = reg_address;

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
	} else if (length == 3) {
		if (i2c_master_recv(codec->control_data, buf, 3) != 3)
			return -EIO;
		value = (buf[0] << 16) | (buf[1] << 8) | buf[2];
	}
	return value;
}

static int adau1701_setprogram(struct snd_soc_codec *codec)
{
	int ret;

	ret = process_sigma_firmware(codec->control_data, ADAU1701_FIRMWARE);

	return ret;
}

static int adau1701_pcm_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 reg;

	reg = ADAU1701_SEROCTL_MASTER | ADAU1701_SEROCTL_OBF16 |
		ADAU1701_SEROCTL_OLF1024;
	adau1701_write_register(codec, ADAU1701_SEROCTL, reg);

	return 0;
}

static void adau1701_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	adau1701_write_register(codec, ADAU1701_SEROCTL, 0);
}

static int adau1701_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 reg;

	reg = adau1701_read_register(codec, ADAU1701_SERITL1);
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		reg &= ~ADAU1701_SERITL1_MSK;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		reg |= ADAU1701_SERITL1_LEFTJ;
		break;
	/* TODO: support TDM */
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	/* TODO: support signal inversions */
	default:
		return -EINVAL;
	}

	/* set iface format*/
	adau1701_write_register(codec, ADAU1701_SERITL1, reg);
	return 0;
}

static int adau1701_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	u16 reg;
	switch (level) {
	case SND_SOC_BIAS_ON:
		reg = adau1701_read_register(codec, ADAU1701_AUXNPOW);
		reg &= ~(ADAU1701_AUXNPOW_AAPD | ADAU1701_AUXNPOW_D0PD |
			ADAU1701_AUXNPOW_D1PD |  ADAU1701_AUXNPOW_D2PD |
			ADAU1701_AUXNPOW_D3PD | ADAU1701_AUXNPOW_VBPD |
			ADAU1701_AUXNPOW_VRPD);
		adau1701_write_register(codec, ADAU1701_AUXNPOW, reg);
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		/* everything off, dac mute, inactive */
		reg = adau1701_read_register(codec, ADAU1701_AUXNPOW);
		reg |= ADAU1701_AUXNPOW_AAPD | ADAU1701_AUXNPOW_D0PD |
			ADAU1701_AUXNPOW_D1PD |  ADAU1701_AUXNPOW_D2PD |
			ADAU1701_AUXNPOW_D3PD | ADAU1701_AUXNPOW_VBPD |
			ADAU1701_AUXNPOW_VRPD;
		adau1701_write_register(codec, ADAU1701_AUXNPOW, reg);
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
	.set_fmt	= adau1701_set_dai_fmt,
};

struct snd_soc_dai_driver adau1701_dai = {
	.name = "adau1701",
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
	int ret;
	u32 reg;
	struct adau1701_priv *adau1701 = dev_get_drvdata(dev);
	struct snd_soc_codec *codec = adau1701->codec;

	reg = ADAU1701_DSPCTRL_DAM | ADAU1701_DSPCTRL_ADM;
	adau1701_write_register(codec, ADAU1701_DSPCTRL, reg);
	ret = adau1701_setprogram(codec);
	reg = ADAU1701_DSPCTRL_DAM | ADAU1701_DSPCTRL_ADM;
	adau1701_write_register(codec, ADAU1701_DSPCTRL, reg);
	if (ret)
		return ret;
	else
		return count;
}
static DEVICE_ATTR(dsp, 0644, NULL, adau1371_dsp_load);

static int adau1701_reg_init(struct snd_soc_codec *codec)
{
	u32 reg;
	int ret;

	/* Load default program */
	ret = adau1701_setprogram(codec);
	if (ret < 0) {
		dev_err(codec->dev, "Loading program data failed\n");
		goto error;
	}
	reg = 0x08;
	adau1701_write_register(codec, ADAU1701_DSPRES, reg);
	adau1701_write_register(codec, ADAU1701_SEROCTL, 0);
	adau1701_write_register(codec, ADAU1701_SERITL1, 0);
	/* Configure the multipurpose pins as serial in/out pins */
	reg = ADAU1701_MPCONF_SDATAP | ADAU1701_MPCONF_SDATAP << 16 |
		ADAU1701_MPCONF_SDATAP << 20;
	adau1701_write_register(codec, ADAU1701_MPCONF0, reg);
	reg = ADAU1701_MPCONF_AUXADC << 8 | ADAU1701_MPCONF_SDATAP << 12 |
		ADAU1701_MPCONF_SDATAP << 16 | ADAU1701_MPCONF_SDATAP << 20;
	adau1701_write_register(codec, ADAU1701_MPCONF1, reg);
	adau1701_write_register(codec, ADAU1701_AUXNPOW, 0);
	reg = ADAU1701_AUXADCE_AAEN;
	adau1701_write_register(codec, ADAU1701_AUXADCE, reg);
	reg = ADAU1701_DACSET_DACEN;
	adau1701_write_register(codec, ADAU1701_DACSET, reg);
	reg = ADAU1701_DSPCTRL_DAM | ADAU1701_DSPCTRL_ADM | ADAU1701_DSPCTRL_CR;
	adau1701_write_register(codec, ADAU1701_DSPCTRL, reg);
#ifdef USE_OSCILLATOR
	/* Power-up the oscillator */
	adau1701_write_register(codec, ADAU1701_OSCIPOW, 0);
#endif
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
	return adau1701_set_bias_level(codec, SND_SOC_BIAS_OFF);
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
	if (ret != 0)
		printk(KERN_ERR "Failed to register adau1701 I2C driver: %d\n",
		       ret);

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

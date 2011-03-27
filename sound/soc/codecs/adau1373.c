/*
 * Driver for ADAU1373 sound codec
 *
 * Copyright 2009-2011 Analog Devices Inc.
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
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "adau1373.h"

/* codec private data */
struct adau1373_priv {
	unsigned int sysclk;
	unsigned int out_chan_mask;
	unsigned int in_chan_mask;
	u8 adau1373_pll_reg[6];
	struct adau1373_platform_data *data;
	struct snd_soc_codec codec;
	enum snd_soc_control_type control_type;
};

/*
 * write a multibyte ADAU1373 register (6byte pll reg)
 */
static int adau1373_write_reg_block(struct snd_soc_codec *codec,
	unsigned int reg, u8 length, u8 *values)
{
	int count = length + 1; /* data plus 8bit register address */
	u8 buf[7] = {0, 0, 0, 0, 0, 0, 0};

	buf[0] = (u8)(reg & 0xFF);

	if (length > 0)
		memcpy(&buf[1], values, length);

	if (codec->hw_write(codec->control_data, buf, count) == count)
		return 0;
	else {
		dev_err(codec->dev, "address block write failed.");
		return -EIO;
	}
}

static int adau1373_volume_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = CHANNELS_OUTPUT;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 31;
	return 0;
}

static int adau1373_volume_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	int i;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1373_priv *chip = snd_soc_codec_get_drvdata(codec);
	u8 *cache = codec->reg_cache;

	for (i = 0; i < CHANNELS_OUTPUT; ++i) {
		if (chip->out_chan_mask & PB_LINE1)
			ucontrol->value.integer.value[i] =
				cache[ADAU_LLN1OPT + i];
		else if (chip->out_chan_mask & PB_LINE2)
			ucontrol->value.integer.value[i] =
				cache[ADAU_LLN2OPT + i];
		else if (chip->out_chan_mask & PB_SPK)
			ucontrol->value.integer.value[i] =
				cache[ADAU_LCDOUTP + i];
/*
		else if (chip->out_chan_mask & PB_EARP)
			ucontrol->value.integer.value[i] =
				cache[ADAU1373_LHPVOL + i];
*/
		else if (chip->out_chan_mask & PB_HP)
			ucontrol->value.integer.value[i] =
				cache[ADAU_LHPOUTP + i];
	}

	return 0;
}

static int adau1373_volume_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	int i;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1373_priv *chip = snd_soc_codec_get_drvdata(codec);
	int change = 0;
	u8 *cache = codec->reg_cache;

	for (i = 0; i < CHANNELS_OUTPUT; ++i) {
		int vol = clamp_t(int, ucontrol->value.integer.value[i], 0, 31);

		if (chip->out_chan_mask & PB_LINE1) {
			if (cache[ADAU_LLN1OPT + i] != vol) {
				change = 1;
				snd_soc_write(codec, ADAU_LLN1OPT + i, vol);
			}
		} else if (chip->out_chan_mask & PB_LINE2) {
			if (cache[ADAU_LLN2OPT + i] != vol) {
				change = 1;
				snd_soc_write(codec, ADAU_LLN2OPT + i, vol);
			}
		} else if (chip->out_chan_mask & PB_HP) {
			if (cache[ADAU_LHPOUTP + i] != vol) {
				change = 1;
				snd_soc_write(codec, ADAU_LHPOUTP + i, vol);
			}
		} else if (chip->out_chan_mask & PB_SPK) {
			if (cache[ADAU_LCDOUTP + i] != vol) {
				change = 1;
				snd_soc_write(codec, ADAU_LCDOUTP + i, vol);
			}
		} else if (chip->out_chan_mask & PB_EARP) {
				snd_soc_write(codec, ADAU_EPCNTRL, cache[ADAU_EPCNTRL] | vol);
		}
	}

	return change;
}

static int adau1373_cap_volume_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = CHANNELS_INPUT;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 31;
	return 0;
}

static int adau1373_cap_volume_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int i;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1373_priv *chip = snd_soc_codec_get_drvdata(codec);
	u8 *cache = codec->reg_cache;

	for (i = 0; i < CHANNELS_INPUT; ++i) {
		if (chip->in_chan_mask & CAP_INPA)
			ucontrol->value.integer.value[i] =
				cache[ADAU_IN1LCTL + i];
		else if (chip->in_chan_mask & CAP_INPB)
			ucontrol->value.integer.value[i] =
				cache[ADAU_IN2LCTL + i];
		else if (chip->in_chan_mask & CAP_INPC)
			ucontrol->value.integer.value[i] =
				cache[ADAU_IN3LCTL + i];
		else if (chip->in_chan_mask & CAP_INPD)
			ucontrol->value.integer.value[i] =
				cache[ADAU_AUXLCTL + i];
	}

	return 0;
}

static int adau1373_cap_volume_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int i, check, vol;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1373_priv *chip = snd_soc_codec_get_drvdata(codec);
	int change = 0;
	u8 *cache = codec->reg_cache;

	for (i = 0; i < CHANNELS_INPUT; ++i) {
		if (chip->in_chan_mask & CAP_INPA)
			check = ADAU_IN1LCTL;
		else if (chip->in_chan_mask & CAP_INPB)
			check = ADAU_IN2LCTL;
		else if (chip->in_chan_mask & CAP_INPC)
			check = ADAU_IN3LCTL;
		else if (chip->in_chan_mask & CAP_INPD)
			check = ADAU_AUXLCTL;
		else
			continue;

		vol = clamp_t(int, ucontrol->value.integer.value[i], 0, 31);
		if (cache[check] != vol) {
			change = 1;
			snd_soc_write(codec, check, vol);
		}
	}

	return change;
}

static int adau1373_play_mute_info(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = CHANNELS_OUTPUT;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int adau1373_play_mute_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u8 reg = snd_soc_read(codec, ADAU_PWDCTL2);
	int curr = (reg & DAC_MUTE_MASK) >> 4;
	int i;

	for (i = 0; i < CHANNELS_OUTPUT; ++i)
		ucontrol->value.integer.value[i] =
			(curr & (1 << i)) ? 1 : 0;

	return 0;
}

static int adau1373_play_mute_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int mute = 0;
	int i;
	u8 reg = snd_soc_read(codec, ADAU_PWDCTL2);
	int curr = (reg & DAC_MUTE_MASK) >> 4;

	for (i = 0; i < CHANNELS_OUTPUT; ++i)
		if (ucontrol->value.integer.value[i])
			mute |= (1 << i);

	if (curr != mute) {
		snd_soc_write(codec, ADAU_PWDCTL2, (reg & ~DAC_MUTE_MASK) | mute << 4);
		return 1;
	}

	return 0;
}

static int adau1373_cap_mute_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = CHANNELS_INPUT;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int adau1373_cap_mute_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u8 reg = snd_soc_read(codec, ADAU_PWDCTL1);
	int curr = (reg & ADC_MUTE_MASK) >> 6;
	int i;

	for (i = 0; i < CHANNELS_INPUT; ++i)
		ucontrol->value.integer.value[i] =
			(curr & (1 << i)) ? 1 : 0;

	return 0;
}

static int adau1373_cap_mute_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int mute = 0;
	int i;
	u8 reg = snd_soc_read(codec, ADAU_PWDCTL1);
	int curr = (reg & ADC_MUTE_MASK) >> 6;

	for (i = 0; i < CHANNELS_INPUT; ++i)
		if (ucontrol->value.integer.value[i])
			mute |= (1 << i);

	if (curr != mute) {
		snd_soc_write(codec, ADAU_PWDCTL1,
			(reg & ~ADC_MUTE_MASK) | mute << 6);
		return 1;
	}

	return 0;
}

#define CAPTURE_SOURCE_NUMBER 5

static int adau1373_mux_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	static char *texts[CAPTURE_SOURCE_NUMBER] = {
		"INPA", "INPB", "INPC", "INPD", "DMIC",
	};

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = CAPTURE_SOURCE_NUMBER;
	if (uinfo->value.enumerated.item >= CAPTURE_SOURCE_NUMBER)
		uinfo->value.enumerated.item = CAPTURE_SOURCE_NUMBER - 1;
	strcpy(uinfo->value.enumerated.name, texts[uinfo->value.enumerated.item]);

	return 0;
}

static int adau1373_mux_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1373_priv *chip = snd_soc_codec_get_drvdata(codec);

	if (chip->in_chan_mask & CAP_INPA)
		ucontrol->value.integer.value[0] = 0;
	else if (chip->in_chan_mask & CAP_INPB)
		ucontrol->value.integer.value[0] = 1;
	else if (chip->in_chan_mask & CAP_INPC)
		ucontrol->value.integer.value[0] = 2;
	else if (chip->in_chan_mask & CAP_INPD)
		ucontrol->value.integer.value[0] = 3;
	else if (chip->in_chan_mask & CAP_DMIC)
		ucontrol->value.integer.value[0] = 4;

	return 0;
}

static int adau1373_mux_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1373_priv *chip = snd_soc_codec_get_drvdata(codec);
	u8 reg, *cache = codec->reg_cache;

	reg = snd_soc_read(codec, ADAU_PWDCTL1);
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		chip->in_chan_mask = CAP_INPA;
		snd_soc_write(codec, ADAU_DMICCTL, 0x00);
		snd_soc_write(codec, ADAU_LADCMIX, INPA_EN);
		snd_soc_write(codec, ADAU_RADCMIX, INPA_EN);
		snd_soc_write(codec, ADAU_IN1LCTL, cache[ADAU_IN1LCTL]);
		snd_soc_write(codec, ADAU_IN1RCTL, cache[ADAU_IN1RCTL]);
		/* Disable other ports */
		snd_soc_write(codec, ADAU_PWDCTL1,
			reg & ~(AIN2PWR | AIN3PWR | AIN4PWR));
		snd_soc_write(codec, ADAU_PWDCTL1, reg | AIN1PWR);
		break;
	case 1:
		chip->in_chan_mask = CAP_INPB;
		snd_soc_write(codec, ADAU_DMICCTL, 0x00);
		snd_soc_write(codec, ADAU_LADCMIX, INPB_EN);
		snd_soc_write(codec, ADAU_RADCMIX, INPB_EN);
		snd_soc_write(codec, ADAU_IN2LCTL, cache[ADAU_IN2LCTL]);
		snd_soc_write(codec, ADAU_IN2RCTL, cache[ADAU_IN2RCTL]);
		snd_soc_write(codec, ADAU_PWDCTL1,
			reg & ~(AIN1PWR | AIN3PWR | AIN4PWR));
		snd_soc_write(codec, ADAU_PWDCTL1, reg | AIN2PWR);
		break;
	case 2:
		chip->in_chan_mask = CAP_INPC;
		snd_soc_write(codec, ADAU_DMICCTL, 0x00);
		snd_soc_write(codec, ADAU_LADCMIX, INPC_EN);
		snd_soc_write(codec, ADAU_RADCMIX, INPC_EN);
		snd_soc_write(codec, ADAU_IN3LCTL, cache[ADAU_IN3LCTL]);
		snd_soc_write(codec, ADAU_IN3RCTL, cache[ADAU_IN3RCTL]);
		snd_soc_write(codec, ADAU_PWDCTL1,
			reg & ~(AIN1PWR | AIN2PWR | AIN4PWR));
		snd_soc_write(codec, ADAU_PWDCTL1, reg | AIN3PWR);
		break;
	case 3:
		chip->in_chan_mask = CAP_INPD;
		snd_soc_write(codec, ADAU_DMICCTL, 0x00);
		snd_soc_write(codec, ADAU_LADCMIX, INPD_EN);
		snd_soc_write(codec, ADAU_RADCMIX, INPD_EN);
		snd_soc_write(codec, ADAU_AUXLCTL, cache[ADAU_AUXLCTL]);
		snd_soc_write(codec, ADAU_AUXRCTL, cache[ADAU_AUXRCTL]);
		snd_soc_write(codec, ADAU_PWDCTL1,
			reg & ~(AIN1PWR | AIN2PWR | AIN3PWR));
		snd_soc_write(codec, ADAU_PWDCTL1, reg | AIN4PWR);
		break;
	case 4:
		chip->in_chan_mask = CAP_DMIC;
		snd_soc_write(codec, ADAU_DMICCTL, 0x01);
		break;
	}

	return 1;
}

#define OUTPUT_NUMBER 5
static int adau1373_play_sel_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{
	static char *texts[OUTPUT_NUMBER] = { "Line1", "Line2", "ClassD", "HeadPhone", "Earpiece" };

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = OUTPUT_NUMBER;
	if (uinfo->value.enumerated.item >= OUTPUT_NUMBER)
		uinfo->value.enumerated.item = OUTPUT_NUMBER - 1;
	strcpy(uinfo->value.enumerated.name, texts[uinfo->value.enumerated.item]);

	return 0;
}

static int adau1373_play_sel_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1373_priv *chip = snd_soc_codec_get_drvdata(codec);

	if (chip->out_chan_mask & PB_LINE1)
		ucontrol->value.enumerated.item[0] = 0;
	if (chip->out_chan_mask & PB_LINE1)
		ucontrol->value.enumerated.item[0] = 1;
	if (chip->out_chan_mask & PB_SPK)
		ucontrol->value.enumerated.item[0] = 2;
	if (chip->out_chan_mask & PB_EARP)
		ucontrol->value.enumerated.item[0] = 3;
	if (chip->out_chan_mask & PB_HP)
		ucontrol->value.enumerated.item[0] = 4;

	return 0;
}

static int adau1373_play_sel_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1373_priv *chip = snd_soc_codec_get_drvdata(codec);
	u8 *cache = codec->reg_cache;
	u8 reg0, reg1;

	if (ucontrol->value.enumerated.item[0] >= OUTPUT_NUMBER)
		return -EINVAL;

	reg0 = snd_soc_read(codec, ADAU_PWDCTL2);
	reg1 = snd_soc_read(codec, ADAU_PWDCTL3);
	chip->out_chan_mask = 0;
	switch (ucontrol->value.enumerated.item[0]) {
	case 0:
		chip->out_chan_mask = PB_LINE1;
		snd_soc_write(codec, ADAU_LLN1MIX, DAC1_LEFT);
		snd_soc_write(codec, ADAU_RLN1MIX, DAC1_RIGHT);
		snd_soc_write(codec, ADAU_LLN1OPT, cache[ADAU_LLN1OPT]);
		snd_soc_write(codec, ADAU_RLN1OPT, cache[ADAU_RLN1OPT]);
		/* Disable other ports */
		snd_soc_write(codec, ADAU_PWDCTL2, reg0 & ~0x0C);
		snd_soc_write(codec, ADAU_PWDCTL2, reg0 | 0x03);
		snd_soc_write(codec, ADAU_PWDCTL3, reg1 & ~0x1E);
		break;
	case 1:
		chip->out_chan_mask = PB_LINE2;
		snd_soc_write(codec, ADAU_LLN2MIX, DAC1_LEFT);
		snd_soc_write(codec, ADAU_RLN2MIX, DAC1_RIGHT);
		snd_soc_write(codec, ADAU_LLN2OPT, cache[ADAU_LLN2OPT]);
		snd_soc_write(codec, ADAU_RLN2OPT, cache[ADAU_RLN2OPT]);
		snd_soc_write(codec, ADAU_PWDCTL2, reg0 & ~0x03);
		snd_soc_write(codec, ADAU_PWDCTL2, reg0 | 0x0C);
		snd_soc_write(codec, ADAU_PWDCTL3, reg1 & ~0x1E);
		break;
	case 2:
		chip->out_chan_mask = PB_SPK;
		snd_soc_write(codec, ADAU_LCDMIX, DAC1_LEFT);
		snd_soc_write(codec, ADAU_RCDMIX, DAC1_RIGHT);
		snd_soc_write(codec, ADAU_LCDOUTP, cache[ADAU_LCDOUTP]);
		snd_soc_write(codec, ADAU_RCDOUTP, cache[ADAU_RCDOUTP]);
		snd_soc_write(codec, ADAU_PWDCTL2, reg0 & ~0x0F);
		snd_soc_write(codec, ADAU_PWDCTL3, reg1 | 0x0C);
		snd_soc_write(codec, ADAU_PWDCTL3, reg1 & ~0x12);
		break;
	case 3:
		chip->out_chan_mask = PB_EARP;
		snd_soc_write(codec, ADAU_EPMIX, DAC1_LEFT);
		snd_soc_write(codec, ADAU_PWDCTL2, reg0 & ~0x0F);
		snd_soc_write(codec, ADAU_PWDCTL3, reg1 | 0x10);
		snd_soc_write(codec, ADAU_PWDCTL3, reg1 & ~0x0E);
		break;
	case 4:
		chip->out_chan_mask = PB_HP;
		snd_soc_write(codec, ADAU_LHPMIX, DAC1_LEFT);
		snd_soc_write(codec, ADAU_RHPMIX, DAC1_RIGHT);
		snd_soc_write(codec, ADAU_LHPOUTP, cache[ADAU_LHPOUTP]);
		snd_soc_write(codec, ADAU_RHPOUTP, cache[ADAU_RHPOUTP]);
		snd_soc_write(codec, ADAU_PWDCTL2, reg0 & ~0x0F);
		snd_soc_write(codec, ADAU_PWDCTL3, reg1 | 0x02);
		snd_soc_write(codec, ADAU_PWDCTL3, reg1 & ~0x1C);
		break;
	}

	return 1;
}

static const struct snd_kcontrol_new adau1373_snd_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Master Playback Volume",
		.info  = adau1373_volume_info,
		.get   = adau1373_volume_get,
		.put   = adau1373_volume_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Playback Switch",
		.info  = adau1373_play_mute_info,
		.get   = adau1373_play_mute_get,
		.put   = adau1373_play_mute_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Capture Volume",
		.info  = adau1373_cap_volume_info,
		.get   = adau1373_cap_volume_get,
		.put   = adau1373_cap_volume_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Record Switch",
		.info  = adau1373_cap_mute_info,
		.get   = adau1373_cap_mute_get,
		.put   = adau1373_cap_mute_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Input Mux",
		.info  = adau1373_mux_info,
		.get   = adau1373_mux_get,
		.put   = adau1373_mux_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Output Mixer",
		.info  = adau1373_play_sel_info,
		.get   = adau1373_play_sel_get,
		.put  = adau1373_play_sel_put,
	}
};

static const struct snd_soc_dapm_widget adau1373_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC", "Capture", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("LINE OUT"),
	SND_SOC_DAPM_OUTPUT("CLASS D"),
	SND_SOC_DAPM_OUTPUT("HEADPHONE"),
	SND_SOC_DAPM_INPUT("INPA"),
	SND_SOC_DAPM_INPUT("INPB"),
	SND_SOC_DAPM_INPUT("INPC"),
	SND_SOC_DAPM_INPUT("INPD"),
};

static const struct snd_soc_dapm_route audio_conn[] = {
	{ "LINE OUT", "Output Mixer", "DAC" },
	{ "CLASS D", "Output Mixer", "DAC" },
	{ "HEADPHONE", "Output Mixer", "DAC" },
	{ "ADC", "Input Mux", "INPA" },
	{ "ADC", "Input Mux", "INPB" },
	{ "ADC", "Input Mux", "INPC" },
	{ "ADC", "Input Mux", "INPD" },
};

static int adau1373_add_widgets(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	snd_soc_dapm_new_controls(dapm, adau1373_dapm_widgets,
				  ARRAY_SIZE(adau1373_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, audio_conn, ARRAY_SIZE(audio_conn));

	return 0;
}

#define adau1373_reset(c) snd_soc_write(c, ADAU_RESET, 0)

static inline int get_coeff(struct adau1373_priv *adau1373, int rate)
{
	const struct _pll_settings *pll_settings = adau1373->data->pll_settings;
	int i;

	for (i = 0; i < adau1373->data->pll_settings_num; ++i)
		if ((pll_settings + i)->rate == rate && (pll_settings + i)->mclk ==
			adau1373->sysclk)
			return i;

	return i;
}

static void adau1373_set_drc(struct snd_soc_codec *codec, u8 *dsettings)
{
	snd_soc_write(codec, ADAU_DRCCTL1, dsettings[0]);
	snd_soc_write(codec, ADAU_DRCCTL2, dsettings[1]);
	snd_soc_write(codec, ADAU_DRCCTL3, dsettings[2]);
	snd_soc_write(codec, ADAU_DRCCTL4, dsettings[3]);
	snd_soc_write(codec, ADAU_DRCCTL5, dsettings[4]);
	snd_soc_write(codec, ADAU_DRCCTL6, dsettings[5]);
	snd_soc_write(codec, ADAU_DRCCTL7, dsettings[6]);
	snd_soc_write(codec, ADAU_DRCCTL8, dsettings[7]);
	snd_soc_write(codec, ADAU_DRCCTL9, dsettings[8]);
	snd_soc_write(codec, ADAU_DRCCTLA, dsettings[9]);
	snd_soc_write(codec, ADAU_DRCCTLB, dsettings[10]);
	snd_soc_write(codec, ADAU_DRCCTLC, dsettings[11]);
	snd_soc_write(codec, ADAU_DRCCTLD, dsettings[12]);
}

/* Set rate and format */
static int adau1373_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	u8 reg;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct adau1373_priv *adau1373 = snd_soc_codec_get_drvdata(codec);
	u8 *pll_reg = adau1373->adau1373_pll_reg;
	const struct _pll_settings *pll_settings = adau1373->data->pll_settings;
	int i = 0;
	u8 dai_ctl;

	switch (params_rate(params)) {
	case 96000:
	case 48000:
	case 16000:
	case 8000:
		i = get_coeff(adau1373, 48000);
		break;
	case 44100:
	case 22050:
		i = get_coeff(adau1373, 44100);
		break;
	default:
		dev_err(codec->dev, "rate : %d isn't supported\n", params_rate(params));
		break;
	}

	if (i == adau1373->data->pll_settings_num)
		return -EINVAL;

	reg = snd_soc_read(codec, ADAU_CLK1SDIV);
	snd_soc_write(codec, ADAU_CLK1SDIV, reg & ~CLKSDIV_COREN);
	/* Divide PLL output(48k * 1024 or 44.1k * 1024) to get wanted rate */
	switch (params_rate(params)) {
	case 96000:
		snd_soc_write(codec, ADAU_CLK1SDIV, (1 << CLKSDIV_CLKDIV_SHIFT));
		break;
	case 48000:
		snd_soc_write(codec, ADAU_CLK1SDIV, (3 << CLKSDIV_CLKDIV_SHIFT));
		break;
	case 44100:
		snd_soc_write(codec, ADAU_CLK1SDIV, (3 << CLKSDIV_CLKDIV_SHIFT));
		break;
	case 22050:
		snd_soc_write(codec, ADAU_CLK1SDIV, (7 << CLKSDIV_CLKDIV_SHIFT));
		break;
	case 16000:
		snd_soc_write(codec, ADAU_CLK1SDIV, (5 << CLKSDIV_CLKDIV_SHIFT | 1));
		break;
	case 8000:
		snd_soc_write(codec, ADAU_CLK1SDIV, (5 << CLKSDIV_CLKDIV_SHIFT | 3));
		break;
	default:
		dev_err(codec->dev, "rate : %d isn't supported\n", params_rate(params));
		break;
	}

	/* Set PLL */
#if 0
	snd_soc_write(codec, ADAU_PLLACTL1,
		((pll_settings + i)->m & 0xff00) >> 8);
	snd_soc_write(codec, ADAU_PLLACTL2, (pll_settings + i)->m
		& 0xff);
	snd_soc_write(codec, ADAU_PLLACTL3,
		((pll_settings + i)->n & 0xff00) >> 8);
	snd_soc_write(codec, ADAU_PLLACTL4, (pll_settings + i)->n
		& 0xff);
#endif
	pll_reg[0] = ((pll_settings + i)->m & 0xff00) >> 8;
	pll_reg[1] = (pll_settings + i)->m & 0xff;
	pll_reg[2] = ((pll_settings + i)->n & 0xff00) >> 8;
	pll_reg[3] = (pll_settings + i)->n & 0xff;
	pll_reg[4] = (pll_settings + i)->integer << 3 |
		 (pll_settings + i)->input_div << 1 | (pll_settings + i)->type;
	pll_reg[5] =  0x00;

	adau1373_write_reg_block(codec, ADAU_PLLACTL1, 6, pll_reg);

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		dai_ctl = WLA_16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		dai_ctl = WLA_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		dai_ctl = WLA_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		dai_ctl = WLA_32;
		break;
	default:
		dai_ctl = 0;
		break;
	}
	reg = snd_soc_read(codec, ADAU_DAIA);
	snd_soc_write(codec, ADAU_DAIA, reg | dai_ctl);

	return 0;
}

static int adau1373_pcm_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct adau1373_priv *adau1373 = snd_soc_codec_get_drvdata(codec);
	u8 *pll_reg = adau1373->adau1373_pll_reg;
	int counter = 0;
	u8 reg;

	reg = snd_soc_read(codec, ADAU_PWDCTL3);
	snd_soc_write(codec, ADAU_PWDCTL3, reg | WHOLEPWR);

	pll_reg[5] = PLLEN;
	adau1373_write_reg_block(codec, ADAU_PLLACTL1, 6, pll_reg);
	/* Chcek if PLL is locked by polling the lock bit */
	do {
		++counter;
		schedule_timeout_interruptible(msecs_to_jiffies(1));
	} while (((codec->hw_read(codec, ADAU_PLLACTL6)) & PLL_LOCKED) == 0
			&& counter < 20);
	if (counter >= 20) {
		dev_err(codec->dev, "failed to initialize PLL\n");
		return -1;

	}

	reg = snd_soc_read(codec, ADAU_CLK1SDIV);
	snd_soc_write(codec, ADAU_CLK1SDIV, reg | CLKSDIV_COREN);
	udelay(10);
	/* Use DAI A */
	snd_soc_write(codec, ADAU_DAIACTL, DAI_EN);

	return 0;
}

static void adau1373_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	u8 reg;

	/* deactivate */
	if (!codec->active) {
		reg = snd_soc_read(codec, ADAU_CLK1SDIV);
		snd_soc_write(codec, ADAU_DAIACTL, 0x00);
		snd_soc_write(codec, ADAU_CLK1SDIV, reg & ~CLKSDIV_COREN);
		snd_soc_write(codec, ADAU_PLLACTL6, 0x00);
	}
}

static int adau1373_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 reg0, reg1;

	reg0 = snd_soc_read(codec, ADAU_PWDCTL1);
	reg1 = snd_soc_read(codec, ADAU_PWDCTL2);
	if (mute) {
		snd_soc_write(codec, ADAU_PWDCTL1, reg0 & ~(LADCPWR | RADCPWR));
		snd_soc_write(codec, ADAU_PWDCTL2, reg1 & ~(LDAC1PWR | RDAC1PWR));
	} else {
		snd_soc_write(codec, ADAU_PWDCTL1, reg0 | LADCPWR | RADCPWR);
		snd_soc_write(codec, ADAU_PWDCTL2, reg1 | LDAC1PWR | RDAC1PWR);
	}
	return 0;
}

static int adau1373_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
				   unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adau1373_priv *adau1373 = snd_soc_codec_get_drvdata(codec);

	adau1373->sysclk = freq;

	return 0;
}

static int adau1373_set_dai_fmt(struct snd_soc_dai *codec_dai,
				unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 dai_ctl = 0;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		dai_ctl |= MSA;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		dai_ctl |= FORMAT_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		dai_ctl |= FORMAT_RJUST;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		dai_ctl |= FORMAT_LJUST;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		dai_ctl |= FORMAT_DSP;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		dai_ctl |= LRPA_INV | BCLK_INV;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		dai_ctl |= BCLK_INV;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		dai_ctl |= LRPA_INV;
		break;
	default:
		return -EINVAL;
	}

	/* set DAIA */
	snd_soc_write(codec, ADAU_DAIA, dai_ctl);

	return 0;
}

static int adau1373_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level)
{
	u8 reg_pwr, reg_clk;

	reg_pwr = snd_soc_read(codec, ADAU_PWDCTL3);
	reg_clk = snd_soc_read(codec, ADAU_CLK1SDIV);
	switch (level) {
	case SND_SOC_BIAS_ON:
		snd_soc_write(codec, ADAU_PWDCTL3, reg_pwr | WHOLEPWR);
		/* vref/mid, osc on, dac unmute */
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		/* everything off except vref/vmid, */
		snd_soc_write(codec, ADAU_PWDCTL3, reg_pwr & ~WHOLEPWR);
		break;
	case SND_SOC_BIAS_OFF:
		break;
	}
	codec->dapm.bias_level = level;

	return 0;
}

#define adau1373_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
			SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000)

#define adau1373_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops adau1373_dai_ops = {
	.prepare	= adau1373_pcm_prepare,
	.hw_params	= adau1373_hw_params,
	.shutdown	= adau1373_shutdown,
	.digital_mute	= adau1373_mute,
	.set_sysclk	= adau1373_set_dai_sysclk,
	.set_fmt	= adau1373_set_dai_fmt,
};

static struct snd_soc_dai_driver adau1373_dai = {
	.name = "ADAU1373",
	.playback = {
		.stream_name  = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates        = adau1373_RATES,
		.formats      = adau1373_FORMATS,
	},
	.capture = {
		.stream_name  = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates        = adau1373_RATES,
		.formats      = adau1373_FORMATS,
	},
	.ops = &adau1373_dai_ops,
};

static int adau1373_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	adau1373_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int adau1373_resume(struct snd_soc_codec *codec)
{
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ADAU1373_CACHEREGNUM; ++i) {
		data[0] = i;
		data[1] = cache[i];
		codec->hw_write(codec->control_data, data, 2);
	}
	adau1373_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

static int adau1373_init(struct snd_soc_codec *codec)
{
	struct adau1373_priv *adau1373 = snd_soc_codec_get_drvdata(codec);
	u8 reg = 0;

	adau1373_reset(codec);
	/* Default, output: speaker; input:INPA */
	adau1373->out_chan_mask = PB_HP;
	adau1373->in_chan_mask = CAP_INPA;

	/* Capture settings */
#if DIGMIC
	/* Digital microphone A */
	snd_soc_write(codec, ADAU_DMICCTL, DMICAEN);
#else
	snd_soc_write(codec, ADAU_INPMODE, 0x00);
	/* Input volume gain: 0 db */
	snd_soc_write(codec, ADAU_IN1LCTL, 0x0D);
	snd_soc_write(codec, ADAU_IN1RCTL, 0x0D);
	snd_soc_write(codec, ADAU_IN2LCTL, 0x0D);
	snd_soc_write(codec, ADAU_IN2RCTL, 0x0D);
	snd_soc_write(codec, ADAU_IN3LCTL, 0x0D);
	snd_soc_write(codec, ADAU_IN3RCTL, 0x0D);
	snd_soc_write(codec, ADAU_AUXLCTL, 0x0D);
	snd_soc_write(codec, ADAU_AUXRCTL, 0x0D);

	/* AIN1 enabled */
	snd_soc_write(codec, ADAU_LADCMIX, INPA_EN);
	snd_soc_write(codec, ADAU_RADCMIX, INPA_EN);

	snd_soc_write(codec, ADAU_MICCTR1, 0x00);
	snd_soc_write(codec, ADAU_EPCNTRL, 0x0C);
#endif
	/* Playback settings */

	/* Headphone enabled */
	snd_soc_write(codec, ADAU_LHPMIX, DAC1_RIGHT);
	snd_soc_write(codec, ADAU_RHPMIX, DAC1_RIGHT);

	snd_soc_write(codec, ADAU_HPCTRL, POPTIME4M);
	snd_soc_write(codec, ADAU_HPCTRL2, LVLTHR400);
	/* 0db */
	snd_soc_write(codec, ADAU_LHPOUTP, 0x1F);
	snd_soc_write(codec, ADAU_RHPOUTP, 0x1F);

	/* clock souce: CLK1, FS, 64 bits per frame */
	snd_soc_write(codec, ADAU_BCLKDIVA, BPFA_64);

	/* Playback: Channel 0, DAIA --> DAC */
	snd_soc_write(codec, ADAU_DINMIXC0, DIN_AIFAPB);
	snd_soc_write(codec, ADAU_DOPMIXC3, DOUT_CH0_DAC);
	/* Capture: Channel 1, ADC --> DAIA */
	snd_soc_write(codec, ADAU_DINMIXC1, DIN_ADC);
	snd_soc_write(codec, ADAU_DOPMIXC0, DOUT_CH1_REC);

	/* PWR on input port A, MIC1 BIAS, right and left ADCs */
	reg = AIN1PWR | MICB1PWR | RADCPWR | LADCPWR;
	snd_soc_write(codec, ADAU_PWDCTL1, reg);
	/* PWR on right and left 0f DAC1 */
	reg = RDAC1PWR | LDAC1PWR;
	snd_soc_write(codec, ADAU_PWDCTL2, reg);
	reg = WHOLEPWR | HPPWR;
	snd_soc_write(codec, ADAU_PWDCTL3, reg);

	adau1373_set_drc(codec, adau1373->data->drc_settings);
	snd_soc_write(codec, ADAU_FDSPSEL1, 0x03);
	/* Enable playback, capture */
	snd_soc_write(codec, ADAU_DIGEN, PBAEN | RECEN | FDSPEN);
	snd_soc_write(codec, 0x3C, 0x07);
	snd_soc_write(codec, 0x3C, 0x05);

	return 0;

}

static int adau1373_probe(struct snd_soc_codec *codec)
{
	struct adau1373_priv *adau1373 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	ret = snd_soc_codec_set_cache_io(codec, 8, 8, adau1373->control_type);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	ret = adau1373_init(codec);
	if (ret < 0) {
		dev_err(codec->dev, "failed to initialize\n");
		return ret;
	}

	snd_soc_add_controls(codec, adau1373_snd_controls,
			     ARRAY_SIZE(adau1373_snd_controls));
	adau1373_add_widgets(codec);

	return 0;
}

/* remove everything here */
static int adau1373_remove(struct snd_soc_codec *codec)
{
	adau1373_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_adau1373 = {
	.probe   = adau1373_probe,
	.remove  = adau1373_remove,
	.suspend = adau1373_suspend,
	.resume  = adau1373_resume,
	.set_bias_level = adau1373_set_bias_level,
	.reg_cache_size = sizeof(adau1373_reg),
	.reg_word_size = sizeof(u16),
	.reg_cache_default = adau1373_reg,
};

static __devinit int adau1373_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct adau1373_priv *adau1373;
	int ret;

	adau1373 = kzalloc(sizeof(*adau1373), GFP_KERNEL);
	if (adau1373 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, adau1373);
	adau1373->control_type = SND_SOC_I2C;

	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_adau1373, &adau1373_dai, 1);
	if (ret)
		kfree(adau1373);

	return ret;
}

static __devexit int adau1373_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id adau1373_i2c_id[] = {
	{ "adau1373", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adau1373_i2c_id);

/* corgi i2c codec control layer */
static struct i2c_driver adau1373_i2c_driver = {
	.driver = {
		.name = "adau1373",
		.owner = THIS_MODULE,
	},
	.probe    = adau1373_i2c_probe,
	.remove   = __devexit_p(adau1373_i2c_remove),
	.id_table = adau1373_i2c_id,
};

static int __init adau1373_modinit(void)
{
	return i2c_add_driver(&adau1373_i2c_driver);
}
module_init(adau1373_modinit);

static void __exit adau1373_exit(void)
{
	i2c_del_driver(&adau1373_i2c_driver);
}
module_exit(adau1373_exit);

MODULE_DESCRIPTION("ASoC ADAU1373 driver");
MODULE_AUTHOR("Cliff Cai");
MODULE_LICENSE("GPL");

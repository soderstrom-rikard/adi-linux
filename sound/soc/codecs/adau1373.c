/*
 * Driver for ADAU1373 sound codec
 *
 * Copyright 2009 Analog Devices Inc.
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

#include "adau1373.h"



struct snd_soc_codec_device soc_codec_dev_adau1373;
static struct snd_soc_codec *adau1373_codec;
struct adau1373_priv {
	unsigned int sysclk;
	unsigned int out_chan_mask;
	unsigned int in_chan_mask;
	struct adau1373_platform_data *data;
	struct snd_soc_codec codec;
};

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
	struct adau1373_priv *chip = codec->private_data;
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
	struct adau1373_priv *chip = codec->private_data;
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
	struct adau1373_priv *chip = codec->private_data;
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
	struct adau1373_priv *chip = codec->private_data;
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
	struct adau1373_priv *chip = codec->private_data;

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
	struct adau1373_priv *chip = codec->private_data;
	u8 reg, *cache = codec->reg_cache;

	reg = snd_soc_read(codec, ADAU_PWDCTL1);
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		chip->in_chan_mask = CAP_INPA;
		snd_soc_write(codec, ADAU_DMICCTL, 0x00);
		snd_soc_write(codec, ADAU_LADCMIX, AIN1_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_RADCMIX, AIN1_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_IN1LCTL, cache[ADAU_IN1LCTL]);
		snd_soc_write(codec, ADAU_IN1RCTL, cache[ADAU_IN1RCTL]);
		/* Disable other ports */
		snd_soc_write(codec, ADAU_PWDCTL1,
			reg & ~(PWRCTLA_INBPD | PWRCTLA_INCPD | PWRCTLA_INDPD));
		snd_soc_write(codec, ADAU_PWDCTL1, reg | PWRCTLA_INAPD);
		break;
	case 1:
		chip->in_chan_mask = CAP_INPB;
		snd_soc_write(codec, ADAU_DMICCTL, 0x00);
		snd_soc_write(codec, ADAU_LADCMIX, AIN2_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_RADCMIX, AIN2_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_IN2LCTL, cache[ADAU_IN2LCTL]);
		snd_soc_write(codec, ADAU_IN2RCTL, cache[ADAU_IN2RCTL]);
		snd_soc_write(codec, ADAU_PWDCTL1,
			reg & ~(PWRCTLA_INAPD | PWRCTLA_INCPD | PWRCTLA_INDPD));
		snd_soc_write(codec, ADAU_PWDCTL1, reg | PWRCTLA_INBPD);
		break;
	case 2:
		chip->in_chan_mask = CAP_INPC;
		snd_soc_write(codec, ADAU_DMICCTL, 0x00);
		snd_soc_write(codec, ADAU_LADCMIX, AIN3_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_RADCMIX, AIN3_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_IN3LCTL, cache[ADAU_IN3LCTL]);
		snd_soc_write(codec, ADAU_IN3RCTL, cache[ADAU_IN3RCTL]);
		snd_soc_write(codec, ADAU_PWDCTL1,
			reg & ~(PWRCTLA_INAPD | PWRCTLA_INBPD | PWRCTLA_INDPD));
		snd_soc_write(codec, ADAU_PWDCTL1, reg | PWRCTLA_INCPD);
		break;
	case 3:
		chip->in_chan_mask = CAP_INPD;
		snd_soc_write(codec, ADAU_DMICCTL, 0x00);
		snd_soc_write(codec, ADAU_LADCMIX, AIN4_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_RADCMIX, AIN4_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_AUXLCTL, cache[ADAU_AUXLCTL]);
		snd_soc_write(codec, ADAU_AUXRCTL, cache[ADAU_AUXRCTL]);
		snd_soc_write(codec, ADAU_PWDCTL1,
			reg & ~(PWRCTLA_INAPD | PWRCTLA_INBPD | PWRCTLA_INCPD));
		snd_soc_write(codec, ADAU_PWDCTL1, reg | PWRCTLA_INDPD);
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
	struct adau1373_priv *chip = codec->private_data;

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
	struct adau1373_priv *chip = codec->private_data;
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
		snd_soc_write(codec, ADAU_LLN1MIX, LDAC_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_RLN1MIX, RDAC_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_LLN1OPT, cache[ADAU_LLN1OPT]);
		snd_soc_write(codec, ADAU_RLN1OPT, cache[ADAU_RLN1OPT]);
		/* Disable other ports */
		snd_soc_write(codec, ADAU_PWDCTL2, reg0 & ~0x0C);
		snd_soc_write(codec, ADAU_PWDCTL2, reg0 | 0x03);
		snd_soc_write(codec, ADAU_PWDCTL3, reg1 & ~0x1E);
		break;
	case 1:
		chip->out_chan_mask = PB_LINE2;
		snd_soc_write(codec, ADAU_LLN2MIX, LDAC_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_RLN2MIX, RDAC_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_LLN2OPT, cache[ADAU_LLN2OPT]);
		snd_soc_write(codec, ADAU_RLN2OPT, cache[ADAU_RLN2OPT]);
		snd_soc_write(codec, ADAU_PWDCTL2, reg0 & ~0x03);
		snd_soc_write(codec, ADAU_PWDCTL2, reg0 | 0x0C);
		snd_soc_write(codec, ADAU_PWDCTL3, reg1 & ~0x1E);
		break;
	case 2:
		chip->out_chan_mask = PB_SPK;
		snd_soc_write(codec, ADAU_LCDMIX, LDAC_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_RCDMIX, RDAC_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_LCDOUTP, cache[ADAU_LCDOUTP]);
		snd_soc_write(codec, ADAU_RCDOUTP, cache[ADAU_RCDOUTP]);
		snd_soc_write(codec, ADAU_PWDCTL2, reg0 & ~0x0F);
		snd_soc_write(codec, ADAU_PWDCTL3, reg1 | 0x0C);
		snd_soc_write(codec, ADAU_PWDCTL3, reg1 & ~0x12);
		break;
	case 3:
		chip->out_chan_mask = PB_EARP;
		snd_soc_write(codec, ADAU_EPMIX, LDAC_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_PWDCTL2, reg0 & ~0x0F);
		snd_soc_write(codec, ADAU_PWDCTL3, reg1 | 0x10);
		snd_soc_write(codec, ADAU_PWDCTL3, reg1 & ~0x0E);
		break;
	case 4:
		chip->out_chan_mask = PB_HP;
		snd_soc_write(codec, ADAU_LHPMIX, LDAC_SIGNAL_ENA);
		snd_soc_write(codec, ADAU_RHPMIX, RDAC_SIGNAL_ENA);
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
	snd_soc_dapm_new_controls(codec, adau1373_dapm_widgets,
				  ARRAY_SIZE(adau1373_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_conn, ARRAY_SIZE(audio_conn));

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

#if 0
static void adau1373_set_drc(struct snd_soc_codec *codec, u8 *dsettings)
{
	snd_soc_write(codec, ADAU1373_DRCCTL1, dsettings[0]);
	snd_soc_write(codec, ADAU1373_DRCCTL2, dsettings[1]);
	snd_soc_write(codec, ADAU1373_DRCSC1, dsettings[2]);
	snd_soc_write(codec, ADAU1373_DRCSC2, dsettings[3]);
	snd_soc_write(codec, ADAU1373_DRCSC3, dsettings[4]);
	snd_soc_write(codec, ADAU1373_DRCGS1, dsettings[5]);
	snd_soc_write(codec, ADAU1373_DRCGS2, dsettings[6]);
	snd_soc_write(codec, ADAU1373_DRCGS3, dsettings[7]);

}
#endif
/* Set rate and format */
static int adau1373_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	u8 reg;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct adau1373_priv *adau1373 = codec->private_data;
	const struct _pll_settings *pll_settings = adau1373->data->pll_settings;
	int i = 0;
	u8 dai_ctl;

	i = get_coeff(adau1373, params_rate(params));
	if (i == adau1373->data->pll_settings_num)
		return -EINVAL;
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
	reg = snd_soc_read(codec, ADAU_CLK1SDIV);
	snd_soc_write(codec, ADAU_CLK1SDIV, reg & ~CLKSDIV_COREN);
	/* Set PLL */
	snd_soc_write(codec, ADAU_PLLACTL6, 0x00);

	snd_soc_write(codec, ADAU_PLLACTL1,
		((pll_settings + i)->m & 0xff00) >> 8);
	snd_soc_write(codec, ADAU_PLLACTL2, (pll_settings + i)->m
		& 0xff);
	snd_soc_write(codec, ADAU_PLLACTL3,
		((pll_settings + i)->n & 0xff00) >> 8);
	snd_soc_write(codec, ADAU_PLLACTL4, (pll_settings + i)->n
		& 0xff);
	snd_soc_write(codec, ADAU_PLLACTL5, (pll_settings + i)->integer << 3 |
		 (pll_settings + i)->input_div << 1 | (pll_settings + i)->type);

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		dai_ctl = DAICTL_WLEN16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		dai_ctl = DAICTL_WLEN20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		dai_ctl = DAICTL_WLEN24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		dai_ctl = DAICTL_WLEN32;
		break;
	default:
		dai_ctl = 0;
		break;
	}
	reg = snd_soc_read(codec, ADAU_DAIA);
	snd_soc_write(codec, ADAU_DAIACTL, reg | dai_ctl);
	snd_soc_write(codec, ADAU_PLLACTL, 0x00);
	snd_soc_write(codec, ADAU_PLLACTL6, 0x01);

	return 0;
}

static int adau1373_pcm_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	int counter = 0;
	u8 reg;

	/* Chcek if PLL is locked by polling the lock bit */
	do {
		++counter;
		schedule_timeout_interruptible(msecs_to_jiffies(1));
	} while (((codec->hw_read(codec, ADAU_PLLACTL6)) & 0x04) == 0
			&& counter < 20);
	if (counter >= 20) {
		dev_err(codec->dev, "failed to initialize PLL\n");
		return -1;

	}
	/* Use DAI A */
	snd_soc_write(codec, ADAU_DAIACTL, 0x01);
	snd_soc_write(codec, ADAU_SRCARTA, 0x90);
	snd_soc_write(codec, ADAU_SRCARTB, 0x00);
	snd_soc_write(codec, ADAU_DEEMPCTL, 0x01);
	udelay(10);

	reg = snd_soc_read(codec, ADAU_CLK1SDIV);
	snd_soc_write(codec, ADAU_CLK1SDIV, reg | CLKSDIV_COREN);

	return 0;
}

static void adau1373_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	u8 reg;

	reg = snd_soc_read(codec, ADAU_CLK1SDIV);
	/* deactivate */
	if (!codec->active)
		snd_soc_write(codec, ADAU_CLK1SDIV, reg & ~CLKSDIV_COREN);
	snd_soc_write(codec, ADAU_PLLACTL6, 0x0);
	snd_soc_write(codec, ADAU_DAIACTL, 0x00);
}

static int adau1373_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 reg0, reg1;

	reg0 = snd_soc_read(codec, ADAU_PWDCTL1);
	reg1 = snd_soc_read(codec, ADAU_PWDCTL2);
	if (mute) {
		snd_soc_write(codec, ADAU_PWDCTL1, reg0 & ~0xc0);
		snd_soc_write(codec, ADAU_PWDCTL2, reg1 & ~0x30);
	} else {
		snd_soc_write(codec, ADAU_PWDCTL1, reg0 | 0xc0);
		snd_soc_write(codec, ADAU_PWDCTL2, reg1 | 0x30);
	}
	return 0;
}

static int adau1373_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
				   unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adau1373_priv *adau1373 = codec->private_data;

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
		dai_ctl |= 0x40;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		dai_ctl |= 0x02;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		dai_ctl |= 0x00;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		dai_ctl |= 0x01;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		dai_ctl |= 0x03;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		dai_ctl |= 0x90;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		dai_ctl |= 0x80;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		dai_ctl |= 0x10;
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
		snd_soc_write(codec, ADAU_PWDCTL3, reg_pwr | 0x01);
		snd_soc_write(codec, ADAU_CLK1SDIV, reg_clk | CLKSDIV_COREN);
		/* vref/mid, osc on, dac unmute */
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		/* everything off except vref/vmid, */
		snd_soc_write(codec, ADAU_CLK1SDIV, reg_clk & ~CLKSDIV_COREN);
		break;
	case SND_SOC_BIAS_OFF:
		snd_soc_write(codec, ADAU_PWDCTL3, reg_pwr & ~0x01);
		snd_soc_write(codec, ADAU_CLK1SDIV, reg_clk & ~CLKSDIV_COREN);
		break;
	}
	codec->bias_level = level;

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

struct snd_soc_dai adau1373_dai = {
	.name = "ADAU1373",
	.playback = {
		.stream_name  = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates        = adau1373_RATES,
		.formats      = adau1373_FORMATS,
	},
	.capture = {
		.stream_name  = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates        = adau1373_RATES,
		.formats      = adau1373_FORMATS,
	},
	.ops = &adau1373_dai_ops,
};
EXPORT_SYMBOL_GPL(adau1373_dai);

static int adau1373_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	adau1373_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int adau1373_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
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
	adau1373_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}

/*
 * initialise the adau1373 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int adau1373_register(struct adau1373_priv *adau1373, enum snd_soc_control_type control)
{
	struct snd_soc_codec *codec = &adau1373->codec;
	int ret = 0;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->name = "adau1373";
	codec->owner = THIS_MODULE;
	codec->set_bias_level = adau1373_set_bias_level;
	codec->dai = &adau1373_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ADAU1373_CACHEREGNUM;
	codec->reg_cache = kzalloc(ADAU1373_CACHEREGNUM, GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, control);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		return ret;
	}

	ret = snd_soc_register_dai(&adau1373_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
		snd_soc_unregister_codec(codec);
		return ret;
	}

	return ret;
}

static void adau1373_unregister(struct adau1373_priv *adau1373)
{
	struct snd_soc_codec *codec = &adau1373->codec;

	adau1373_set_bias_level(codec, SND_SOC_BIAS_OFF);
	kfree(codec->reg_cache);
	snd_soc_unregister_dai(&adau1373_dai);
	snd_soc_unregister_codec(codec);
	kfree(adau1373);
	adau1373_codec = NULL;
}

static int adau1373_init(struct adau1373_priv *adau1373)
{

	struct snd_soc_codec *codec = &adau1373->codec;
	u8 reg = 0;

	adau1373_reset(codec);
	/* Default, output: speaker; input:INPA */
	adau1373->out_chan_mask = PB_HP;
	adau1373->in_chan_mask = CAP_INPA;

	/* Capture settings */
#ifdef DIGMIC
	/* Digital microphone A */
	snd_soc_write(codec, ADAU_DMICCTL, 0x01);
#else

	snd_soc_write(codec, ADAU1373_INPUTMODE, 0x00);
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
	snd_soc_write(codec, ADAU_LADCMIX, 0x01);
	snd_soc_write(codec, ADAU_RADCMIX, 0x01);

	snd_soc_write(codec, ADAU_MICCTR1, 0x00);
	snd_soc_write(codec, ADAU_EPCNTRL, 0x0C);
#endif
	/* Playback settings*/

	/* Headphone enabled */
	snd_soc_write(codec, ADAU_LHPMIX, 0x10);
	snd_soc_write(codec, ADAU_RHPMIX, 0x20);

	snd_soc_write(codec, ADAU_HPCTRL, 0x10);
	snd_soc_write(codec, ADAU_HPCTRL2, 0x20);
	/* 0db */
	snd_soc_write(codec, ADAU_RCDOUTP, 0x1F);
	snd_soc_write(codec, ADAU_LCDOUTP, 0x1F);

	/* clock souce: PLL1, FS, 64 bits per frame */
	snd_soc_write(codec, ADAU_BCLKDIVA, 0x1A);



	/* PWR on input port A, MIC1 BIAS, right and left ADCs */
	reg = 0xB1;
	snd_soc_write(codec, ADAU_PWDCTL1, reg);
	/* PWR on right and left DACs */
	reg = 0x30;
	snd_soc_write(codec, ADAU_PWDCTL2, reg);
	reg = 0x03;
	snd_soc_write(codec, ADAU_PWDCTL3, reg);
#if 0
	/* Increase the driven ability of DAIA, maybe not necessary in real use */
	snd_soc_write(codec, ADAU1373_PAD_CTL, PADCTL_DAIA);

	/* Enable Dynamic range control */
	adau1373_set_drc(codec, adau1373->data->drc_settings);
	snd_soc_write(codec, ADAU1373_DRCMODE,
		DRCMODE_RIGHT_ENA | DRCMODE_LEFT_ENA | DRCMODE_NGEN);
	/* Playback signal input */
	snd_soc_write(codec, ADAU1373_DSPMODE, DSPMODE_PLAYBACK_ENA);
#endif
	/* Enable playback, capture */
	snd_soc_write(codec, ADAU_DIGEN, 0x0B);

	return 0;

}

static int adau1373_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct adau1373_priv *adau1373;
	int ret = 0;

	socdev->card->codec = adau1373_codec;
	codec = adau1373_codec;
	adau1373 = codec->private_data;
	adau1373->data = pdev->dev.platform_data;

	ret = adau1373_init(adau1373);
	if (ret < 0)
		dev_err(codec->dev, "failed to initialize\n");
	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms: %d\n", ret);
		goto pcm_err;
	}

	snd_soc_add_controls(codec, adau1373_snd_controls,
			     ARRAY_SIZE(adau1373_snd_controls));
	adau1373_add_widgets(codec);
pcm_err:

	return ret;
}

/* remove everything here */
static int adau1373_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_adau1373 = {
	.probe   = adau1373_probe,
	.remove  = adau1373_remove,
	.suspend = adau1373_suspend,
	.resume  = adau1373_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_adau1373);

static __devinit int adau1373_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct adau1373_priv *adau1373;
	struct snd_soc_codec *codec;
	int ret = 0;

	adau1373 = kzalloc(sizeof(struct adau1373_priv), GFP_KERNEL);
	if (adau1373 == NULL)
		return -ENOMEM;
	codec = &adau1373->codec;
	codec->private_data = adau1373;
	codec->hw_write = (hw_write_t)i2c_master_send;

	i2c_set_clientdata(i2c, adau1373);
	codec->control_data = i2c;

	codec->dev = &i2c->dev;
	adau1373_codec = codec;
	ret = adau1373_register(adau1373, SND_SOC_I2C);
	if (ret < 0)
		dev_err(&i2c->dev, "failed to initialize\n");

	return ret;
}

static __devexit int adau1373_i2c_remove(struct i2c_client *client)
{
	struct adau1373_priv *adau1373 = i2c_get_clientdata(client);
	adau1373_unregister(adau1373);
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
	int ret;

	ret = i2c_add_driver(&adau1373_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register adau1373 I2C driver: %d\n",
		       ret);
	}

	return ret;
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

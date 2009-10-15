/*
 * Driver for ADAU1371 sound codec
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

#include "adau1371.h"

struct snd_soc_codec_device soc_codec_dev_adau1371;
static struct snd_soc_codec *adau1371_codec;
struct adau1371_priv {
	unsigned int sysclk;
	unsigned int out_chan_mask;
	unsigned int in_chan_mask;
	struct adau1371_platform_data *data;
	struct snd_soc_codec codec;
};

/*
 * read adau1371 register cache
 */
static inline unsigned int adau1371_read_reg_cache(struct snd_soc_codec *codec,
						   unsigned int reg)
{
	u8 *cache = codec->reg_cache;
	if (reg == ADAU1371_RESET)
		return 0;
	else if (reg > ADAU1371_CACHEREGNUM)
		return -1;
	else
		return cache[reg];
}

/*
 * write adau1371 register cache
 */
static inline void adau1371_write_reg_cache(struct snd_soc_codec *codec,
					    unsigned int reg, u8 value)
{
	u8 *cache = codec->reg_cache;
	if (reg >= ADAU1371_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * write to the adau1371 register space
 */
static int adau1371_write(struct snd_soc_codec *codec, unsigned int reg,
			  unsigned int value)
{
	u8 data[2] = { reg, value, };

	adau1371_write_reg_cache(codec, reg, value);
	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;
	else {
		dev_err(codec->dev, "%s: writing to %#x failed\n", codec->name, reg);
		return -EIO;
	}
}

static int adau1371_volume_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = CHANNELS_OUTPUT;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 31;
	return 0;
}

static int adau1371_volume_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	int i;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1371_priv *chip = codec->private_data;
	u8 *cache = codec->reg_cache;

	for (i = 0; i < CHANNELS_OUTPUT; ++i) {
		if (chip->out_chan_mask & PB_LINE)
			ucontrol->value.integer.value[i] =
				cache[ADAU1371_LLINEVOL + i];
		else if (chip->out_chan_mask & PB_CD)
			ucontrol->value.integer.value[i] =
				cache[ADAU1371_LCDVOL + i];
		else if (chip->out_chan_mask & PB_HP)
			ucontrol->value.integer.value[i] =
				cache[ADAU1371_LHPVOL + i];
	}

	return 0;
}

static int adau1371_volume_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	int i;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1371_priv *chip = codec->private_data;
	int change = 0;
	u8 *cache = codec->reg_cache;

	for (i = 0; i < CHANNELS_OUTPUT; ++i) {
		int vol = clamp_t(int, ucontrol->value.integer.value[i], 0, 31);

		if (chip->out_chan_mask & PB_LINE) {
			if (cache[ADAU1371_LLINEVOL + i] != vol) {
				change = 1;
				adau1371_write(codec, ADAU1371_LLINEVOL + i, vol);
			}
		} else if (chip->out_chan_mask & PB_CD) {
			if (cache[ADAU1371_LCDVOL + i] != vol) {
				change = 1;
				adau1371_write(codec, ADAU1371_LCDVOL + i, vol);
			}
		} else if (chip->out_chan_mask & PB_HP) {
			if (cache[ADAU1371_LHPVOL + i] != vol) {
				change = 1;
				adau1371_write(codec, ADAU1371_LHPVOL + i, vol);
			}
		}
	}

	return change;
}

static int adau1371_cap_volume_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = CHANNELS_INPUT;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 31;
	return 0;
}

static int adau1371_cap_volume_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int i;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1371_priv *chip = codec->private_data;
	u8 *cache = codec->reg_cache;

	for (i = 0; i < CHANNELS_INPUT; ++i) {
		if (chip->in_chan_mask & CAP_INPA)
			ucontrol->value.integer.value[i] =
				cache[ADAU1371_INALVOL + i];
		else if (chip->in_chan_mask & CAP_INPB)
			ucontrol->value.integer.value[i] =
				cache[ADAU1371_INBLVOL + i];
		else if (chip->in_chan_mask & CAP_INPC)
			ucontrol->value.integer.value[i] =
				cache[ADAU1371_INCLVOL + i];
		else if (chip->in_chan_mask & CAP_INPD)
			ucontrol->value.integer.value[i] =
				cache[ADAU1371_INDLVOL + i];
	}

	return 0;
}

static int adau1371_cap_volume_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int i, check, vol;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1371_priv *chip = codec->private_data;
	int change = 0;
	u8 *cache = codec->reg_cache;

	for (i = 0; i < CHANNELS_INPUT; ++i) {
		if (chip->in_chan_mask & CAP_INPA)
			check = ADAU1371_INALVOL;
		else if (chip->in_chan_mask & CAP_INPB)
			check = ADAU1371_INBLVOL;
		else if (chip->in_chan_mask & CAP_INPC)
			check = ADAU1371_INCLVOL;
		else if (chip->in_chan_mask & CAP_INPD)
			check = ADAU1371_INDLVOL;
		else
			continue;

		vol = clamp_t(int, ucontrol->value.integer.value[i], 0, 31);
		if (cache[check] != vol) {
			change = 1;
			adau1371_write(codec, check, vol);
		}
	}

	return change;
}

static int adau1371_play_mute_info(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = CHANNELS_OUTPUT;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int adau1371_play_mute_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u8 reg = adau1371_read_reg_cache(codec, ADAU1371_PWRCTLA);
	int curr = (reg & DAC_MUTE_MASK) >> 4;
	int i;

	for (i = 0; i < CHANNELS_OUTPUT; ++i)
		ucontrol->value.integer.value[i] =
			(curr & (1 << i)) ? 1 : 0;

	return 0;
}

static int adau1371_play_mute_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int mute = 0;
	int i;
	u8 reg = adau1371_read_reg_cache(codec, ADAU1371_PWRCTLB);
	int curr = (reg & DAC_MUTE_MASK) >> 4;

	for (i = 0; i < CHANNELS_OUTPUT; ++i)
		if (ucontrol->value.integer.value[i])
			mute |= (1 << i);

	if (curr != mute) {
		adau1371_write(codec, ADAU1371_PWRCTLB, (reg & ~DAC_MUTE_MASK) | mute << 4);
		return 1;
	}

	return 0;
}

static int adau1371_cap_mute_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = CHANNELS_INPUT;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int adau1371_cap_mute_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u8 reg = adau1371_read_reg_cache(codec, ADAU1371_PWRCTLA);
	int curr = (reg & ADC_MUTE_MASK) >> 6;
	int i;

	for (i = 0; i < CHANNELS_INPUT; ++i)
		ucontrol->value.integer.value[i] =
			(curr & (1 << i)) ? 1 : 0;

	return 0;
}

static int adau1371_cap_mute_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int mute = 0;
	int i;
	u8 reg = adau1371_read_reg_cache(codec, ADAU1371_PWRCTLA);
	int curr = (reg & ADC_MUTE_MASK) >> 6;

	for (i = 0; i < CHANNELS_INPUT; ++i)
		if (ucontrol->value.integer.value[i])
			mute |= (1 << i);

	if (curr != mute) {
		adau1371_write(codec, ADAU1371_PWRCTLA,
			(reg & ~ADC_MUTE_MASK) | mute << 6);
		return 1;
	}

	return 0;
}

#define CAPTURE_SOURCE_NUMBER 4

static int adau1371_mux_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	static char *texts[CAPTURE_SOURCE_NUMBER] = {
		"INPA", "INPB", "INPC", "INPD",
	};

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = CAPTURE_SOURCE_NUMBER;
	if (uinfo->value.enumerated.item >= CAPTURE_SOURCE_NUMBER)
		uinfo->value.enumerated.item = CAPTURE_SOURCE_NUMBER - 1;
	strcpy(uinfo->value.enumerated.name, texts[uinfo->value.enumerated.item]);

	return 0;
}

static int adau1371_mux_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1371_priv *chip = codec->private_data;

	if (chip->in_chan_mask & CAP_INPA)
		ucontrol->value.integer.value[0] = 0;
	else if (chip->in_chan_mask & CAP_INPB)
		ucontrol->value.integer.value[0] = 1;
	else if (chip->in_chan_mask & CAP_INPC)
		ucontrol->value.integer.value[0] = 2;
	else if (chip->in_chan_mask & CAP_INPD)
		ucontrol->value.integer.value[0] = 3;

	return 0;
}

static int adau1371_mux_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1371_priv *chip = codec->private_data;
	u8 reg, *cache = codec->reg_cache;

	reg = adau1371_read_reg_cache(codec, ADAU1371_PWRCTLA);
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		chip->in_chan_mask = CAP_INPA;
		adau1371_write(codec, ADAU1371_LADCMIX, AIN1_SIGNAL_ENA);
		adau1371_write(codec, ADAU1371_RADCMIX, AIN1_SIGNAL_ENA);
		adau1371_write(codec, ADAU1371_INALVOL, cache[ADAU1371_INALVOL]);
		adau1371_write(codec, ADAU1371_INARVOL, cache[ADAU1371_INARVOL]);
		/* Disable other ports */
		adau1371_write(codec, ADAU1371_PWRCTLA,
			reg & ~(PWRCTLA_INBPD | PWRCTLA_INCPD | PWRCTLA_INDPD));
		adau1371_write(codec, ADAU1371_PWRCTLA, reg | PWRCTLA_INAPD);
		break;
	case 1:
		chip->in_chan_mask = CAP_INPB;
		adau1371_write(codec, ADAU1371_LADCMIX, AIN2_SIGNAL_ENA);
		adau1371_write(codec, ADAU1371_RADCMIX, AIN2_SIGNAL_ENA);
		adau1371_write(codec, ADAU1371_INBLVOL, cache[ADAU1371_INBLVOL]);
		adau1371_write(codec, ADAU1371_INBRVOL, cache[ADAU1371_INBRVOL]);
		adau1371_write(codec, ADAU1371_PWRCTLA,
			reg & ~(PWRCTLA_INAPD | PWRCTLA_INCPD | PWRCTLA_INDPD));
		adau1371_write(codec, ADAU1371_PWRCTLA, reg | PWRCTLA_INBPD);
		break;
	case 2:
		chip->in_chan_mask = CAP_INPC;
		adau1371_write(codec, ADAU1371_LADCMIX, AIN3_SIGNAL_ENA);
		adau1371_write(codec, ADAU1371_RADCMIX, AIN3_SIGNAL_ENA);
		adau1371_write(codec, ADAU1371_INCLVOL, cache[ADAU1371_INCLVOL]);
		adau1371_write(codec, ADAU1371_INCRVOL, cache[ADAU1371_INCRVOL]);
		adau1371_write(codec, ADAU1371_PWRCTLA,
			reg & ~(PWRCTLA_INAPD | PWRCTLA_INBPD | PWRCTLA_INDPD));
		adau1371_write(codec, ADAU1371_PWRCTLA, reg | PWRCTLA_INCPD);
		break;
	case 3:
		chip->in_chan_mask = CAP_INPD;
		adau1371_write(codec, ADAU1371_LADCMIX, AIN4_SIGNAL_ENA);
		adau1371_write(codec, ADAU1371_RADCMIX, AIN4_SIGNAL_ENA);
		adau1371_write(codec, ADAU1371_INDLVOL, cache[ADAU1371_INDLVOL]);
		adau1371_write(codec, ADAU1371_INDRVOL, cache[ADAU1371_INDRVOL]);
		adau1371_write(codec, ADAU1371_PWRCTLA,
			reg & ~(PWRCTLA_INAPD | PWRCTLA_INBPD | PWRCTLA_INCPD));
		adau1371_write(codec, ADAU1371_PWRCTLA, reg | PWRCTLA_INDPD);
		break;
	}

	return 1;
}

#define OUTPUT_NUMBER 3
static int adau1371_play_sel_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{
	static char *texts[OUTPUT_NUMBER] = { "Line", "ClassD", "HeadPhone" };

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 3;
	if (uinfo->value.enumerated.item >= OUTPUT_NUMBER)
		uinfo->value.enumerated.item = OUTPUT_NUMBER - 1;
	strcpy(uinfo->value.enumerated.name, texts[uinfo->value.enumerated.item]);

	return 0;
}

static int adau1371_play_sel_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1371_priv *chip = codec->private_data;

	if (chip->out_chan_mask & PB_LINE)
		ucontrol->value.enumerated.item[0] = 0;
	if (chip->out_chan_mask & PB_CD)
		ucontrol->value.enumerated.item[0] = 1;
	if (chip->out_chan_mask & PB_HP)
		ucontrol->value.enumerated.item[0] = 2;

	return 0;
}

static int adau1371_play_sel_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1371_priv *chip = codec->private_data;
	u8 *cache = codec->reg_cache;
	u8 reg;

	if (ucontrol->value.enumerated.item[0] >= OUTPUT_NUMBER)
		return -EINVAL;

	reg = adau1371_read_reg_cache(codec, ADAU1371_PWRCTLB);
	chip->out_chan_mask = 0;
	switch (ucontrol->value.enumerated.item[0]) {
	case 0:
		chip->out_chan_mask = PB_LINE;
		adau1371_write(codec, ADAU1371_LLINEMIX, LDAC_SIGNAL_ENA);
		adau1371_write(codec, ADAU1371_RLINEMIX, RDAC_SIGNAL_ENA);
		adau1371_write(codec, ADAU1371_LLINEVOL, cache[ADAU1371_LLINEVOL]);
		adau1371_write(codec, ADAU1371_RLINEVOL, cache[ADAU1371_RLINEVOL]);
		/* Disable other ports */
		adau1371_write(codec, ADAU1371_PWRCTLB,
			reg & ~(PWRCTLB_LCDPD | PWRCTLB_RCDPD | PWRCTLB_HPPD));
		adau1371_write(codec, ADAU1371_PWRCTLB, reg | PWRCTLB_LLNPD | PWRCTLB_RLNPD);
		break;
	case 1:
		chip->out_chan_mask = PB_CD;
		adau1371_write(codec, ADAU1371_LCDMIX, LDAC_SIGNAL_ENA);
		adau1371_write(codec, ADAU1371_RCDMIX, RDAC_SIGNAL_ENA);
		adau1371_write(codec, ADAU1371_LCDVOL, cache[ADAU1371_LCDVOL]);
		adau1371_write(codec, ADAU1371_RCDVOL, cache[ADAU1371_RCDVOL]);
		adau1371_write(codec, ADAU1371_PWRCTLB,
			reg & ~(PWRCTLB_LLNPD | PWRCTLB_RLNPD | PWRCTLB_HPPD));
		adau1371_write(codec, ADAU1371_PWRCTLB, reg | PWRCTLB_LCDPD | PWRCTLB_RCDPD);
		break;
	case 2:
		chip->out_chan_mask = PB_HP;
		adau1371_write(codec, ADAU1371_LHPMIX, LDAC_SIGNAL_ENA);
		adau1371_write(codec, ADAU1371_RHPMIX, RDAC_SIGNAL_ENA);
		adau1371_write(codec, ADAU1371_LHPVOL, cache[ADAU1371_LHPVOL]);
		adau1371_write(codec, ADAU1371_RHPVOL, cache[ADAU1371_RHPVOL]);
		adau1371_write(codec, ADAU1371_PWRCTLB,
			reg & ~(PWRCTLB_LLNPD | PWRCTLB_RLNPD | PWRCTLB_LCDPD | PWRCTLB_RCDPD));
		adau1371_write(codec, ADAU1371_PWRCTLB, reg | PWRCTLB_HPPD);
		break;
	}

	return 1;
}

static const struct snd_kcontrol_new adau1371_snd_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Master Playback Volume",
		.info  = adau1371_volume_info,
		.get   = adau1371_volume_get,
		.put   = adau1371_volume_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Playback Switch",
		.info  = adau1371_play_mute_info,
		.get   = adau1371_play_mute_get,
		.put   = adau1371_play_mute_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Capture Volume",
		.info  = adau1371_cap_volume_info,
		.get   = adau1371_cap_volume_get,
		.put   = adau1371_cap_volume_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Record Switch",
		.info  = adau1371_cap_mute_info,
		.get   = adau1371_cap_mute_get,
		.put   = adau1371_cap_mute_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Input Mux",
		.info  = adau1371_mux_info,
		.get   = adau1371_mux_get,
		.put   = adau1371_mux_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Output Mixer",
		.info  = adau1371_play_sel_info,
		.get   = adau1371_play_sel_get,
		.put  = adau1371_play_sel_put,
	}
};

static const struct snd_soc_dapm_widget adau1371_dapm_widgets[] = {
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

static int adau1371_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, adau1371_dapm_widgets,
				  ARRAY_SIZE(adau1371_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_conn, ARRAY_SIZE(audio_conn));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

#define adau1371_reset(c) adau1371_write(c, ADAU1371_RESET, 0)

static inline int get_coeff(struct adau1371_priv *adau1371, int rate)
{
	const struct _pll_settings *pll_settings = adau1371->data->pll_settings;
	int i;

	for (i = 0; i < adau1371->data->pll_settings_num; ++i)
		if ((pll_settings + i)->rate == rate && (pll_settings + i)->mclk ==
			adau1371->sysclk)
			return i;

	return i;
}

static void adau1371_set_drc(struct snd_soc_codec *codec, u8 *dsettings)
{
	adau1371_write(codec, ADAU1371_DRCCTL1, dsettings[0]);
	adau1371_write(codec, ADAU1371_DRCCTL2, dsettings[1]);
	adau1371_write(codec, ADAU1371_DRCSC1, dsettings[2]);
	adau1371_write(codec, ADAU1371_DRCSC2, dsettings[3]);
	adau1371_write(codec, ADAU1371_DRCSC3, dsettings[4]);
	adau1371_write(codec, ADAU1371_DRCGS1, dsettings[5]);
	adau1371_write(codec, ADAU1371_DRCGS2, dsettings[6]);
	adau1371_write(codec, ADAU1371_DRCGS3, dsettings[7]);

}

/* Set rate and format */
static int adau1371_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	u8 reg;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct adau1371_priv *adau1371 = codec->private_data;
	const struct _pll_settings *pll_settings = adau1371->data->pll_settings;
	int i = 0;
	u8 dai_ctl;

	i = get_coeff(adau1371, params_rate(params));
	if (i == adau1371->data->pll_settings_num)
		return -EINVAL;
	/* Divide PLL output(48k * 1024 or 44.1k * 1024) to get wanted rate */
	switch (params_rate(params)) {
	case 96000:
		adau1371_write(codec, ADAU1371_CLKSDIV, (1 << CLKSDIV_CLKDIV_SHIFT));
		break;
	case 48000:
		adau1371_write(codec, ADAU1371_CLKSDIV, (3 << CLKSDIV_CLKDIV_SHIFT));
		break;
	case 44100:
		adau1371_write(codec, ADAU1371_CLKSDIV, (3 << CLKSDIV_CLKDIV_SHIFT));
		break;
	case 22050:
		adau1371_write(codec, ADAU1371_CLKSDIV, (7 << CLKSDIV_CLKDIV_SHIFT));
		break;
	case 16000:
		adau1371_write(codec, ADAU1371_CLKSDIV, (5 << CLKSDIV_CLKDIV_SHIFT | 1));
		break;
	case 8000:
		adau1371_write(codec, ADAU1371_CLKSDIV, (5 << CLKSDIV_CLKDIV_SHIFT | 3));
		break;
	default:
		dev_err(codec->dev, "rate : %d isn't supported\n", params_rate(params));
		break;
	}
	reg = adau1371_read_reg_cache(codec, ADAU1371_CLKSDIV);
	adau1371_write(codec, ADAU1371_CLKSDIV, reg & ~CLKSDIV_COREN);
	/* Set PLL */
	adau1371_write(codec, ADAU1371_PLLCTLB, 0x00);
	adau1371_write(codec, ADAU1371_PLLMHI,
		((pll_settings + i)->m & 0xff00) >> 8);
	adau1371_write(codec, ADAU1371_PLLMLOW, (pll_settings + i)->m
		& 0xff);
	adau1371_write(codec, ADAU1371_PLLNHI,
		((pll_settings + i)->n & 0xff00) >> 8);
	adau1371_write(codec, ADAU1371_PLLNLOW, (pll_settings + i)->n
		& 0xff);
	adau1371_write(codec, ADAU1371_PLLCTLA, (pll_settings + i)->integer << 3 |
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
	reg = adau1371_read_reg_cache(codec, ADAU1371_DAIACTL);
	adau1371_write(codec, ADAU1371_DAIACTL, reg | dai_ctl);

	return 0;
}

static int adau1371_pcm_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	u8 reg;

	/* Enable PLL and wait 5 ms for PLL lock */
	adau1371_write(codec, ADAU1371_PLLCTLB, PLLCTLB_PLLEN);
	msleep(5);

	/* Use DAI A */
	adau1371_write(codec, ADAU1371_SRCDAICTL, SRCDAICTL_DAIA_ENA);
	udelay(10);

	reg = adau1371_read_reg_cache(codec, ADAU1371_CLKSDIV);
	adau1371_write(codec, ADAU1371_CLKSDIV, reg | CLKSDIV_COREN);

	return 0;
}

static void adau1371_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	u8 reg;

	reg = adau1371_read_reg_cache(codec, ADAU1371_CLKSDIV);
	/* deactivate */
	if (!codec->active)
		adau1371_write(codec, ADAU1371_CLKSDIV, reg & ~CLKSDIV_COREN);
	adau1371_write(codec, ADAU1371_PLLCTLB, 0x0);
	adau1371_write(codec, ADAU1371_SRCDAICTL, 0x0);
}

static int adau1371_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 reg;

	reg = adau1371_read_reg_cache(codec, ADAU1371_PWRCTLB);
	if (mute)
		adau1371_write(codec, ADAU1371_PWRCTLB, reg & ~DAC_MUTE_MASK);
	else
		adau1371_write(codec, ADAU1371_PWRCTLB, reg | DAC_MUTE_MASK);

	return 0;
}

static int adau1371_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
				   unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adau1371_priv *adau1371 = codec->private_data;

	adau1371->sysclk = freq;

	return 0;
}

static int adau1371_set_dai_fmt(struct snd_soc_dai *codec_dai,
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
		dai_ctl |= DAICTL_FMI2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		dai_ctl |= DAICTL_FMRJUST;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		dai_ctl |= DAICTL_FMLJUST;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		dai_ctl |= DAICTL_FMDSP;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		dai_ctl |= DAICTL_BLKINVA | DAICTL_LRPA;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		dai_ctl |= DAICTL_BLKINVA;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		dai_ctl |= DAICTL_LRPA;
		break;
	default:
		return -EINVAL;
	}

	/* set DAIA */
	adau1371_write(codec, ADAU1371_DAIACTL, dai_ctl);

	return 0;
}

static int adau1371_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level)
{
	u8 reg_pwr, reg_clk;

	reg_pwr = adau1371_read_reg_cache(codec, ADAU1371_PWRCTLB);
	reg_clk = adau1371_read_reg_cache(codec, ADAU1371_CLKSDIV);
	switch (level) {
	case SND_SOC_BIAS_ON:
		adau1371_write(codec, ADAU1371_PWRCTLB, reg_pwr | PWRCTLB_PWDB);
		adau1371_write(codec, ADAU1371_CLKSDIV, reg_clk | CLKSDIV_COREN);
		/* vref/mid, osc on, dac unmute */
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		/* everything off except vref/vmid, */
		adau1371_write(codec, ADAU1371_CLKSDIV, reg_clk & ~CLKSDIV_COREN);
		break;
	case SND_SOC_BIAS_OFF:
		adau1371_write(codec, ADAU1371_PWRCTLB, reg_pwr & ~PWRCTLB_PWDB);
		adau1371_write(codec, ADAU1371_CLKSDIV, reg_clk & ~CLKSDIV_COREN);
		break;
	}
	codec->bias_level = level;

	return 0;
}

#define adau1371_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
			SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000)

#define adau1371_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops adau1371_dai_ops = {
	.prepare	= adau1371_pcm_prepare,
	.hw_params	= adau1371_hw_params,
	.shutdown	= adau1371_shutdown,
	.digital_mute	= adau1371_mute,
	.set_sysclk	= adau1371_set_dai_sysclk,
	.set_fmt	= adau1371_set_dai_fmt,
};

struct snd_soc_dai adau1371_dai = {
	.name = "ADAU1371",
	.playback = {
		.stream_name  = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates        = adau1371_RATES,
		.formats      = adau1371_FORMATS,
	},
	.capture = {
		.stream_name  = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates        = adau1371_RATES,
		.formats      = adau1371_FORMATS,
	},
	.ops = &adau1371_dai_ops,
};
EXPORT_SYMBOL_GPL(adau1371_dai);

static int adau1371_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	adau1371_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int adau1371_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ADAU1371_CACHEREGNUM; ++i) {
		data[0] = i;
		data[1] = cache[i];
		codec->hw_write(codec->control_data, data, 2);
	}
	adau1371_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	adau1371_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}

/*
 * initialise the adau1371 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int adau1371_register(struct adau1371_priv *adau1371)
{
	struct snd_soc_codec *codec = &adau1371->codec;
	int ret = 0;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->name = "adau1371";
	codec->owner = THIS_MODULE;
	codec->read = adau1371_read_reg_cache;
	codec->write = adau1371_write;
	codec->set_bias_level = adau1371_set_bias_level;
	codec->dai = &adau1371_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ADAU1371_CACHEREGNUM;
	codec->reg_cache = kzalloc(ADAU1371_CACHEREGNUM, GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		return ret;
	}

	ret = snd_soc_register_dai(&adau1371_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
		snd_soc_unregister_codec(codec);
		return ret;
	}

	return ret;
}

static void adau1371_unregister(struct adau1371_priv *adau1371)
{
	struct snd_soc_codec *codec = &adau1371->codec;

	adau1371_set_bias_level(codec, SND_SOC_BIAS_OFF);
	kfree(codec->reg_cache);
	snd_soc_unregister_dai(&adau1371_dai);
	snd_soc_unregister_codec(codec);
	kfree(adau1371);
	adau1371_codec = NULL;
}

static int adau1371_init(struct adau1371_priv *adau1371)
{

	struct snd_soc_codec *codec = &adau1371->codec;
	u8 reg = 0;

	adau1371_reset(codec);
	/* By default line out and input A are enabled */
	adau1371->out_chan_mask = PB_LINE;
	adau1371->in_chan_mask = CAP_INPA;

	/* Playback mix settings, line out switched to DACs */
	adau1371_write(codec, ADAU1371_LLINEMIX, LDAC_SIGNAL_ENA);
	adau1371_write(codec, ADAU1371_RLINEMIX, RDAC_SIGNAL_ENA);
	/* Line out volume gain:10 db by default */
	adau1371_write(codec, ADAU1371_LLINEVOL, 0x17);
	adau1371_write(codec, ADAU1371_RLINEVOL, 0x17);
	adau1371_write(codec, ADAU1371_DAIAPBLVOL, 0x80);
	adau1371_write(codec, ADAU1371_DAIAPBRVOL, 0x80);

	/* Capture mix settings, AIN1 switched to ADCs */
	adau1371_write(codec, ADAU1371_LADCMIX, AIN1_SIGNAL_ENA);
	adau1371_write(codec, ADAU1371_RADCMIX, AIN1_SIGNAL_ENA);
	/* Input volume gain:10 db by default */
	adau1371_write(codec, ADAU1371_INPUTMODE, 0x00);
	adau1371_write(codec, ADAU1371_INALVOL, 0x17);
	adau1371_write(codec, ADAU1371_INARVOL, 0x17);
	adau1371_write(codec, ADAU1371_DAIRECLVOL, 0x80);
	adau1371_write(codec, ADAU1371_DAIRECRVOL, 0x80);
	/* Should be set on REVB to enable ADCs */
	adau1371_write(codec, ADAU1371_CODECCTL, 0x10);
	/* 64 bits per frame */
	adau1371_write(codec, ADAU1371_BCLKDIV, BCLKDIV_BPFA64);
	/* PWR on input port A,right and left ADCs */
	reg = PWRCTLA_INAPD | PWRCTLA_MICBPD | PWRCTLA_LADCPD | PWRCTLA_RADCPD;
	adau1371_write(codec, ADAU1371_PWRCTLA, reg);
	/* PWR on line out,right and left DACs and whole chip */
	reg = PWRCTLB_RLNPD | PWRCTLB_LLNPD | PWRCTLB_RDACPD |
		PWRCTLB_LDACPD | PWRCTLB_PWDB;
	adau1371_write(codec, ADAU1371_PWRCTLB, reg);
	/* Increase the driven ability of DAIA, maybe not necessary in real use */
	adau1371_write(codec, ADAU1371_PAD_CTL, PADCTL_DAIA);

	/* Enable Dynamic range control */
	adau1371_set_drc(codec, adau1371->data->drc_settings);
	adau1371_write(codec, ADAU1371_DRCMODE,
		DRCMODE_RIGHT_ENA | DRCMODE_LEFT_ENA | DRCMODE_NGEN);
	/* Playback signal input */
	adau1371_write(codec, ADAU1371_DSPMODE, DSPMODE_PLAYBACK_ENA);

	/* Enable playback, capture and DSP function */
	adau1371_write(codec, ADAU1371_DIGEN, DIGEN_PBEN | DIGEN_RECEN | DIGEN_FDSPEN);

	return 0;

}

static int adau1371_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct adau1371_priv *adau1371;
	int ret = 0;

	socdev->card->codec = adau1371_codec;
	codec = adau1371_codec;
	adau1371 = codec->private_data;
	adau1371->data = pdev->dev.platform_data;

	ret = adau1371_init(adau1371);
	if (ret < 0)
		dev_err(codec->dev, "failed to initialize\n");
	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms: %d\n", ret);
		goto pcm_err;
	}

	snd_soc_add_controls(codec, adau1371_snd_controls,
			     ARRAY_SIZE(adau1371_snd_controls));
	adau1371_add_widgets(codec);
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

/* remove everything here */
static int adau1371_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_adau1371 = {
	.probe   = adau1371_probe,
	.remove  = adau1371_remove,
	.suspend = adau1371_suspend,
	.resume  = adau1371_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_adau1371);

static __devinit int adau1371_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct adau1371_priv *adau1371;
	struct snd_soc_codec *codec;
	int ret = 0;

	adau1371 = kzalloc(sizeof(struct adau1371_priv), GFP_KERNEL);
	if (adau1371 == NULL)
		return -ENOMEM;
	codec = &adau1371->codec;
	codec->private_data = adau1371;
	codec->hw_write = (hw_write_t)i2c_master_send;

	i2c_set_clientdata(i2c, adau1371);
	codec->control_data = i2c;

	codec->dev = &i2c->dev;
	adau1371_codec = codec;
	ret = adau1371_register(adau1371);
	if (ret < 0)
		dev_err(&i2c->dev, "failed to initialize\n");

	return ret;
}

static __devexit int adau1371_i2c_remove(struct i2c_client *client)
{
	struct adau1371_priv *adau1371 = i2c_get_clientdata(client);
	adau1371_unregister(adau1371);
	return 0;
}

static const struct i2c_device_id adau1371_i2c_id[] = {
	{ "adau1371", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adau1371_i2c_id);

/* corgi i2c codec control layer */
static struct i2c_driver adau1371_i2c_driver = {
	.driver = {
		.name = "adau1371",
		.owner = THIS_MODULE,
	},
	.probe    = adau1371_i2c_probe,
	.remove   = __devexit_p(adau1371_i2c_remove),
	.id_table = adau1371_i2c_id,
};

static int __init adau1371_modinit(void)
{
	int ret;

	ret = i2c_add_driver(&adau1371_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register adau1371 I2C driver: %d\n",
		       ret);
	}

	return ret;
}
module_init(adau1371_modinit);

static void __exit adau1371_exit(void)
{
	i2c_del_driver(&adau1371_i2c_driver);
}
module_exit(adau1371_exit);

MODULE_DESCRIPTION("ASoC ADAU1371 driver");
MODULE_AUTHOR("Cliff Cai");
MODULE_LICENSE("GPL");

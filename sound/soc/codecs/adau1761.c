/*
 * Driver for ADAU1761 sound codec
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

#include "adau1761.h"

#define AUDIO_NAME "adau1761"
#define ADAU1761_VERSION "0.20"
#define ADAU1761_FIRMWARE "adau1761.bin"

#define CAP_MIC  1
#define CAP_LINE 2
#define CAPTURE_SOURCE_NUMBER 2
#define ADAU1761_DIG_MIC 0

struct snd_soc_codec_device soc_codec_dev_adau1761;
static struct snd_soc_codec *adau1761_codec;
/* codec private data */
struct adau1761_priv {
	unsigned int sysclk;
	unsigned int in_source;
	unsigned int out_route;
	unsigned int pll_out;
	struct work_struct resume_work;
	struct snd_soc_codec codec;
	int dapm_state_suspend;
	struct platform_device *pdev;
	u8 pll_enable;
	u8 adau1761_pll_reg[6];
	u8 rate_index;
	/* dapm */
	u8 dapm_lineL;
	u8 dapm_lineR;
	u8 dapm_hpL;
	u8 dapm_hpR;
};

/*
 * write register cache
 */
static inline int adau1761_write_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	u8 *cache = codec->reg_cache;

	if (reg < ADAU_FIRSTREG)
		reg = reg + ADAU_FIRSTREG;

	if ((reg < ADAU_FIRSTREG) || (reg > ADAU_LASTREG))
		return -1;

	cache[reg - ADAU_FIRSTREG] = value;

	return 0;
}

/*
 * read a multi-byte ADAU1761 register (6byte pll reg)
 */
static int adau1761_read_reg_block(struct snd_soc_codec *codec,
	unsigned int reg, u8 len)
{
	u8 buf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	u8 addr[2];
	unsigned int i;

	if (reg < ADAU_FIRSTREG)
		reg = reg + ADAU_FIRSTREG;

	if ((reg < ADAU_FIRSTREG) || (reg > ADAU_LASTREG))
		return -EIO;

	addr[0] = (u8)(reg >> 8);
	addr[1] = (u8)(reg & 0xFF);

	/* write the 2byte read address */
	if (codec->hw_write(codec->control_data, addr, 2) != 2) {
		dev_err(codec->dev, "read_reg_byte:address write failed.");
		return -EIO;
	}

	if (i2c_master_recv(codec->control_data, buf, len) != len)
		return -EIO;

	for (i = 0; i < len; i++)
		adau1761_write_reg_cache(codec, reg+i, (unsigned int)buf[i]);

	return 0;
}
/*
 * write a multibyte ADAU1761 register (6byte pll reg)
 */
static int adau1761_write_reg_block(struct snd_soc_codec *codec,
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
 * adau1761 controls
 */
static int adau1761_mux_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1761_priv *adau1761 = codec->private_data;

	if (adau1761->in_source & CAP_MIC)
		ucontrol->value.integer.value[0] = 0x0;
	else
		ucontrol->value.integer.value[0] = 0x1;

	return 0;
}

static int adau1761_mux_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1761_priv *adau1761 = codec->private_data;
	int src = ucontrol->value.integer.value[0];
	u8 regvalue = 0;


	if (src == 0) {/* Select Mic */
		adau1761->in_source = CAP_MIC;
#ifdef ADAU1761_DIG_MIC
		regvalue = (snd_soc_read(codec, ADAU_ADCCTL0) & 0xFB)|0x4;
		snd_soc_write(codec, ADAU_ADCCTL0, regvalue);
#else
		snd_soc_write(codec, ADAU_RECMBIA, RECMBIA_DISABLE);
		regvalue = (snd_soc_read(codec, ADAU_RECVLCL)
				| RECVLC_ENABLE_MASK);
		snd_soc_write(codec, ADAU_RECVLCL, regvalue);
		regvalue = (snd_soc_read(codec, ADAU_RECVLCR)
				| RECVLC_ENABLE_MASK);
		snd_soc_write(codec, ADAU_RECVLCR, regvalue);
		snd_soc_write(codec, ADAU_RECMLC1, RECMLC_MIC_0DB);
		snd_soc_write(codec, ADAU_RECMRC1, RECMLC_MIC_0DB);
#endif
	} else if (src == 1) {/* Select Line */
		adau1761->in_source = CAP_LINE;
#ifdef ADAU1761_DIG_MIC
		regvalue = (snd_soc_read(codec, ADAU_ADCCTL0) & 0xFB);
		snd_soc_write(codec, ADAU_ADCCTL0, regvalue);
#endif
		snd_soc_write(codec, ADAU_RECMBIA, RECMBIA_DISABLE);
		regvalue = (snd_soc_read(codec, ADAU_RECVLCL)
				& RECVLC_DISABLE_MASK);
		snd_soc_write(codec, ADAU_RECVLCL, regvalue);
		regvalue = (snd_soc_read(codec, ADAU_RECVLCR)
				& RECVLC_DISABLE_MASK);
		snd_soc_write(codec, ADAU_RECVLCR, regvalue);
		snd_soc_write(codec, ADAU_RECMLC1, RECMLC_LINE_0DB);
		snd_soc_write(codec, ADAU_RECMRC1, RECMLC_LINE_0DB);
	}

	return 0;
}

static int adau1761_mic_boost_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1761_priv *adau1761 = codec->private_data;

	if (adau1761->in_source & CAP_MIC)
		ucontrol->value.integer.value[0] =
		(RECMLC_MIC_20DB ==
			snd_soc_read(codec, ADAU_RECMLC1));
	else
		ucontrol->value.integer.value[0] = 0x0;

	ucontrol->value.integer.value[1] = ucontrol->value.integer.value[0];

	return 0;
}

static int adau1761_mic_boost_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct adau1761_priv *adau1761 = codec->private_data;
	int val = ucontrol->value.integer.value[0];
	u8 regvalue = 0;

	if (adau1761->in_source & CAP_MIC) {
		regvalue = (val) ? RECMLC_MIC_20DB : RECMLC_MIC_0DB;
		if (snd_soc_read(codec, ADAU_RECMLC1) != regvalue) {
			snd_soc_write(codec, ADAU_RECMLC1, regvalue);
			snd_soc_write(codec, ADAU_RECMRC1, regvalue);
			return 1;
		}
	}

	return 0;
}

static const char *adau1761_input_select[] = {"Mic", "Line"};
static const struct soc_enum adau1761_enums[] = {
	SOC_ENUM_SINGLE(ADAU_RECMLC1-ADAU_FIRSTREG, 0, 2, adau1761_input_select),
};

static const struct snd_kcontrol_new adau1761_snd_controls[] = {
SOC_DOUBLE_R("Master Playback Volume", ADAU_DACCTL1-ADAU_FIRSTREG,
	ADAU_DACCTL2-ADAU_FIRSTREG, 0, 255, 1),
SOC_DOUBLE_R("Capture Volume", ADAU_ADCCTL1-ADAU_FIRSTREG,
	ADAU_ADCCTL2-ADAU_FIRSTREG, 0, 255, 1),
SOC_DOUBLE_R("Capture Switch", ADAU_RECMLC0-ADAU_FIRSTREG,
	ADAU_RECMRC0-ADAU_FIRSTREG, 0, 1, 0),
SOC_ENUM_EXT("Capture Source", adau1761_enums[0],
	adau1761_mux_get, adau1761_mux_put),
SOC_SINGLE_EXT("Mic Boost (+20dB)", ADAU_RECMLC1-ADAU_FIRSTREG, 0, 1, 0,
	adau1761_mic_boost_get, adau1761_mic_boost_put),
SOC_DOUBLE_R("Headphone Playback Volume", ADAU_PLBHPVL-ADAU_FIRSTREG,
	ADAU_PLBHPVR-ADAU_FIRSTREG, 2, 63, 0),
SOC_DOUBLE_R("Line Playback Volume", ADAU_PLBLOVL-ADAU_FIRSTREG,
	ADAU_PLBLOVR-ADAU_FIRSTREG, 2, 63, 0),
};

/*
 * _DAPM_
 */

static int adau1761_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 reg = 0;

	if (mute) {
		/* mute outputs */
		reg = (snd_soc_read(codec, ADAU_PLBMLC0) & 0xFE) | 0x0;
		snd_soc_write(codec, ADAU_PLBMLC0, reg);
		reg = (snd_soc_read(codec, ADAU_PLBMRC0) & 0xFE) | 0x0;
		snd_soc_write(codec, ADAU_PLBMRC0, reg);

		/* mute inputs */
		reg = (snd_soc_read(codec, ADAU_RECMLC0) & 0xFE) | 0x0;
		snd_soc_write(codec, ADAU_RECMLC0, reg);
		reg = (snd_soc_read(codec, ADAU_RECMRC0) & 0xFE) | 0x0;
		snd_soc_write(codec, ADAU_RECMRC0, reg);
	} else {

		/* un-mute inputs */
		reg = (snd_soc_read(codec, ADAU_RECMLC0) & 0xFE) | 0x1;
		snd_soc_write(codec, ADAU_RECMLC0, reg);
		reg = (snd_soc_read(codec, ADAU_RECMRC0) & 0xFE) | 0x1;
		snd_soc_write(codec, ADAU_RECMRC0, reg);

		/* un-mute outputs */
		reg = (snd_soc_read(codec, ADAU_PLBMLC0) & 0xFE) | 0x1;
		snd_soc_write(codec, ADAU_PLBMLC0, reg);
		reg = (snd_soc_read(codec, ADAU_PLBMRC0) & 0xFE) | 0x1;
		snd_soc_write(codec, ADAU_PLBMRC0, reg);
	}

	return 0;
}

/* Left Mixer */
static const struct snd_kcontrol_new adau1761_left_mixer_controls[] = {
SOC_DAPM_SINGLE("LineLeft Bypass Switch", ADAU_PLBLOVL-ADAU_FIRSTREG, 1, 1, 0),
SOC_DAPM_SINGLE("HPLeft Bypass Switch", ADAU_PLBHPVL-ADAU_FIRSTREG, 1, 1, 0),
};

/* Right mixer */
static const struct snd_kcontrol_new adau1761_right_mixer_controls[] = {
SOC_DAPM_SINGLE("LineRight Bypass Switch", ADAU_PLBLOVR-ADAU_FIRSTREG, 1, 1, 0),
SOC_DAPM_SINGLE("HPRight Bypass Switch", ADAU_PLBHPVR-ADAU_FIRSTREG, 1, 1, 0),
};

static const struct snd_soc_dapm_widget adau1761_dapm_widgets[] = {

SND_SOC_DAPM_MIXER("Left Mixer", ADAU_PLBPWRM-ADAU_FIRSTREG, 2, 1, \
	&adau1761_left_mixer_controls[0], ARRAY_SIZE(adau1761_left_mixer_controls)),
SND_SOC_DAPM_MIXER("Left Out", ADAU_PLBPWRM-ADAU_FIRSTREG, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("Left Line Mixer", ADAU_PLBMLLO-ADAU_FIRSTREG, 0, 0, NULL, 0),
SND_SOC_DAPM_OUTPUT("LOUT"),
SND_SOC_DAPM_OUTPUT("LHPOUT"),

SND_SOC_DAPM_MIXER("Right Mixer", ADAU_PLBPWRM-ADAU_FIRSTREG, 3, 1, \
	&adau1761_right_mixer_controls[0], ARRAY_SIZE(adau1761_right_mixer_controls)),
SND_SOC_DAPM_MIXER("Right Out", ADAU_PLBPWRM-ADAU_FIRSTREG, 1, 0, NULL, 0),
SND_SOC_DAPM_MIXER("Right Line Mixer", ADAU_PLBMRLO-ADAU_FIRSTREG, 0, 0, NULL, 0),
SND_SOC_DAPM_OUTPUT("ROUT"),
SND_SOC_DAPM_OUTPUT("RHPOUT"),

SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
SND_SOC_DAPM_MIXER("DAC Enable Left", ADAU_DACCTL0-ADAU_FIRSTREG, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("DAC Enable Right", ADAU_DACCTL0-ADAU_FIRSTREG, 1, 0, NULL, 0),
SND_SOC_DAPM_MIXER("HP Bias Left", ADAU_PLBPWRM-ADAU_FIRSTREG, 6, 1, NULL, 0),
SND_SOC_DAPM_MIXER("HP Bias Right", ADAU_PLBLRMC-ADAU_FIRSTREG, 0, 0, NULL, 0),

SND_SOC_DAPM_ADC("ADC", "Capture", SND_SOC_NOPM, 0, 0),
SND_SOC_DAPM_MIXER("ADC Left", ADAU_ADCCTL0-ADAU_FIRSTREG, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("ADC Right", ADAU_ADCCTL0-ADAU_FIRSTREG, 1, 0, NULL, 0),

#if !defined(ADAU1761_DIG_MIC)
SND_SOC_DAPM_MICBIAS("Mic Bias", ADAU_RECMBIA-ADAU_FIRSTREG, 0, 0),
SND_SOC_DAPM_MIXER("Left Mic Mixer", ADAU_RECVLCL-ADAU_FIRSTREG, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("Right Mic Mixer", ADAU_RECVLCR-ADAU_FIRSTREG, 0, 0, NULL, 0),
#else
SND_SOC_DAPM_MICBIAS("Mic Bias Left", SND_SOC_NOPM, 1, 0),
SND_SOC_DAPM_MICBIAS("Mic Bias Right", SND_SOC_NOPM, 1, 0),
SND_SOC_DAPM_MIXER("Left Mic Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("Right Mic Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
#endif

SND_SOC_DAPM_MIXER("Left Input", ADAU_RECPWRM-ADAU_FIRSTREG, 1, 1, NULL, 0),
SND_SOC_DAPM_MIXER("Right Input", ADAU_RECPWRM-ADAU_FIRSTREG, 2, 1, NULL, 0),

SND_SOC_DAPM_INPUT("LMICIN"),
SND_SOC_DAPM_INPUT("RMICIN"),
SND_SOC_DAPM_INPUT("LLINEIN"),
SND_SOC_DAPM_INPUT("RLINEIN"),
};

static const struct snd_soc_dapm_route audio_conns[] = {
	/* DAC */
	{"DAC Enable Left", NULL, "DAC"},
	{"DAC Enable Right", NULL, "DAC"},

	/* mixers */
	{"Left Mixer", NULL, "DAC Enable Left"},
	{"Right Mixer", NULL, "DAC Enable Right"},

	/* outputs */
	{"Left Out", NULL, "Left Mixer"},
	{"Right Out", NULL, "Right Mixer"},

	/* line Out */
	{"Left Line Mixer", NULL, "Left Out"},
	{"Right Line Mixer", NULL, "Right Out"},
	{"LOUT", "LineLeft Bypass Switch", "Left Line Mixer"},
	{"ROUT", "LineRight Bypass Switch", "Right Line Mixer"},

	/* headphone out */
	{"HP Bias Left", NULL, "Left Out"},
	{"HP Bias Right", NULL, "Right Out"},
	{"LHPOUT", "HPLeft Bypass Switch", "HP Bias Left"},
	{"RHPOUT", "HPRight Bypass Switch", "HP Bias Right"},

	/* inputs */
	{"Left Input", NULL, "LLINEIN"},
	{"Right Input", NULL, "RLINEIN"},
	{"Mic Bias", NULL, "LMICIN"},
	{"Mic Bias", NULL, "RMICIN"},
	{"Left Mic Mixer", NULL, "Mic Bias"},
	{"Right Mic Mixer", NULL, "Mic Bias"},
	{"ADC Left", NULL, "Left Input"},
	{"ADC Right", NULL, "Right Input"},
	{"ADC Left", NULL, "Left Mic Mixer"},
	{"ADC Right", NULL, "Right Mic Mixer"},
	{"ADC", NULL, "ADC Left"},
	{"ADC", NULL, "ADC Right"},

};

static int adau1761_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, adau1761_dapm_widgets,
				  ARRAY_SIZE(adau1761_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_conns, ARRAY_SIZE(audio_conns));
	return 0;
}

/* PLL dividors */
struct _pll_div {
	u32 mclk;
	u32 pll_freq;
	u16 den;
	u16 num;
	u8  param;
};

static const struct _pll_div clock_dividers[] = {
	{ 12000000, 45158400, 625, 477, /*44.1kHz*/
		(PLLCTRL_INTPART_R3|PLLCTRL_INPUT_DIV1|PLLCTRL_TYPE_FRAC) },
	{12000000, 49152000, 125, 12, /*48kHz*/
		(PLLCTRL_INTPART_R4|PLLCTRL_INPUT_DIV1|PLLCTRL_TYPE_FRAC) },
	{12288000, 45158400, 40, 27, /*44.1Khz*/
		(PLLCTRL_INTPART_R3|PLLCTRL_INPUT_DIV1|PLLCTRL_TYPE_FRAC) },
	{12288000, 49152000, 0, 0, /*48kHz*/
		(PLLCTRL_INTPART_R4|PLLCTRL_INPUT_DIV1|PLLCTRL_TYPE_INT) },
};

static inline int get_pll_settings(int mclk, int pll_out)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(clock_dividers); i++) {
		if (clock_dividers[i].mclk == mclk
			&& clock_dividers[i].pll_freq == pll_out)
			return i;
	}
	return 0;
}

static int adau1761_pll_init(struct snd_soc_codec *codec)
{
	struct adau1761_priv *adau1761 = codec->private_data;
	u8 *pll_reg = adau1761->adau1761_pll_reg;
	int ix = 0;
	/* Clear dsp Run bit */
	snd_soc_write(codec, ADAU_DSP_RUN, 0x00);

	/* Init ADAU1761 clocking */
	snd_soc_write(codec, ADAU_CLKCTRL,
		(CLKCTRL_SRC_PLL | CLKCTRL_FRQ_1024 | CLKCTRL_DISABLE));

	ix = get_pll_settings(adau1761->sysclk, adau1761->pll_out);

	pll_reg[0] = (clock_dividers[ix].den >> 8);
	pll_reg[1] = (clock_dividers[ix].den & 0xFF);
	pll_reg[2] = (clock_dividers[ix].num >> 8);
	pll_reg[3] = (clock_dividers[ix].num & 0xFF);
	pll_reg[4] = clock_dividers[ix].param;
	pll_reg[5] = PLLCTRL_DISABLE;
	adau1761_write_reg_block(codec, ADAU_PLLCTRL, 6, pll_reg);

	adau1761->pll_enable = 0;

	return 0;
}

static int adau1761_pll_enable(struct snd_soc_codec *codec, int enable)
{
	struct adau1761_priv *adau1761 = codec->private_data;
	u8 *pll_reg = adau1761->adau1761_pll_reg;
	int counter = 0;

	if (enable) {
		pll_reg[5]  = PLLCTRL_ENABLE;
		adau1761_write_reg_block(codec, ADAU_PLLCTRL, 6, pll_reg);

		/* wait for PLL lock*/
		do {
			++counter;
			schedule_timeout_interruptible(msecs_to_jiffies(1));
			adau1761_read_reg_block(codec, ADAU_PLLCTRL, 6);
		} while (0 == (snd_soc_read(codec, ADAU_PLLCTRL + 5) & 0x2)
			&& counter < 20);
		if (counter >= 20)
			return -1;

		adau1761->pll_enable = 1;

		/* Init ADAU1761 clocking */
		snd_soc_write(codec, ADAU_CLKCTRL,
			(CLKCTRL_SRC_PLL | CLKCTRL_FRQ_1024 | CLKCTRL_ENABLE));
		/* restore dsp state */
		snd_soc_write(codec, ADAU_DSP_RUN, 0x01);
	}

	return 0;

}

static int adau1761_setprogram(struct snd_soc_codec *codec)
{
	int ret = 0;

	snd_soc_write(codec, ADAU_DSP_ENA, 0x01);
	/* don't waste time writing program for adau1361 */
	if (codec->hw_read(codec, ADAU_DSP_ENA) != 0x01)
		return -ENODEV;
	ret = process_sigma_firmware(codec->control_data, ADAU1761_FIRMWARE);

	return ret;
}

static int adau1761_reg_init(struct snd_soc_codec *codec)
{
	struct adau1761_mode_register regdata;
	struct adau1761_mode_register *registers = 0;
	int i;
#ifdef ADAU1761_DIG_MIC
	int mode = 1;
#else /* analog mic */
	int mode = 0;
#endif
	adau1761_pll_init(codec);
	adau1761_pll_enable(codec, 1);

	/* Load deault regsiter settings */
	for (i = 0; i < RESET_REGISTER_COUNT; ++i) {
		regdata = adau1761_reset[i];
		snd_soc_write(codec, regdata.regaddress, regdata.regvalue);
	}
	/* Load mode registers */
	registers = adau1761_mode_registers[mode];
	for (i = 0; i < MODE_REGISTER_COUNT; ++i) {
		regdata = registers[i];
		snd_soc_write(codec, regdata.regaddress, regdata.regvalue);
	}
	/* Load default program */
	adau1761_setprogram(codec);
	snd_soc_write(codec, ADAU_SPRTCT1, 0x00);
	/* unmute outputs */
	snd_soc_write(codec, ADAU_PLBHPVL, DAPM_HP_DEF);
	snd_soc_write(codec, ADAU_PLBHPVR, DAPM_HP_DEF);
	snd_soc_write(codec, ADAU_PLBLOVL, DAPM_LINE_DEF);
	snd_soc_write(codec, ADAU_PLBLOVR, DAPM_LINE_DEF);

	return 0;
}

struct _srate_set {
	int fs;
	u8 reg;
};

static const struct _srate_set srate_iface[] = {
	{8000, 0x1},
	{11025, 0x2},
	{12000, 0x2},
	{16000, 0x3},
	{22050, 0x4},
	{24000, 0x4},
	{32000, 0x5},
	{44100, 0x0},
	{48000, 0x0},
	{88200, 0x6},
	{96000, 0x6},
};

static int adau1761_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct adau1761_priv *adau1761 = codec->private_data;
	int rate = params_rate(params);
	int i;

	/* initialize the PLL */
	if (adau1761_pll_init(codec) != 0)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(srate_iface); i++) {
		if (srate_iface[i].fs == rate) {
			adau1761->rate_index = i;
			break;
		}
	}

	return 0;
}

static int adau1761_pcm_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct adau1761_priv *adau1761 = codec->private_data;
	u8 reg = 0;
	int ret = 0;

	reg = srate_iface[adau1761->rate_index].reg;
	ret = adau1761_pll_enable(codec, 1);
	if (ret)
		dev_err(codec->dev, "Failed to initialize PLL");

	snd_soc_write(codec, ADAU_SER_SRT, reg);
	/* Set DSP Rate to follow the serial rate */
	snd_soc_write(codec, ADAU_DSP_SR1, 0x07);

	reg = (snd_soc_read(codec, ADAU_CONVCT0) & 0xF8) | reg;
	snd_soc_write(codec, ADAU_CONVCT0, reg);

	return ret;
}

static void adau1761_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	u8 reg;

	snd_soc_write(codec, ADAU_DSP_RUN, 0x0);
	reg = snd_soc_read(codec, ADAU_CLKCTRL);
	snd_soc_write(codec, ADAU_CLKCTRL, reg & ~0x1);

}

static int adau1761_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 reg = 0;

	/* set master/slave audio interface */
	reg = (snd_soc_read(codec, ADAU_SPRTCT0) & 0xFE);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:  /*master*/
		reg |= 0x1;
		snd_soc_write(codec, ADAU_CLK_EN1, CLK_EN1_MASTER);
		break;
	case SND_SOC_DAIFMT_CBS_CFS: /*slave*/
		reg |= 0x0;
		snd_soc_write(codec, ADAU_CLK_EN1, CLK_EN1_SLAVE);
		break;
	default:
		return 0;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
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
	snd_soc_write(codec, ADAU_SPRTCT0, reg);
	return 0;
}

/*
 * Clock after PLL and dividers
 */
static int adau1761_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adau1761_priv *adau1761 = codec->private_data;

	switch (freq) {
	case 12000000:
		adau1761->sysclk = freq;
		return 0;
	case 12288000:
		adau1761->sysclk = freq;
		return 0;
	}

	/* supported 12MHz MCLK only for now */
	return -EINVAL;
}

static int adau1761_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, int source, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adau1761_priv *adau1761 = codec->private_data;

	/* fixed MCLK only supported for now */
	if (adau1761->sysclk != freq_in)
		return -EINVAL;

	/* Only update pll when freq changes */
	if (adau1761->pll_enable && adau1761->pll_out == freq_out)
		return 0;

	switch (freq_out) {
	case 45158400:
		adau1761->pll_out = freq_out;
		break;
	case 49152000:
		adau1761->pll_out = freq_out;
		break;
	default:
		dev_err(codec->dev, "adau1761_set_dai_pll: undefined pll freq:%d", freq_out);
		return -EINVAL;
	}

	return 0;
}


static int adau1761_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		snd_soc_write(codec, ADAU_CLKCTRL,
			(CLKCTRL_SRC_PLL | CLKCTRL_FRQ_1024 | CLKCTRL_DISABLE));
		break;
	case SND_SOC_BIAS_OFF:
		/* everything off, dac mute, inactive */
		snd_soc_write(codec, ADAU_RECPWRM, RECPWRM_LOW_PWR);
		snd_soc_write(codec, ADAU_PLBPWRM, PLBPWRM_LOW_PWR);
		snd_soc_write(codec, ADAU_PLBCTRL, PLBCTRL_POP_OFF);
		snd_soc_write(codec, ADAU_CLKCTRL,
			(CLKCTRL_SRC_PLL | CLKCTRL_FRQ_1024 | CLKCTRL_DISABLE));
		break;

	}
	codec->bias_level = level;
	return 0;
}

#define ADAU1761_RATES (SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_22050 |\
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_88200 |\
		SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
		SNDRV_PCM_RATE_96000)

#define ADAU1761_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops adau1761_dai_ops = {
	.hw_params	= adau1761_hw_params,
	.prepare	= adau1761_pcm_prepare,
	.shutdown	= adau1761_shutdown,
	.digital_mute	= adau1761_mute,
	.set_fmt	= adau1761_set_dai_fmt,
	.set_sysclk	= adau1761_set_dai_sysclk,
	.set_pll	= adau1761_set_dai_pll,
};

struct snd_soc_dai adau1761_dai = {
	.name = "ADAU1761",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = ADAU1761_RATES,
		.formats = ADAU1761_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = ADAU1761_RATES,
		.formats = ADAU1761_FORMATS,
	},
	.ops = &adau1761_dai_ops,
};
EXPORT_SYMBOL_GPL(adau1761_dai);

static int adau1761_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	adau1761_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static void adau1761_resume_wq_handler(struct work_struct *work)
{
	struct adau1761_priv *adau1761 = container_of(work, struct adau1761_priv, resume_work);
	struct snd_soc_codec *codec = &adau1761->codec;
	unsigned int i, v;

	adau1761_pll_init(codec);
	adau1761_pll_enable(codec, 1);
	/* Load program */
	adau1761_setprogram(codec);

	/* sync reg_cache with the hardware */
	for (i = ADAU_FIRSTREG; i <= ADAU_LASTREG; ++i) {
		/* skip over the 6byte PLL control register */
		if (i >= ADAU_PLLCTRL && i < ADAU_MICCTRL)
			continue;
		/* skip reserved registers */
		if (i > ADAU_MCLKPAD && i < ADAU_DSP_CRC)
			continue;

		v = snd_soc_read(codec, i);
		if (snd_soc_write(codec, i, v) != 0) {
			dev_err(codec->dev, "ERROR WRITING %.4X AT REG %x\n", v, i);
			return;
		}
	}

	snd_soc_write(codec, ADAU_PLBCTRL, PLBCTRL_POP_ON);
	snd_soc_write(codec, ADAU_RECPWRM, RECPWRM_RUN_PWR);
	snd_soc_write(codec, ADAU_PLBPWRM, PLBPWRM_RUN_PWR);

	adau1761_set_bias_level(codec, SND_SOC_BIAS_ON);

}

static int adau1761_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	struct adau1761_priv *adau1761 = codec->private_data;

	adau1761->pdev = pdev;
	schedule_work(&adau1761->resume_work);
	return 0;
}

/*
 * initialise the adau1761 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int adau1761_register(struct adau1761_priv *adau1761, enum snd_soc_control_type control)
{
	struct snd_soc_codec *codec = &adau1761->codec;
	int ret = 0;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	codec->name = "adau1761";
	codec->owner = THIS_MODULE;
	codec->set_bias_level = adau1761_set_bias_level;
	codec->dai = &adau1761_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ADAU_NUMCACHEREG;
	codec->reg_cache = kzalloc(ADAU_NUMCACHEREG, GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;
	ret = snd_soc_codec_set_cache_io(codec, 16, 8, control);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		return ret;
	}

	ret = snd_soc_register_dai(&adau1761_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
		snd_soc_unregister_codec(codec);
		return ret;
	}

	return ret;
}

static void adau1761_unregister(struct adau1761_priv *adau1761)
{
	struct snd_soc_codec *codec = &adau1761->codec;

	adau1761_set_bias_level(codec, SND_SOC_BIAS_OFF);
	kfree(codec->reg_cache);
	snd_soc_unregister_dai(&adau1761_dai);
	snd_soc_unregister_codec(codec);
	kfree(adau1761);
	adau1761_codec = NULL;
}

static ssize_t adau1371_dsp_load(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct snd_soc_device *socdev = dev_get_drvdata(dev);
	struct snd_soc_codec *codec = socdev->card->codec;
	int ret = 0;

	ret = adau1761_setprogram(codec);
	if (ret)
		return ret;
	else
		return count;
}
static DEVICE_ATTR(dsp, 0644, NULL, adau1371_dsp_load);

static int adau1761_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct adau1761_priv *adau1761;
	int ret = 0;

	socdev->card->codec = adau1761_codec;
	codec = adau1761_codec;
	adau1761 = codec->private_data;
	adau1761->in_source = CAP_MIC; /*default is mic input*/
	adau1761->sysclk = ADAU1761_MCLK_RATE;
	adau1761->pll_out = ADAU1761_PLL_FREQ_48;
	adau1761->dapm_lineL = DAPM_LINE_DEF;
	adau1761->dapm_lineR = DAPM_LINE_DEF;
	adau1761->dapm_hpL = DAPM_HP_DEF;
	adau1761->dapm_hpR = DAPM_HP_DEF;
	adau1761->pdev = pdev;

	ret = adau1761_reg_init(codec);
	if (ret < 0)
		dev_err(codec->dev, "failed to initialize\n");

	ret = device_create_file(codec->dev, &dev_attr_dsp);
	if (ret)
		dev_err(socdev->dev, "device_create_file() failed\n");
	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms: %d\n", ret);
		goto pcm_err;
	}

	snd_soc_add_controls(codec, adau1761_snd_controls,
			     ARRAY_SIZE(adau1761_snd_controls));
	adau1761_add_widgets(codec);
pcm_err:
	device_remove_file(codec->dev, &dev_attr_dsp);
	return ret;
}

/* remove everything here */
static int adau1761_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_adau1761 = {
	.probe = 	adau1761_probe,
	.remove = 	adau1761_remove,
	.suspend = 	adau1761_suspend,
	.resume =	adau1761_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_adau1761);


static __devinit int adau1761_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct adau1761_priv *adau1761;
	struct snd_soc_codec *codec;
	int ret = 0;

	adau1761 = kzalloc(sizeof(struct adau1761_priv), GFP_KERNEL);
	if (adau1761 == NULL)
		return -ENOMEM;
	codec = &adau1761->codec;
	codec->private_data = adau1761;
	codec->hw_write = (hw_write_t)i2c_master_send;

	i2c_set_clientdata(i2c, adau1761);
	codec->control_data = i2c;

	codec->dev = &i2c->dev;
	adau1761_codec = codec;

	INIT_WORK(&adau1761->resume_work, adau1761_resume_wq_handler);
	ret = adau1761_register(adau1761, SND_SOC_I2C);
	if (ret < 0)
		dev_err(&i2c->dev, "failed to initialize\n");

	return ret;
}

static __devexit int adau1761_i2c_remove(struct i2c_client *client)
{
	struct adau1761_priv *adau1761 = i2c_get_clientdata(client);
	adau1761_unregister(adau1761);
	return 0;
}

static const struct i2c_device_id adau1761_i2c_id[] = {
	{ "adau1761", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adau1761_i2c_id);

/* corgi i2c codec control layer */
static struct i2c_driver adau1761_i2c_driver = {
	.driver = {
		.name = "adau1761",
		.owner = THIS_MODULE,
	},
	.probe    = adau1761_i2c_probe,
	.remove   = __devexit_p(adau1761_i2c_remove),
	.id_table = adau1761_i2c_id,
};

static int __init adau1761_modinit(void)
{
	int ret;

	ret = i2c_add_driver(&adau1761_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register adau1761 I2C driver: %d\n",
		       ret);
	}

	return ret;
}
module_init(adau1761_modinit);

static void __exit adau1761_exit(void)
{
	i2c_del_driver(&adau1761_i2c_driver);
}
module_exit(adau1761_exit);

MODULE_DESCRIPTION("ASoC ADAU1761 driver");
MODULE_AUTHOR("John McCarty, Cliff Cai");
MODULE_LICENSE("GPL");

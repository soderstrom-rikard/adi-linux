/*
 * Driver for ADAU1381 sound codec
 *
 * Copyright 2010-2011 Analog Devices Inc.
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
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "adau1381.h"

#define CAP_MIC  1
#define CAP_LINE 2
#define CAPTURE_SOURCE_NUMBER 2
#define ADAU1381_DIG_MIC 0

/* codec private data */
struct adau1381_priv {
	unsigned int sysclk;
	unsigned int in_source;
	unsigned int out_route;
	unsigned int pll_out;
	struct work_struct resume_work;
	struct snd_soc_codec codec;
	int dapm_state_suspend;
	struct platform_device *pdev;
	u8 pll_enable;
	u8 adau1381_pll_reg[6];
	u8 rate_index;
	/* dapm */
	u8 dapm_lineL;
	u8 dapm_lineR;
	u8 dapm_hpL;
	u8 dapm_hpR;
	enum snd_soc_control_type control_type;
};

struct adau1381_setup_data {
	unsigned short i2c_bus;
	unsigned short i2c_address;
};

struct adau1381_mode_register {
	u16 regaddress;
	u16 regvalue;
};

/*
 * Reset Mode - ADC capture/DAC playback
 * (AInput mixers 0db, AOuput mixers 0db, HP out ON)
 */
static struct adau1381_mode_register adau1381_reset[RESET_REGISTER_COUNT] = {
	/* mute outputs */
	{ADAU_RGUCTRL, 0x00},
	{ADAU_RECCTRL, 0x00},
	{ADAU_RECPWRM, 0x00},
	{ADAU_RECGAIL, 0x07},
	{ADAU_RECGAIR, 0x07},
	{ADAU_RECMBIA, RECMBIA_DISABLE},
	{ADAU_SPRTCT0, ADAU_SRPT_CTRL0},
	{ADAU_SPRTCT1, 0x21}, /* 0x21 = 32bclocks frame, 0x41 = 48 */
	{ADAU_CONVCT0, 0x00},
	{ADAU_CONVCT1, 0x00},
	{ADAU_ADCCTL0, 0x00},
	{ADAU_LADCATT, 0x00},
	{ADAU_RADCATT, 0x00},
	{ADAU_PLMLCTL, 0x20},
	{ADAU_PLMRCTL, 0x20},
	{ADAU_PLBMCTL, 0x00},
	{ADAU_PLBCAMP, 0x00},
	{ADAU_RLOMUTE, 0x02},
	{ADAU_LLOMUTE, 0x02},
	{ADAU_PLSPCTL, 0x01},
	{ADAU_ZXDETCT, 0x01},
	{ADAU_PLBPWRM, 0x00},
	{ADAU_DACCTRL, 0x03},
	{ADAU_LDACATT, 0x00},
	{ADAU_RDACATT, 0x00},
	{ADAU_SERPAD0, 0xAA},
	{ADAU_SERPAD1, 0x00},
	{ADAU_COMPAD0, 0xAA},
	{ADAU_COMPAD1, 0x00},
	{ADAU_MCKOCTL, 0x00},
};

static struct adau1381_mode_register adau1381_mode0[MODE_REGISTER_COUNT] = {
	/* analog mic */
	{ADAU_RECCTRL, 0x00},
};

static struct adau1381_mode_register adau1381_mode1[MODE_REGISTER_COUNT] = {
	/* digital mic */
	{ADAU_RECCTRL, 0x10},
};

static struct adau1381_mode_register *adau1381_mode_registers[] = {
	adau1381_mode0,
	adau1381_mode1,
};

/*
 * write register cache
 */
static inline int adau1381_write_reg_cache(struct snd_soc_codec *codec,
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
 * read a multi-byte ADAU1381 register (6byte pll reg)
 */
static int adau1381_read_reg_block(struct snd_soc_codec *codec,
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
		adau1381_write_reg_cache(codec, reg+i, (unsigned int)buf[i]);

	return 0;
}

/*
 * write a multibyte ADAU1381 register (6byte pll reg)
 */
static int adau1381_write_reg_block(struct snd_soc_codec *codec,
	unsigned int reg, u8 length, u8 *values)
{
	int count = length + 2; /* data plus 16bit register address */
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

static const struct snd_kcontrol_new adau1381_snd_controls[] = {
SOC_DOUBLE_R("Master Playback Volume", ADAU_LDACATT-ADAU_FIRSTREG,
	ADAU_RDACATT-ADAU_FIRSTREG, 0, 255, 1),
SOC_DOUBLE_R("Capture Volume", ADAU_LADCATT-ADAU_FIRSTREG,
	ADAU_RADCATT-ADAU_FIRSTREG, 0, 255, 1),
};

/*
 * _DAPM_
 */

static const struct snd_soc_dapm_widget adau1381_dapm_widgets[] = {

SND_SOC_DAPM_MIXER("Left Out", ADAU_PLBPWRM-ADAU_FIRSTREG, 0, 0, NULL, 0),
SND_SOC_DAPM_OUTPUT("LOUT"),
SND_SOC_DAPM_OUTPUT("LHPOUT"),

SND_SOC_DAPM_MIXER("Right Out", ADAU_PLBPWRM-ADAU_FIRSTREG, 1, 0, NULL, 0),
SND_SOC_DAPM_OUTPUT("ROUT"),
SND_SOC_DAPM_OUTPUT("RHPOUT"),

SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
SND_SOC_DAPM_MIXER("HP Bias Left", ADAU_PLBPWRM-ADAU_FIRSTREG, 6, 1, NULL, 0),

SND_SOC_DAPM_ADC("ADC", "Capture", SND_SOC_NOPM, 0, 0),
SND_SOC_DAPM_MIXER("ADC Left", ADAU_ADCCTL0-ADAU_FIRSTREG, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("ADC Right", ADAU_ADCCTL0-ADAU_FIRSTREG, 1, 0, NULL, 0),

#if !defined(ADAU1381_DIG_MIC)
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

static int adau1381_add_widgets(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	snd_soc_dapm_new_controls(dapm, adau1381_dapm_widgets,
				  ARRAY_SIZE(adau1381_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, audio_conns, ARRAY_SIZE(audio_conns));

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
	{ 12000000, 45158400, 625, 477, /* 44.1kHz */
		(PLLCTRL_INTPART_R3|PLLCTRL_INPUT_DIV1|PLLCTRL_TYPE_FRAC) },
	{12000000, 49152000, 125, 12, /* 48kHz */
		(PLLCTRL_INTPART_R4|PLLCTRL_INPUT_DIV1|PLLCTRL_TYPE_FRAC) },
	{12288000, 45158400, 40, 27, /* 44.1Khz */
		(PLLCTRL_INTPART_R3|PLLCTRL_INPUT_DIV1|PLLCTRL_TYPE_FRAC) },
	{12288000, 49152000, 0, 0, /* 48kHz */
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

static int adau1381_pll_init(struct snd_soc_codec *codec)
{
	struct adau1381_priv *adau1381 = snd_soc_codec_get_drvdata(codec);
	u8 *pll_reg = adau1381->adau1381_pll_reg;
	int ix = 0;

	/* Init ADAU1381 clocking */
	snd_soc_write(codec, ADAU_CLKCTRL,
		(CLKCTRL_SRC_PLL | CLKCTRL_FRQ_1024 | CLKCTRL_DISABLE));

	ix = get_pll_settings(adau1381->sysclk, adau1381->pll_out);

	pll_reg[0] = (clock_dividers[ix].den >> 8);
	pll_reg[1] = (clock_dividers[ix].den & 0xFF);
	pll_reg[2] = (clock_dividers[ix].num >> 8);
	pll_reg[3] = (clock_dividers[ix].num & 0xFF);
	pll_reg[4] = clock_dividers[ix].param;
	pll_reg[5] = PLLCTRL_DISABLE;
	adau1381_write_reg_block(codec, ADAU_PLLCTRL, 6, pll_reg);

	adau1381->pll_enable = 0;

	return 0;
}

static int adau1381_pll_enable(struct snd_soc_codec *codec, int enable)
{
	struct adau1381_priv *adau1381 = snd_soc_codec_get_drvdata(codec);
	u8 *pll_reg = adau1381->adau1381_pll_reg;
	int counter = 0;

	if (enable) {
		pll_reg[5]  = PLLCTRL_ENABLE;
		adau1381_write_reg_block(codec, ADAU_PLLCTRL, 6, pll_reg);

		/* wait for PLL lock */
		do {
			++counter;
			schedule_timeout_interruptible(msecs_to_jiffies(1));
			adau1381_read_reg_block(codec, ADAU_PLLCTRL, 6);
		} while (0 == (snd_soc_read(codec, ADAU_PLLCTRL + 5) & 0x2)
			&& counter < 20);
		if (counter >= 20)
			return -1;

		adau1381->pll_enable = 1;

		/* Init ADAU1381 clocking */
		snd_soc_write(codec, ADAU_CLKCTRL,
			(CLKCTRL_SRC_PLL | CLKCTRL_FRQ_1024 | CLKCTRL_ENABLE));
	}

	return 0;

}

static int adau1381_reg_init(struct snd_soc_codec *codec)
{
	struct adau1381_mode_register regdata;
	struct adau1381_mode_register *registers = 0;
	int i;
#ifdef ADAU1381_DIG_MIC
	int mode = 1;
#else /* analog mic */
	int mode = 0;
#endif

	/* Load deault regsiter settings */
	for (i = 0; i < RESET_REGISTER_COUNT; ++i) {
		regdata = adau1381_reset[i];
		snd_soc_write(codec, regdata.regaddress, regdata.regvalue);
	}
	/* Load mode registers */
	registers = adau1381_mode_registers[mode];
	for (i = 0; i < MODE_REGISTER_COUNT; ++i) {
		regdata = registers[i];
		snd_soc_write(codec, regdata.regaddress, regdata.regvalue);
	}

	snd_soc_write(codec, ADAU_SPRTCT1, 0x00);



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

static const struct _srate_set erate_iface[] = {
	{8000, 0x6},
	{11025, 0x5},
	{12000, 0x5},
	{16000, 0x4},
	{22050, 0x3},
	{24000, 0x3},
	{32000, 0x2},
	{44100, 0x1},
	{48000, 0x1},
	{88200, 0x0},
	{96000, 0x0},
};

static int adau1381_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct adau1381_priv *adau1381 = snd_soc_codec_get_drvdata(codec);
	int rate = params_rate(params);
	int i;

	/* initialize the PLL */
	if (adau1381_pll_init(codec) != 0)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(srate_iface); i++) {
		if (srate_iface[i].fs == rate) {
			adau1381->rate_index = i;
			break;
		}
	}

	return 0;
}

static int adau1381_pcm_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct adau1381_priv *adau1381 = snd_soc_codec_get_drvdata(codec);
	u8 reg = 0;
	int ret = 0;

	reg = srate_iface[adau1381->rate_index].reg;
	ret = adau1381_pll_enable(codec, 1);
	if (ret)
		dev_err(codec->dev, "Failed to initialize PLL");
	snd_soc_write(codec, ADAU_DIGPWR0, 0xF3);
	snd_soc_write(codec, ADAU_DIGPWR0, 0x0D);
	reg = (snd_soc_read(codec, ADAU_CONVCT0) & 0xF8) | reg;
	snd_soc_write(codec, ADAU_CONVCT0, reg);
	reg = (snd_soc_read(codec, ADAU_SAMRATE)) | reg;
	snd_soc_write(codec, ADAU_SAMRATE, reg);

	reg = (snd_soc_read(codec, ADAU_FRMRATE)) | reg;
	snd_soc_write(codec, ADAU_FRMRATE, reg);
	snd_soc_write(codec, ADAU_ENGIRUN, 0x01);

	return ret;
}

static void adau1381_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	u8 reg;

	snd_soc_write(codec, ADAU_DIGPWR0, 0x0);
	snd_soc_write(codec, ADAU_DIGPWR0, 0x0);
	reg = snd_soc_read(codec, ADAU_CLKCTRL);
	snd_soc_write(codec, ADAU_CLKCTRL, reg & ~0x1);
	snd_soc_write(codec, ADAU_FRMRATE, 0x7F);
	msleep(3);
	snd_soc_write(codec, ADAU_ENGIRUN, 0x0);

}

static int adau1381_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 reg = 0;

	/* set master/slave audio interface */
	reg = (snd_soc_read(codec, ADAU_SPRTCT0) & 0xFE);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:  /* master */
		reg |= 0x1;
		break;
	case SND_SOC_DAIFMT_CBS_CFS: /* slave */
		reg &= ~0x1;
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

	/* set I2S iface format */
	snd_soc_write(codec, ADAU_SPRTCT0, reg);
	return 0;
}

/*
 * Clock after PLL and dividers
 */
static int adau1381_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adau1381_priv *adau1381 = snd_soc_codec_get_drvdata(codec);

	switch (freq) {
	case 12000000:
		adau1381->sysclk = freq;
		return 0;
	case 12288000:
		adau1381->sysclk = freq;
		return 0;
	}

	/* supported 12MHz MCLK only for now */
	return -EINVAL;
}

static int adau1381_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, int source, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adau1381_priv *adau1381 = snd_soc_codec_get_drvdata(codec);

	/* fixed MCLK only supported for now */
	if (adau1381->sysclk != freq_in)
		return -EINVAL;

	/* Only update pll when freq changes */
	if (adau1381->pll_enable && adau1381->pll_out == freq_out)
		return 0;

	switch (freq_out) {
	case 45158400:
		adau1381->pll_out = freq_out;
		break;
	case 49152000:
		adau1381->pll_out = freq_out;
		break;
	default:
		dev_err(codec->dev, "adau1381_set_dai_pll: undefined pll freq:%d", freq_out);
		return -EINVAL;
	}

	return 0;
}


static int adau1381_set_bias_level(struct snd_soc_codec *codec,
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
		snd_soc_write(codec, ADAU_CLKCTRL,
			(CLKCTRL_SRC_PLL | CLKCTRL_FRQ_1024 | CLKCTRL_DISABLE));
		break;

	}
	codec->dapm.bias_level = level;
	return 0;
}

#define ADAU1381_RATES (SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_22050 |\
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_88200 |\
		SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
		SNDRV_PCM_RATE_96000)

#define ADAU1381_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops adau1381_dai_ops = {
	.hw_params	= adau1381_hw_params,
	.prepare	= adau1381_pcm_prepare,
	.shutdown	= adau1381_shutdown,
	.set_fmt	= adau1381_set_dai_fmt,
	.set_sysclk	= adau1381_set_dai_sysclk,
	.set_pll	= adau1381_set_dai_pll,
};

static struct snd_soc_dai_driver adau1381_dai = {
	.name = "ADAU1381",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = ADAU1381_RATES,
		.formats = ADAU1381_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = ADAU1381_RATES,
		.formats = ADAU1381_FORMATS,
	},
	.ops = &adau1381_dai_ops,
};

static int adau1381_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	adau1381_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static void adau1381_resume_wq_handler(struct work_struct *work)
{
	struct adau1381_priv *adau1381 = container_of(work, struct adau1381_priv, resume_work);
	struct snd_soc_codec *codec = &adau1381->codec;
	unsigned int i, v;

	adau1381_pll_init(codec);
	adau1381_pll_enable(codec, 1);

	/* sync reg_cache with the hardware */
	for (i = ADAU_FIRSTREG; i <= ADAU_LASTREG; ++i) {

		v = snd_soc_read(codec, i);
		if (snd_soc_write(codec, i, v) != 0) {
			dev_err(codec->dev, "ERROR WRITING %.4X AT REG %x\n", v, i);
			return;
		}
	}

	snd_soc_write(codec, ADAU_RECPWRM, RECPWRM_RUN_PWR);
	snd_soc_write(codec, ADAU_PLBPWRM, PLBPWRM_RUN_PWR);

	adau1381_set_bias_level(codec, SND_SOC_BIAS_ON);

}

static int adau1381_resume(struct snd_soc_codec *codec)
{
	struct adau1381_priv *adau1381 = snd_soc_codec_get_drvdata(codec);

	schedule_work(&adau1381->resume_work);
	return 0;
}

static int adau1381_probe(struct snd_soc_codec *codec)
{
	struct adau1381_priv *adau1381 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	ret = snd_soc_codec_set_cache_io(codec, 16, 8, adau1381->control_type);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	ret = adau1381_reg_init(codec);
	if (ret < 0) {
		dev_err(codec->dev, "failed to initialize\n");
		return ret;
	}

	snd_soc_add_controls(codec, adau1381_snd_controls,
			     ARRAY_SIZE(adau1381_snd_controls));
	adau1381_add_widgets(codec);

	return 0;
}

/* remove everything here */
static int adau1381_remove(struct snd_soc_codec *codec)
{
	adau1381_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_adau1381 = {
	.probe   = adau1381_probe,
	.remove  = adau1381_remove,
	.suspend = adau1381_suspend,
	.resume  = adau1381_resume,
	.set_bias_level = adau1381_set_bias_level,
	.reg_cache_size = ADAU_NUMCACHEREG,
	.reg_word_size = sizeof(u16),
	.reg_cache_default = adau1381_reg,
};

static __devinit int adau1381_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct adau1381_priv *adau1381;
	int ret;

	adau1381 = kzalloc(sizeof(*adau1381), GFP_KERNEL);
	if (adau1381 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, adau1381);
	adau1381->control_type = SND_SOC_I2C;

	INIT_WORK(&adau1381->resume_work, adau1381_resume_wq_handler);
	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_adau1381, &adau1381_dai, 1);
	if (ret)
		kfree(adau1381);

	return ret;
}

static __devexit int adau1381_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id adau1381_i2c_id[] = {
	{ "adau1381", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adau1381_i2c_id);

/* corgi i2c codec control layer */
static struct i2c_driver adau1381_i2c_driver = {
	.driver = {
		.name = "adau1381",
		.owner = THIS_MODULE,
	},
	.probe    = adau1381_i2c_probe,
	.remove   = __devexit_p(adau1381_i2c_remove),
	.id_table = adau1381_i2c_id,
};

static int __init adau1381_modinit(void)
{
	return i2c_add_driver(&adau1381_i2c_driver);
}
module_init(adau1381_modinit);

static void __exit adau1381_exit(void)
{
	i2c_del_driver(&adau1381_i2c_driver);
}
module_exit(adau1381_exit);

MODULE_DESCRIPTION("ASoC ADAU1381 driver");
MODULE_AUTHOR("Cliff Cai");
MODULE_LICENSE("GPL");

/*
 * Driver for ADAU1373 sound codec
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _ADAU1373_H
#define _ADAU1373_H

/* ADAU1373 Codec Register definitions */

#define DIGMIC 0

#define ADAU_INPMODE	0x00
#define ADAU_IN1LCTL	0x01
#define ADAU_IN1RCTL	0x02
#define ADAU_IN2LCTL	0x03
#define ADAU_IN2RCTL	0x04
#define ADAU_IN3LCTL	0x05
#define ADAU_IN3RCTL	0x06
#define ADAU_AUXLCTL	0x07
#define ADAU_AUXRCTL	0x08
#define ADAU_LLN1OPT	0x09
#define ADAU_RLN1OPT	0x0A
#define ADAU_LLN2OPT	0x0B
#define ADAU_RLN2OPT	0x0C
#define ADAU_LCDOUTP	0x0D
#define ADAU_RCDOUTP	0x0E
#define ADAU_LHPOUTP	0x0F
#define ADAU_RHPOUTP	0x10
#define ADAU_ADCGAIN	0x11

#define ADAU_LADCMIX	0x12
#define ADAU_RADCMIX	0x13
#define ADAU_LLN1MIX	0x14
#define ADAU_LLN2MIX	0x15
#define ADAU_RLN1MIX	0x16
#define ADAU_RLN2MIX	0x17
#define ADAU_LCDMIX	0x18
#define ADAU_RCDMIX	0x19
#define ADAU_LHPMIX	0x1A
#define ADAU_RHPMIX	0x1B
#define ADAU_EPMIX	0x1C

#define ADAU_HPCTRL	0x1D
#define ADAU_HPCTRL2	0x1E
#define ADAU_LSCTRL	0x1F

#define ADAU_BSTCTRL	0x20
#define ADAU_EPCNTRL	0x21
#define ADAU_MICCTR1	0x22
#define ADAU_MICCTR2	0x23
#define ADAU_OPTCTRL	0x24
#define ADAU_PWDCTL1	0x25
#define ADAU_PWDCTL2	0x26
#define ADAU_PWDCTL3	0x27


#define ADAU_PLLACTL	0x28
#define ADAU_PLLACTL1	0x29
#define ADAU_PLLACTL2	0x2A
#define ADAU_PLLACTL3	0x2B
#define ADAU_PLLACTL4	0x2C
#define ADAU_PLLACTL5	0x2D
#define ADAU_PLLACTL6	0x2E

#define ADAU_PLLBCTL	0x2F
#define ADAU_PLLBCTL1	0x30
#define ADAU_PLLBCTL2	0x31
#define ADAU_PLLBCTL3	0x32
#define ADAU_PLLBCTL4	0x33
#define ADAU_PLLBCTL5	0x34
#define ADAU_PLLBCTL6	0x35

#define ADAU_HEADDET	0x36
#define ADAU_CHIPSTA	0x39
#define ADAU_CLK1SDIV	0x40
#define ADAU_CLK1ODIV	0x41
#define ADAU_CLK2SDIV	0x42
#define ADAU_CLK2ODIV	0x43


#define ADAU_DAIA	0x44
#define ADAU_DAIB	0x45
#define ADAU_DAIC	0x46

#define ADAU_BCLKDIVA	0x47
#define ADAU_BCLKDIVB	0x48
#define ADAU_BCLKDIVC	0x49

#define ADAU_SRCARTA	0x4A
#define ADAU_SRCARTB	0x4B
#define ADAU_SRCBRTA	0x4C
#define ADAU_SRCBRTB	0x4D
#define ADAU_SRCCRTA	0x4E
#define ADAU_SRCCRTB	0x4F

#define ADAU_DEEMPCTL	0x50
#define ADAU_DAIACTL	0x51
#define ADAU_DAIBCTL	0x52
#define ADAU_DAICCTL	0x53


#define ADAU_DINMIXC0	0x56
#define ADAU_DINMIXC1	0x57
#define ADAU_DINMIXC2	0x58
#define ADAU_DINMIXC3	0x59
#define ADAU_DINMIXC4	0x5A

#define ADAU_DOPMIXC0	0x5B
#define ADAU_DOPMIXC1	0x5B
#define ADAU_DOPMIXC2	0x5D
#define ADAU_DOPMIXC3	0x5E
#define ADAU_DOPMIXC4	0x5F

#define ADAU_VOLMOD1	0x60
#define ADAU_VOLMOD2	0x61

#define ADAU_DAPBLVOL	0x62
#define ADAU_DAPBRVOL	0x63
#define ADAU_DBPBLVOL	0x64
#define ADAU_DBPBRVOL	0x65
#define ADAU_DCPBLVOL	0x66
#define ADAU_DCPBRVOL	0x67


#define ADAU_DARCLVOL	0x68
#define ADAU_DARCRVOL	0x69
#define ADAU_DBRCLVOL	0x6A
#define ADAU_DBRCRVOL	0x6B
#define ADAU_DCRCLVOL	0x6C
#define ADAU_DCRCRVOL	0x6D


#define ADAU_PBALVOL	0x6E
#define ADAU_PBARVOL	0x6F
#define ADAU_PBBLVOL	0x70
#define ADAU_PBBRVOL	0x71


#define ADAU_RECLVOL	0x72
#define ADAU_RECRVOL	0x73
#define ADAU_RECLVOL	0x72
#define ADAU_RECRVOL	0x73
#define ADAU_DRECLVOL	0x74
#define ADAU_DRECRVOL	0x75

#define ADAU_ALCCTL0	0x76
#define ADAU_ALCCTL1	0x77
#define ADAU_ALCCTL2	0x78
#define ADAU_ALCCTL3	0x79
#define ADAU_ALCCTL4	0x7A
#define ADAU_ALCCTL5	0x7B
#define ADAU_ALCCTL6	0x7C

#define ADAU_PBALPCTL	0xE0
#define ADAU_PBBLPCTL	0xE1

#define ADAU_DMICCTL	0xE2
#define ADAU_PADCTL1	0xE9
#define ADAU_PADCTL2	0xEA
#define ADAU_DIGEN	0xEB
#define ADAU_CHIPIDH	0xEC
#define ADAU_CHIPIDM	0xED
#define ADAU_CHIPIDL	0xEE
#define ADAU_RESET	0xFF


struct adau1361_mode_settings{
	u8  regaddress;
	u8  regvalue;
};

/*
 * ADAU1373 Codec Register Field definitions
 * (Mask value to extract the corresponding Register field)
 */

#define VOL_MASK		0x1F
#define AIN1_SIGNAL_ENA		0x01
#define AIN2_SIGNAL_ENA		0x02
#define AIN3_SIGNAL_ENA		0x04
#define AIN4_SIGNAL_ENA		0x08
#define LDAC_SIGNAL_ENA		0x10
#define RDAC_SIGNAL_ENA		0x20

/* PWR Management */
#define PWRCTLA_INAPD		0x01
#define PWRCTLA_INBPD		0x02
#define PWRCTLA_INCPD		0x04
#define PWRCTLA_INDPD		0x08
#define PWRCTLA_PASSPD		0x10
#define PWRCTLA_MICBPD		0x20
#define PWRCTLA_LADCPD		0x40
#define PWRCTLA_RADCPD		0x80

#define PWRCTLB_PWDB		0x01
#define PWRCTLB_HPPD		0x02
#define PWRCTLB_LCDPD		0x04
#define PWRCTLB_RCDPD		0x08
#define PWRCTLB_LDACPD		0x10
#define PWRCTLB_RDACPD		0x20
#define PWRCTLB_LLNPD		0x40
#define PWRCTLB_RLNPD		0x80

/* PLL Control */

#define PLLCTLA_X_SHIFT		1
#define PLLCTLA_R_SHIFT		3
#define PLLCTLB_PLLEN		0x01	/* PLL enable */
#define PLLCTLB_LOCK		0x20	/* Lock poll */

/*Clock GEN*/

#define CLKSDIV_COREN		0x80	/* Core clock enable */
#define CLKSDIV_PLL_BYPASS	0x40	/* Bypass PLL */
#define CLKSDIV_CLKDIV_SHIFT	3
#define CLKSDIV_MCLKDIV_SHIFT	0

/* DAI Control */

#define DAICTL_FMRJUST		0x00	/* Audio interface mode */
#define DAICTL_FMLJUST		0x01
#define DAICTL_FMI2S		0x02
#define DAICTL_FMDSP		0x03

#define DAICTL_WLEN16		0x00	/* Word length */
#define DAICTL_WLEN20		0x04
#define DAICTL_WLEN24		0x08
#define DAICTL_WLEN32		0x0c

#define DAICTL_LRPA		0x10
#define DAICTL_SWAPA		0x20
#define DAICTL_MSA		0x40	/* Codec in master mode */
#define DAICTL_BLKINVA		0x80

/* Number of bit clock per frame */
#define BCLKDIV_BPFA256		0x00
#define BCLKDIV_BPFA128		0x01
#define BCLKDIV_BPFA64		0x02
#define BCLKDIV_BPFA32		0x03

/* SRC/DAI Control */
#define SRCDAICTL_DAIA_ENA	0x01
#define SRCDAICTL_DAIB_ENA	0x02
#define SRCDAICTL_SRCREC_ENA	0x04
#define SRCDAICTL_SRCPB_ENA	0x08
/* DIGMIC */
#define DIGMIC_EN		0x01
/* DIGEN */
#define DIGEN_PBEN		0x01
#define DIGEN_RECEN		0x02
#define DIGEN_FDSPEN		0x04

#define PB_LINE1		0x01
#define PB_LINE2		0x02
#define PB_SPK			0x04
#define PB_EARP			0x08
#define PB_HP			0x10

#define CAP_INPA		0x01
#define CAP_INPB		0x02
#define CAP_INPC		0x04
#define CAP_INPD		0x08
#define CAP_DMIC		0x10

#define ADC_MUTE_MASK		0xc0
#define DAC_MUTE_MASK		0x30

/* DRC */
#define DRCMODE_NGEN		0x01
#define DRCMODE_RIGHT_ENA	0x04
#define DRCMODE_LEFT_ENA	0x08

/* DSP MODE */
#define DSPMODE_PLAYBACK_ENA	0x01
#define DSPMODE_CAPTURE_ENA	0x02

/* PAD_CTL */
#define PADCTL_DAIA		0x01
#define PADCTL_DAIB		0x02
#define PADCTL_GPIO		0x04
#define PADCTL_I2C		0x08
#define PADCTL_I2CFLT		0x10

#define BE_SHIFT		2
#define EQ_SHIFT		4
#define HPF_SHIFT		6

#define ADAU1373_CACHEREGNUM	0x100

#define ADAU1373_SYSCLK		0
#define ADAU1373_DAI		0

#define CHANNELS_INPUT		2
#define CHANNELS_OUTPUT		2

struct adau1373_setup_data {
	int i2c_bus;
	unsigned short i2c_address;
};

struct _pll_settings {
	u32 mclk;
	u32 rate;
	u16 n;
	u16 m;
	u8 input_div:2;
	u8 integer:4;
	u8 type:1;
};

struct _srate_set {
	int fs;
	u8 reg;
};

struct adau1373_platform_data {
	u8 pll_settings_num;
	const struct _pll_settings *pll_settings;
	u8 drc_settings[8];
};

extern struct snd_soc_dai adau1373_dai;
extern struct snd_soc_codec_device soc_codec_dev_adau1373;

#endif

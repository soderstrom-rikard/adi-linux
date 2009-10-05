/*
 * Driver for ADAU1371 sound codec
 *
 * Copyright 2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _ADAU1371_H
#define _ADAU1371_H

/* ADAU1371 Codec Register definitions */

#define ADAU1371_INPUTMODE	0x00
#define ADAU1371_INALVOL	0x01
#define ADAU1371_INARVOL	0x02
#define ADAU1371_INBLVOL	0x03
#define ADAU1371_INBRVOL	0x04
#define ADAU1371_INCLVOL	0x05
#define ADAU1371_INCRVOL	0x06
#define ADAU1371_INDLVOL	0x07
#define ADAU1371_INDRVOL	0x08
#define ADAU1371_LLINEVOL	0x09
#define ADAU1371_RLINEVOL	0x0A
#define ADAU1371_LCDVOL		0x0B
#define ADAU1371_RCDVOL		0x0C
#define ADAU1371_LHPVOL		0x0D
#define ADAU1371_RHPVOL		0x0E

#define ADAU1371_HPCTRL		0x0F
#define ADAU1371_LSCTRL		0x10
#define ADAU1371_LINECTL	0x11
#define ADAU1371_ADCGAIN	0x12

#define ADAU1371_LADCMIX	0x13
#define ADAU1371_RADCMIX	0x14
#define ADAU1371_LLINEMIX	0x15
#define ADAU1371_RLINEMIX	0x16
#define ADAU1371_LCDMIX		0x17
#define ADAU1371_RCDMIX		0x18
#define ADAU1371_LHPMIX		0x19
#define ADAU1371_RHPMIX		0x1A

#define ADAU1371_REFCTL		0x1B
#define ADAU1371_CODECCTL	0x1C
#define ADAU1371_PWRCTLA	0x1D
#define ADAU1371_PWRCTLB	0x1E
#define ADAU1371_HEADDECT	0x1F

#define ADAU1371_PLLMHI		0x20
#define ADAU1371_PLLMLOW	0x21
#define ADAU1371_PLLNHI		0x22
#define ADAU1371_PLLNLOW	0x23
#define ADAU1371_PLLCTLA	0x24
#define ADAU1371_PLLCTLB	0x25

#define ADAU1371_CHIPSTAT	0x26
#define ADAU1371_CODECSTAT	0x27
#define ADAU1371_HPSTAT		0x28
#define ADAU1371_ADDCTL		0x29

#define ADAU1371_CLKSDIV	0x30
#define ADAU1371_CLKODIV	0x31
#define ADAU1371_DAIACTL	0x32
#define ADAU1371_DAIBCTL	0x33
#define ADAU1371_BCLKDIV	0x34
#define ADAU1371_SRCA		0x35
#define ADAU1371_SRCB		0x36
#define ADAU1371_SRCC		0x37
#define ADAU1371_SRCMIX		0x38
#define ADAU1371_SRCDAICTL	0x39
#define ADAU1371_DAIAPBLVOL	0x3A
#define ADAU1371_DAIAPBRVOL	0x3B
#define ADAU1371_DAIBPBLVOL	0x3C
#define ADAU1371_DAIBPBRVOL	0x3D
#define ADAU1371_DAIRECLVOL	0x3E
#define ADAU1371_DAIRECRVOL	0x3F
#define ADAU1371_CODECPBLVOL	0x40
#define ADAU1371_CODECPBRVOL	0x41
#define ADAU1371_CODECRECLVOL	0x42
#define ADAU1371_CODECRECRVOL	0x43
#define ADAU1371_VOLMOD		0x44
/* Dynamic range control */
#define ADAU1371_DRCCTL1	0x51
#define ADAU1371_DRCCTL2	0x52
#define ADAU1371_DRCSC1		0x53
#define ADAU1371_DRCSC2		0x54
#define ADAU1371_DRCSC3		0x55
#define ADAU1371_DRCGS1		0x56
#define ADAU1371_DRCGS2		0x57
#define ADAU1371_DRCGS3		0x58
#define ADAU1371_DRCMODE	0x59

#define ADAU1371_DSPMODE	0xA2

#define ADAU1371_DIGMIC		0xB1
#define ADAU1371_PAD_CTL	0xB7
#define ADAU1371_DIGEN		0xC0
#define ADAU1371_RESET		0xFF

/*
 * ADAU1371 Codec Register Field definitions
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

#define PB_LINE			0x01
#define PB_CD			0x02
#define PB_HP			0x04

#define CAP_INPA		0x01
#define CAP_INPB		0x02
#define CAP_INPC		0x04
#define CAP_INPD		0x08

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

#define ADAU1371_CACHEREGNUM	0x100

#define ADAU1371_SYSCLK		0
#define ADAU1371_DAI		0

#define CHANNELS_INPUT		2
#define CHANNELS_OUTPUT		2

struct adau1371_setup_data {
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

struct adau1371_platform_data {
	u8 pll_settings_num;
	const struct _pll_settings *pll_settings;
	u8 drc_settings[8];
};

extern struct snd_soc_dai adau1371_dai;
extern struct snd_soc_codec_device soc_codec_dev_adau1371;

#endif

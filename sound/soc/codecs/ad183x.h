/*
 * AD183X Audio Codec driver supporting AD1835A, AD183X, AD1837A, AD1838A, AD1839A
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __AD183X_H__
#define __AD183X_H__

#define AD183X_DAC_CTRL1               0
#define AD183X_DAC_POWERDOWN           2
#define AD183X_DAC_SERFMT_MASK	       0xE0
#define AD183X_DAC_SERFMT_PCK256       (0x4 << 5)
#define AD183X_DAC_SERFMT_PCK128       (0x5 << 5)
#define AD183X_DAC_WORD_LEN_MASK       0x18

#define AD183X_DAC_CTRL2               1
#define AD183X_DACL1_MUTE              0
#define AD183X_DACR1_MUTE              1
#define AD183X_DACL2_MUTE              2
#define AD183X_DACR2_MUTE              3
#define AD183X_DACL3_MUTE              4
#define AD183X_DACR3_MUTE              5
#define AD183X_DACL4_MUTE              6
#define AD183X_DACR4_MUTE              7

#define AD183X_DAC_L1_VOL              2
#define AD183X_DAC_R1_VOL              3
#define AD183X_DAC_L2_VOL              4
#define AD183X_DAC_R2_VOL              5
#define AD183X_DAC_L3_VOL              6
#define AD183X_DAC_R3_VOL              7
#define AD183X_DAC_L4_VOL              8
#define AD183X_DAC_R4_VOL              9

#define AD183X_ADC_CTRL1               12
#define AD183X_ADC_POWERDOWN           7
#define AD183X_ADC_HIGHPASS_FILTER     8

#define AD183X_ADC_CTRL2               13
#define AD183X_ADCL1_MUTE 		0
#define AD183X_ADCR1_MUTE 		1
#define AD183X_ADCL2_MUTE 		2
#define AD183X_ADCR2_MUTE 		3
#define AD183X_ADC_WORD_LEN_MASK       0x30
#define AD183X_ADC_SERFMT_MASK	       (7 << 6)
#define AD183X_ADC_SERFMT_PCK256       (0x4 << 6)
#define AD183X_ADC_SERFMT_PCK128       (0x5 << 6)
#define AD183X_ADC_AUX                 (0x6 << 6)

#define AD183X_ADC_CTRL3               14

#define AD183X_NUM_REGS                16

extern struct snd_soc_dai ad183x_dai;
extern struct snd_soc_codec_device soc_codec_dev_ad183x;
#endif

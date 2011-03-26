/*
 * definitions for AD74111 registers
 *
 * Copyright 2006-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __AD74111_H__
#define __AD74111_H__

#define AD_READ			0x0000
#define AD_WRITE		0x8000

/* Control register A */
#define CTRL_REG_A		(0 << 11)

#define REGA_REFAMP		(1 << 2)
#define REGA_REF		(1 << 3)
#define REGA_DAC		(1 << 4)
#define REGA_ADC		(1 << 5)
#define REGA_ADC_INPAMP		(1 << 6)

/* Control register B */
#define CTRL_REG_B		(1 << 11)

#define REGB_FCLKDIV(x)		((x) & 0x3)
#define REGB_SCLKDIV(x)		(((x) & 0x3) << 2)
#define REGB_TCLKDIV(x)		(((x) & 0x3) << 4)

/* Control register C */
#define CTRL_REG_C		(2 << 11)

#define REGC_ADC_HP		(1 << 0)
#define REGC_DAC_DEEMPH(x)	(((x) & 0x3) << 1)
#define REGC_LG_DELAY		(1 << 3)
#define REGC_WORD_WIDTH(x)	(((x) & 0x3) << 4)

/* Control register D */
#define CTRL_REG_D		(3 << 11)

#define REGD_MASTER		(1 << 0)
#define REGD_FDCLK		(1 << 1)
#define REGD_DSP_MODE		(1 << 2)
#define REGD_MIX_MODE		(1 << 3)
#define REGD_MFS		(1 << 9)

/* Control register E */
#define CTRL_REG_E		(4 << 11)

#define REGE_DAC_MUTE		(1 << 0)
#define REGE_ADC_MUTE		(1 << 1)
#define REGE_ADC_GAIN(x)	(((x) & 0x7) << 2)
#define REGE_ADC_PEAKEN		(1 << 5)

/* Control register F */
#define CTRL_REG_F		(5 << 11)
#define REGF_DAC_VOL(x)		((x) & 0x3F)

/* Control register G */
#define CTRL_REG_G		(6 << 11)

#endif

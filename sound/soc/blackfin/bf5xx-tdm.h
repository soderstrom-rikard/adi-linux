/*
 * sound/soc/blackfin/bf5xx-tdm.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _BF5XX_TDM_H
#define _BF5XX_TDM_H

struct bf5xx_tdm_port {
	u16 tcr1;
	u16 rcr1;
	u16 tcr2;
	u16 rcr2;
	/* which slot used for the corresponding audio channel? */
	int slot_seq;
	int configured;
};

extern struct snd_soc_dai bf5xx_tdm_dai;

#endif

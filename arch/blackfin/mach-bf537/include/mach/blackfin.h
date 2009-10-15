/*
 * Copyright 2005-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later
 */

#ifndef _MACH_BLACKFIN_H_
#define _MACH_BLACKFIN_H_

#define BF537_FAMILY

#include "bf537.h"
#include "defBF534.h"
#include "anomaly.h"

#if defined(CONFIG_BF537) || defined(CONFIG_BF536)
#include "defBF537.h"
#endif

#if !defined(__ASSEMBLY__)
#include "cdefBF534.h"

#if defined(CONFIG_BF537) || defined(CONFIG_BF536)
#include "cdefBF537.h"
#endif
#endif

/* PLL_DIV Masks													*/
#define CCLK_DIV1 CSEL_DIV1	/*          CCLK = VCO / 1                                  */
#define CCLK_DIV2 CSEL_DIV2	/*          CCLK = VCO / 2                                  */
#define CCLK_DIV4 CSEL_DIV4	/*          CCLK = VCO / 4                                  */
#define CCLK_DIV8 CSEL_DIV8	/*          CCLK = VCO / 8                                  */

#endif

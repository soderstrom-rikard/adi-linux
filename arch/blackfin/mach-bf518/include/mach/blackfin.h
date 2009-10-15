/*
 * Copyright 2008-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later
 */

#ifndef _MACH_BLACKFIN_H_
#define _MACH_BLACKFIN_H_

#include "bf518.h"
#include "defBF512.h"
#include "anomaly.h"

#if defined(CONFIG_BF518)
#include "defBF518.h"
#endif

#if defined(CONFIG_BF516)
#include "defBF516.h"
#endif

#if defined(CONFIG_BF514)
#include "defBF514.h"
#endif

#if defined(CONFIG_BF512)
#include "defBF512.h"
#endif

#if !defined(__ASSEMBLY__)
#include "cdefBF512.h"

#if defined(CONFIG_BF518)
#include "cdefBF518.h"
#endif

#if defined(CONFIG_BF516)
#include "cdefBF516.h"
#endif

#if defined(CONFIG_BF514)
#include "cdefBF514.h"
#endif
#endif

/* PLL_DIV Masks													*/
#define CCLK_DIV1 CSEL_DIV1	/*          CCLK = VCO / 1                                  */
#define CCLK_DIV2 CSEL_DIV2	/*          CCLK = VCO / 2                                  */
#define CCLK_DIV4 CSEL_DIV4	/*          CCLK = VCO / 4                                  */
#define CCLK_DIV8 CSEL_DIV8	/*          CCLK = VCO / 8                                  */

#endif

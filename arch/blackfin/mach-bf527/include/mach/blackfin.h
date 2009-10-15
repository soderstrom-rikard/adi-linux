/*
 * Copyright 2007-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later
 */

#ifndef _MACH_BLACKFIN_H_
#define _MACH_BLACKFIN_H_

#include "bf527.h"
#include "defBF522.h"
#include "anomaly.h"

#if defined(CONFIG_BF527) || defined(CONFIG_BF526)
#include "defBF527.h"
#endif

#if defined(CONFIG_BF525) || defined(CONFIG_BF524)
#include "defBF525.h"
#endif

#if !defined(__ASSEMBLY__)
#include "cdefBF522.h"

#if defined(CONFIG_BF527) || defined(CONFIG_BF526)
#include "cdefBF527.h"
#endif

#if defined(CONFIG_BF525) || defined(CONFIG_BF524)
#include "cdefBF525.h"
#endif
#endif

/* PLL_DIV Masks													*/
#define CCLK_DIV1 CSEL_DIV1	/*          CCLK = VCO / 1                                  */
#define CCLK_DIV2 CSEL_DIV2	/*          CCLK = VCO / 2                                  */
#define CCLK_DIV4 CSEL_DIV4	/*          CCLK = VCO / 4                                  */
#define CCLK_DIV8 CSEL_DIV8	/*          CCLK = VCO / 8                                  */

#endif

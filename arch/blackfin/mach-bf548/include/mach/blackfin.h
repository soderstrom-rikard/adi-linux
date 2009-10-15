/*
 * Copyright 2007-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _MACH_BLACKFIN_H_
#define _MACH_BLACKFIN_H_

#include "bf548.h"
#include "anomaly.h"

#ifdef CONFIG_BF542
#include "defBF542.h"
#endif

#ifdef CONFIG_BF544
#include "defBF544.h"
#endif

#ifdef CONFIG_BF547
#include "defBF547.h"
#endif

#ifdef CONFIG_BF548
#include "defBF548.h"
#endif

#ifdef CONFIG_BF549
#include "defBF549.h"
#endif

#if !defined(__ASSEMBLY__)
#ifdef CONFIG_BF542
#include "cdefBF542.h"
#endif
#ifdef CONFIG_BF544
#include "cdefBF544.h"
#endif
#ifdef CONFIG_BF547
#include "cdefBF547.h"
#endif
#ifdef CONFIG_BF548
#include "cdefBF548.h"
#endif
#ifdef CONFIG_BF549
#include "cdefBF549.h"
#endif

#endif

/* PLL_DIV Masks */
#define CCLK_DIV1 CSEL_DIV1	/* CCLK = VCO / 1 */
#define CCLK_DIV2 CSEL_DIV2	/* CCLK = VCO / 2 */
#define CCLK_DIV4 CSEL_DIV4	/* CCLK = VCO / 4 */
#define CCLK_DIV8 CSEL_DIV8	/* CCLK = VCO / 8 */

#endif

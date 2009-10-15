/*
 * Copyright 2005-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _MACH_BLACKFIN_H_
#define _MACH_BLACKFIN_H_

#define BF561_FAMILY

#include "bf561.h"
#include "defBF561.h"
#include "anomaly.h"

#if !defined(__ASSEMBLY__)
#include "cdefBF561.h"
#endif

#define bfin_read_FIO_FLAG_D() bfin_read_FIO0_FLAG_D()
#define bfin_write_FIO_FLAG_D(val) bfin_write_FIO0_FLAG_D(val)
#define bfin_read_FIO_DIR() bfin_read_FIO0_DIR()
#define bfin_write_FIO_DIR(val) bfin_write_FIO0_DIR(val)
#define bfin_read_FIO_INEN() bfin_read_FIO0_INEN()
#define bfin_write_FIO_INEN(val) bfin_write_FIO0_INEN(val)

#define SIC_IWR0 SICA_IWR0
#define SIC_IWR1 SICA_IWR1
#define SIC_IAR0 SICA_IAR0
#define bfin_write_SIC_IMASK0 bfin_write_SICA_IMASK0
#define bfin_write_SIC_IMASK1 bfin_write_SICA_IMASK1
#define bfin_write_SIC_IWR0   bfin_write_SICA_IWR0
#define bfin_write_SIC_IWR1   bfin_write_SICA_IWR1

#define bfin_read_SIC_IMASK0 bfin_read_SICA_IMASK0
#define bfin_read_SIC_IMASK1 bfin_read_SICA_IMASK1
#define bfin_read_SIC_IWR0   bfin_read_SICA_IWR0
#define bfin_read_SIC_IWR1   bfin_read_SICA_IWR1
#define bfin_read_SIC_ISR0   bfin_read_SICA_ISR0
#define bfin_read_SIC_ISR1   bfin_read_SICA_ISR1

#define bfin_read_SIC_IMASK(x)		bfin_read32(SICA_IMASK0 + (x << 2))
#define bfin_write_SIC_IMASK(x, val)	bfin_write32((SICA_IMASK0 + (x << 2)), val)
#define bfin_read_SICB_IMASK(x)		bfin_read32(SICB_IMASK0 + (x << 2))
#define bfin_write_SICB_IMASK(x, val)	bfin_write32((SICB_IMASK0 + (x << 2)), val)
#define bfin_read_SIC_ISR(x)		bfin_read32(SICA_ISR0 + (x << 2))
#define bfin_write_SIC_ISR(x, val)	bfin_write32((SICA_ISR0 + (x << 2)), val)
#define bfin_read_SICB_ISR(x)		bfin_read32(SICB_ISR0 + (x << 2))
#define bfin_write_SICB_ISR(x, val)	bfin_write32((SICB_ISR0 + (x << 2)), val)

#endif				/* _MACH_BLACKFIN_H_ */

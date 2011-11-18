/*
 * Copyright 2004-2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _BFIN_IO_H
#define _BFIN_IO_H

#include <linux/compiler.h>
#include <linux/types.h>
#include <asm/byteorder.h>
#include <asm/blackfin.h>

#define DECLARE_BFIN_RAW_READX(size, type, asm, asm_sign) \
static inline type __raw_read##size(const volatile void __iomem *addr) \
{ \
	unsigned int val; \
	int tmp; \
	__asm__ __volatile__ ( \
		"cli %1;" \
		"NOP; NOP; SSYNC;" \
		"%0 = "#asm" [%2] "#asm_sign";" \
		"sti %1;" \
		: "=d"(val), "=d"(tmp) \
		: "a"(addr) \
	); \
	return (type) val; \
}
DECLARE_BFIN_RAW_READX(b, u8, b, (z))
#define __raw_readb __raw_readb
DECLARE_BFIN_RAW_READX(w, u16, w, (z))
#define __raw_readw __raw_readw
DECLARE_BFIN_RAW_READX(l, u32, , )
#define __raw_readl __raw_readl

#ifdef CONFIG_BF60x
#define DECLARE_BFIN_RAW_WRITEX(size, type) \
static inline void __raw_write##size(type val, volatile void __iomem *addr) \
{ \
	unsigned int tmp = 0; \
	unsigned int tmp_addr = (unsigned int)addr; \
	int is_smc = 0; \
	if ((tmp_addr & ~(ASYNC_BANK0_SIZE - 1)) == ASYNC_BANK0_BASE) { \
		tmp = bfin_read32(SMC_B0CTL); \
		bfin_write32(SMC_B0CTL, tmp | 0x2000); \
		SSYNC(); \
		is_smc = 1; \
	} \
	*(volatile type __force *) addr = (type) val; \
	if (is_smc) { \
		tmp = bfin_read32(SMC_B0CTL); \
		bfin_write32(SMC_B0CTL, tmp & ~0x2000); \
	} \
}
DECLARE_BFIN_RAW_WRITEX(b, u8)
#define __raw_writeb __raw_writeb
DECLARE_BFIN_RAW_WRITEX(w, u16)
#define __raw_writew __raw_writew
DECLARE_BFIN_RAW_WRITEX(l, u32)
#define __raw_writel __raw_writel
#endif

extern void outsb(unsigned long port, const void *addr, unsigned long count);
extern void outsw(unsigned long port, const void *addr, unsigned long count);
extern void outsw_8(unsigned long port, const void *addr, unsigned long count);
extern void outsl(unsigned long port, const void *addr, unsigned long count);
#define outsb outsb
#define outsw outsw
#define outsl outsl

extern void insb(unsigned long port, void *addr, unsigned long count);
extern void insw(unsigned long port, void *addr, unsigned long count);
extern void insw_8(unsigned long port, void *addr, unsigned long count);
extern void insl(unsigned long port, void *addr, unsigned long count);
extern void insl_16(unsigned long port, void *addr, unsigned long count);
#define insb insb
#define insw insw
#define insl insl

extern void dma_outsb(unsigned long port, const void *addr, unsigned short count);
extern void dma_outsw(unsigned long port, const void *addr, unsigned short count);
extern void dma_outsl(unsigned long port, const void *addr, unsigned short count);

extern void dma_insb(unsigned long port, void *addr, unsigned short count);
extern void dma_insw(unsigned long port, void *addr, unsigned short count);
extern void dma_insl(unsigned long port, void *addr, unsigned short count);

/**
 * I/O write barrier
 *
 * Ensure ordering of I/O space writes. This will make sure that writes
 * following the barrier will arrive after all previous writes.
 */
#define mmiowb() do { SSYNC(); wmb(); } while (0)

#include <asm-generic/io.h>

#endif

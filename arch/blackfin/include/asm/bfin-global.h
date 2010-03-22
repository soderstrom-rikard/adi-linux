/*
 * Global extern defines for blackfin
 *
 * Copyright 2006-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _BFIN_GLOBAL_H_
#define _BFIN_GLOBAL_H_

#ifndef __ASSEMBLY__

#include <linux/linkage.h>
#include <linux/types.h>

#if defined(CONFIG_DMA_UNCACHED_4M)
# define DMA_UNCACHED_REGION (4 * 1024 * 1024)
#elif defined(CONFIG_DMA_UNCACHED_2M)
# define DMA_UNCACHED_REGION (2 * 1024 * 1024)
#elif defined(CONFIG_DMA_UNCACHED_1M)
# define DMA_UNCACHED_REGION (1024 * 1024)
#elif defined(CONFIG_DMA_UNCACHED_512K)
# define DMA_UNCACHED_REGION (512 * 1024)
#elif defined(CONFIG_DMA_UNCACHED_256K)
# define DMA_UNCACHED_REGION (256 * 1024)
#elif defined(CONFIG_DMA_UNCACHED_128K)
# define DMA_UNCACHED_REGION (128 * 1024)
#else
# define DMA_UNCACHED_REGION (0)
#endif

extern void bfin_setup_caches(unsigned int cpu);
extern void bfin_setup_cpudata(unsigned int cpu);

extern unsigned long get_cclk(void);
extern unsigned long get_sclk(void);
extern unsigned long sclk_to_usecs(unsigned long sclk);
extern unsigned long usecs_to_sclk(unsigned long usecs);

struct pt_regs;
#if defined(CONFIG_DEBUG_VERBOSE)
extern void dump_bfin_process(struct pt_regs *regs);
extern void dump_bfin_mem(struct pt_regs *regs);
extern void dump_bfin_trace_buffer(void);
#else
#define dump_bfin_process(regs)
#define dump_bfin_mem(regs)
#define dump_bfin_trace_buffer()
#endif

/* init functions only */
extern int init_arch_irq(void);
extern void init_exception_vectors(void);
extern void program_IAR(void);

extern asmlinkage void lower_to_irq14(void);
extern asmlinkage void bfin_return_from_exception(void);
extern asmlinkage void asm_do_IRQ(unsigned int irq, struct pt_regs *regs);
extern int bfin_internal_set_wake(unsigned int irq, unsigned int state);

extern void *l1_data_A_sram_alloc(size_t);
extern void *l1_data_B_sram_alloc(size_t);
extern void *l1_inst_sram_alloc(size_t);
extern void *l1_data_sram_alloc(size_t);
extern void *l1_data_sram_zalloc(size_t);
extern void *l2_sram_alloc(size_t);
extern void *l2_sram_zalloc(size_t);
extern int l1_data_A_sram_free(const void*);
extern int l1_data_B_sram_free(const void*);
extern int l1_inst_sram_free(const void*);
extern int l1_data_sram_free(const void*);
extern int l2_sram_free(const void *);
extern int sram_free(const void*);

#define L1_INST_SRAM		0x00000001
#define L1_DATA_A_SRAM		0x00000002
#define L1_DATA_B_SRAM		0x00000004
#define L1_DATA_SRAM		0x00000006
#define L2_SRAM			0x00000008
extern void *sram_alloc_with_lsl(size_t, unsigned long);
extern int sram_free_with_lsl(const void*);

extern void *isram_memcpy(void *dest, const void *src, size_t n);

extern const char bfin_board_name[];

extern unsigned long bfin_sic_iwr[];
extern unsigned vr_wakeup;
extern u16 _bfin_swrst; /* shadow for Software Reset Register (SWRST) */

#include <asm/irq.h>

#define SUPPLE_0_WAKEUP ((IRQ_SUPPLE_0 - (IRQ_CORETMR + 1)) % 32)

static inline void bfin_iwr_set_pll(unsigned long *iwr0,
			unsigned long *iwr1, unsigned long *iwr2)
{
#ifdef CONFIG_SMP
	unsigned long SICA_SICB_OFF =
			((bfin_read_DSPID() & 0xff) ? 0x1000 : 0);
#else
# define SICA_SICB_OFF 0
#endif

#ifdef SIC_IWR0
	*iwr0 = bfin_read32(SIC_IWR0 + SICA_SICB_OFF);
#ifdef SIC_IWR1
	*iwr1 = bfin_read32(SIC_IWR1 + SICA_SICB_OFF);
#ifdef SIC_IWR2
	*iwr2 = bfin_read32(SIC_IWR2);
	bfin_write32(SIC_IWR2, 0);
#endif
	bfin_write32(SIC_IWR1 + SICA_SICB_OFF, 0);
#endif
	bfin_write32(SIC_IWR0 + SICA_SICB_OFF, IWR_ENABLE(0));
#else
	*iwr0 = bfin_read32(SIC_IWR);
	bfin_write32(SIC_IWR, IWR_ENABLE(0));
#endif
}

#if defined(CONFIG_HOTPLUG_CPU) || \
	(defined(CONFIG_CPU_VOLTAGE) && defined(CONFIG_SMP))
static inline void bfin_iwr_set_sup0(unsigned long *iwr0,
			unsigned long *iwr1, unsigned long *iwr2)
{
#ifdef CONFIG_SMP
	unsigned long SICA_SICB_OFF =
			((bfin_read_DSPID() & 0xff) ? 0x1000 : 0);
#else
# define SICA_SICB_OFF 0
#endif
	*iwr0 = bfin_read32(SIC_IWR0 + SICA_SICB_OFF);
	*iwr1 = bfin_read32(SIC_IWR1 + SICA_SICB_OFF);
	bfin_write32(SIC_IWR0 + SICA_SICB_OFF, 0);
	bfin_write32(SIC_IWR1 + SICA_SICB_OFF, IWR_ENABLE(SUPPLE_0_WAKEUP));
}
#endif

static inline void bfin_iwr_restore(unsigned long iwr0,
			unsigned long iwr1, unsigned long iwr2)
{
#ifdef CONFIG_SMP
	unsigned long SICA_SICB_OFF =
			((bfin_read_DSPID() & 0xff) ? 0x1000 : 0);
#else
# define SICA_SICB_OFF 0
#endif

#ifdef SIC_IWR0
	bfin_write32(SIC_IWR0 + SICA_SICB_OFF, iwr0);
#ifdef SIC_IWR1
	bfin_write32(SIC_IWR1 + SICA_SICB_OFF, iwr1);
#ifdef SIC_IWR2
	bfin_write32(SIC_IWR2, iwr2);
#endif
#endif
#else
	bfin_write32(SIC_IWR, iwr0);
#endif
}

/* Writing to PLL_CTL initiates a PLL relock sequence. */
static inline void bfin_write_PLL_CTL(unsigned int val)
{
	unsigned long flags = 0;
	unsigned long iwr0, iwr1, iwr2;

	if (val == bfin_read_PLL_CTL())
		return;

	local_irq_save_hw(flags);
	bfin_iwr_set_pll(&iwr0, &iwr1, &iwr2);

	bfin_write16(PLL_CTL, val);
	SSYNC();
	asm("IDLE;");

	bfin_iwr_restore(iwr0, iwr1, iwr2);
	local_irq_restore_hw(flags);
}

/* Writing to VR_CTL initiates a PLL relock sequence. */
static inline void bfin_write_VR_CTL(unsigned int val)
{
	unsigned long flags = 0;
	unsigned long iwr0, iwr1, iwr2;

	if (val == bfin_read_VR_CTL())
		return;

	local_irq_save_hw(flags);
	bfin_iwr_set_pll(&iwr0, &iwr1, &iwr2);

	bfin_write16(VR_CTL, val);
	SSYNC();
	asm("IDLE;");

	bfin_iwr_restore(iwr0, iwr1, iwr2);
	local_irq_restore_hw(flags);
}


#ifdef BF533_FAMILY

#if ANOMALY_05000311
#define BFIN_WRITE_FIO_FLAG(name) \
static inline void bfin_write_FIO_FLAG_##name(unsigned short val) \
{ \
	unsigned long flags; \
	local_irq_save_hw(flags); \
	bfin_write16(FIO_FLAG_##name, val); \
	bfin_read_CHIPID(); \
	local_irq_restore_hw(flags); \
}
BFIN_WRITE_FIO_FLAG(D)
BFIN_WRITE_FIO_FLAG(C)
BFIN_WRITE_FIO_FLAG(S)
BFIN_WRITE_FIO_FLAG(T)

#define BFIN_READ_FIO_FLAG(name) \
static inline u16 bfin_read_FIO_FLAG_##name(void) \
{ \
	unsigned long flags; \
	u16 ret; \
	local_irq_save_hw(flags); \
	ret = bfin_read16(FIO_FLAG_##name); \
	bfin_read_CHIPID(); \
	local_irq_restore_hw(flags); \
	return ret; \
}
BFIN_READ_FIO_FLAG(D)
BFIN_READ_FIO_FLAG(C)
BFIN_READ_FIO_FLAG(S)
BFIN_READ_FIO_FLAG(T)

#else
#define bfin_write_FIO_FLAG_D(val)           bfin_write16(FIO_FLAG_D, val)
#define bfin_write_FIO_FLAG_C(val)           bfin_write16(FIO_FLAG_C, val)
#define bfin_write_FIO_FLAG_S(val)           bfin_write16(FIO_FLAG_S, val)
#define bfin_write_FIO_FLAG_T(val)           bfin_write16(FIO_FLAG_T, val)
#define bfin_read_FIO_FLAG_T()               bfin_read16(FIO_FLAG_T)
#define bfin_read_FIO_FLAG_C()               bfin_read16(FIO_FLAG_C)
#define bfin_read_FIO_FLAG_S()               bfin_read16(FIO_FLAG_S)
#define bfin_read_FIO_FLAG_D()               bfin_read16(FIO_FLAG_D)
#endif				/* ANOMALY_05000311 */

#endif				/* BF533_FAMILY */

#endif				/* !__ASSEMBLY__ */

#endif				/* _BLACKFIN_H_ */

/*
 * Copyright 2005-2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _MACH_COMMON_PLL_H
#define _MACH_COMMON_PLL_H

#include <asm/blackfin.h>
#include <asm/irqflags.h>

#ifndef bfin_iwr_restore
static inline void
bfin_iwr_restore(unsigned long iwr0, unsigned long iwr1, unsigned long iwr2)
{
#ifdef SIC_IWR
	bfin_write_SIC_IWR(iwr0);
#else
	bfin_write_SIC_IWR0(iwr0);
# ifdef SIC_IWR1
	bfin_write_SIC_IWR1(iwr1);
# endif
# ifdef SIC_IWR2
	bfin_write_SIC_IWR2(iwr2);
# endif
#endif
}
#endif

#ifndef bfin_iwr_save
static inline void
bfin_iwr_save(unsigned long niwr0, unsigned long niwr1, unsigned long niwr2,
              unsigned long *iwr0, unsigned long *iwr1, unsigned long *iwr2)
{
#ifdef SIC_IWR
	*iwr0 = bfin_read_SIC_IWR();
#else
	*iwr0 = bfin_read_SIC_IWR0();
# ifdef SIC_IWR1
	*iwr1 = bfin_read_SIC_IWR1();
# endif
# ifdef SIC_IWR2
	*iwr2 = bfin_read_SIC_IWR2();
# endif
#endif
	bfin_iwr_restore(niwr0, niwr1, niwr2);
}
#endif

/* Writing to PLL_CTL initiates a PLL relock sequence */
static inline void bfin_write_PLL_CTL(unsigned int val)
{
	unsigned long flags, iwr0, iwr1, iwr2;

	if (val == bfin_read_PLL_CTL())
		return;

	local_irq_save_hw(flags);
	/* Enable the PLL Wakeup bit in SIC IWR */
	bfin_iwr_save(IWR_ENABLE(0), 0, 0, &iwr0, &iwr1, &iwr2);

	bfin_write_PLL_CTL(val);
	SSYNC();
	asm("IDLE;");

	bfin_iwr_restore(iwr0, iwr1, iwr2);
	local_irq_restore_hw(flags);
}

/* Writing to VR_CTL initiates a PLL relock sequence */
static inline void bfin_write_VR_CTL(unsigned int val)
{
	unsigned long flags, iwr0, iwr1, iwr2;

	if (val == bfin_read_VR_CTL())
		return;

	local_irq_save_hw(flags);
	/* Enable the PLL Wakeup bit in SIC IWR */
	bfin_iwr_save(IWR_ENABLE(0), 0, 0, &iwr0, &iwr1, &iwr2);

	bfin_write_VR_CTL(val);
	SSYNC();
	asm("IDLE;");

	bfin_iwr_restore(iwr0, iwr1, iwr2);
	local_irq_restore_hw(flags);
}

#endif

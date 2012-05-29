/*
 * arch/blackfin/mach-common/clocks-init.c - reprogram clocks / memory
 *
 * Copyright 2004-2008 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/blackfin.h>

#include <asm/dma.h>
#include <asm/clocks.h>
#include <asm/mem_init.h>
#include <asm/dpmc.h>

__attribute__((l1_text))
static void do_sync(void)
{
	__builtin_bfin_ssync();
}

__attribute__((l1_text))
void init_clocks(void)
{
	/* Kill any active DMAs as they may trigger external memory accesses
	 * in the middle of reprogramming things, and that'll screw us up.
	 * For example, any automatic DMAs left by U-Boot for splash screens.
	 */

#ifdef CONFIG_BF60x
	int i, dlldatacycle, dll_ctl;

	if (bfin_read_DMC0_STAT() & MEMINITDONE) {
		bfin_write_DMC0_CTL(bfin_read_DMC0_CTL() | SRREQ);
		do_sync();
		while (!(bfin_read_DMC0_STAT() & SRACK))
			continue;
	}

	/* Don't set the same value of MSEL and DF to CGU_CTL */
	if ((bfin_read32(CGU0_CTL) & (MSEL_MASK | DF_MASK))
		!= CGU_CTL_VAL) {
		bfin_write32(CGU0_DIV, CGU_DIV_VAL);
		bfin_write32(CGU0_CTL, CGU_CTL_VAL);
		while ((bfin_read32(CGU0_STAT) & (CLKSALGN | PLLBP)) ||
			!(bfin_read32(CGU0_STAT) & PLOCK))
			continue;
	}

	bfin_write32(CGU0_DIV, CGU_DIV_VAL | UPDT);
	while (bfin_read32(CGU0_STAT) & CLKSALGN)
		continue;

	if (bfin_read_DMC0_STAT() & MEMINITDONE) {
		bfin_write_DMC0_CTL(bfin_read_DMC0_CTL() & ~SRREQ);
		do_sync();
		while (bfin_read_DMC0_STAT() & SRACK)
			continue;
	}

	for (i = 0; i < 7; i++) {
		if (ddr_config_table[i].ddr_clk == CONFIG_BFIN_DCLK) {
			bfin_write_DMC0_CFG(ddr_config_table[i].dmc_ddrcfg);
			bfin_write_DMC0_TR0(ddr_config_table[i].dmc_ddrtr0);
			bfin_write_DMC0_TR1(ddr_config_table[i].dmc_ddrtr1);
			bfin_write_DMC0_TR2(ddr_config_table[i].dmc_ddrtr2);
			bfin_write_DMC0_MR(ddr_config_table[i].dmc_ddrmr);
			bfin_write_DMC0_EMR1(ddr_config_table[i].dmc_ddrmr1);
			bfin_write_DMC0_CTL(ddr_config_table[i].dmc_ddrctl);
			break;
		}
	}

	do_sync();
	while (!(bfin_read_DMC0_STAT() & MEMINITDONE))
		continue;

	dlldatacycle = (bfin_read_DMC0_STAT() & PHYRDPHASE) >> PHYRDPHASE_OFFSET;
	dll_ctl = bfin_read_DMC0_DLLCTL();
	dll_ctl &= ~DATACYC;
	bfin_write_DMC0_DLLCTL(dll_ctl | (dlldatacycle << DATACYC_OFFSET));

	do_sync();
	while (!(bfin_read_DMC0_STAT() & DLLCALDONE))
		continue;
#else
	size_t i;
	for (i = 0; i < MAX_DMA_CHANNELS; ++i) {
		struct dma_register *dma = dma_io_base_addr[i];
		dma->cfg = 0;
	}

	do_sync();

#ifdef SIC_IWR0
	bfin_write_SIC_IWR0(IWR_ENABLE(0));
# ifdef SIC_IWR1
	/* BF52x system reset does not properly reset SIC_IWR1 which
	 * will screw up the bootrom as it relies on MDMA0/1 waking it
	 * up from IDLE instructions.  See this report for more info:
	 * http://blackfin.uclinux.org/gf/tracker/4323
	 */
	if (ANOMALY_05000435)
		bfin_write_SIC_IWR1(IWR_ENABLE(10) | IWR_ENABLE(11));
	else
		bfin_write_SIC_IWR1(IWR_DISABLE_ALL);
# endif
# ifdef SIC_IWR2
	bfin_write_SIC_IWR2(IWR_DISABLE_ALL);
# endif
#else
	bfin_write_SIC_IWR(IWR_ENABLE(0));
#endif
	do_sync();
#ifdef EBIU_SDGCTL
	bfin_write_EBIU_SDGCTL(bfin_read_EBIU_SDGCTL() | SRFS);
	do_sync();
#endif

#ifdef CLKBUFOE
	bfin_write16(VR_CTL, bfin_read_VR_CTL() | CLKBUFOE);
	do_sync();
	__asm__ __volatile__("IDLE;");
#endif
	bfin_write_PLL_LOCKCNT(0x300);
	do_sync();
	/* We always write PLL_CTL thus avoiding Anomaly 05000242 */
	bfin_write16(PLL_CTL, PLL_CTL_VAL);
	__asm__ __volatile__("IDLE;");
	bfin_write_PLL_DIV(CONFIG_CCLK_ACT_DIV | CONFIG_SCLK_DIV);
#ifdef EBIU_SDGCTL
	bfin_write_EBIU_SDRRC(mem_SDRRC);
	bfin_write_EBIU_SDGCTL((bfin_read_EBIU_SDGCTL() & SDGCTL_WIDTH) | mem_SDGCTL);
#else
	bfin_write_EBIU_RSTCTL(bfin_read_EBIU_RSTCTL() & ~(SRREQ));
	do_sync();
	bfin_write_EBIU_RSTCTL(bfin_read_EBIU_RSTCTL() | 0x1);
	bfin_write_EBIU_DDRCTL0(mem_DDRCTL0);
	bfin_write_EBIU_DDRCTL1(mem_DDRCTL1);
	bfin_write_EBIU_DDRCTL2(mem_DDRCTL2);
#ifdef CONFIG_MEM_EBIU_DDRQUE
	bfin_write_EBIU_DDRQUE(CONFIG_MEM_EBIU_DDRQUE);
#endif
#endif
#endif
	do_sync();
	bfin_read16(0);

}

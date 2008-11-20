/*
 * File:         arch/blackfin/mach-bf537/head.S
 * Based on:     arch/blackfin/mach-bf533/head.S
 * Author:       Jeff Dionne <jeff@uclinux.org> COPYRIGHT 1998 D. Jeff Dionne
 *
 * Created:      1998
 * Description:  Startup code for Blackfin BF537
 *
 * Modified:
 *               Copyright 2004-2006 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/blackfin.h>

#include <asm/dma.h>
#include <asm/clocks.h>
#include <asm/mem_init.h>

#ifdef ANOMALY_05000265 /* Add 250 mV of hysteresis to SPORT input pins */
#define PLL_CTL_VAL (((CONFIG_VCO_MULT & 63) << 9) | CLKIN_HALF | (PLL_BYPASS << 8) | 0x8000)
#else
#define PLL_CTL_VAL (((CONFIG_VCO_MULT & 63) << 9) | CLKIN_HALF | (PLL_BYPASS << 8))
#endif

__attribute__((l1_text))
void do_sync(void)
{
	__asm__ __volatile__("NOP;NOP;NOP;SSYNC;");
}

__attribute__((l1_text))
void init_clocks(void)
{
	int i;
	struct dma_register *dma;

	for (i = 0; i < MAX_BLACKFIN_DMA_CHANNEL; i++) {
		dma = dma_io_base_addr[i];
		dma->cfg = 0;
	}

	do_sync();

#if defined(CONFIG_BF54x) || defined(CONFIG_BF52x)  || defined(CONFIG_BF561) || \
	defined(CONFIG_BF538) || defined(CONFIG_BF539) || defined(CONFIG_BF51x)
	bfin_write_SIC_IWR0(IWR_ENABLE(0));
#if defined(CONFIG_BF52x) || defined(CONFIG_BF51x)
	/* BF52x system reset does not properly reset SIC_IWR1 which
	 * will screw up the bootrom as it relies on MDMA0/1 waking it
	 * up from IDLE instructions.  See this report for more info:
	 * http://blackfin.uclinux.org/gf/tracker/4323
	 */
	if (ANOMALY_05000435)
		bfin_write_SIC_IWR1(IWR_ENABLE(10) | IWR_ENABLE(11));
	else
		bfin_write_SIC_IWR1(IWR_DISABLE_ALL);
#else
	bfin_write_SIC_IWR1(IWR_DISABLE_ALL);
#endif
# ifdef CONFIG_BF54x
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
	bfin_write16(PLL_CTL, PLL_CTL_VAL);
	__asm__ __volatile__("IDLE;");
	bfin_write_PLL_DIV(CONFIG_CCLK_ACT_DIV | CONFIG_SCLK_DIV);
#ifdef EBIU_SDGCTL
	bfin_write_EBIU_SDRRC(mem_SDRRC);
	bfin_write_EBIU_SDGCTL(mem_SDGCTL);
#else
	bfin_write_EBIU_RSTCTL(bfin_read_EBIU_RSTCTL() & ~(SRREQ));
	do_sync();
	bfin_write_EBIU_RSTCTL(bfin_read_EBIU_RSTCTL() | 0x1);
	bfin_write_EBIU_DDRCTL0(mem_DDRCTL0);
	bfin_write_EBIU_DDRCTL1(mem_DDRCTL1);
	bfin_write_EBIU_DDRCTL2(mem_DDRCTL2);
#endif
	do_sync();
	bfin_read16(0);
}

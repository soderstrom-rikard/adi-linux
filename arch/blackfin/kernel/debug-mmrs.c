/*
 * debugfs interface to core/system MMRs
 *
 * Copyright 2007-2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later
 */

#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <asm/blackfin.h>
#include <asm/bfin_can.h>
#include <asm/bfin_dma.h>
#include <asm/bfin_ppi.h>
#include <asm/bfin_serial.h>
#include <asm/bfin5xx_spi.h>
#include <asm/bfin_twi.h>

#define _d(name, bits, addr, perms) debugfs_create_x##bits(name, perms, parent, (u##bits *)addr)
#define d(name, bits, addr)         _d(name, bits, addr, S_IRUSR|S_IWUSR)
#define d_RO(name, bits, addr)      _d(name, bits, addr, S_IRUSR)
#define d_WO(name, bits, addr)      _d(name, bits, addr, S_IWUSR)

#define D_RO(name, bits) d_RO(#name, bits, name)
#define D_WO(name, bits) d_WO(#name, bits, name)
#define D32(name)        d(#name, 32, name)
#define D16(name)        d(#name, 16, name)

#define REGS_OFF(peri, mmr) offsetof(struct bfin_##peri##_regs, mmr)
#define __REGS(peri, sname, rname) \
	do { \
		struct bfin_##peri##_regs r; \
		void *addr = (void *)(base + REGS_OFF(peri, rname)); \
		strcpy(_buf, sname); \
		if (sizeof(r.rname) == 2) \
			debugfs_create_x16(buf, S_IRUSR|S_IWUSR, parent, addr); \
		else \
			debugfs_create_x32(buf, S_IRUSR|S_IWUSR, parent, addr); \
	} while (0)
#define REGS_STR_PFX(buf, pfx, num) \
	({ \
		buf + (num >= 0 ? \
			sprintf(buf, #pfx "%i_", num) : \
			sprintf(buf, #pfx "_")); \
	})

/*
 * Core registers (not memory mapped)
 */
extern u32 last_seqstat;

static int debug_cclk_get(void *data, u64 *val)
{
	*val = get_cclk();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_debug_cclk, debug_cclk_get, NULL, "0x%08llx\n");

static int debug_sclk_get(void *data, u64 *val)
{
	*val = get_sclk();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_debug_sclk, debug_sclk_get, NULL, "0x%08llx\n");

#define DEFINE_SYSREG(sr, pre, post) \
static int sysreg_##sr##_get(void *data, u64 *val) \
{ \
	unsigned long tmp; \
	pre; \
	__asm__ __volatile__("%0 = " #sr ";" : "=d"(tmp)); \
	*val = tmp; \
	return 0; \
} \
static int sysreg_##sr##_set(void *data, u64 val) \
{ \
	unsigned long tmp = val; \
	__asm__ __volatile__(#sr " = %0;" : : "d"(tmp)); \
	post; \
	return 0; \
} \
DEFINE_SIMPLE_ATTRIBUTE(fops_sysreg_##sr, sysreg_##sr##_get, sysreg_##sr##_set, "0x%08llx\n")

DEFINE_SYSREG(cycles, , );
DEFINE_SYSREG(cycles2, __asm__ __volatile__("%0 = cycles;" : "=d"(tmp)), );
DEFINE_SYSREG(emudat, , );
DEFINE_SYSREG(seqstat, , );
DEFINE_SYSREG(syscfg, , CSYNC());
#define D_SYSREG(sr) debugfs_create_file(#sr, S_IRUSR|S_IWUSR, parent, NULL, &fops_sysreg_##sr)

/*
 * CAN
 */
#define CAN_OFF(mmr)  REGS_OFF(can, mmr)
#define __CAN(uname, lname) __REGS(can, #uname, lname)
static void __init __maybe_unused
bfin_debug_mmrs_can(struct dentry *parent, unsigned long base, int num)
{
	static struct dentry *am, *mb;
	int i, j;
	char buf[32], *_buf = REGS_STR_PFX(buf, CAN, num);

	if (!am) {
		am = debugfs_create_dir("am", parent);
		mb = debugfs_create_dir("mb", parent);
	}

	__CAN(MC1, mc1);
	__CAN(MD1, md1);
	__CAN(TRS1, trs1);
	__CAN(TRR1, trr1);
	__CAN(TA1, ta1);
	__CAN(AA1, aa1);
	__CAN(RMP1, rmp1);
	__CAN(RML1, rml1);
	__CAN(MBTIF1, mbtif1);
	__CAN(MBRIF1, mbrif1);
	__CAN(MBIM1, mbim1);
	__CAN(RFH1, rfh1);
	__CAN(OPSS1, opss1);

	__CAN(MC2, mc2);
	__CAN(MD2, md2);
	__CAN(TRS2, trs2);
	__CAN(TRR2, trr2);
	__CAN(TA2, ta2);
	__CAN(AA2, aa2);
	__CAN(RMP2, rmp2);
	__CAN(RML2, rml2);
	__CAN(MBTIF2, mbtif2);
	__CAN(MBRIF2, mbrif2);
	__CAN(MBIM2, mbim2);
	__CAN(RFH2, rfh2);
	__CAN(OPSS2, opss2);

	__CAN(CLOCK, clock);
	__CAN(TIMING, timing);
	__CAN(DEBUG, debug);
	__CAN(STATUS, status);
	__CAN(CEC, cec);
	__CAN(GIS, gis);
	__CAN(GIM, gim);
	__CAN(GIF, gif);
	__CAN(CONTROL, control);
	__CAN(INTR, intr);
	__CAN(VERSION, version);
	__CAN(MBTD, mbtd);
	__CAN(EWR, ewr);
	__CAN(ESR, esr);
	/*__CAN(UCREG, ucreg); no longer exists */
	__CAN(UCCNT, uccnt);
	__CAN(UCRC, ucrc);
	__CAN(UCCNF, uccnf);
	__CAN(VERSION2, version2);

	for (i = 0; i < 32; ++i) {
		sprintf(_buf, "AM%02iL", i);
		debugfs_create_x16(buf, S_IRUSR|S_IWUSR, am,
			(u16 *)(base + CAN_OFF(msk[i].aml)));
		sprintf(_buf, "AM%02iH", i);
		debugfs_create_x16(buf, S_IRUSR|S_IWUSR, am,
			(u16 *)(base + CAN_OFF(msk[i].amh)));

		for (j = 0; j < 3; ++j) {
			sprintf(_buf, "MB%02i_DATA%i", i, j);
			debugfs_create_x16(buf, S_IRUSR|S_IWUSR, mb,
				(u16 *)(base + CAN_OFF(chl[i].data[j*2])));
		}
		sprintf(_buf, "MB%02i_LENGTH", i);
		debugfs_create_x16(buf, S_IRUSR|S_IWUSR, mb,
			(u16 *)(base + CAN_OFF(chl[i].dlc)));
		sprintf(_buf, "MB%02i_TIMESTAMP", i);
		debugfs_create_x16(buf, S_IRUSR|S_IWUSR, mb,
			(u16 *)(base + CAN_OFF(chl[i].tsv)));
		sprintf(_buf, "MB%02i_ID0", i);
		debugfs_create_x16(buf, S_IRUSR|S_IWUSR, mb,
			(u16 *)(base + CAN_OFF(chl[i].id0)));
		sprintf(_buf, "MB%02i_ID1", i);
		debugfs_create_x16(buf, S_IRUSR|S_IWUSR, mb,
			(u16 *)(base + CAN_OFF(chl[i].id1)));
	}
}
#define CAN(num) bfin_debug_mmrs_can(parent, CAN##num##_MC1, num)

/*
 * DMA
 */
#define __DMA(uname, lname) __REGS(dma, #uname, lname)
static void __init __maybe_unused
bfin_debug_mmrs_dma(struct dentry *parent, unsigned long base, int num, char mdma, const char *pfx)
{
	char buf[32], *_buf;

	if (mdma)
		_buf = buf + sprintf(buf, "%s_%c%i_", pfx, mdma, num);
	else
		_buf = buf + sprintf(buf, "%s%i_", pfx, num);

	__DMA(NEXT_DESC_PTR, next_desc_ptr);
	__DMA(START_ADDR, start_addr);
	__DMA(CONFIG, config);
	__DMA(X_COUNT, x_count);
	__DMA(X_MODIFY, x_modify);
	__DMA(Y_COUNT, y_count);
	__DMA(Y_MODIFY, y_modify);
	__DMA(CURR_DESC_PTR, curr_desc_ptr);
	__DMA(CURR_ADDR, curr_addr);
	__DMA(IRQ_STATUS, irq_status);
	__DMA(PERIPHERAL_MAP, peripheral_map);
	__DMA(CURR_X_COUNT, curr_x_count);
	__DMA(CURR_Y_COUNT, curr_y_count);
}
#define _DMA(num, base, mdma, pfx) bfin_debug_mmrs_dma(parent, base, num, mdma, pfx "DMA")
#define DMA(num)  _DMA(num, DMA##num##_NEXT_DESC_PTR, 0, "")
#define _MDMA(num, x) \
	do { \
		_DMA(num, x##MDMA_D##num##_CONFIG, 'D', #x); \
		_DMA(num, x##MDMA_S##num##_CONFIG, 'S', #x); \
	} while (0)
#define MDMA(num) _MDMA(num, )
#define IMDMA(num) _MDMA(num, I)

/*
 * EPPI
 */
#define __EPPI(uname, lname) __REGS(eppi, #uname, lname)
static void __init __maybe_unused
bfin_debug_mmrs_eppi(struct dentry *parent, unsigned long base, int num)
{
	char buf[32], *_buf = REGS_STR_PFX(buf, EPPI, num);
	__EPPI(STATUS, status);
	__EPPI(HCOUNT, hcount);
	__EPPI(HDELAY, hdelay);
	__EPPI(VCOUNT, vcount);
	__EPPI(VDELAY, vdelay);
	__EPPI(FRAME, frame);
	__EPPI(LINE, line);
	__EPPI(CLKDIV, clkdiv);
	__EPPI(CONTROL, control);
	__EPPI(FS1W_HBL, fs1w_hbl);
	__EPPI(FS1P_AVPL, fs1p_avpl);
	__EPPI(FS2W_LVB, fs2w_lvb);
	__EPPI(FS2P_LAVF, fs2p_lavf);
	__EPPI(CLIP, clip);
}
#define EPPI(num) bfin_debug_mmrs_eppi(parent, EPPI##num##_STATUS, num)

/*
 * General Purpose Timers
 */
#define GPTIMER_OFF(mmr) (TIMER0_##mmr - TIMER0_CONFIG)
#define __GPTIMER(name) \
	do { \
		strcpy(_buf, #name); \
		debugfs_create_x16(buf, S_IRUSR|S_IWUSR, parent, (u16 *)(base + GPTIMER_OFF(name))); \
	} while (0)
static void __init __maybe_unused
bfin_debug_mmrs_gptimer(struct dentry *parent, unsigned long base, int num)
{
	char buf[32], *_buf = REGS_STR_PFX(buf, TIMER, num);
	__GPTIMER(CONFIG);
	__GPTIMER(COUNTER);
	__GPTIMER(PERIOD);
	__GPTIMER(WIDTH);
}
#define GPTIMER(num) bfin_debug_mmrs_gptimer(parent, TIMER##num##_CONFIG, num)

/*
 * Handshake MDMA
 */
#define __HMDMA(uname, lname) __REGS(hmdma, #uname, lname)
static void __init __maybe_unused
bfin_debug_mmrs_hmdma(struct dentry *parent, unsigned long base, int num)
{
	char buf[32], *_buf = REGS_STR_PFX(buf, HMDMA, num);
	__HMDMA(CONTROL, control);
	__HMDMA(ECINIT, ecinit);
	__HMDMA(BCINIT, bcinit);
	__HMDMA(ECURGENT, ecurgent);
	__HMDMA(ECOVERFLOW, ecoverflow);
	__HMDMA(ECOUNT, ecount);
	__HMDMA(BCOUNT, bcount);
}
#define HMDMA(num) bfin_debug_mmrs_hmdma(parent, HMDMA##num##_CONTROL, num)

/*
 * PPI
 */
#define __PPI(uname, lname) __REGS(ppi, #uname, lname)
static void __init __maybe_unused
bfin_debug_mmrs_ppi(struct dentry *parent, unsigned long base, int num)
{
	char buf[32], *_buf = REGS_STR_PFX(buf, PPI, num);
	__PPI(CONTROL, control);
	__PPI(STATUS, status);
	__PPI(COUNT, count);
	__PPI(DELAY, delay);
	__PPI(FRAME, frame);
}
#define PPI(num) bfin_debug_mmrs_ppi(parent, PPI##num##_STATUS, num)

/*
 * SPI
 */
#define __SPI(uname, lname) __REGS(spi, #uname, lname)
static void __init __maybe_unused
bfin_debug_mmrs_spi(struct dentry *parent, unsigned long base, int num)
{
	char buf[32], *_buf = REGS_STR_PFX(buf, SPI, num);
	__SPI(CTL, ctl);
	__SPI(FLG, flg);
	__SPI(STAT, stat);
	__SPI(TDBR, tdbr);
	__SPI(RDBR, rdbr);
	__SPI(BAUD, baud);
	__SPI(SHADOW, shadow);
}
#define SPI(num) bfin_debug_mmrs_spi(parent, SPI##num##_REGBASE, num)

/*
 * SPORT
 */
static inline int sport_width(void *mmr)
{
	unsigned long lmmr = (unsigned long)mmr;
	if ((lmmr & 0xff) == 0x10)
		/* SPORT#_TX has 0x10 offset -> SPORT#_TCR2 has 0x04 offset */
		lmmr -= 0xc;
	else
		/* SPORT#_RX has 0x18 offset -> SPORT#_RCR2 has 0x24 offset */
		lmmr += 0xc;
	/* extract SLEN field from control register 2 and add 1 */
	return (bfin_read16(lmmr) & 0x1f) + 1;
}
static int sport_set(void *mmr, u64 val)
{
	unsigned long flags;
	local_irq_save(flags);
	if (sport_width(mmr) <= 16)
		bfin_write16(mmr, val);
	else
		bfin_write32(mmr, val);
	local_irq_restore(flags);
	return 0;
}
static int sport_get(void *mmr, u64 *val)
{
	unsigned long flags;
	local_irq_save(flags);
	if (sport_width(mmr) <= 16)
		*val = bfin_read16(mmr);
	else
		*val = bfin_read32(mmr);
	local_irq_restore(flags);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_sport, sport_get, sport_set, "0x%08llx\n");
/*DEFINE_SIMPLE_ATTRIBUTE(fops_sport_ro, sport_get, NULL, "0x%08llx\n");*/
DEFINE_SIMPLE_ATTRIBUTE(fops_sport_wo, NULL, sport_set, "0x%08llx\n");
#define SPORT_OFF(mmr) (SPORT0_##mmr - SPORT0_TCR1)
#define _D_SPORT(name, perms, fops) \
	do { \
		strcpy(_buf, #name); \
		debugfs_create_file(buf, perms, parent, (void *)(base + SPORT_OFF(name)), fops); \
	} while (0)
#define __SPORT_RW(name) _D_SPORT(name, S_IRUSR|S_IWUSR, &fops_sport)
#define __SPORT_RO(name) _D_SPORT(name, S_IRUSR, &fops_sport_ro)
#define __SPORT_WO(name) _D_SPORT(name, S_IWUSR, &fops_sport_wo)
#define __SPORT(name, bits) \
	do { \
		strcpy(_buf, #name); \
		debugfs_create_x##bits(buf, S_IRUSR|S_IWUSR, parent, (u##bits *)(base + SPORT_OFF(name))); \
	} while (0)
static void __init __maybe_unused
bfin_debug_mmrs_sport(struct dentry *parent, unsigned long base, int num)
{
	char buf[32], *_buf = REGS_STR_PFX(buf, SPORT, num);
	__SPORT(CHNL, 16);
	__SPORT(MCMC1, 16);
	__SPORT(MCMC2, 16);
	__SPORT(MRCS0, 32);
	__SPORT(MRCS1, 32);
	__SPORT(MRCS2, 32);
	__SPORT(MRCS3, 32);
	__SPORT(MTCS0, 32);
	__SPORT(MTCS1, 32);
	__SPORT(MTCS2, 32);
	__SPORT(MTCS3, 32);
	__SPORT(RCLKDIV, 16);
	__SPORT(RCR1, 16);
	__SPORT(RCR2, 16);
	__SPORT(RFSDIV, 16);
	__SPORT_RW(RX);
	__SPORT(STAT, 16);
	__SPORT(TCLKDIV, 16);
	__SPORT(TCR1, 16);
	__SPORT(TCR2, 16);
	__SPORT(TFSDIV, 16);
	__SPORT_WO(TX);
}
#define SPORT(num) bfin_debug_mmrs_sport(parent, SPORT##num##_TCR1, num)

/*
 * TWI
 */
#define __TWI(uname, lname) __REGS(twi, #uname, lname)
static void __init __maybe_unused
bfin_debug_mmrs_twi(struct dentry *parent, unsigned long base, int num)
{
	char buf[32], *_buf = REGS_STR_PFX(buf, TWI, num);
	__TWI(CLKDIV, clkdiv);
	__TWI(CONTROL, control);
	__TWI(SLAVE_CTL, slave_ctl);
	__TWI(SLAVE_STAT, slave_stat);
	__TWI(SLAVE_ADDR, slave_addr);
	__TWI(MASTER_CTL, master_ctl);
	__TWI(MASTER_STAT, master_stat);
	__TWI(MASTER_ADDR, master_addr);
	__TWI(INT_STAT, int_stat);
	__TWI(INT_MASK, int_mask);
	__TWI(FIFO_CTL, fifo_ctl);
	__TWI(FIFO_STAT, fifo_stat);
	__TWI(XMT_DATA8, xmt_data8);
	__TWI(XMT_DATA16, xmt_data16);
	__TWI(RCV_DATA8, rcv_data8);
	__TWI(RCV_DATA16, rcv_data16);
}
#define TWI(num) bfin_debug_mmrs_twi(parent, TWI##num##_CLKDIV, num)

/*
 * UART
 */
#define __UART(uname, lname) __REGS(uart, #uname, lname)
static void __init __maybe_unused
bfin_debug_mmrs_uart(struct dentry *parent, unsigned long base, int num)
{
	char buf[32], *_buf = REGS_STR_PFX(buf, UART, num);
#ifdef BFIN_UART_BF54X_STYLE
	__UART(DLL, dll);
	__UART(DLH, dlh);
	__UART(GCTL, gctl);
	__UART(LCR, lcr);
	__UART(MCR, mcr);
	__UART(LSR, lsr);
	__UART(MSR, msr);
	__UART(SCR, scr);
	__UART(IER_SET, ier_set);
	__UART(IER_CLEAR, ier_clear);
	__UART(THR, thr);
	__UART(RBR, rbr);
#else
	__UART(DLL, dll);
	__UART(THR, thr);
	__UART(RBR, rbr);
	__UART(DLH, dlh);
	__UART(IER, ier);
	__UART(IIR, iir);
	__UART(LCR, lcr);
	__UART(MCR, mcr);
	__UART(LSR, lsr);
	__UART(MSR, msr);
	__UART(SCR, scr);
	__UART(GCTL, gctl);
#endif
}
#define UART(num) bfin_debug_mmrs_uart(parent, UART##num##_DLL, num)

/*
 * The actual debugfs generation
 */
static struct dentry *debug_mmrs_dentry;

static int __init bfin_debug_mmrs_init(void)
{
	struct dentry *top, *parent;

	pr_info("debug-mmrs: setting up Blackfin MMR debugfs\n");

	top = debugfs_create_dir("blackfin", NULL);
	if (top == NULL)
		return -1;

	parent = debugfs_create_dir("core_regs", top);
	debugfs_create_file("cclk", S_IRUSR, parent, NULL, &fops_debug_cclk);
	debugfs_create_file("sclk", S_IRUSR, parent, NULL, &fops_debug_sclk);
	debugfs_create_x32("last_seqstat", S_IRUSR, parent, &last_seqstat);
	D_SYSREG(cycles);
	D_SYSREG(cycles2);
	D_SYSREG(emudat);
	D_SYSREG(seqstat);
	D_SYSREG(syscfg);

	/* Core MMRs */
	parent = debugfs_create_dir("ctimer", top);
	D32(TCNTL);
	D32(TCOUNT);
	D32(TPERIOD);
	D32(TSCALE);

	parent = debugfs_create_dir("cec", top);
	D32(EVT0);
	D32(EVT1);
	D32(EVT2);
	D32(EVT3);
	D32(EVT4);
	D32(EVT5);
	D32(EVT6);
	D32(EVT7);
	D32(EVT8);
	D32(EVT9);
	D32(EVT10);
	D32(EVT11);
	D32(EVT12);
	D32(EVT13);
	D32(EVT14);
	D32(EVT15);
	D32(EVT_OVERRIDE);
	D32(IMASK);
	D32(IPEND);
	D32(ILAT);
	D32(IPRIO);

	parent = debugfs_create_dir("debug", top);
	D32(DBGSTAT);
	D32(DSPID);

	parent = debugfs_create_dir("mmu", top);
	D32(SRAM_BASE_ADDRESS);
	D32(DCPLB_ADDR0);
	D32(DCPLB_ADDR10);
	D32(DCPLB_ADDR11);
	D32(DCPLB_ADDR12);
	D32(DCPLB_ADDR13);
	D32(DCPLB_ADDR14);
	D32(DCPLB_ADDR15);
	D32(DCPLB_ADDR1);
	D32(DCPLB_ADDR2);
	D32(DCPLB_ADDR3);
	D32(DCPLB_ADDR4);
	D32(DCPLB_ADDR5);
	D32(DCPLB_ADDR6);
	D32(DCPLB_ADDR7);
	D32(DCPLB_ADDR8);
	D32(DCPLB_ADDR9);
	D32(DCPLB_DATA0);
	D32(DCPLB_DATA10);
	D32(DCPLB_DATA11);
	D32(DCPLB_DATA12);
	D32(DCPLB_DATA13);
	D32(DCPLB_DATA14);
	D32(DCPLB_DATA15);
	D32(DCPLB_DATA1);
	D32(DCPLB_DATA2);
	D32(DCPLB_DATA3);
	D32(DCPLB_DATA4);
	D32(DCPLB_DATA5);
	D32(DCPLB_DATA6);
	D32(DCPLB_DATA7);
	D32(DCPLB_DATA8);
	D32(DCPLB_DATA9);
	D32(DCPLB_FAULT_ADDR);
	D32(DCPLB_STATUS);
	D32(DMEM_CONTROL);
	D32(DTEST_COMMAND);
	D32(DTEST_DATA0);
	D32(DTEST_DATA1);

	D32(ICPLB_ADDR0);
	D32(ICPLB_ADDR1);
	D32(ICPLB_ADDR2);
	D32(ICPLB_ADDR3);
	D32(ICPLB_ADDR4);
	D32(ICPLB_ADDR5);
	D32(ICPLB_ADDR6);
	D32(ICPLB_ADDR7);
	D32(ICPLB_ADDR8);
	D32(ICPLB_ADDR9);
	D32(ICPLB_ADDR10);
	D32(ICPLB_ADDR11);
	D32(ICPLB_ADDR12);
	D32(ICPLB_ADDR13);
	D32(ICPLB_ADDR14);
	D32(ICPLB_ADDR15);
	D32(ICPLB_DATA0);
	D32(ICPLB_DATA1);
	D32(ICPLB_DATA2);
	D32(ICPLB_DATA3);
	D32(ICPLB_DATA4);
	D32(ICPLB_DATA5);
	D32(ICPLB_DATA6);
	D32(ICPLB_DATA7);
	D32(ICPLB_DATA8);
	D32(ICPLB_DATA9);
	D32(ICPLB_DATA10);
	D32(ICPLB_DATA11);
	D32(ICPLB_DATA12);
	D32(ICPLB_DATA13);
	D32(ICPLB_DATA14);
	D32(ICPLB_DATA15);
	D32(ICPLB_FAULT_ADDR);
	D32(ICPLB_STATUS);
	D32(IMEM_CONTROL);
	if (!ANOMALY_05000481) {
		D32(ITEST_COMMAND);
		D32(ITEST_DATA0);
		D32(ITEST_DATA1);
	}

	parent = debugfs_create_dir("perf", top);
	D32(PFCNTR0);
	D32(PFCNTR1);
	D32(PFCTL);

	parent = debugfs_create_dir("trace", top);
	D32(TBUF);
	D32(TBUFCTL);
	D32(TBUFSTAT);

	parent = debugfs_create_dir("watchpoint", top);
	D32(WPIACTL);
	D32(WPIA0);
	D32(WPIA1);
	D32(WPIA2);
	D32(WPIA3);
	D32(WPIA4);
	D32(WPIA5);
	D32(WPIACNT0);
	D32(WPIACNT1);
	D32(WPIACNT2);
	D32(WPIACNT3);
	D32(WPIACNT4);
	D32(WPIACNT5);
	D32(WPDACTL);
	D32(WPDA0);
	D32(WPDA1);
	D32(WPDACNT0);
	D32(WPDACNT1);
	D32(WPSTAT);

	/* System MMRs */
#ifdef ATAPI_CONTROL
	parent = debugfs_create_dir("atapi", top);
	D16(ATAPI_CONTROL);
	D16(ATAPI_DEV_ADDR);
	D16(ATAPI_DEV_RXBUF);
	D16(ATAPI_DEV_TXBUF);
	D16(ATAPI_DMA_TFRCNT);
	D16(ATAPI_INT_MASK);
	D16(ATAPI_INT_STATUS);
	D16(ATAPI_LINE_STATUS);
	D16(ATAPI_MULTI_TIM_0);
	D16(ATAPI_MULTI_TIM_1);
	D16(ATAPI_MULTI_TIM_2);
	D16(ATAPI_PIO_TFRCNT);
	D16(ATAPI_PIO_TIM_0);
	D16(ATAPI_PIO_TIM_1);
	D16(ATAPI_REG_TIM_0);
	D16(ATAPI_SM_STATE);
	D16(ATAPI_STATUS);
	D16(ATAPI_TERMINATE);
	D16(ATAPI_UDMAOUT_TFRCNT);
	D16(ATAPI_ULTRA_TIM_0);
	D16(ATAPI_ULTRA_TIM_1);
	D16(ATAPI_ULTRA_TIM_2);
	D16(ATAPI_ULTRA_TIM_3);
	D16(ATAPI_UMAIN_TFRCNT);
	D16(ATAPI_XFER_LEN);
#endif

#if defined(CAN_MC1) || defined(CAN0_MC1) || defined(CAN1_MC1)
	parent = debugfs_create_dir("can", top);
# ifdef CAN_MC1
	bfin_debug_mmrs_can(parent, CAN_MC1, -1);
# endif
# ifdef CAN0_MC1
	CAN(0);
# endif
# ifdef CAN1_MC1
	CAN(1);
# endif
#endif

#ifdef CNT_COMMAND
	parent = debugfs_create_dir("counter", top);
	D16(CNT_COMMAND);
	D16(CNT_CONFIG);
	D32(CNT_COUNTER);
	D16(CNT_DEBOUNCE);
	D16(CNT_IMASK);
	D32(CNT_MAX);
	D32(CNT_MIN);
	D16(CNT_STATUS);
#endif

	parent = debugfs_create_dir("dmac", top);
#ifdef DMA_TC_CNT
	D16(DMAC_TC_CNT);
	D16(DMAC_TC_PER);
#endif
#ifdef DMAC0_TC_CNT
	D16(DMAC0_TC_CNT);
	D16(DMAC0_TC_PER);
#endif
#ifdef DMAC1_TC_CNT
	D16(DMAC1_TC_CNT);
	D16(DMAC1_TC_PER);
#endif
#ifdef DMAC1_PERIMUX
	D16(DMAC1_PERIMUX);
#endif

#ifndef __ADSPBF561__
	parent = debugfs_create_dir("dma", top);
	DMA(0);
	DMA(1);
	DMA(1);
	DMA(2);
	DMA(3);
	DMA(4);
	DMA(5);
	DMA(6);
	DMA(7);
#ifdef DMA8_CONFIG
	DMA(8);
	DMA(9);
	DMA(10);
	DMA(11);
#endif
#ifdef DMA12_CONFIG
	DMA(12);
	DMA(13);
	DMA(14);
	DMA(15);
	DMA(16);
	DMA(17);
	DMA(18);
	DMA(19);
#endif
#ifdef DMA20_CONFIG
	DMA(20);
	DMA(21);
	DMA(22);
	DMA(23);
#endif
#endif

	parent = debugfs_create_dir("ebiu_amc", top);
	D32(EBIU_AMBCTL0);
	D32(EBIU_AMBCTL1);
	D16(EBIU_AMGCTL);
#ifdef EBIU_MBSCTL
	D16(EBIU_MBSCTL);
	D32(EBIU_ARBSTAT);
	D32(EBIU_MODE);
	D16(EBIU_FCTL);
#endif

#ifdef EBIU_SDGCTL
	parent = debugfs_create_dir("ebiu_sdram", top);
# ifdef __ADSPBF561__
	D32(EBIU_SDBCTL);
# else
	D16(EBIU_SDBCTL);
# endif
	D32(EBIU_SDGCTL);
	D16(EBIU_SDRRC);
	D16(EBIU_SDSTAT);
#endif

#ifdef EBIU_DDRACCT
	parent = debugfs_create_dir("ebiu_ddr", top);
	D32(EBIU_DDRACCT);
	D32(EBIU_DDRARCT);
	D32(EBIU_DDRBRC0);
	D32(EBIU_DDRBRC1);
	D32(EBIU_DDRBRC2);
	D32(EBIU_DDRBRC3);
	D32(EBIU_DDRBRC4);
	D32(EBIU_DDRBRC5);
	D32(EBIU_DDRBRC6);
	D32(EBIU_DDRBRC7);
	D32(EBIU_DDRBWC0);
	D32(EBIU_DDRBWC1);
	D32(EBIU_DDRBWC2);
	D32(EBIU_DDRBWC3);
	D32(EBIU_DDRBWC4);
	D32(EBIU_DDRBWC5);
	D32(EBIU_DDRBWC6);
	D32(EBIU_DDRBWC7);
	D32(EBIU_DDRCTL0);
	D32(EBIU_DDRCTL1);
	D32(EBIU_DDRCTL2);
	D32(EBIU_DDRCTL3);
	D32(EBIU_DDRGC0);
	D32(EBIU_DDRGC1);
	D32(EBIU_DDRGC2);
	D32(EBIU_DDRGC3);
	D32(EBIU_DDRMCCL);
	D32(EBIU_DDRMCEN);
	D32(EBIU_DDRQUE);
	D32(EBIU_DDRTACT);
	D32(EBIU_ERRADD);
	D16(EBIU_ERRMST);
	D16(EBIU_RSTCTL);
#endif

#ifdef EMAC_ADDRHI
	parent = debugfs_create_dir("emac", top);
	D32(EMAC_ADDRHI);
	D32(EMAC_ADDRLO);
	D32(EMAC_FLC);
	D32(EMAC_HASHHI);
	D32(EMAC_HASHLO);
	D32(EMAC_MMC_CTL);
	D32(EMAC_MMC_RIRQE);
	D32(EMAC_MMC_RIRQS);
	D32(EMAC_MMC_TIRQE);
	D32(EMAC_MMC_TIRQS);
	D32(EMAC_OPMODE);
	D32(EMAC_RXC_ALIGN);
	D32(EMAC_RXC_ALLFRM);
	D32(EMAC_RXC_ALLOCT);
	D32(EMAC_RXC_BROAD);
	D32(EMAC_RXC_DMAOVF);
	D32(EMAC_RXC_EQ64);
	D32(EMAC_RXC_FCS);
	D32(EMAC_RXC_GE1024);
	D32(EMAC_RXC_LNERRI);
	D32(EMAC_RXC_LNERRO);
	D32(EMAC_RXC_LONG);
	D32(EMAC_RXC_LT1024);
	D32(EMAC_RXC_LT128);
	D32(EMAC_RXC_LT256);
	D32(EMAC_RXC_LT512);
	D32(EMAC_RXC_MACCTL);
	D32(EMAC_RXC_MULTI);
	D32(EMAC_RXC_OCTET);
	D32(EMAC_RXC_OK);
	D32(EMAC_RXC_OPCODE);
	D32(EMAC_RXC_PAUSE);
	D32(EMAC_RXC_SHORT);
	D32(EMAC_RXC_TYPED);
	D32(EMAC_RXC_UNICST);
	D32(EMAC_RX_IRQE);
	D32(EMAC_RX_STAT);
	D32(EMAC_RX_STKY);
	D32(EMAC_STAADD);
	D32(EMAC_STADAT);
	D32(EMAC_SYSCTL);
	D32(EMAC_SYSTAT);
	D32(EMAC_TXC_1COL);
	D32(EMAC_TXC_ABORT);
	D32(EMAC_TXC_ALLFRM);
	D32(EMAC_TXC_ALLOCT);
	D32(EMAC_TXC_BROAD);
	D32(EMAC_TXC_CRSERR);
	D32(EMAC_TXC_DEFER);
	D32(EMAC_TXC_DMAUND);
	D32(EMAC_TXC_EQ64);
	D32(EMAC_TXC_GE1024);
	D32(EMAC_TXC_GT1COL);
	D32(EMAC_TXC_LATECL);
	D32(EMAC_TXC_LT1024);
	D32(EMAC_TXC_LT128);
	D32(EMAC_TXC_LT256);
	D32(EMAC_TXC_LT512);
	D32(EMAC_TXC_MACCTL);
	D32(EMAC_TXC_MULTI);
	D32(EMAC_TXC_OCTET);
	D32(EMAC_TXC_OK);
	D32(EMAC_TXC_UNICST);
	D32(EMAC_TXC_XS_COL);
	D32(EMAC_TXC_XS_DFR);
	D32(EMAC_TX_IRQE);
	D32(EMAC_TX_STAT);
	D32(EMAC_TX_STKY);
	D32(EMAC_VLAN1);
	D32(EMAC_VLAN2);
	D32(EMAC_WKUP_CTL);
	D32(EMAC_WKUP_FFCMD);
	D32(EMAC_WKUP_FFCRC0);
	D32(EMAC_WKUP_FFCRC1);
	D32(EMAC_WKUP_FFMSK0);
	D32(EMAC_WKUP_FFMSK1);
	D32(EMAC_WKUP_FFMSK2);
	D32(EMAC_WKUP_FFMSK3);
	D32(EMAC_WKUP_FFOFF);
# ifdef EMAC_PTP_ACCR
	D32(EMAC_PTP_ACCR);
	D32(EMAC_PTP_ADDEND);
	D32(EMAC_PTP_ALARMHI);
	D32(EMAC_PTP_ALARMLO);
	D16(EMAC_PTP_CTL);
	D32(EMAC_PTP_FOFF);
	D32(EMAC_PTP_FV1);
	D32(EMAC_PTP_FV2);
	D32(EMAC_PTP_FV3);
	D16(EMAC_PTP_ID_OFF);
	D32(EMAC_PTP_ID_SNAP);
	D16(EMAC_PTP_IE);
	D16(EMAC_PTP_ISTAT);
	D32(EMAC_PTP_OFFSET);
	D32(EMAC_PTP_PPS_PERIOD);
	D32(EMAC_PTP_PPS_STARTHI);
	D32(EMAC_PTP_PPS_STARTLO);
	D32(EMAC_PTP_RXSNAPHI);
	D32(EMAC_PTP_RXSNAPLO);
	D32(EMAC_PTP_TIMEHI);
	D32(EMAC_PTP_TIMELO);
	D32(EMAC_PTP_TXSNAPHI);
	D32(EMAC_PTP_TXSNAPLO);
# endif
#endif

#if defined(EPPI0_STATUS) || defined(EPPI1_STATUS) || defined(EPPI2_STATUS)
	parent = debugfs_create_dir("eppi", top);
# ifdef EPPI0_STATUS
	EPPI(0);
# endif
# ifdef EPPI1_STATUS
	EPPI(1);
# endif
# ifdef EPPI2_STATUS
	EPPI(2);
# endif
#endif

	parent = debugfs_create_dir("gptimer", top);
#ifdef TIMER_DISABLE
	D16(TIMER_DISABLE);
	D16(TIMER_ENABLE);
	D32(TIMER_STATUS);
#endif
#ifdef TIMER_DISABLE0
	D16(TIMER_DISABLE0);
	D16(TIMER_ENABLE0);
	D32(TIMER_STATUS0);
#endif
#ifdef TIMER_DISABLE1
	D16(TIMER_DISABLE1);
	D16(TIMER_ENABLE1);
	D32(TIMER_STATUS1);
#endif
	/* XXX: Should convert BF561 MMR names */
#ifdef TMRS4_DISABLE
	D16(TMRS4_DISABLE);
	D16(TMRS4_ENABLE);
	D32(TMRS4_STATUS);
	D16(TMRS8_DISABLE);
	D16(TMRS8_ENABLE);
	D32(TMRS8_STATUS);
#endif
	GPTIMER(0);
	GPTIMER(1);
	GPTIMER(2);
#ifdef TIMER3_CONFIG
	GPTIMER(3);
	GPTIMER(4);
	GPTIMER(5);
	GPTIMER(6);
	GPTIMER(7);
#endif
#ifdef TIMER8_CONFIG
	GPTIMER(8);
	GPTIMER(9);
	GPTIMER(10);
#endif
#ifdef TIMER11_CONFIG
	GPTIMER(11);
#endif

#ifdef HMDMA0_CONTROL
	parent = debugfs_create_dir("hmdma", top);
	HMDMA(0);
	HMDMA(1);
#endif

#ifdef HOST_CONTROL
	parent = debugfs_create_dir("hostdp", top);
	D16(HOST_CONTROL);
	D16(HOST_STATUS);
	D16(HOST_TIMEOUT);
#endif

#ifdef IMDMA_S0_CONFIG
	parent = debugfs_create_dir("imdma", top);
	IMDMA(0);
	IMDMA(1);
#endif

#ifdef KPAD_CTL
	parent = debugfs_create_dir("keypad", top);
	D16(KPAD_CTL);
	D16(KPAD_PRESCALE);
	D16(KPAD_MSEL);
	D16(KPAD_ROWCOL);
	D16(KPAD_STAT);
	D16(KPAD_SOFTEVAL);
#endif

	parent = debugfs_create_dir("mdma", top);
	MDMA(0);
	MDMA(1);
#ifdef MDMA_D2_CONFIG
	MDMA(2);
	MDMA(3);
#endif

#ifdef MXVR_CONFIG
	parent = debugfs_create_dir("mxvr", top);
	D16(MXVR_CONFIG);
# ifdef MXVR_PLL_CTL_0
	D32(MXVR_PLL_CTL_0);
# endif
	D32(MXVR_STATE_0);
	D32(MXVR_STATE_1);
	D32(MXVR_INT_STAT_0);
	D32(MXVR_INT_STAT_1);
	D32(MXVR_INT_EN_0);
	D32(MXVR_INT_EN_1);
	D16(MXVR_POSITION);
	D16(MXVR_MAX_POSITION);
	D16(MXVR_DELAY);
	D16(MXVR_MAX_DELAY);
	D32(MXVR_LADDR);
	D16(MXVR_GADDR);
	D32(MXVR_AADDR);
	D32(MXVR_ALLOC_0);
	D32(MXVR_ALLOC_1);
	D32(MXVR_ALLOC_2);
	D32(MXVR_ALLOC_3);
	D32(MXVR_ALLOC_4);
	D32(MXVR_ALLOC_5);
	D32(MXVR_ALLOC_6);
	D32(MXVR_ALLOC_7);
	D32(MXVR_ALLOC_8);
	D32(MXVR_ALLOC_9);
	D32(MXVR_ALLOC_10);
	D32(MXVR_ALLOC_11);
	D32(MXVR_ALLOC_12);
	D32(MXVR_ALLOC_13);
	D32(MXVR_ALLOC_14);
	D32(MXVR_SYNC_LCHAN_0);
	D32(MXVR_SYNC_LCHAN_1);
	D32(MXVR_SYNC_LCHAN_2);
	D32(MXVR_SYNC_LCHAN_3);
	D32(MXVR_SYNC_LCHAN_4);
	D32(MXVR_SYNC_LCHAN_5);
	D32(MXVR_SYNC_LCHAN_6);
	D32(MXVR_SYNC_LCHAN_7);
	D32(MXVR_DMA0_CONFIG);
	D32(MXVR_DMA0_START_ADDR);
	D16(MXVR_DMA0_COUNT);
	D32(MXVR_DMA0_CURR_ADDR);
	D16(MXVR_DMA0_CURR_COUNT);
	D32(MXVR_DMA1_CONFIG);
	D32(MXVR_DMA1_START_ADDR);
	D16(MXVR_DMA1_COUNT);
	D32(MXVR_DMA1_CURR_ADDR);
	D16(MXVR_DMA1_CURR_COUNT);
	D32(MXVR_DMA2_CONFIG);
	D32(MXVR_DMA2_START_ADDR);
	D16(MXVR_DMA2_COUNT);
	D32(MXVR_DMA2_CURR_ADDR);
	D16(MXVR_DMA2_CURR_COUNT);
	D32(MXVR_DMA3_CONFIG);
	D32(MXVR_DMA3_START_ADDR);
	D16(MXVR_DMA3_COUNT);
	D32(MXVR_DMA3_CURR_ADDR);
	D16(MXVR_DMA3_CURR_COUNT);
	D32(MXVR_DMA4_CONFIG);
	D32(MXVR_DMA4_START_ADDR);
	D16(MXVR_DMA4_COUNT);
	D32(MXVR_DMA4_CURR_ADDR);
	D16(MXVR_DMA4_CURR_COUNT);
	D32(MXVR_DMA5_CONFIG);
	D32(MXVR_DMA5_START_ADDR);
	D16(MXVR_DMA5_COUNT);
	D32(MXVR_DMA5_CURR_ADDR);
	D16(MXVR_DMA5_CURR_COUNT);
	D32(MXVR_DMA6_CONFIG);
	D32(MXVR_DMA6_START_ADDR);
	D16(MXVR_DMA6_COUNT);
	D32(MXVR_DMA6_CURR_ADDR);
	D16(MXVR_DMA6_CURR_COUNT);
	D32(MXVR_DMA7_CONFIG);
	D32(MXVR_DMA7_START_ADDR);
	D16(MXVR_DMA7_COUNT);
	D32(MXVR_DMA7_CURR_ADDR);
	D16(MXVR_DMA7_CURR_COUNT);
	D16(MXVR_AP_CTL);
	D32(MXVR_APRB_START_ADDR);
	D32(MXVR_APRB_CURR_ADDR);
	D32(MXVR_APTB_START_ADDR);
	D32(MXVR_APTB_CURR_ADDR);
	D32(MXVR_CM_CTL);
	D32(MXVR_CMRB_START_ADDR);
	D32(MXVR_CMRB_CURR_ADDR);
	D32(MXVR_CMTB_START_ADDR);
	D32(MXVR_CMTB_CURR_ADDR);
	D32(MXVR_RRDB_START_ADDR);
	D32(MXVR_RRDB_CURR_ADDR);
	D32(MXVR_PAT_DATA_0);
	D32(MXVR_PAT_EN_0);
	D32(MXVR_PAT_DATA_1);
	D32(MXVR_PAT_EN_1);
	D16(MXVR_FRAME_CNT_0);
	D16(MXVR_FRAME_CNT_1);
	D32(MXVR_ROUTING_0);
	D32(MXVR_ROUTING_1);
	D32(MXVR_ROUTING_2);
	D32(MXVR_ROUTING_3);
	D32(MXVR_ROUTING_4);
	D32(MXVR_ROUTING_5);
	D32(MXVR_ROUTING_6);
	D32(MXVR_ROUTING_7);
	D32(MXVR_ROUTING_8);
	D32(MXVR_ROUTING_9);
	D32(MXVR_ROUTING_10);
	D32(MXVR_ROUTING_11);
	D32(MXVR_ROUTING_12);
	D32(MXVR_ROUTING_13);
	D32(MXVR_ROUTING_14);
# ifdef MXVR_PLL_CTL_1
	D32(MXVR_PLL_CTL_1);
# endif
	D16(MXVR_BLOCK_CNT);
# ifdef MXVR_CLK_CTL
	D32(MXVR_CLK_CTL);
# endif
# ifdef MXVR_CDRPLL_CTL
	D32(MXVR_CDRPLL_CTL);
# endif
# ifdef MXVR_FMPLL_CTL
	D32(MXVR_FMPLL_CTL);
# endif
# ifdef MXVR_PIN_CTL
	D16(MXVR_PIN_CTL);
# endif
# ifdef MXVR_SCLK_CNT
	D16(MXVR_SCLK_CNT);
# endif
#endif

#ifdef NFC_ADDR
	parent = debugfs_create_dir("nfc", top);
	D_WO(NFC_ADDR, 16);
	D_WO(NFC_CMD, 16);
	D_RO(NFC_COUNT, 16);
	D16(NFC_CTL);
	D_WO(NFC_DATA_RD, 16);
	D_WO(NFC_DATA_WR, 16);
	D_RO(NFC_ECC0, 16);
	D_RO(NFC_ECC1, 16);
	D_RO(NFC_ECC2, 16);
	D_RO(NFC_ECC3, 16);
	D16(NFC_IRQMASK);
	D16(NFC_IRQSTAT);
	D_WO(NFC_PGCTL, 16);
	D_RO(NFC_READ, 16);
	D16(NFC_RST);
	D_RO(NFC_STAT, 16);
#endif

#ifdef OTP_CONTROL
	parent = debugfs_create_dir("otp", top);
	D16(OTP_CONTROL);
	D16(OTP_BEN);
	D16(OTP_STATUS);
	D32(OTP_TIMING);
	D32(OTP_DATA0);
	D32(OTP_DATA1);
	D32(OTP_DATA2);
	D32(OTP_DATA3);
#endif

#ifdef PIXC_CTL
	parent = debugfs_create_dir("pixc", top);
	D16(PIXC_CTL);
	D16(PIXC_PPL);
	D16(PIXC_LPF);
	D16(PIXC_AHSTART);
	D16(PIXC_AHEND);
	D16(PIXC_AVSTART);
	D16(PIXC_AVEND);
	D16(PIXC_ATRANSP);
	D16(PIXC_BHSTART);
	D16(PIXC_BHEND);
	D16(PIXC_BVSTART);
	D16(PIXC_BVEND);
	D16(PIXC_BTRANSP);
	D16(PIXC_INTRSTAT);
	D32(PIXC_RYCON);
	D32(PIXC_GUCON);
	D32(PIXC_BVCON);
	D32(PIXC_CCBIAS);
	D32(PIXC_TC);
#endif

	parent = debugfs_create_dir("pll", top);
	D16(PLL_CTL);
	D16(PLL_DIV);
	D16(PLL_LOCKCNT);
	D16(PLL_STAT);
	D16(VR_CTL);
	D32(CHIPID);	/* it's part of this hardware block */

#if defined(PPI_STATUS) || defined(PPI0_STATUS) || defined(PPI1_STATUS)
	parent = debugfs_create_dir("ppi", top);
# ifdef PPI_STATUS
	bfin_debug_mmrs_ppi(parent, PPI_STATUS, -1);
# endif
# ifdef PPI0_STATUS
	PPI(0);
# endif
# ifdef PPI1_STATUS
	PPI(1);
# endif
#endif

#ifdef PWM_CTRL
	parent = debugfs_create_dir("pwm", top);
	D16(PWM_CTRL);
	D16(PWM_STAT);
	D16(PWM_TM);
	D16(PWM_DT);
	D16(PWM_GATE);
	D16(PWM_CHA);
	D16(PWM_CHB);
	D16(PWM_CHC);
	D16(PWM_SEG);
	D16(PWM_SYNCWT);
	D16(PWM_CHAL);
	D16(PWM_CHBL);
	D16(PWM_CHCL);
	D16(PWM_LSI);
	D16(PWM_STAT2);
#endif

#ifdef RSI_CONFIG
	parent = debugfs_create_dir("rsi", top);
	D32(RSI_ARGUMENT);
	D16(RSI_CEATA_CONTROL);
	D16(RSI_CLK_CONTROL);
	D16(RSI_COMMAND);
	D16(RSI_CONFIG);
	D16(RSI_DATA_CNT);
	D16(RSI_DATA_CONTROL);
	D16(RSI_DATA_LGTH);
	D32(RSI_DATA_TIMER);
	D16(RSI_EMASK);
	D16(RSI_ESTAT);
	D32(RSI_FIFO);
	D16(RSI_FIFO_CNT);
	D32(RSI_MASK0);
	D32(RSI_MASK1);
	D16(RSI_PID0);
	D16(RSI_PID1);
	D16(RSI_PID2);
	D16(RSI_PID3);
	D16(RSI_PWR_CONTROL);
	D16(RSI_RD_WAIT_EN);
	D32(RSI_RESPONSE0);
	D32(RSI_RESPONSE1);
	D32(RSI_RESPONSE2);
	D32(RSI_RESPONSE3);
	D16(RSI_RESP_CMD);
	D32(RSI_STATUS);
	D_WO(RSI_STATUSCL, 16);
#endif

#ifdef RTC_ALARM
	parent = debugfs_create_dir("rtc", top);
	D32(RTC_ALARM);
	D16(RTC_ICTL);
	D16(RTC_ISTAT);
	D16(RTC_PREN);
	D32(RTC_STAT);
	D16(RTC_SWCNT);
#endif

#ifdef SDH_CFG
	parent = debugfs_create_dir("sdh", top);
	D32(SDH_ARGUMENT);
	D16(SDH_CFG);
	D16(SDH_CLK_CTL);
	D16(SDH_COMMAND);
	D_RO(SDH_DATA_CNT, 16);
	D16(SDH_DATA_CTL);
	D16(SDH_DATA_LGTH);
	D32(SDH_DATA_TIMER);
	D16(SDH_E_MASK);
	D16(SDH_E_STATUS);
	D32(SDH_FIFO);
	D_RO(SDH_FIFO_CNT, 16);
	D32(SDH_MASK0);
	D32(SDH_MASK1);
	D_RO(SDH_PID0, 16);
	D_RO(SDH_PID1, 16);
	D_RO(SDH_PID2, 16);
	D_RO(SDH_PID3, 16);
	D_RO(SDH_PID4, 16);
	D_RO(SDH_PID5, 16);
	D_RO(SDH_PID6, 16);
	D_RO(SDH_PID7, 16);
	D16(SDH_PWR_CTL);
	D16(SDH_RD_WAIT_EN);
	D_RO(SDH_RESPONSE0, 32);
	D_RO(SDH_RESPONSE1, 32);
	D_RO(SDH_RESPONSE2, 32);
	D_RO(SDH_RESPONSE3, 32);
	D_RO(SDH_RESP_CMD, 16);
	D_RO(SDH_STATUS, 32);
	D_WO(SDH_STATUS_CLR, 16);
#endif

#ifdef SECURE_CONTROL
	parent = debugfs_create_dir("security", top);
	D16(SECURE_CONTROL);
	D16(SECURE_STATUS);
	D32(SECURE_SYSSWT);
#endif

	parent = debugfs_create_dir("sic", top);
	D16(SWRST);
	D16(SYSCR);
	D16(SIC_RVECT);
	D32(SIC_IAR0);
	D32(SIC_IAR1);
	D32(SIC_IAR2);
#ifdef SIC_IAR3
	D32(SIC_IAR3);
#endif
#ifdef SIC_IAR4
	D32(SIC_IAR4);
	D32(SIC_IAR5);
	D32(SIC_IAR6);
#endif
#ifdef SIC_IAR7
	D32(SIC_IAR7);
#endif
#ifdef SIC_IAR8
	D32(SIC_IAR8);
	D32(SIC_IAR9);
	D32(SIC_IAR10);
	D32(SIC_IAR11);
#endif
#ifdef SIC_IMASK
	D32(SIC_IMASK);
	D32(SIC_ISR);
	D32(SIC_IWR);
#endif
#ifdef SIC_IMASK0
	D32(SIC_IMASK0);
	D32(SIC_IMASK1);
	D32(SIC_ISR0);
	D32(SIC_ISR1);
	D32(SIC_IWR0);
	D32(SIC_IWR1);
#endif
#ifdef SIC_IMASK2
	D32(SIC_IMASK2);
	D32(SIC_ISR2);
	D32(SIC_IWR2);
#endif
#ifdef SICB_RVECT
	D16(SICB_SWRST);
	D16(SICB_SYSCR);
	D16(SICB_RVECT);
	D32(SICB_IAR0);
	D32(SICB_IAR1);
	D32(SICB_IAR2);
	D32(SICB_IAR3);
	D32(SICB_IAR4);
	D32(SICB_IAR5);
	D32(SICB_IAR6);
	D32(SICB_IAR7);
	D32(SICB_IMASK0);
	D32(SICB_IMASK1);
	D32(SICB_ISR0);
	D32(SICB_ISR1);
	D32(SICB_IWR0);
	D32(SICB_IWR1);
#endif

	parent = debugfs_create_dir("spi", top);
#ifdef SPI0_REGBASE
	SPI(0);
#endif
#ifdef SPI1_REGBASE
	SPI(1);
#endif
#ifdef SPI2_REGBASE
	SPI(2);
#endif

	parent = debugfs_create_dir("sport", top);
#ifdef SPORT0_STAT
	SPORT(0);
#endif
#ifdef SPORT1_STAT
	SPORT(1);
#endif
#ifdef SPORT2_STAT
	SPORT(2);
#endif
#ifdef SPORT3_STAT
	SPORT(3);
#endif

#if defined(TWI_CLKDIV) || defined(TWI0_CLKDIV) || defined(TWI1_CLKDIV)
	parent = debugfs_create_dir("twi", top);
# ifdef TWI_CLKDIV
	bfin_debug_mmrs_twi(parent, TWI_CLKDIV, -1);
# endif
# ifdef TWI0_CLKDIV
	TWI(0);
# endif
# ifdef TWI1_CLKDIV
	TWI(1);
# endif
#endif

	parent = debugfs_create_dir("uart", top);
#ifdef BFIN_UART_DLL
	bfin_debug_mmrs_uart(parent, BFIN_UART_DLL, -1);
#endif
#ifdef UART0_DLL
	UART(0);
#endif
#ifdef UART1_DLL
	UART(1);
#endif
#ifdef UART2_DLL
	UART(2);
#endif
#ifdef UART3_DLL
	UART(3);
#endif

#ifdef USB_FADDR
	parent = debugfs_create_dir("usb", top);
	D16(USB_FADDR);
	D16(USB_POWER);
	D16(USB_INTRTX);
	D16(USB_INTRRX);
	D16(USB_INTRTXE);
	D16(USB_INTRRXE);
	D16(USB_INTRUSB);
	D16(USB_INTRUSBE);
	D16(USB_FRAME);
	D16(USB_INDEX);
	D16(USB_TESTMODE);
	D16(USB_GLOBINTR);
	D16(USB_GLOBAL_CTL);
	D16(USB_TX_MAX_PACKET);
	D16(USB_CSR0);
	D16(USB_TXCSR);
	D16(USB_RX_MAX_PACKET);
	D16(USB_RXCSR);
	D16(USB_COUNT0);
	D16(USB_RXCOUNT);
	D16(USB_TXTYPE);
	D16(USB_NAKLIMIT0);
	D16(USB_TXINTERVAL);
	D16(USB_RXTYPE);
	D16(USB_RXINTERVAL);
	D16(USB_TXCOUNT);
	D16(USB_EP0_FIFO);
	D16(USB_EP1_FIFO);
	D16(USB_EP2_FIFO);
	D16(USB_EP3_FIFO);
	D16(USB_EP4_FIFO);
	D16(USB_EP5_FIFO);
	D16(USB_EP6_FIFO);
	D16(USB_EP7_FIFO);
	D16(USB_OTG_DEV_CTL);
	D16(USB_OTG_VBUS_IRQ);
	D16(USB_OTG_VBUS_MASK);
	D16(USB_LINKINFO);
	D16(USB_VPLEN);
	D16(USB_HS_EOF1);
	D16(USB_FS_EOF1);
	D16(USB_LS_EOF1);
	D16(USB_APHY_CNTRL);
	D16(USB_APHY_CALIB);
	D16(USB_APHY_CNTRL2);
	D16(USB_PHY_TEST);
	D16(USB_PLLOSC_CTRL);
	D16(USB_SRP_CLKDIV);
	D16(USB_EP_NI0_TXMAXP);
	D16(USB_EP_NI0_TXCSR);
	D16(USB_EP_NI0_RXMAXP);
	D16(USB_EP_NI0_RXCSR);
	D16(USB_EP_NI0_RXCOUNT);
	D16(USB_EP_NI0_TXTYPE);
	D16(USB_EP_NI0_TXINTERVAL);
	D16(USB_EP_NI0_RXTYPE);
	D16(USB_EP_NI0_RXINTERVAL);
	D16(USB_EP_NI0_TXCOUNT);
	D16(USB_EP_NI1_TXMAXP);
	D16(USB_EP_NI1_TXCSR);
	D16(USB_EP_NI1_RXMAXP);
	D16(USB_EP_NI1_RXCSR);
	D16(USB_EP_NI1_RXCOUNT);
	D16(USB_EP_NI1_TXTYPE);
	D16(USB_EP_NI1_TXINTERVAL);
	D16(USB_EP_NI1_RXTYPE);
	D16(USB_EP_NI1_RXINTERVAL);
	D16(USB_EP_NI1_TXCOUNT);
	D16(USB_EP_NI2_TXMAXP);
	D16(USB_EP_NI2_TXCSR);
	D16(USB_EP_NI2_RXMAXP);
	D16(USB_EP_NI2_RXCSR);
	D16(USB_EP_NI2_RXCOUNT);
	D16(USB_EP_NI2_TXTYPE);
	D16(USB_EP_NI2_TXINTERVAL);
	D16(USB_EP_NI2_RXTYPE);
	D16(USB_EP_NI2_RXINTERVAL);
	D16(USB_EP_NI2_TXCOUNT);
	D16(USB_EP_NI3_TXMAXP);
	D16(USB_EP_NI3_TXCSR);
	D16(USB_EP_NI3_RXMAXP);
	D16(USB_EP_NI3_RXCSR);
	D16(USB_EP_NI3_RXCOUNT);
	D16(USB_EP_NI3_TXTYPE);
	D16(USB_EP_NI3_TXINTERVAL);
	D16(USB_EP_NI3_RXTYPE);
	D16(USB_EP_NI3_RXINTERVAL);
	D16(USB_EP_NI3_TXCOUNT);
	D16(USB_EP_NI4_TXMAXP);
	D16(USB_EP_NI4_TXCSR);
	D16(USB_EP_NI4_RXMAXP);
	D16(USB_EP_NI4_RXCSR);
	D16(USB_EP_NI4_RXCOUNT);
	D16(USB_EP_NI4_TXTYPE);
	D16(USB_EP_NI4_TXINTERVAL);
	D16(USB_EP_NI4_RXTYPE);
	D16(USB_EP_NI4_RXINTERVAL);
	D16(USB_EP_NI4_TXCOUNT);
	D16(USB_EP_NI5_TXMAXP);
	D16(USB_EP_NI5_TXCSR);
	D16(USB_EP_NI5_RXMAXP);
	D16(USB_EP_NI5_RXCSR);
	D16(USB_EP_NI5_RXCOUNT);
	D16(USB_EP_NI5_TXTYPE);
	D16(USB_EP_NI5_TXINTERVAL);
	D16(USB_EP_NI5_RXTYPE);
	D16(USB_EP_NI5_RXINTERVAL);
	D16(USB_EP_NI5_TXCOUNT);
	D16(USB_EP_NI6_TXMAXP);
	D16(USB_EP_NI6_TXCSR);
	D16(USB_EP_NI6_RXMAXP);
	D16(USB_EP_NI6_RXCSR);
	D16(USB_EP_NI6_RXCOUNT);
	D16(USB_EP_NI6_TXTYPE);
	D16(USB_EP_NI6_TXINTERVAL);
	D16(USB_EP_NI6_RXTYPE);
	D16(USB_EP_NI6_RXINTERVAL);
	D16(USB_EP_NI6_TXCOUNT);
	D16(USB_EP_NI7_TXMAXP);
	D16(USB_EP_NI7_TXCSR);
	D16(USB_EP_NI7_RXMAXP);
	D16(USB_EP_NI7_RXCSR);
	D16(USB_EP_NI7_RXCOUNT);
	D16(USB_EP_NI7_TXTYPE);
	D16(USB_EP_NI7_TXINTERVAL);
	D16(USB_EP_NI7_RXTYPE);
	D16(USB_EP_NI7_RXINTERVAL);
	D16(USB_EP_NI7_TXCOUNT);
	D16(USB_DMA_INTERRUPT);
	D16(USB_DMA0CONTROL);
	D16(USB_DMA0ADDRLOW);
	D16(USB_DMA0ADDRHIGH);
	D16(USB_DMA0COUNTLOW);
	D16(USB_DMA0COUNTHIGH);
	D16(USB_DMA1CONTROL);
	D16(USB_DMA1ADDRLOW);
	D16(USB_DMA1ADDRHIGH);
	D16(USB_DMA1COUNTLOW);
	D16(USB_DMA1COUNTHIGH);
	D16(USB_DMA2CONTROL);
	D16(USB_DMA2ADDRLOW);
	D16(USB_DMA2ADDRHIGH);
	D16(USB_DMA2COUNTLOW);
	D16(USB_DMA2COUNTHIGH);
	D16(USB_DMA3CONTROL);
	D16(USB_DMA3ADDRLOW);
	D16(USB_DMA3ADDRHIGH);
	D16(USB_DMA3COUNTLOW);
	D16(USB_DMA3COUNTHIGH);
	D16(USB_DMA4CONTROL);
	D16(USB_DMA4ADDRLOW);
	D16(USB_DMA4ADDRHIGH);
	D16(USB_DMA4COUNTLOW);
	D16(USB_DMA4COUNTHIGH);
	D16(USB_DMA5CONTROL);
	D16(USB_DMA5ADDRLOW);
	D16(USB_DMA5ADDRHIGH);
	D16(USB_DMA5COUNTLOW);
	D16(USB_DMA5COUNTHIGH);
	D16(USB_DMA6CONTROL);
	D16(USB_DMA6ADDRLOW);
	D16(USB_DMA6ADDRHIGH);
	D16(USB_DMA6COUNTLOW);
	D16(USB_DMA6COUNTHIGH);
	D16(USB_DMA7CONTROL);
	D16(USB_DMA7ADDRLOW);
	D16(USB_DMA7ADDRHIGH);
	D16(USB_DMA7COUNTLOW);
	D16(USB_DMA7COUNTHIGH);
#endif

#ifdef WDOG_CNT
	parent = debugfs_create_dir("watchdog", top);
	D32(WDOG_CNT);
	D16(WDOG_CTL);
	D32(WDOG_STAT);
#endif
#ifdef WDOGA_CNT
	parent = debugfs_create_dir("watchdog", top);
	D32(WDOGA_CNT);
	D16(WDOGA_CTL);
	D32(WDOGA_STAT);
	D32(WDOGB_CNT);
	D16(WDOGB_CTL);
	D32(WDOGB_STAT);
#endif

#ifdef __ADSPBF51x__
# define USE_BF51x 1
#else
# define USE_BF51x 0
#endif
	if (USE_BF51x) {

		parent = debugfs_create_dir("GPIO PIN", top);
		d("PORTF_FER", 16, 0xFFC03200);
		d("PORTF_DRIVE", 16, 0xFFC03220);
		d("PORTF_HYSTERESIS", 16, 0xFFC03240);
		d("PORTF_MUX", 16, 0xFFC03210);
		d("PORTG_FER", 16, 0xFFC03204);
		d("PORTG_DRIVE", 16, 0xFFC03224);
		d("PORTG_HYSTERESIS", 16, 0xFFC03244);
		d("PORTG_MUX", 16, 0xFFC03214);
		d("PORTH_FER", 16, 0xFFC03208);
		d("PORTH_DRIVE", 16, 0xFFC03228);
		d("PORTH_HYSTERESIS", 16, 0xFFC03248);
		d("PORTH_MUX", 16, 0xFFC03218);

		parent = debugfs_create_dir("NON-GPIO", top);
		d("NONGPIO_DRIVE", 16, 0xFFC03280);
		d("NONGPIO_HYSTERESIS", 16, 0xFFC03288);

		parent = debugfs_create_dir("Port I-O", top);
		d("PORTFIO", 16, 0xFFC00700);
		d("PORTFIO_BOTH", 16, 0xFFC0073C);
		d("PORTFIO_CLEAR", 16, 0xFFC00704);
		d("PORTFIO_DIR", 16, 0xFFC00730);
		d("PORTFIO_EDGE", 16, 0xFFC00738);
		d("PORTFIO_INEN", 16, 0xFFC00740);
		d("PORTFIO_MASKA", 16, 0xFFC00710);
		d("PORTFIO_MASKA_CLEAR", 16, 0xFFC00714);
		d("PORTFIO_MASKA_SET", 16, 0xFFC00718);
		d("PORTFIO_MASKA_TOGGLE", 16, 0xFFC0071C);
		d("PORTFIO_MASKB", 16, 0xFFC00720);
		d("PORTFIO_MASKB_CLEAR", 16, 0xFFC00724);
		d("PORTFIO_MASKB_SET", 16, 0xFFC00728);
		d("PORTFIO_MASKB_TOGGLE", 16, 0xFFC0072C);
		d("PORTFIO_POLAR", 16, 0xFFC00734);
		d("PORTFIO_SET", 16, 0xFFC00708);
		d("PORTFIO_TOGGLE", 16, 0xFFC0070C);
		d("PORTGIO", 16, 0xFFC01500);
		d("PORTGIO_BOTH", 16, 0xFFC0153C);
		d("PORTGIO_CLEAR", 16, 0xFFC01504);
		d("PORTGIO_DIR", 16, 0xFFC01530);
		d("PORTGIO_EDGE", 16, 0xFFC01538);
		d("PORTGIO_INEN", 16, 0xFFC01540);
		d("PORTGIO_MASKA", 16, 0xFFC01510);
		d("PORTGIO_MASKA_CLEAR", 16, 0xFFC01514);
		d("PORTGIO_MASKA_SET", 16, 0xFFC01518);
		d("PORTGIO_MASKA_TOGGLE", 16, 0xFFC0151C);
		d("PORTGIO_MASKB", 16, 0xFFC01520);
		d("PORTGIO_MASKB_CLEAR", 16, 0xFFC01524);
		d("PORTGIO_MASKB_SET", 16, 0xFFC01528);
		d("PORTGIO_MASKB_TOGGLE", 16, 0xFFC0152C);
		d("PORTGIO_POLAR", 16, 0xFFC01534);
		d("PORTGIO_SET", 16, 0xFFC01508);
		d("PORTGIO_TOGGLE", 16, 0xFFC0150C);
		d("PORTHIO", 16, 0xFFC01700);
		d("PORTHIO_BOTH", 16, 0xFFC0173C);
		d("PORTHIO_CLEAR", 16, 0xFFC01704);
		d("PORTHIO_DIR", 16, 0xFFC01730);
		d("PORTHIO_EDGE", 16, 0xFFC01738);
		d("PORTHIO_INEN", 16, 0xFFC01740);
		d("PORTHIO_MASKA", 16, 0xFFC01710);
		d("PORTHIO_MASKA_CLEAR", 16, 0xFFC01714);
		d("PORTHIO_MASKA_SET", 16, 0xFFC01718);
		d("PORTHIO_MASKA_TOGGLE", 16, 0xFFC0171C);
		d("PORTHIO_MASKB", 16, 0xFFC01720);
		d("PORTHIO_MASKB_CLEAR", 16, 0xFFC01724);
		d("PORTHIO_MASKB_SET", 16, 0xFFC01728);
		d("PORTHIO_MASKB_TOGGLE", 16, 0xFFC0172C);
		d("PORTHIO_POLAR", 16, 0xFFC01734);
		d("PORTHIO_SET", 16, 0xFFC01708);
		d("PORTHIO_TOGGLE", 16, 0xFFC0170C);

	}	/* BF51x */

#ifdef __ADSPBF52x__
# define USE_BF52x 1
#else
# define USE_BF52x 0
#endif
	if (USE_BF52x) {

		parent = debugfs_create_dir("GPIO PIN", top);
		d("PORTF_FER", 16, 0xFFC03200);
		d("PORTF_DRIVE", 16, 0xFFC03220);
		d("PORTF_HYSTERESIS", 16, 0xFFC03240);
		d("PORTF_MUX", 16, 0xFFC03210);
		d("PORTF_SLEW", 16, 0xFFC03230);
		d("PORTG_FER", 16, 0xFFC03204);
		d("PORTG_DRIVE", 16, 0xFFC03224);
		d("PORTG_HYSTERESIS", 16, 0xFFC03244);
		d("PORTG_MUX", 16, 0xFFC03214);
		d("PORTG_SLEW", 16, 0xFFC03234);
		d("PORTH_FER", 16, 0xFFC03208);
		d("PORTH_DRIVE", 16, 0xFFC03228);
		d("PORTH_HYSTERESIS", 16, 0xFFC03248);
		d("PORTH_MUX", 16, 0xFFC03218);
		d("PORTH_SLEW", 16, 0xFFC03238);

		parent = debugfs_create_dir("NON-GPIO", top);
		d("NONGPIO_DRIVE", 16, 0xFFC03280);
		d("NONGPIO_HYSTERESIS", 16, 0xFFC03288);
		d("NONGPIO_SLEW", 16, 0xFFC03284);

		parent = debugfs_create_dir("Port I-O", top);
		d("PORTFIO", 16, 0xFFC00700);
		d("PORTFIO_BOTH", 16, 0xFFC0073C);
		d("PORTFIO_CLEAR", 16, 0xFFC00704);
		d("PORTFIO_DIR", 16, 0xFFC00730);
		d("PORTFIO_EDGE", 16, 0xFFC00738);
		d("PORTFIO_INEN", 16, 0xFFC00740);
		d("PORTFIO_MASKA", 16, 0xFFC00710);
		d("PORTFIO_MASKA_CLEAR", 16, 0xFFC00714);
		d("PORTFIO_MASKA_SET", 16, 0xFFC00718);
		d("PORTFIO_MASKA_TOGGLE", 16, 0xFFC0071C);
		d("PORTFIO_MASKB", 16, 0xFFC00720);
		d("PORTFIO_MASKB_CLEAR", 16, 0xFFC00724);
		d("PORTFIO_MASKB_SET", 16, 0xFFC00728);
		d("PORTFIO_MASKB_TOGGLE", 16, 0xFFC0072C);
		d("PORTFIO_POLAR", 16, 0xFFC00734);
		d("PORTFIO_SET", 16, 0xFFC00708);
		d("PORTFIO_TOGGLE", 16, 0xFFC0070C);
		d("PORTGIO", 16, 0xFFC01500);
		d("PORTGIO_BOTH", 16, 0xFFC0153C);
		d("PORTGIO_CLEAR", 16, 0xFFC01504);
		d("PORTGIO_DIR", 16, 0xFFC01530);
		d("PORTGIO_EDGE", 16, 0xFFC01538);
		d("PORTGIO_INEN", 16, 0xFFC01540);
		d("PORTGIO_MASKA", 16, 0xFFC01510);
		d("PORTGIO_MASKA_CLEAR", 16, 0xFFC01514);
		d("PORTGIO_MASKA_SET", 16, 0xFFC01518);
		d("PORTGIO_MASKA_TOGGLE", 16, 0xFFC0151C);
		d("PORTGIO_MASKB", 16, 0xFFC01520);
		d("PORTGIO_MASKB_CLEAR", 16, 0xFFC01524);
		d("PORTGIO_MASKB_SET", 16, 0xFFC01528);
		d("PORTGIO_MASKB_TOGGLE", 16, 0xFFC0152C);
		d("PORTGIO_POLAR", 16, 0xFFC01534);
		d("PORTGIO_SET", 16, 0xFFC01508);
		d("PORTGIO_TOGGLE", 16, 0xFFC0150C);
		d("PORTHIO", 16, 0xFFC01700);
		d("PORTHIO_BOTH", 16, 0xFFC0173C);
		d("PORTHIO_CLEAR", 16, 0xFFC01704);
		d("PORTHIO_DIR", 16, 0xFFC01730);
		d("PORTHIO_EDGE", 16, 0xFFC01738);
		d("PORTHIO_INEN", 16, 0xFFC01740);
		d("PORTHIO_MASKA", 16, 0xFFC01710);
		d("PORTHIO_MASKA_CLEAR", 16, 0xFFC01714);
		d("PORTHIO_MASKA_SET", 16, 0xFFC01718);
		d("PORTHIO_MASKA_TOGGLE", 16, 0xFFC0171C);
		d("PORTHIO_MASKB", 16, 0xFFC01720);
		d("PORTHIO_MASKB_CLEAR", 16, 0xFFC01724);
		d("PORTHIO_MASKB_SET", 16, 0xFFC01728);
		d("PORTHIO_MASKB_TOGGLE", 16, 0xFFC0172C);
		d("PORTHIO_POLAR", 16, 0xFFC01734);
		d("PORTHIO_SET", 16, 0xFFC01708);
		d("PORTHIO_TOGGLE", 16, 0xFFC0170C);

	}	/* BF52x */

#ifdef __ADSPBF531__
# define USE_BF531 1
#else
# define USE_BF531 0
#endif
#ifdef __ADSPBF532__
# define USE_BF532 1
#else
# define USE_BF532 0
#endif
#ifdef __ADSPBF533__
# define USE_BF533 1
#else
# define USE_BF533 0
#endif
	if (USE_BF531 || USE_BF532 || USE_BF533) {

		parent = debugfs_create_dir("Extended Registers", top);
		d("FIO_BOTH", 16, 0xFFC0073C);
		d("FIO_DIR", 16, 0xFFC00730);
		d("FIO_EDGE", 16, 0xFFC00738);
		d("FIO_FLAG_C", 16, 0xFFC00704);
		d("FIO_FLAG_D", 16, 0xFFC00700);
		d("FIO_FLAG_S", 16, 0xFFC00708);
		d("FIO_FLAG_T", 16, 0xFFC0070C);
		d("FIO_INEN", 16, 0xFFC00740);
		d("FIO_MASKA_C", 16, 0xFFC00714);
		d("FIO_MASKA_D", 16, 0xFFC00710);
		d("FIO_MASKA_S", 16, 0xFFC00718);
		d("FIO_MASKA_T", 16, 0xFFC0071C);
		d("FIO_MASKB_C", 16, 0xFFC00724);
		d("FIO_MASKB_D", 16, 0xFFC00720);
		d("FIO_MASKB_S", 16, 0xFFC00728);
		d("FIO_MASKB_T", 16, 0xFFC0072C);
		d("FIO_POLAR", 16, 0xFFC00734);

	}	/* BF531 BF532 BF533 */

#ifdef __ADSPBF534__
# define USE_BF534 1
#else
# define USE_BF534 0
#endif
#ifdef __ADSPBF536__
# define USE_BF536 1
#else
# define USE_BF536 0
#endif
#ifdef __ADSPBF537__
# define USE_BF537 1
#else
# define USE_BF537 0
#endif
	if (USE_BF534 || USE_BF536 || USE_BF537) {

		parent = debugfs_create_dir("Pin Control", top);
		d("PORTF_FER", 16, 0xFFC03200);
		d("PORTG_FER", 16, 0xFFC03204);
		d("PORTH_FER", 16, 0xFFC03208);
		d("PORT_MUX", 16, 0xFFC0320C);

		parent = debugfs_create_dir("Port I-O", top);
		d("PORTFIO", 16, 0xFFC00700);
		d("PORTFIO_BOTH", 16, 0xFFC0073C);
		d("PORTFIO_CLEAR", 16, 0xFFC00704);
		d("PORTFIO_DIR", 16, 0xFFC00730);
		d("PORTFIO_EDGE", 16, 0xFFC00738);
		d("PORTFIO_INEN", 16, 0xFFC00740);
		d("PORTFIO_MASKA", 16, 0xFFC00710);
		d("PORTFIO_MASKA_CLEAR", 16, 0xFFC00714);
		d("PORTFIO_MASKA_SET", 16, 0xFFC00718);
		d("PORTFIO_MASKA_TOGGLE", 16, 0xFFC0071C);
		d("PORTFIO_MASKB", 16, 0xFFC00720);
		d("PORTFIO_MASKB_CLEAR", 16, 0xFFC00724);
		d("PORTFIO_MASKB_SET", 16, 0xFFC00728);
		d("PORTFIO_MASKB_TOGGLE", 16, 0xFFC0072C);
		d("PORTFIO_POLAR", 16, 0xFFC00734);
		d("PORTFIO_SET", 16, 0xFFC00708);
		d("PORTFIO_TOGGLE", 16, 0xFFC0070C);
		d("PORTGIO", 16, 0xFFC01500);
		d("PORTGIO_BOTH", 16, 0xFFC0153C);
		d("PORTGIO_CLEAR", 16, 0xFFC01504);
		d("PORTGIO_DIR", 16, 0xFFC01530);
		d("PORTGIO_EDGE", 16, 0xFFC01538);
		d("PORTGIO_INEN", 16, 0xFFC01540);
		d("PORTGIO_MASKA", 16, 0xFFC01510);
		d("PORTGIO_MASKA_CLEAR", 16, 0xFFC01514);
		d("PORTGIO_MASKA_SET", 16, 0xFFC01518);
		d("PORTGIO_MASKA_TOGGLE", 16, 0xFFC0151C);
		d("PORTGIO_MASKB", 16, 0xFFC01520);
		d("PORTGIO_MASKB_CLEAR", 16, 0xFFC01524);
		d("PORTGIO_MASKB_SET", 16, 0xFFC01528);
		d("PORTGIO_MASKB_TOGGLE", 16, 0xFFC0152C);
		d("PORTGIO_POLAR", 16, 0xFFC01534);
		d("PORTGIO_SET", 16, 0xFFC01508);
		d("PORTGIO_TOGGLE", 16, 0xFFC0150C);
		d("PORTHIO", 16, 0xFFC01700);
		d("PORTHIO_BOTH", 16, 0xFFC0173C);
		d("PORTHIO_CLEAR", 16, 0xFFC01704);
		d("PORTHIO_DIR", 16, 0xFFC01730);
		d("PORTHIO_EDGE", 16, 0xFFC01738);
		d("PORTHIO_INEN", 16, 0xFFC01740);
		d("PORTHIO_MASKA", 16, 0xFFC01710);
		d("PORTHIO_MASKA_CLEAR", 16, 0xFFC01714);
		d("PORTHIO_MASKA_SET", 16, 0xFFC01718);
		d("PORTHIO_MASKA_TOGGLE", 16, 0xFFC0171C);
		d("PORTHIO_MASKB", 16, 0xFFC01720);
		d("PORTHIO_MASKB_CLEAR", 16, 0xFFC01724);
		d("PORTHIO_MASKB_SET", 16, 0xFFC01728);
		d("PORTHIO_MASKB_TOGGLE", 16, 0xFFC0172C);
		d("PORTHIO_POLAR", 16, 0xFFC01734);
		d("PORTHIO_SET", 16, 0xFFC01708);
		d("PORTHIO_TOGGLE", 16, 0xFFC0170C);

	}	/* BF534 BF536 BF537 */

#ifdef __ADSPBF538__
# define USE_BF538 1
#else
# define USE_BF538 0
#endif
#ifdef __ADSPBF539__
# define USE_BF539 1
#else
# define USE_BF539 0
#endif
	if (USE_BF538 || USE_BF539) {

		parent = debugfs_create_dir("GPIO Port C", top);
		d("PORTCIO", 16, 0xFFC01510);
		d("PORTCIO_CLEAR", 16, 0xFFC01520);
		d("PORTCIO_DIR", 16, 0xFFC01550);
		d("PORTCIO_FER", 16, 0xFFC01500);
		d("PORTCIO_INEN", 16, 0xFFC01560);
		d("PORTCIO_SET", 16, 0xFFC01530);
		d("PORTCIO_TOGGLE", 16, 0xFFC01540);

		parent = debugfs_create_dir("GPIO Port D", top);
		d("PORTDIO", 16, 0xFFC01514);
		d("PORTDIO_CLEAR", 16, 0xFFC01524);
		d("PORTDIO_DIR", 16, 0xFFC01554);
		d("PORTDIO_FER", 16, 0xFFC01504);
		d("PORTDIO_INEN", 16, 0xFFC01564);
		d("PORTDIO_SET", 16, 0xFFC01534);
		d("PORTDIO_TOGGLE", 16, 0xFFC01544);

		parent = debugfs_create_dir("GPIO Port E", top);
		d("PORTEIO", 16, 0xFFC01518);
		d("PORTEIO_CLEAR", 16, 0xFFC01528);
		d("PORTEIO_DIR", 16, 0xFFC01558);
		d("PORTEIO_FER", 16, 0xFFC01508);
		d("PORTEIO_INEN", 16, 0xFFC01568);
		d("PORTEIO_SET", 16, 0xFFC01538);
		d("PORTEIO_TOGGLE", 16, 0xFFC01548);

		parent = debugfs_create_dir("GPIO Port F", top);
		d("PORTFIO", 16, 0xFFC00700);
		d("PORTFIO_BOTH", 16, 0xFFC0073C);
		d("PORTFIO_CLEAR", 16, 0xFFC00704);
		d("PORTFIO_DIR", 16, 0xFFC00730);
		d("PORTFIO_EDGE", 16, 0xFFC00738);
		d("PORTFIO_INEN", 16, 0xFFC00740);
		d("PORTFIO_MASKA", 16, 0xFFC00710);
		d("PORTFIO_MASKA_CLEAR", 16, 0xFFC00714);
		d("PORTFIO_MASKA_SET", 16, 0xFFC00718);
		d("PORTFIO_MASKA_TOGGLE", 16, 0xFFC0071C);
		d("PORTFIO_MASKB", 16, 0xFFC00720);
		d("PORTFIO_MASKB_CLEAR", 16, 0xFFC00724);
		d("PORTFIO_MASKB_SET", 16, 0xFFC00728);
		d("PORTFIO_MASKB_TOGGLE", 16, 0xFFC0072C);
		d("PORTFIO_POLAR", 16, 0xFFC00734);
		d("PORTFIO_SET", 16, 0xFFC00708);
		d("PORTFIO_TOGGLE", 16, 0xFFC0070C);

	}	/* BF538 BF539 */

#ifdef __ADSPBF54x__
# define USE_BF54x 1
#else
# define USE_BF54x 0
#endif
	if (USE_BF54x) {

		parent = debugfs_create_dir("PINT_0", top);
		d("PINT0_ASSIGN", 32, 0xFFC0140C);
		d("PINT0_EDGE_CLEAR", 32, 0xFFC01414);
		d("PINT0_EDGE_SET", 32, 0xFFC01410);
		d("PINT0_INVERT_CLEAR", 32, 0xFFC0141C);
		d("PINT0_INVERT_SET", 32, 0xFFC01418);
		d("PINT0_IRQ", 32, 0xFFC01408);
		d("PINT0_LATCH", 32, 0xFFC01424);
		d("PINT0_MASK_CLEAR", 32, 0xFFC01404);
		d("PINT0_MASK_SET", 32, 0xFFC01400);
		d("PINT0_PINSTATE", 32, 0xFFC01420);

		parent = debugfs_create_dir("PINT_1", top);
		d("PINT1_ASSIGN", 32, 0xFFC0143C);
		d("PINT1_EDGE_CLEAR", 32, 0xFFC01444);
		d("PINT1_EDGE_SET", 32, 0xFFC01440);
		d("PINT1_INVERT_CLEAR", 32, 0xFFC0144C);
		d("PINT1_INVERT_SET", 32, 0xFFC01448);
		d("PINT1_IRQ", 32, 0xFFC01438);
		d("PINT1_LATCH", 32, 0xFFC01454);
		d("PINT1_MASK_CLEAR", 32, 0xFFC01434);
		d("PINT1_MASK_SET", 32, 0xFFC01430);
		d("PINT1_PINSTATE", 32, 0xFFC01450);

		parent = debugfs_create_dir("PINT_2", top);
		d("PINT2_ASSIGN", 32, 0xFFC0146C);
		d("PINT2_EDGE_CLEAR", 32, 0xFFC01474);
		d("PINT2_EDGE_SET", 32, 0xFFC01470);
		d("PINT2_INVERT_CLEAR", 32, 0xFFC0147C);
		d("PINT2_INVERT_SET", 32, 0xFFC01478);
		d("PINT2_IRQ", 32, 0xFFC01468);
		d("PINT2_LATCH", 32, 0xFFC01484);
		d("PINT2_MASK_CLEAR", 32, 0xFFC01464);
		d("PINT2_MASK_SET", 32, 0xFFC01460);
		d("PINT2_PINSTATE", 32, 0xFFC01480);

		parent = debugfs_create_dir("PINT_3", top);
		d("PINT3_ASSIGN", 32, 0xFFC0149C);
		d("PINT3_EDGE_CLEAR", 32, 0xFFC014A4);
		d("PINT3_EDGE_SET", 32, 0xFFC014A0);
		d("PINT3_INVERT_CLEAR", 32, 0xFFC014AC);
		d("PINT3_INVERT_SET", 32, 0xFFC014A8);
		d("PINT3_IRQ", 32, 0xFFC01498);
		d("PINT3_LATCH", 32, 0xFFC014B4);
		d("PINT3_MASK_CLEAR", 32, 0xFFC01494);
		d("PINT3_MASK_SET", 32, 0xFFC01490);
		d("PINT3_PINSTATE", 32, 0xFFC014B0);

		parent = debugfs_create_dir("Port_A", top);
		d("PORTA", 16, 0xFFC014C4);
		d("PORTA_CLEAR", 16, 0xFFC014CC);
		d("PORTA_DIR_CLEAR", 16, 0xFFC014D4);
		d("PORTA_DIR_SET", 16, 0xFFC014D0);
		d("PORTA_FER", 16, 0xFFC014C0);
		d("PORTA_INEN", 16, 0xFFC014D8);
		d("PORTA_MUX", 32, 0xFFC014DC);
		d("PORTA_SET", 16, 0xFFC014C8);

		parent = debugfs_create_dir("Port_B", top);
		d("PORTB", 16, 0xFFC014E4);
		d("PORTB_CLEAR", 16, 0xFFC014EC);
		d("PORTB_DIR_CLEAR", 16, 0xFFC014F4);
		d("PORTB_DIR_SET", 16, 0xFFC014F0);
		d("PORTB_FER", 16, 0xFFC014E0);
		d("PORTB_INEN", 16, 0xFFC014F8);
		d("PORTB_MUX", 32, 0xFFC014FC);
		d("PORTB_SET", 16, 0xFFC014E8);

		parent = debugfs_create_dir("Port_C", top);
		d("PORTC", 16, 0xFFC01504);
		d("PORTC_CLEAR", 16, 0xFFC0150C);
		d("PORTC_DIR_CLEAR", 16, 0xFFC01514);
		d("PORTC_DIR_SET", 16, 0xFFC01510);
		d("PORTC_FER", 16, 0xFFC01500);
		d("PORTC_INEN", 16, 0xFFC01518);
		d("PORTC_MUX", 32, 0xFFC0151C);
		d("PORTC_SET", 16, 0xFFC01508);

		parent = debugfs_create_dir("Port_D", top);
		d("PORTD", 16, 0xFFC01524);
		d("PORTD_CLEAR", 16, 0xFFC0152C);
		d("PORTD_DIR_CLEAR", 16, 0xFFC01534);
		d("PORTD_DIR_SET", 16, 0xFFC01530);
		d("PORTD_FER", 16, 0xFFC01520);
		d("PORTD_INEN", 16, 0xFFC01538);
		d("PORTD_MUX", 32, 0xFFC0153C);
		d("PORTD_SET", 16, 0xFFC01528);

		parent = debugfs_create_dir("Port_E", top);
		d("PORTE", 16, 0xFFC01544);
		d("PORTE_CLEAR", 16, 0xFFC0154C);
		d("PORTE_DIR_CLEAR", 16, 0xFFC01554);
		d("PORTE_DIR_SET", 16, 0xFFC01550);
		d("PORTE_FER", 16, 0xFFC01540);
		d("PORTE_INEN", 16, 0xFFC01558);
		d("PORTE_MUX", 32, 0xFFC0155C);
		d("PORTE_SET", 16, 0xFFC01548);

		parent = debugfs_create_dir("Port_F", top);
		d("PORTF", 16, 0xFFC01564);
		d("PORTF_CLEAR", 16, 0xFFC0156C);
		d("PORTF_DIR_CLEAR", 16, 0xFFC01574);
		d("PORTF_DIR_SET", 16, 0xFFC01570);
		d("PORTF_FER", 16, 0xFFC01560);
		d("PORTF_INEN", 16, 0xFFC01578);
		d("PORTF_MUX", 32, 0xFFC0157C);
		d("PORTF_SET", 16, 0xFFC01568);

		parent = debugfs_create_dir("Port_G", top);
		d("PORTG", 16, 0xFFC01584);
		d("PORTG_CLEAR", 16, 0xFFC0158C);
		d("PORTG_DIR_CLEAR", 16, 0xFFC01594);
		d("PORTG_DIR_SET", 16, 0xFFC01590);
		d("PORTG_FER", 16, 0xFFC01580);
		d("PORTG_INEN", 16, 0xFFC01598);
		d("PORTG_MUX", 32, 0xFFC0159C);
		d("PORTG_SET", 16, 0xFFC01588);

		parent = debugfs_create_dir("Port_H", top);
		d("PORTH", 16, 0xFFC015A4);
		d("PORTH_CLEAR", 16, 0xFFC015AC);
		d("PORTH_DIR_CLEAR", 16, 0xFFC015B4);
		d("PORTH_DIR_SET", 16, 0xFFC015B0);
		d("PORTH_FER", 16, 0xFFC015A0);
		d("PORTH_INEN", 16, 0xFFC015B8);
		d("PORTH_MUX", 32, 0xFFC015BC);
		d("PORTH_SET", 16, 0xFFC015A8);

		parent = debugfs_create_dir("Port_I", top);
		d("PORTI", 16, 0xFFC015C4);
		d("PORTI_CLEAR", 16, 0xFFC015CC);
		d("PORTI_DIR_CLEAR", 16, 0xFFC015D4);
		d("PORTI_DIR_SET", 16, 0xFFC015D0);
		d("PORTI_FER", 16, 0xFFC015C0);
		d("PORTI_INEN", 16, 0xFFC015D8);
		d("PORTI_MUX", 32, 0xFFC015DC);
		d("PORTI_SET", 16, 0xFFC015C8);

		parent = debugfs_create_dir("Port_J", top);
		d("PORTJ", 16, 0xFFC015E4);
		d("PORTJ_CLEAR", 16, 0xFFC015EC);
		d("PORTJ_DIR_CLEAR", 16, 0xFFC015F4);
		d("PORTJ_DIR_SET", 16, 0xFFC015F0);
		d("PORTJ_FER", 16, 0xFFC015E0);
		d("PORTJ_INEN", 16, 0xFFC015F8);
		d("PORTJ_MUX", 32, 0xFFC015FC);
		d("PORTJ_SET", 16, 0xFFC015E8);

	}	/* BF54x */

#ifdef __ADSPBF561__
# define USE_BF561 1
#else
# define USE_BF561 0
#endif
	if (USE_BF561) {

		parent = debugfs_create_dir("DMA1 Channel-0", top);
		d("DMA1_0_CONFIG", 16, 0xFFC01C08);
		d("DMA1_0_CURR_ADDR", 32, 0xFFC01C24);
		d("DMA1_0_CURR_DESC_PTR", 32, 0xFFC01C20);
		d("DMA1_0_CURR_X_COUNT", 16, 0xFFC01C30);
		d("DMA1_0_CURR_Y_COUNT", 16, 0xFFC01C38);
		d("DMA1_0_IRQ_STATUS", 16, 0xFFC01C28);
		d("DMA1_0_NEXT_DESC_PTR", 32, 0xFFC01C00);
		d("DMA1_0_PERIPHERAL_MAP", 16, 0xFFC01C2C);
		d("DMA1_0_START_ADDR", 32, 0xFFC01C04);
		d("DMA1_0_X_COUNT", 16, 0xFFC01C10);
		d("DMA1_0_X_MODIFY", 16, 0xFFC01C14);
		d("DMA1_0_Y_COUNT", 16, 0xFFC01C18);
		d("DMA1_0_Y_MODIFY", 16, 0xFFC01C1C);

		parent = debugfs_create_dir("DMA1 Channel-10", top);
		d("DMA1_10_CONFIG", 16, 0xFFC01E88);
		d("DMA1_10_CURR_ADDR", 32, 0xFFC01EA4);
		d("DMA1_10_CURR_DESC_PTR", 32, 0xFFC01EA0);
		d("DMA1_10_CURR_X_COUNT", 16, 0xFFC01EB0);
		d("DMA1_10_CURR_Y_COUNT", 16, 0xFFC01EB8);
		d("DMA1_10_IRQ_STATUS", 16, 0xFFC01EA8);
		d("DMA1_10_NEXT_DESC_PTR", 32, 0xFFC01E80);
		d("DMA1_10_PERIPHERAL_MAP", 16, 0xFFC01EAC);
		d("DMA1_10_START_ADDR", 32, 0xFFC01E84);
		d("DMA1_10_X_COUNT", 16, 0xFFC01E90);
		d("DMA1_10_X_MODIFY", 16, 0xFFC01E94);
		d("DMA1_10_Y_COUNT", 16, 0xFFC01E98);
		d("DMA1_10_Y_MODIFY", 16, 0xFFC01E9C);

		parent = debugfs_create_dir("DMA1 Channel-11", top);
		d("DMA1_11_CONFIG", 16, 0xFFC01EC8);
		d("DMA1_11_CURR_ADDR", 32, 0xFFC01EE4);
		d("DMA1_11_CURR_DESC_PTR", 32, 0xFFC01EE0);
		d("DMA1_11_CURR_X_COUNT", 16, 0xFFC01EF0);
		d("DMA1_11_CURR_Y_COUNT", 16, 0xFFC01EF8);
		d("DMA1_11_IRQ_STATUS", 16, 0xFFC01EE8);
		d("DMA1_11_NEXT_DESC_PTR", 32, 0xFFC01EC0);
		d("DMA1_11_PERIPHERAL_MAP", 16, 0xFFC01EEC);
		d("DMA1_11_START_ADDR", 32, 0xFFC01EC4);
		d("DMA1_11_X_COUNT", 16, 0xFFC01ED0);
		d("DMA1_11_X_MODIFY", 16, 0xFFC01ED4);
		d("DMA1_11_Y_COUNT", 16, 0xFFC01ED8);
		d("DMA1_11_Y_MODIFY", 16, 0xFFC01EDC);

		parent = debugfs_create_dir("DMA1 Channel-1", top);
		d("DMA1_1_CONFIG", 16, 0xFFC01C48);
		d("DMA1_1_CURR_ADDR", 32, 0xFFC01C64);
		d("DMA1_1_CURR_DESC_PTR", 32, 0xFFC01C60);
		d("DMA1_1_CURR_X_COUNT", 16, 0xFFC01C70);
		d("DMA1_1_CURR_Y_COUNT", 16, 0xFFC01C78);
		d("DMA1_1_IRQ_STATUS", 16, 0xFFC01C68);
		d("DMA1_1_NEXT_DESC_PTR", 32, 0xFFC01C40);
		d("DMA1_1_PERIPHERAL_MAP", 16, 0xFFC01C6C);
		d("DMA1_1_START_ADDR", 32, 0xFFC01C44);
		d("DMA1_1_X_COUNT", 16, 0xFFC01C50);
		d("DMA1_1_X_MODIFY", 16, 0xFFC01C54);
		d("DMA1_1_Y_COUNT", 16, 0xFFC01C58);
		d("DMA1_1_Y_MODIFY", 16, 0xFFC01C5C);

		parent = debugfs_create_dir("DMA1 Channel-2", top);
		d("DMA1_2_CONFIG", 16, 0xFFC01C88);
		d("DMA1_2_CURR_ADDR", 32, 0xFFC01CA4);
		d("DMA1_2_CURR_DESC_PTR", 32, 0xFFC01CA0);
		d("DMA1_2_CURR_X_COUNT", 16, 0xFFC01CB0);
		d("DMA1_2_CURR_Y_COUNT", 16, 0xFFC01CB8);
		d("DMA1_2_IRQ_STATUS", 16, 0xFFC01CA8);
		d("DMA1_2_NEXT_DESC_PTR", 32, 0xFFC01C80);
		d("DMA1_2_PERIPHERAL_MAP", 16, 0xFFC01CAC);
		d("DMA1_2_START_ADDR", 32, 0xFFC01C84);
		d("DMA1_2_X_COUNT", 16, 0xFFC01C90);
		d("DMA1_2_X_MODIFY", 16, 0xFFC01C94);
		d("DMA1_2_Y_COUNT", 16, 0xFFC01C98);
		d("DMA1_2_Y_MODIFY", 16, 0xFFC01C9C);

		parent = debugfs_create_dir("DMA1 Channel-3", top);
		d("DMA1_3_CONFIG", 16, 0xFFC01CC8);
		d("DMA1_3_CURR_ADDR", 32, 0xFFC01CE4);
		d("DMA1_3_CURR_DESC_PTR", 32, 0xFFC01CE0);
		d("DMA1_3_CURR_X_COUNT", 16, 0xFFC01CF0);
		d("DMA1_3_CURR_Y_COUNT", 16, 0xFFC01CF8);
		d("DMA1_3_IRQ_STATUS", 16, 0xFFC01CE8);
		d("DMA1_3_NEXT_DESC_PTR", 32, 0xFFC01CC0);
		d("DMA1_3_PERIPHERAL_MAP", 16, 0xFFC01CEC);
		d("DMA1_3_START_ADDR", 32, 0xFFC01CC4);
		d("DMA1_3_X_COUNT", 16, 0xFFC01CD0);
		d("DMA1_3_X_MODIFY", 16, 0xFFC01CD4);
		d("DMA1_3_Y_COUNT", 16, 0xFFC01CD8);
		d("DMA1_3_Y_MODIFY", 16, 0xFFC01CDC);

		parent = debugfs_create_dir("DMA1 Channel-4", top);
		d("DMA1_4_CONFIG", 16, 0xFFC01D08);
		d("DMA1_4_CURR_ADDR", 32, 0xFFC01D24);
		d("DMA1_4_CURR_DESC_PTR", 32, 0xFFC01D20);
		d("DMA1_4_CURR_X_COUNT", 16, 0xFFC01D30);
		d("DMA1_4_CURR_Y_COUNT", 16, 0xFFC01D38);
		d("DMA1_4_IRQ_STATUS", 16, 0xFFC01D28);
		d("DMA1_4_NEXT_DESC_PTR", 32, 0xFFC01D00);
		d("DMA1_4_PERIPHERAL_MAP", 16, 0xFFC01D2C);
		d("DMA1_4_START_ADDR", 32, 0xFFC01D04);
		d("DMA1_4_X_COUNT", 16, 0xFFC01D10);
		d("DMA1_4_X_MODIFY", 16, 0xFFC01D14);
		d("DMA1_4_Y_COUNT", 16, 0xFFC01D18);
		d("DMA1_4_Y_MODIFY", 16, 0xFFC01D1C);

		parent = debugfs_create_dir("DMA1 Channel-5", top);
		d("DMA1_5_CONFIG", 16, 0xFFC01D48);
		d("DMA1_5_CURR_ADDR", 32, 0xFFC01D64);
		d("DMA1_5_CURR_DESC_PTR", 32, 0xFFC01D60);
		d("DMA1_5_CURR_X_COUNT", 16, 0xFFC01D70);
		d("DMA1_5_CURR_Y_COUNT", 16, 0xFFC01D78);
		d("DMA1_5_IRQ_STATUS", 16, 0xFFC01D68);
		d("DMA1_5_NEXT_DESC_PTR", 32, 0xFFC01D40);
		d("DMA1_5_PERIPHERAL_MAP", 16, 0xFFC01D6C);
		d("DMA1_5_START_ADDR", 32, 0xFFC01D44);
		d("DMA1_5_X_COUNT", 16, 0xFFC01D50);
		d("DMA1_5_X_MODIFY", 16, 0xFFC01D54);
		d("DMA1_5_Y_COUNT", 16, 0xFFC01D58);
		d("DMA1_5_Y_MODIFY", 16, 0xFFC01D5C);

		parent = debugfs_create_dir("DMA1 Channel-6", top);
		d("DMA1_6_CONFIG", 16, 0xFFC01D88);
		d("DMA1_6_CURR_ADDR", 32, 0xFFC01DA4);
		d("DMA1_6_CURR_DESC_PTR", 32, 0xFFC01DA0);
		d("DMA1_6_CURR_X_COUNT", 16, 0xFFC01DB0);
		d("DMA1_6_CURR_Y_COUNT", 16, 0xFFC01DB8);
		d("DMA1_6_IRQ_STATUS", 16, 0xFFC01DA8);
		d("DMA1_6_NEXT_DESC_PTR", 32, 0xFFC01D80);
		d("DMA1_6_PERIPHERAL_MAP", 16, 0xFFC01DAC);
		d("DMA1_6_START_ADDR", 32, 0xFFC01D84);
		d("DMA1_6_X_COUNT", 16, 0xFFC01D90);
		d("DMA1_6_X_MODIFY", 16, 0xFFC01D94);
		d("DMA1_6_Y_COUNT", 16, 0xFFC01D98);
		d("DMA1_6_Y_MODIFY", 16, 0xFFC01D9C);

		parent = debugfs_create_dir("DMA1 Channel-7", top);
		d("DMA1_7_CONFIG", 16, 0xFFC01DC8);
		d("DMA1_7_CURR_ADDR", 32, 0xFFC01DE4);
		d("DMA1_7_CURR_DESC_PTR", 32, 0xFFC01DE0);
		d("DMA1_7_CURR_X_COUNT", 16, 0xFFC01DF0);
		d("DMA1_7_CURR_Y_COUNT", 16, 0xFFC01DF8);
		d("DMA1_7_IRQ_STATUS", 16, 0xFFC01DE8);
		d("DMA1_7_NEXT_DESC_PTR", 32, 0xFFC01DC0);
		d("DMA1_7_PERIPHERAL_MAP", 16, 0xFFC01DEC);
		d("DMA1_7_START_ADDR", 32, 0xFFC01DC4);
		d("DMA1_7_X_COUNT", 16, 0xFFC01DD0);
		d("DMA1_7_X_MODIFY", 16, 0xFFC01DD4);
		d("DMA1_7_Y_COUNT", 16, 0xFFC01DD8);
		d("DMA1_7_Y_MODIFY", 16, 0xFFC01DDC);

		parent = debugfs_create_dir("DMA1 Channel-8", top);
		d("DMA1_8_CONFIG", 16, 0xFFC01E08);
		d("DMA1_8_CURR_ADDR", 32, 0xFFC01E24);
		d("DMA1_8_CURR_DESC_PTR", 32, 0xFFC01E20);
		d("DMA1_8_CURR_X_COUNT", 16, 0xFFC01E30);
		d("DMA1_8_CURR_Y_COUNT", 16, 0xFFC01E38);
		d("DMA1_8_IRQ_STATUS", 16, 0xFFC01E28);
		d("DMA1_8_NEXT_DESC_PTR", 32, 0xFFC01E00);
		d("DMA1_8_PERIPHERAL_MAP", 16, 0xFFC01E2C);
		d("DMA1_8_START_ADDR", 32, 0xFFC01E04);
		d("DMA1_8_X_COUNT", 16, 0xFFC01E10);
		d("DMA1_8_X_MODIFY", 16, 0xFFC01E14);
		d("DMA1_8_Y_COUNT", 16, 0xFFC01E18);
		d("DMA1_8_Y_MODIFY", 16, 0xFFC01E1C);

		parent = debugfs_create_dir("DMA1 Channel-9", top);
		d("DMA1_9_CONFIG", 16, 0xFFC01E48);
		d("DMA1_9_CURR_ADDR", 32, 0xFFC01E64);
		d("DMA1_9_CURR_DESC_PTR", 32, 0xFFC01E60);
		d("DMA1_9_CURR_X_COUNT", 16, 0xFFC01E70);
		d("DMA1_9_CURR_Y_COUNT", 16, 0xFFC01E78);
		d("DMA1_9_IRQ_STATUS", 16, 0xFFC01E68);
		d("DMA1_9_NEXT_DESC_PTR", 32, 0xFFC01E40);
		d("DMA1_9_PERIPHERAL_MAP", 16, 0xFFC01E6C);
		d("DMA1_9_START_ADDR", 32, 0xFFC01E44);
		d("DMA1_9_X_COUNT", 16, 0xFFC01E50);
		d("DMA1_9_X_MODIFY", 16, 0xFFC01E54);
		d("DMA1_9_Y_COUNT", 16, 0xFFC01E58);
		d("DMA1_9_Y_MODIFY", 16, 0xFFC01E5C);

		parent = debugfs_create_dir("DMA2 Channel 0", top);
		d("DMA2_0_CONFIG", 16, 0xFFC00C08);
		d("DMA2_0_CURR_ADDR", 32, 0xFFC00C24);
		d("DMA2_0_CURR_DESC_PTR", 32, 0xFFC00C20);
		d("DMA2_0_CURR_X_COUNT", 16, 0xFFC00C30);
		d("DMA2_0_CURR_Y_COUNT", 16, 0xFFC00C38);
		d("DMA2_0_IRQ_STATUS", 16, 0xFFC00C28);
		d("DMA2_0_NEXT_DESC_PTR", 32, 0xFFC00C00);
		d("DMA2_0_PERIPHERAL_MAP", 16, 0xFFC00C2C);
		d("DMA2_0_START_ADDR", 32, 0xFFC00C04);
		d("DMA2_0_X_COUNT", 16, 0xFFC00C10);
		d("DMA2_0_X_MODIFY", 16, 0xFFC00C14);
		d("DMA2_0_Y_COUNT", 16, 0xFFC00C18);
		d("DMA2_0_Y_MODIFY", 16, 0xFFC00C1C);

		parent = debugfs_create_dir("DMA2 Channel 10", top);
		d("DMA2_10_CONFIG", 16, 0xFFC00E88);
		d("DMA2_10_CURR_ADDR", 32, 0xFFC00EA4);
		d("DMA2_10_CURR_DESC_PTR", 32, 0xFFC00EA0);
		d("DMA2_10_CURR_X_COUNT", 16, 0xFFC00EB0);
		d("DMA2_10_CURR_Y_COUNT", 16, 0xFFC00EB8);
		d("DMA2_10_IRQ_STATUS", 16, 0xFFC00EA8);
		d("DMA2_10_NEXT_DESC_PTR", 32, 0xFFC00E80);
		d("DMA2_10_PERIPHERAL_MAP", 16, 0xFFC00EAC);
		d("DMA2_10_START_ADDR", 32, 0xFFC00E84);
		d("DMA2_10_X_COUNT", 16, 0xFFC00E90);
		d("DMA2_10_X_MODIFY", 16, 0xFFC00E94);
		d("DMA2_10_Y_COUNT", 16, 0xFFC00E98);
		d("DMA2_10_Y_MODIFY", 16, 0xFFC00E9C);

		parent = debugfs_create_dir("DMA2 Channel 11", top);
		d("DMA2_11_CONFIG", 16, 0xFFC00EC8);
		d("DMA2_11_CURR_ADDR", 32, 0xFFC00EE4);
		d("DMA2_11_CURR_DESC_PTR", 32, 0xFFC00EE0);
		d("DMA2_11_CURR_X_COUNT", 16, 0xFFC00EF0);
		d("DMA2_11_CURR_Y_COUNT", 16, 0xFFC00EF8);
		d("DMA2_11_IRQ_STATUS", 16, 0xFFC00EE8);
		d("DMA2_11_NEXT_DESC_PTR", 32, 0xFFC00EC0);
		d("DMA2_11_PERIPHERAL_MAP", 16, 0xFFC00EEC);
		d("DMA2_11_START_ADDR", 32, 0xFFC00EC4);
		d("DMA2_11_X_COUNT", 16, 0xFFC00ED0);
		d("DMA2_11_X_MODIFY", 16, 0xFFC00ED4);
		d("DMA2_11_Y_COUNT", 16, 0xFFC00ED8);
		d("DMA2_11_Y_MODIFY", 16, 0xFFC00EDC);

		parent = debugfs_create_dir("DMA2 Channel 1", top);
		d("DMA2_1_CONFIG", 16, 0xFFC00C48);
		d("DMA2_1_CURR_ADDR", 32, 0xFFC00C64);
		d("DMA2_1_CURR_DESC_PTR", 32, 0xFFC00C60);
		d("DMA2_1_CURR_X_COUNT", 16, 0xFFC00C70);
		d("DMA2_1_CURR_Y_COUNT", 16, 0xFFC00C78);
		d("DMA2_1_IRQ_STATUS", 16, 0xFFC00C68);
		d("DMA2_1_NEXT_DESC_PTR", 32, 0xFFC00C40);
		d("DMA2_1_PERIPHERAL_MAP", 16, 0xFFC00C6C);
		d("DMA2_1_START_ADDR", 32, 0xFFC00C44);
		d("DMA2_1_X_COUNT", 16, 0xFFC00C50);
		d("DMA2_1_X_MODIFY", 16, 0xFFC00C54);
		d("DMA2_1_Y_COUNT", 16, 0xFFC00C58);
		d("DMA2_1_Y_MODIFY", 16, 0xFFC00C5C);

		parent = debugfs_create_dir("DMA2 Channel 2", top);
		d("DMA2_2_CONFIG", 16, 0xFFC00C88);
		d("DMA2_2_CURR_ADDR", 32, 0xFFC00CA4);
		d("DMA2_2_CURR_DESC_PTR", 32, 0xFFC00CA0);
		d("DMA2_2_CURR_X_COUNT", 16, 0xFFC00CB0);
		d("DMA2_2_CURR_Y_COUNT", 16, 0xFFC00CB8);
		d("DMA2_2_IRQ_STATUS", 16, 0xFFC00CA8);
		d("DMA2_2_NEXT_DESC_PTR", 32, 0xFFC00C80);
		d("DMA2_2_PERIPHERAL_MAP", 16, 0xFFC00CAC);
		d("DMA2_2_START_ADDR", 32, 0xFFC00C84);
		d("DMA2_2_X_COUNT", 16, 0xFFC00C90);
		d("DMA2_2_X_MODIFY", 16, 0xFFC00C94);
		d("DMA2_2_Y_COUNT", 16, 0xFFC00C98);
		d("DMA2_2_Y_MODIFY", 16, 0xFFC00C9C);

		parent = debugfs_create_dir("DMA2 Channel 3", top);
		d("DMA2_3_CONFIG", 16, 0xFFC00CC8);
		d("DMA2_3_CURR_ADDR", 32, 0xFFC00CE4);
		d("DMA2_3_CURR_DESC_PTR", 32, 0xFFC00CE0);
		d("DMA2_3_CURR_X_COUNT", 16, 0xFFC00CF0);
		d("DMA2_3_CURR_Y_COUNT", 16, 0xFFC00CF8);
		d("DMA2_3_IRQ_STATUS", 16, 0xFFC00CE8);
		d("DMA2_3_NEXT_DESC_PTR", 32, 0xFFC00CC0);
		d("DMA2_3_PERIPHERAL_MAP", 16, 0xFFC00CEC);
		d("DMA2_3_START_ADDR", 32, 0xFFC00CC4);
		d("DMA2_3_X_COUNT", 16, 0xFFC00CD0);
		d("DMA2_3_X_MODIFY", 16, 0xFFC00CD4);
		d("DMA2_3_Y_COUNT", 16, 0xFFC00CD8);
		d("DMA2_3_Y_MODIFY", 16, 0xFFC00CDC);

		parent = debugfs_create_dir("DMA2 Channel 4", top);
		d("DMA2_4_CONFIG", 16, 0xFFC00D08);
		d("DMA2_4_CURR_ADDR", 32, 0xFFC00D24);
		d("DMA2_4_CURR_DESC_PTR", 32, 0xFFC00D20);
		d("DMA2_4_CURR_X_COUNT", 16, 0xFFC00D30);
		d("DMA2_4_CURR_Y_COUNT", 16, 0xFFC00D38);
		d("DMA2_4_IRQ_STATUS", 16, 0xFFC00D28);
		d("DMA2_4_NEXT_DESC_PTR", 32, 0xFFC00D00);
		d("DMA2_4_PERIPHERAL_MAP", 16, 0xFFC00D2C);
		d("DMA2_4_START_ADDR", 32, 0xFFC00D04);
		d("DMA2_4_X_COUNT", 16, 0xFFC00D10);
		d("DMA2_4_X_MODIFY", 16, 0xFFC00D14);
		d("DMA2_4_Y_COUNT", 16, 0xFFC00D18);
		d("DMA2_4_Y_MODIFY", 16, 0xFFC00D1C);

		parent = debugfs_create_dir("DMA2 Channel 5", top);
		d("DMA2_5_CONFIG", 16, 0xFFC00D48);
		d("DMA2_5_CURR_ADDR", 32, 0xFFC00D64);
		d("DMA2_5_CURR_DESC_PTR", 32, 0xFFC00D60);
		d("DMA2_5_CURR_X_COUNT", 16, 0xFFC00D70);
		d("DMA2_5_CURR_Y_COUNT", 16, 0xFFC00D78);
		d("DMA2_5_IRQ_STATUS", 16, 0xFFC00D68);
		d("DMA2_5_NEXT_DESC_PTR", 32, 0xFFC00D40);
		d("DMA2_5_PERIPHERAL_MAP", 16, 0xFFC00D6C);
		d("DMA2_5_START_ADDR", 32, 0xFFC00D44);
		d("DMA2_5_X_COUNT", 16, 0xFFC00D50);
		d("DMA2_5_X_MODIFY", 16, 0xFFC00D54);
		d("DMA2_5_Y_COUNT", 16, 0xFFC00D58);
		d("DMA2_5_Y_MODIFY", 16, 0xFFC00D5C);

		parent = debugfs_create_dir("DMA2 Channel 6", top);
		d("DMA2_6_CONFIG", 16, 0xFFC00D88);
		d("DMA2_6_CURR_ADDR", 32, 0xFFC00DA4);
		d("DMA2_6_CURR_DESC_PTR", 32, 0xFFC00DA0);
		d("DMA2_6_CURR_X_COUNT", 16, 0xFFC00DB0);
		d("DMA2_6_CURR_Y_COUNT", 16, 0xFFC00DB8);
		d("DMA2_6_IRQ_STATUS", 16, 0xFFC00DA8);
		d("DMA2_6_NEXT_DESC_PTR", 32, 0xFFC00D80);
		d("DMA2_6_PERIPHERAL_MAP", 16, 0xFFC00DAC);
		d("DMA2_6_START_ADDR", 32, 0xFFC00D84);
		d("DMA2_6_X_COUNT", 16, 0xFFC00D90);
		d("DMA2_6_X_MODIFY", 16, 0xFFC00D94);
		d("DMA2_6_Y_COUNT", 16, 0xFFC00D98);
		d("DMA2_6_Y_MODIFY", 16, 0xFFC00D9C);

		parent = debugfs_create_dir("DMA2 Channel 7", top);
		d("DMA2_7_CONFIG", 16, 0xFFC00DC8);
		d("DMA2_7_CURR_ADDR", 32, 0xFFC00DE4);
		d("DMA2_7_CURR_DESC_PTR", 32, 0xFFC00DE0);
		d("DMA2_7_CURR_X_COUNT", 16, 0xFFC00DF0);
		d("DMA2_7_CURR_Y_COUNT", 16, 0xFFC00DF8);
		d("DMA2_7_IRQ_STATUS", 16, 0xFFC00DE8);
		d("DMA2_7_NEXT_DESC_PTR", 32, 0xFFC00DC0);
		d("DMA2_7_PERIPHERAL_MAP", 16, 0xFFC00DEC);
		d("DMA2_7_START_ADDR", 32, 0xFFC00DC4);
		d("DMA2_7_X_COUNT", 16, 0xFFC00DD0);
		d("DMA2_7_X_MODIFY", 16, 0xFFC00DD4);
		d("DMA2_7_Y_COUNT", 16, 0xFFC00DD8);
		d("DMA2_7_Y_MODIFY", 16, 0xFFC00DDC);

		parent = debugfs_create_dir("DMA2 Channel 8", top);
		d("DMA2_8_CONFIG", 16, 0xFFC00E08);
		d("DMA2_8_CURR_ADDR", 32, 0xFFC00E24);
		d("DMA2_8_CURR_DESC_PTR", 32, 0xFFC00E20);
		d("DMA2_8_CURR_X_COUNT", 16, 0xFFC00E30);
		d("DMA2_8_CURR_Y_COUNT", 16, 0xFFC00E38);
		d("DMA2_8_IRQ_STATUS", 16, 0xFFC00E28);
		d("DMA2_8_NEXT_DESC_PTR", 32, 0xFFC00E00);
		d("DMA2_8_PERIPHERAL_MAP", 16, 0xFFC00E2C);
		d("DMA2_8_START_ADDR", 32, 0xFFC00E04);
		d("DMA2_8_X_COUNT", 16, 0xFFC00E10);
		d("DMA2_8_X_MODIFY", 16, 0xFFC00E14);
		d("DMA2_8_Y_COUNT", 16, 0xFFC00E18);
		d("DMA2_8_Y_MODIFY", 16, 0xFFC00E1C);

		parent = debugfs_create_dir("DMA2 Channel 9", top);
		d("DMA2_9_CONFIG", 16, 0xFFC00E48);
		d("DMA2_9_CURR_ADDR", 32, 0xFFC00E64);
		d("DMA2_9_CURR_DESC_PTR", 32, 0xFFC00E60);
		d("DMA2_9_CURR_X_COUNT", 16, 0xFFC00E70);
		d("DMA2_9_CURR_Y_COUNT", 16, 0xFFC00E78);
		d("DMA2_9_IRQ_STATUS", 16, 0xFFC00E68);
		d("DMA2_9_NEXT_DESC_PTR", 32, 0xFFC00E40);
		d("DMA2_9_PERIPHERAL_MAP", 16, 0xFFC00E6C);
		d("DMA2_9_START_ADDR", 32, 0xFFC00E44);
		d("DMA2_9_X_COUNT", 16, 0xFFC00E50);
		d("DMA2_9_X_MODIFY", 16, 0xFFC00E54);
		d("DMA2_9_Y_COUNT", 16, 0xFFC00E58);
		d("DMA2_9_Y_MODIFY", 16, 0xFFC00E5C);

		parent = debugfs_create_dir("Flag 0", top);
		d("FIO0_BOTH", 16, 0xFFC0073C);
		d("FIO0_DIR", 16, 0xFFC00730);
		d("FIO0_EDGE", 16, 0xFFC00738);
		d("FIO0_FLAG_C", 16, 0xFFC00704);
		d("FIO0_FLAG_D", 16, 0xFFC00700);
		d("FIO0_FLAG_S", 16, 0xFFC00708);
		d("FIO0_FLAG_T", 16, 0xFFC0070C);
		d("FIO0_INEN", 16, 0xFFC00740);
		d("FIO0_MASKA_C", 16, 0xFFC00714);
		d("FIO0_MASKA_D", 16, 0xFFC00710);
		d("FIO0_MASKA_S", 16, 0xFFC00718);
		d("FIO0_MASKA_T", 16, 0xFFC0071C);
		d("FIO0_MASKB_C", 16, 0xFFC00724);
		d("FIO0_MASKB_D", 16, 0xFFC00720);
		d("FIO0_MASKB_S", 16, 0xFFC00728);
		d("FIO0_MASKB_T", 16, 0xFFC0072C);
		d("FIO0_POLAR", 16, 0xFFC00734);

		parent = debugfs_create_dir("Flag 1", top);
		d("FIO1_BOTH", 16, 0xFFC0153C);
		d("FIO1_DIR", 16, 0xFFC01530);
		d("FIO1_EDGE", 16, 0xFFC01538);
		d("FIO1_FLAG_C", 16, 0xFFC01504);
		d("FIO1_FLAG_D", 16, 0xFFC01500);
		d("FIO1_FLAG_S", 16, 0xFFC01508);
		d("FIO1_FLAG_T", 16, 0xFFC0150C);
		d("FIO1_INEN", 16, 0xFFC01540);
		d("FIO1_MASKA_C", 16, 0xFFC01514);
		d("FIO1_MASKA_D", 16, 0xFFC01510);
		d("FIO1_MASKA_S", 16, 0xFFC01518);
		d("FIO1_MASKA_T", 16, 0xFFC0151C);
		d("FIO1_MASKB_C", 16, 0xFFC01524);
		d("FIO1_MASKB_D", 16, 0xFFC01520);
		d("FIO1_MASKB_S", 16, 0xFFC01528);
		d("FIO1_MASKB_T", 16, 0xFFC0152C);
		d("FIO1_POLAR", 16, 0xFFC01534);

		parent = debugfs_create_dir("Flag 2", top);
		d("FIO2_BOTH", 16, 0xFFC0173C);
		d("FIO2_DIR", 16, 0xFFC01730);
		d("FIO2_EDGE", 16, 0xFFC01738);
		d("FIO2_FLAG_C", 16, 0xFFC01704);
		d("FIO2_FLAG_D", 16, 0xFFC01700);
		d("FIO2_FLAG_S", 16, 0xFFC01708);
		d("FIO2_FLAG_T", 16, 0xFFC0170C);
		d("FIO2_INEN", 16, 0xFFC01740);
		d("FIO2_MASKA_C", 16, 0xFFC01714);
		d("FIO2_MASKA_D", 16, 0xFFC01710);
		d("FIO2_MASKA_S", 16, 0xFFC01718);
		d("FIO2_MASKA_T", 16, 0xFFC0171C);
		d("FIO2_MASKB_C", 16, 0xFFC01724);
		d("FIO2_MASKB_D", 16, 0xFFC01720);
		d("FIO2_MASKB_S", 16, 0xFFC01728);
		d("FIO2_MASKB_T", 16, 0xFFC0172C);
		d("FIO2_POLAR", 16, 0xFFC01734);

	}	/* BF561 */

	debug_mmrs_dentry = top;

	return 0;
}
module_init(bfin_debug_mmrs_init);

static void __exit bfin_debug_mmrs_exit(void)
{
	debugfs_remove_recursive(debug_mmrs_dentry);
}
module_exit(bfin_debug_mmrs_exit);

MODULE_LICENSE("GPL");

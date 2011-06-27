/*
 * Copyright 2004-2008 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/linkage.h>
#include <asm/blackfin.h>
#include <mach/irq.h>
#include <asm/dpmc.h>

.section .l1.text

ENTRY(_sleep_mode)
	[--SP] = ( R7:0, P5:0 );
	[--SP] =  RETS;

	call _set_sic_iwr;

	P0.H = hi(PLL_CTL);
	P0.L = lo(PLL_CTL);
	R1 = W[P0](z);
	BITSET (R1, 3);
	W[P0] = R1.L;

	CLI R2;
	SSYNC;
	IDLE;
	STI R2;

	call _test_pll_locked;

	R0 = IWR_ENABLE(0);
	R1 = IWR_DISABLE_ALL;
	R2 = IWR_DISABLE_ALL;

	call _set_sic_iwr;

	P0.H = hi(PLL_CTL);
	P0.L = lo(PLL_CTL);
	R7 = w[p0](z);
	BITCLR (R7, 3);
	BITCLR (R7, 5);
	w[p0] = R7.L;
	IDLE;
	call _test_pll_locked;

	RETS = [SP++];
	( R7:0, P5:0 ) = [SP++];
	RTS;
ENDPROC(_sleep_mode)

/*
 * This func never returns as it puts the part into hibernate, and
 * is only called from do_hibernate, so we don't bother saving or
 * restoring any of the normal C runtime state.  When we wake up,
 * the entry point will be in do_hibernate and not here.
 *
 * We accept just one argument -- the value to write to VR_CTL.
 */
ENTRY(_hibernate_mode)
	/* Save/setup the regs we need early for minor pipeline optimization */
	R4 = R0;
	P3.H = hi(VR_CTL);
	P3.L = lo(VR_CTL);

	/* Disable all wakeup sources */
	R0 = IWR_DISABLE_ALL;
	R1 = IWR_DISABLE_ALL;
	R2 = IWR_DISABLE_ALL;
	call _set_sic_iwr;
	call _set_dram_srfs;
	SSYNC;

	/* Finally, we climb into our cave to hibernate */
	W[P3] = R4.L;
	CLI R2;
	IDLE;
.Lforever:
	jump .Lforever;
ENDPROC(_hibernate_mode)

ENTRY(_sleep_deeper)
	[--SP] = ( R7:0, P5:0 );
	[--SP] =  RETS;

	CLI R4;

	P3 = R0;
	P4 = R1;
	P5 = R2;

	R0 = IWR_ENABLE(0);
	R1 = IWR_DISABLE_ALL;
	R2 = IWR_DISABLE_ALL;

	call _set_sic_iwr;
	call _set_dram_srfs;	/* Set SDRAM Self Refresh */

	P0.H = hi(PLL_DIV);
	P0.L = lo(PLL_DIV);
	R6 = W[P0](z);
	R0.L = 0xF;
	W[P0] = R0.l;		/* Set Max VCO to SCLK divider */

	P0.H = hi(PLL_CTL);
	P0.L = lo(PLL_CTL);
	R5 = W[P0](z);
	R0.L = (CONFIG_MIN_VCO_HZ/CONFIG_CLKIN_HZ) << 9;
	W[P0] = R0.l;		/* Set Min CLKIN to VCO multiplier */

	SSYNC;
	IDLE;

	call _test_pll_locked;

	P0.H = hi(VR_CTL);
	P0.L = lo(VR_CTL);
	R7 = W[P0](z);
	R1 = 0x6;
	R1 <<= 16;
	R2 = 0x0404(Z);
	R1 = R1|R2;

	R2 = DEPOSIT(R7, R1);
	W[P0] = R2;		/* Set Min Core Voltage */

	SSYNC;
	IDLE;

	call _test_pll_locked;

	R0 = P3;
	R1 = P4;
	R3 = P5;
	call _set_sic_iwr;	/* Set Awake from IDLE */

	P0.H = hi(PLL_CTL);
	P0.L = lo(PLL_CTL);
	R0 = W[P0](z);
	BITSET (R0, 3);
	W[P0] = R0.L;		/* Turn CCLK OFF */
	SSYNC;
	IDLE;

	call _test_pll_locked;

	R0 = IWR_ENABLE(0);
	R1 = IWR_DISABLE_ALL;
	R2 = IWR_DISABLE_ALL;

	call _set_sic_iwr;	/* Set Awake from IDLE PLL */

	P0.H = hi(VR_CTL);
	P0.L = lo(VR_CTL);
	W[P0]= R7;

	SSYNC;
	IDLE;

	call _test_pll_locked;

	P0.H = hi(PLL_DIV);
	P0.L = lo(PLL_DIV);
	W[P0]= R6;		/* Restore CCLK and SCLK divider */

	P0.H = hi(PLL_CTL);
	P0.L = lo(PLL_CTL);
	w[p0] = R5;		/* Restore VCO multiplier */
	IDLE;
	call _test_pll_locked;

	call _unset_dram_srfs;	/* SDRAM Self Refresh Off */

	STI R4;

	RETS = [SP++];
	( R7:0, P5:0 ) = [SP++];
	RTS;
ENDPROC(_sleep_deeper)

ENTRY(_set_dram_srfs)
	/*  set the dram to self refresh mode */
	SSYNC;
#if defined(EBIU_RSTCTL)	/* DDR */
	P0.H = hi(EBIU_RSTCTL);
	P0.L = lo(EBIU_RSTCTL);
	R2 = [P0];
	BITSET(R2, 3); /* SRREQ enter self-refresh mode */
	[P0] = R2;
	SSYNC;
1:
	R2 = [P0];
	CC = BITTST(R2, 4);
	if !CC JUMP 1b;
#else 				/* SDRAM */
	P0.L = lo(EBIU_SDGCTL);
	P0.H = hi(EBIU_SDGCTL);
	R2 = [P0];
	BITSET(R2, 24); /* SRFS enter self-refresh mode */
	[P0] = R2;
	SSYNC;

	P0.L = lo(EBIU_SDSTAT);
	P0.H = hi(EBIU_SDSTAT);
1:
	R2 = w[P0];
	SSYNC;
	cc = BITTST(R2, 1); /* SDSRA poll self-refresh status */
	if !cc jump 1b;

	P0.L = lo(EBIU_SDGCTL);
	P0.H = hi(EBIU_SDGCTL);
	R2 = [P0];
	BITCLR(R2, 0); /* SCTLE disable CLKOUT */
	[P0] = R2;
#endif
	RTS;
ENDPROC(_set_dram_srfs)

ENTRY(_unset_dram_srfs)
	/*  set the dram out of self refresh mode */
#if defined(EBIU_RSTCTL)	/* DDR */
	P0.H = hi(EBIU_RSTCTL);
	P0.L = lo(EBIU_RSTCTL);
	R2 = [P0];
	BITCLR(R2, 3); /* clear SRREQ bit */
	[P0] = R2;
#elif defined(EBIU_SDGCTL)	/* SDRAM */

	P0.L = lo(EBIU_SDGCTL); /* release CLKOUT from self-refresh */
	P0.H = hi(EBIU_SDGCTL);
	R2 = [P0];
	BITSET(R2, 0); /* SCTLE enable CLKOUT */
	[P0] = R2
	SSYNC;

	P0.L = lo(EBIU_SDGCTL); /* release SDRAM from self-refresh */
	P0.H = hi(EBIU_SDGCTL);
	R2 = [P0];
	BITCLR(R2, 24); /* clear SRFS bit */
	[P0] = R2
#endif
	SSYNC;
	RTS;
ENDPROC(_unset_dram_srfs)

ENTRY(_set_sic_iwr)
#ifdef SIC_IWR0
	P0.H = hi(SYSMMR_BASE);
	P0.L = lo(SYSMMR_BASE);
	[P0 + (SIC_IWR0 - SYSMMR_BASE)] = R0;
	[P0 + (SIC_IWR1 - SYSMMR_BASE)] = R1;
# ifdef SIC_IWR2
	[P0 + (SIC_IWR2 - SYSMMR_BASE)] = R2;
# endif
#else
	P0.H = hi(SIC_IWR);
	P0.L = lo(SIC_IWR);
	[P0] = R0;
#endif

	SSYNC;
	RTS;
ENDPROC(_set_sic_iwr)

ENTRY(_test_pll_locked)
	P0.H = hi(PLL_STAT);
	P0.L = lo(PLL_STAT);
1:
	R0 = W[P0] (Z);
	CC = BITTST(R0,5);
	IF !CC JUMP 1b;
	RTS;
ENDPROC(_test_pll_locked)

.section .text

#define PM_REG0  R7
#define PM_REG1  R6
#define PM_REG2  R5
#define PM_REG3  R4
#define PM_REG4  R3
#define PM_REG5  R2
#define PM_REG6  R1
#define PM_REG7  R0
#define PM_REG8  P5
#define PM_REG9  P4
#define PM_REG10 P3
#define PM_REG11 P2
#define PM_REG12 P1
#define PM_REG13 P0

#define PM_REGSET0  R7:7
#define PM_REGSET1  R7:6
#define PM_REGSET2  R7:5
#define PM_REGSET3  R7:4
#define PM_REGSET4  R7:3
#define PM_REGSET5  R7:2
#define PM_REGSET6  R7:1
#define PM_REGSET7  R7:0
#define PM_REGSET8  R7:0, P5:5
#define PM_REGSET9  R7:0, P5:4
#define PM_REGSET10 R7:0, P5:3
#define PM_REGSET11 R7:0, P5:2
#define PM_REGSET12 R7:0, P5:1
#define PM_REGSET13 R7:0, P5:0

#define _PM_PUSH(n, x, w, base) PM_REG##n = w[FP + ((x) - (base))];
#define _PM_POP(n, x, w, base)  w[FP + ((x) - (base))] = PM_REG##n;
#define PM_PUSH_SYNC(n)         [--sp] = (PM_REGSET##n);
#define PM_POP_SYNC(n)          (PM_REGSET##n) = [sp++];
#define PM_PUSH(n, x)           _PM_PUSH(n, x, , COREMMR_BASE)
#define PM_POP(n, x)            _PM_POP(n, x, , COREMMR_BASE)
#define PM_SYS_PUSH(n, x)       _PM_PUSH(n, x, , SYSMMR_BASE)
#define PM_SYS_POP(n, x)        _PM_POP(n, x, , SYSMMR_BASE)
#define PM_SYS_PUSH16(n, x)     _PM_PUSH(n, x, w, SYSMMR_BASE)
#define PM_SYS_POP16(n, x)      _PM_POP(n, x, w, SYSMMR_BASE)

ENTRY(_do_hibernate)
	/*
	 * Save the core regs early so we can blow them away when
	 * saving/restoring MMR states
	 */
	[--sp] = (R7:0, P5:0);
	[--sp] = fp;
	[--sp] = usp;

	[--sp] = i0;
	[--sp] = i1;
	[--sp] = i2;
	[--sp] = i3;

	[--sp] = m0;
	[--sp] = m1;
	[--sp] = m2;
	[--sp] = m3;

	[--sp] = l0;
	[--sp] = l1;
	[--sp] = l2;
	[--sp] = l3;

	[--sp] = b0;
	[--sp] = b1;
	[--sp] = b2;
	[--sp] = b3;
	[--sp] = a0.x;
	[--sp] = a0.w;
	[--sp] = a1.x;
	[--sp] = a1.w;

	[--sp] = LC0;
	[--sp] = LC1;
	[--sp] = LT0;
	[--sp] = LT1;
	[--sp] = LB0;
	[--sp] = LB1;

	/* We can't push RETI directly as that'll change IPEND[4] */
	r7 = RETI;
	[--sp] = RETS;
	[--sp] = ASTAT;
	[--sp] = CYCLES;
	[--sp] = CYCLES2;
	[--sp] = SYSCFG;
	[--sp] = RETX;
	[--sp] = SEQSTAT;
	[--sp] = r7;

	/* Save first func arg in M3 */
	M3 = R0;

	/* Save system MMRs */
	FP.H = hi(SYSMMR_BASE);
	FP.L = lo(SYSMMR_BASE);

#ifdef SIC_IMASK0
	PM_SYS_PUSH(0, SIC_IMASK0)
	PM_SYS_PUSH(1, SIC_IMASK1)
# ifdef SIC_IMASK2
	PM_SYS_PUSH(2, SIC_IMASK2)
# endif
#else
	PM_SYS_PUSH(0, SIC_IMASK)
#endif
#ifdef SIC_IAR0
	PM_SYS_PUSH(3, SIC_IAR0)
	PM_SYS_PUSH(4, SIC_IAR1)
	PM_SYS_PUSH(5, SIC_IAR2)
#endif
#ifdef SIC_IAR3
	PM_SYS_PUSH(6, SIC_IAR3)
#endif
#ifdef SIC_IAR4
	PM_SYS_PUSH(7, SIC_IAR4)
	PM_SYS_PUSH(8, SIC_IAR5)
	PM_SYS_PUSH(9, SIC_IAR6)
#endif
#ifdef SIC_IAR7
	PM_SYS_PUSH(10, SIC_IAR7)
#endif
#ifdef SIC_IAR8
	PM_SYS_PUSH(11, SIC_IAR8)
	PM_SYS_PUSH(12, SIC_IAR9)
	PM_SYS_PUSH(13, SIC_IAR10)
#endif
	PM_PUSH_SYNC(13)
#ifdef SIC_IAR11
	PM_SYS_PUSH(0, SIC_IAR11)
#endif

#ifdef SIC_IWR
	PM_SYS_PUSH(1, SIC_IWR)
#endif
#ifdef SIC_IWR0
	PM_SYS_PUSH(1, SIC_IWR0)
#endif
#ifdef SIC_IWR1
	PM_SYS_PUSH(2, SIC_IWR1)
#endif
#ifdef SIC_IWR2
	PM_SYS_PUSH(3, SIC_IWR2)
#endif

#ifdef PINT0_ASSIGN
	PM_SYS_PUSH(4, PINT0_MASK_SET)
	PM_SYS_PUSH(5, PINT1_MASK_SET)
	PM_SYS_PUSH(6, PINT2_MASK_SET)
	PM_SYS_PUSH(7, PINT3_MASK_SET)
	PM_SYS_PUSH(8, PINT0_ASSIGN)
	PM_SYS_PUSH(9, PINT1_ASSIGN)
	PM_SYS_PUSH(10, PINT2_ASSIGN)
	PM_SYS_PUSH(11, PINT3_ASSIGN)
	PM_SYS_PUSH(12, PINT0_INVERT_SET)
	PM_SYS_PUSH(13, PINT1_INVERT_SET)
	PM_PUSH_SYNC(13)
	PM_SYS_PUSH(0, PINT2_INVERT_SET)
	PM_SYS_PUSH(1, PINT3_INVERT_SET)
	PM_SYS_PUSH(2, PINT0_EDGE_SET)
	PM_SYS_PUSH(3, PINT1_EDGE_SET)
	PM_SYS_PUSH(4, PINT2_EDGE_SET)
	PM_SYS_PUSH(5, PINT3_EDGE_SET)
#endif

	PM_SYS_PUSH16(6, SYSCR)

	PM_SYS_PUSH16(7, EBIU_AMGCTL)
	PM_SYS_PUSH(8, EBIU_AMBCTL0)
	PM_SYS_PUSH(9, EBIU_AMBCTL1)
#ifdef EBIU_FCTL
	PM_SYS_PUSH(10, EBIU_MBSCTL)
	PM_SYS_PUSH(11, EBIU_MODE)
	PM_SYS_PUSH(12, EBIU_FCTL)
	PM_PUSH_SYNC(12)
#else
	PM_PUSH_SYNC(9)
#endif

#ifdef PORTCIO_FER
	PM_SYS_PUSH16(0, PORTCIO_DIR)
	PM_SYS_PUSH16(1, PORTCIO_INEN)
	PM_SYS_PUSH16(2, PORTCIO)
	PM_SYS_PUSH16(3, PORTCIO_FER)
	PM_SYS_PUSH16(4, PORTDIO_DIR)
	PM_SYS_PUSH16(5, PORTDIO_INEN)
	PM_SYS_PUSH16(6, PORTDIO)
	PM_SYS_PUSH16(7, PORTDIO_FER)
	PM_SYS_PUSH16(8, PORTEIO_DIR)
	PM_SYS_PUSH16(9, PORTEIO_INEN)
	PM_SYS_PUSH16(10, PORTEIO)
	PM_SYS_PUSH16(11, PORTEIO_FER)
	PM_PUSH_SYNC(11)
#endif

	/* Save Core MMRs */
	I0.H = hi(COREMMR_BASE);
	I0.L = lo(COREMMR_BASE);
	I1 = I0;
	I2 = I0;
	I3 = I0;
	B0 = I0;
	B1 = I0;
	B2 = I0;
	B3 = I0;
	I1.L = lo(DCPLB_ADDR0);
	I2.L = lo(DCPLB_DATA0);
	I3.L = lo(ICPLB_ADDR0);
	B0.L = lo(ICPLB_DATA0);
	B1.L = lo(EVT2);
	B2.L = lo(IMASK);
	B3.L = lo(TCNTL);

	/* DCPLB Addr */
	FP = I1;
	R7 = [FP++];	/* DCPLB_ADDR0 */
	R6 = [FP++];	/* DCPLB_ADDR1 */
	R5 = [FP++];	/* DCPLB_ADDR2 */
	R4 = [FP++];	/* DCPLB_ADDR3 */
	R3 = [FP++];	/* DCPLB_ADDR4 */
	R2 = [FP++];	/* DCPLB_ADDR5 */
	R1 = [FP++];	/* DCPLB_ADDR6 */
	R0 = [FP++];	/* DCPLB_ADDR7 */
	P5 = [FP++];	/* DCPLB_ADDR8 */
	P4 = [FP++];	/* DCPLB_ADDR9 */
	P3 = [FP++];	/* DCPLB_ADDR10 */
	P2 = [FP++];	/* DCPLB_ADDR11 */
	P1 = [FP++];	/* DCPLB_ADDR12 */
	P0 = [FP++];	/* DCPLB_ADDR13 */
	[--sp] = (R7:0, P5:0);
	R7 = [FP++];	/* DCPLB_ADDR14 */
	R6 = [FP++];	/* DCPLB_ADDR15 */

	/* DCPLB Data */
	FP = I2;
	R5 = [FP++];	/* DCPLB_DATA0 */
	R4 = [FP++];	/* DCPLB_DATA1 */
	R3 = [FP++];	/* DCPLB_DATA2 */
	R2 = [FP++];	/* DCPLB_DATA3 */
	R1 = [FP++];	/* DCPLB_DATA4 */
	R0 = [FP++];	/* DCPLB_DATA5 */
	P5 = [FP++];	/* DCPLB_DATA6 */
	P4 = [FP++];	/* DCPLB_DATA7 */
	P3 = [FP++];	/* DCPLB_DATA8 */
	P2 = [FP++];	/* DCPLB_DATA9 */
	P1 = [FP++];	/* DCPLB_DATA10 */
	P0 = [FP++];	/* DCPLB_DATA11 */
	[--sp] = (R7:0, P5:0);
	R7 = [FP++];	/* DCPLB_DATA12 */
	R6 = [FP++];	/* DCPLB_DATA13 */
	R5 = [FP++];	/* DCPLB_DATA14 */
	R4 = [FP++];	/* DCPLB_DATA15 */

	/* ICPLB Addr */
	FP = I3;
	R3 = [FP++];	/* ICPLB_ADDR0 */
	R2 = [FP++];	/* ICPLB_ADDR1 */
	R1 = [FP++];	/* ICPLB_ADDR2 */
	R0 = [FP++];	/* ICPLB_ADDR3 */
	P5 = [FP++];	/* ICPLB_ADDR4 */
	P4 = [FP++];	/* ICPLB_ADDR5 */
	P3 = [FP++];	/* ICPLB_ADDR6 */
	P2 = [FP++];	/* ICPLB_ADDR7 */
	P1 = [FP++];	/* ICPLB_ADDR8 */
	P0 = [FP++];	/* ICPLB_ADDR9 */
	[--sp] = (R7:0, P5:0);
	R7 = [FP++];	/* ICPLB_ADDR10 */
	R6 = [FP++];	/* ICPLB_ADDR11 */
	R5 = [FP++];	/* ICPLB_ADDR12 */
	R4 = [FP++];	/* ICPLB_ADDR13 */
	R3 = [FP++];	/* ICPLB_ADDR14 */
	R2 = [FP++];	/* ICPLB_ADDR15 */

	/* ICPLB Data */
	FP = B0;
	R1 = [FP++];	/* ICPLB_DATA0 */
	R0 = [FP++];	/* ICPLB_DATA1 */
	P5 = [FP++];	/* ICPLB_DATA2 */
	P4 = [FP++];	/* ICPLB_DATA3 */
	P3 = [FP++];	/* ICPLB_DATA4 */
	P2 = [FP++];	/* ICPLB_DATA5 */
	P1 = [FP++];	/* ICPLB_DATA6 */
	P0 = [FP++];	/* ICPLB_DATA7 */
	[--sp] = (R7:0, P5:0);
	R7 = [FP++];	/* ICPLB_DATA8 */
	R6 = [FP++];	/* ICPLB_DATA9 */
	R5 = [FP++];	/* ICPLB_DATA10 */
	R4 = [FP++];	/* ICPLB_DATA11 */
	R3 = [FP++];	/* ICPLB_DATA12 */
	R2 = [FP++];	/* ICPLB_DATA13 */
	R1 = [FP++];	/* ICPLB_DATA14 */
	R0 = [FP++];	/* ICPLB_DATA15 */

	/* Event Vectors */
	FP = B1;
	P5 = [FP++];	/* EVT2 */
	P4 = [FP++];	/* EVT3 */
	FP += 4;		/* EVT4 */
	P3 = [FP++];	/* EVT5 */
	P2 = [FP++];	/* EVT6 */
	P1 = [FP++];	/* EVT7 */
	P0 = [FP++];	/* EVT8 */
	[--sp] = (R7:0, P5:0);
	R7 = [FP++];	/* EVT9 */
	R6 = [FP++];	/* EVT10 */
	R5 = [FP++];	/* EVT11 */
	R4 = [FP++];	/* EVT12 */
	R3 = [FP++];	/* EVT13 */
	R2 = [FP++];	/* EVT14 */
	R1 = [FP++];	/* EVT15 */

	/* CEC */
	FP = B2;
	R0 = [FP++];	/* IMASK */
	FP += 4;		/* IPEND */
	P5 = [FP++];	/* ILAT */
	P4 = [FP++];	/* IPRIO */

	/* Core Timer */
	FP = B3;
	P3 = [FP++];	/* TCNTL */
	P2 = [FP++];	/* TPERIOD */
	P1 = [FP++];	/* TSCALE */
	P0 = [FP++];	/* TCOUNT */
	[--sp] = (R7:0, P5:0);

	/* Misc non-contiguous registers */
	FP = I0;
	R7 = [FP + (DMEM_CONTROL - COREMMR_BASE)];
	R6 = [FP + (IMEM_CONTROL - COREMMR_BASE)];
	R5 = [FP + (TBUFCTL - COREMMR_BASE)];
	[--sp] = (R7:5);

	/* Setup args to hibernate mode early for pipeline optimization */
	R0 = M3;
	P1.H = _hibernate_mode;
	P1.L = _hibernate_mode;

	/* Save Magic, return address and Stack Pointer */
	P0 = 0;
	R1.H = 0xDEAD;	/* Hibernate Magic */
	R1.L = 0xBEEF;
	R2.H = .Lpm_resume_here;
	R2.L = .Lpm_resume_here;
	[P0++] = R1;	/* Store Hibernate Magic */
	[P0++] = R2;	/* Save Return Address */
	[P0++] = SP;	/* Save Stack Pointer */

	/* Must use an indirect call as we need to jump to L1 */
	call (P1); /* Goodbye */

.Lpm_resume_here:

	/* Restore Core MMRs */
	I0.H = hi(COREMMR_BASE);
	I0.L = lo(COREMMR_BASE);
	I1 = I0;
	I2 = I0;
	I3 = I0;
	B0 = I0;
	B1 = I0;
	B2 = I0;
	B3 = I0;
	I1.L = lo(DCPLB_ADDR15);
	I2.L = lo(DCPLB_DATA15);
	I3.L = lo(ICPLB_ADDR15);
	B0.L = lo(ICPLB_DATA15);
	B1.L = lo(EVT15);
	B2.L = lo(IPRIO);
	B3.L = lo(TCOUNT);

	/* Misc non-contiguous registers */
	FP = I0;
	(R7:5) = [sp++];
	[FP + (TBUFCTL - COREMMR_BASE)] = R5;
	[FP + (IMEM_CONTROL - COREMMR_BASE)] = R6;
	[FP + (DMEM_CONTROL - COREMMR_BASE)] = R7;

	/* Core Timer */
	(R7:0, P5:0) = [sp++];
	FP = B3;
	[FP--] = P0;	/* TCOUNT */
	[FP--] = P1;	/* TSCALE */
	[FP--] = P2;	/* TPERIOD */
	[FP--] = P3;	/* TCNTL */

	/* CEC */
	FP = B2;
	[FP--] = P4;	/* IPRIO */
	[FP--] = P5;	/* ILAT */
	FP += -4;		/* IPEND */
	[FP--] = R0;	/* IMASK */

	/* Event Vectors */
	FP = B1;
	[FP--] = R1;	/* EVT15 */
	[FP--] = R2;	/* EVT14 */
	[FP--] = R3;	/* EVT13 */
	[FP--] = R4;	/* EVT12 */
	[FP--] = R5;	/* EVT11 */
	[FP--] = R6;	/* EVT10 */
	[FP--] = R7;	/* EVT9 */
	(R7:0, P5:0) = [sp++];
	[FP--] = P0;	/* EVT8 */
	[FP--] = P1;	/* EVT7 */
	[FP--] = P2;	/* EVT6 */
	[FP--] = P3;	/* EVT5 */
	FP += -4;		/* EVT4 */
	[FP--] = P4;	/* EVT3 */
	[FP--] = P5;	/* EVT2 */

	/* ICPLB Data */
	FP = B0;
	[FP--] = R0;	/* ICPLB_DATA15 */
	[FP--] = R1;	/* ICPLB_DATA14 */
	[FP--] = R2;	/* ICPLB_DATA13 */
	[FP--] = R3;	/* ICPLB_DATA12 */
	[FP--] = R4;	/* ICPLB_DATA11 */
	[FP--] = R5;	/* ICPLB_DATA10 */
	[FP--] = R6;	/* ICPLB_DATA9 */
	[FP--] = R7;	/* ICPLB_DATA8 */
	(R7:0, P5:0) = [sp++];
	[FP--] = P0;	/* ICPLB_DATA7 */
	[FP--] = P1;	/* ICPLB_DATA6 */
	[FP--] = P2;	/* ICPLB_DATA5 */
	[FP--] = P3;	/* ICPLB_DATA4 */
	[FP--] = P4;	/* ICPLB_DATA3 */
	[FP--] = P5;	/* ICPLB_DATA2 */
	[FP--] = R0;	/* ICPLB_DATA1 */
	[FP--] = R1;	/* ICPLB_DATA0 */

	/* ICPLB Addr */
	FP = I3;
	[FP--] = R2;	/* ICPLB_ADDR15 */
	[FP--] = R3;	/* ICPLB_ADDR14 */
	[FP--] = R4;	/* ICPLB_ADDR13 */
	[FP--] = R5;	/* ICPLB_ADDR12 */
	[FP--] = R6;	/* ICPLB_ADDR11 */
	[FP--] = R7;	/* ICPLB_ADDR10 */
	(R7:0, P5:0) = [sp++];
	[FP--] = P0;	/* ICPLB_ADDR9 */
	[FP--] = P1;	/* ICPLB_ADDR8 */
	[FP--] = P2;	/* ICPLB_ADDR7 */
	[FP--] = P3;	/* ICPLB_ADDR6 */
	[FP--] = P4;	/* ICPLB_ADDR5 */
	[FP--] = P5;	/* ICPLB_ADDR4 */
	[FP--] = R0;	/* ICPLB_ADDR3 */
	[FP--] = R1;	/* ICPLB_ADDR2 */
	[FP--] = R2;	/* ICPLB_ADDR1 */
	[FP--] = R3;	/* ICPLB_ADDR0 */

	/* DCPLB Data */
	FP = I2;
	[FP--] = R4;	/* DCPLB_DATA15 */
	[FP--] = R5;	/* DCPLB_DATA14 */
	[FP--] = R6;	/* DCPLB_DATA13 */
	[FP--] = R7;	/* DCPLB_DATA12 */
	(R7:0, P5:0) = [sp++];
	[FP--] = P0;	/* DCPLB_DATA11 */
	[FP--] = P1;	/* DCPLB_DATA10 */
	[FP--] = P2;	/* DCPLB_DATA9 */
	[FP--] = P3;	/* DCPLB_DATA8 */
	[FP--] = P4;	/* DCPLB_DATA7 */
	[FP--] = P5;	/* DCPLB_DATA6 */
	[FP--] = R0;	/* DCPLB_DATA5 */
	[FP--] = R1;	/* DCPLB_DATA4 */
	[FP--] = R2;	/* DCPLB_DATA3 */
	[FP--] = R3;	/* DCPLB_DATA2 */
	[FP--] = R4;	/* DCPLB_DATA1 */
	[FP--] = R5;	/* DCPLB_DATA0 */

	/* DCPLB Addr */
	FP = I1;
	[FP--] = R6;	/* DCPLB_ADDR15 */
	[FP--] = R7;	/* DCPLB_ADDR14 */
	(R7:0, P5:0) = [sp++];
	[FP--] = P0;	/* DCPLB_ADDR13 */
	[FP--] = P1;	/* DCPLB_ADDR12 */
	[FP--] = P2;	/* DCPLB_ADDR11 */
	[FP--] = P3;	/* DCPLB_ADDR10 */
	[FP--] = P4;	/* DCPLB_ADDR9 */
	[FP--] = P5;	/* DCPLB_ADDR8 */
	[FP--] = R0;	/* DCPLB_ADDR7 */
	[FP--] = R1;	/* DCPLB_ADDR6 */
	[FP--] = R2;	/* DCPLB_ADDR5 */
	[FP--] = R3;	/* DCPLB_ADDR4 */
	[FP--] = R4;	/* DCPLB_ADDR3 */
	[FP--] = R5;	/* DCPLB_ADDR2 */
	[FP--] = R6;	/* DCPLB_ADDR1 */
	[FP--] = R7;	/* DCPLB_ADDR0 */

	/* Restore System MMRs */
	FP.H = hi(SYSMMR_BASE);
	FP.L = lo(SYSMMR_BASE);

#ifdef PORTCIO_FER
	PM_POP_SYNC(11)
	PM_SYS_POP16(11, PORTEIO_FER)
	PM_SYS_POP16(10, PORTEIO)
	PM_SYS_POP16(9, PORTEIO_INEN)
	PM_SYS_POP16(8, PORTEIO_DIR)
	PM_SYS_POP16(7, PORTDIO_FER)
	PM_SYS_POP16(6, PORTDIO)
	PM_SYS_POP16(5, PORTDIO_INEN)
	PM_SYS_POP16(4, PORTDIO_DIR)
	PM_SYS_POP16(3, PORTCIO_FER)
	PM_SYS_POP16(2, PORTCIO)
	PM_SYS_POP16(1, PORTCIO_INEN)
	PM_SYS_POP16(0, PORTCIO_DIR)
#endif

#ifdef EBIU_FCTL
	PM_POP_SYNC(12)
	PM_SYS_POP(12, EBIU_FCTL)
	PM_SYS_POP(11, EBIU_MODE)
	PM_SYS_POP(10, EBIU_MBSCTL)
#else
	PM_POP_SYNC(9)
#endif
	PM_SYS_POP(9, EBIU_AMBCTL1)
	PM_SYS_POP(8, EBIU_AMBCTL0)
	PM_SYS_POP16(7, EBIU_AMGCTL)

	PM_SYS_POP16(6, SYSCR)

#ifdef PINT0_ASSIGN
	PM_SYS_POP(5, PINT3_EDGE_SET)
	PM_SYS_POP(4, PINT2_EDGE_SET)
	PM_SYS_POP(3, PINT1_EDGE_SET)
	PM_SYS_POP(2, PINT0_EDGE_SET)
	PM_SYS_POP(1, PINT3_INVERT_SET)
	PM_SYS_POP(0, PINT2_INVERT_SET)
	PM_POP_SYNC(13)
	PM_SYS_POP(13, PINT1_INVERT_SET)
	PM_SYS_POP(12, PINT0_INVERT_SET)
	PM_SYS_POP(11, PINT3_ASSIGN)
	PM_SYS_POP(10, PINT2_ASSIGN)
	PM_SYS_POP(9, PINT1_ASSIGN)
	PM_SYS_POP(8, PINT0_ASSIGN)
	PM_SYS_POP(7, PINT3_MASK_SET)
	PM_SYS_POP(6, PINT2_MASK_SET)
	PM_SYS_POP(5, PINT1_MASK_SET)
	PM_SYS_POP(4, PINT0_MASK_SET)
#endif

#ifdef SIC_IWR2
	PM_SYS_POP(3, SIC_IWR2)
#endif
#ifdef SIC_IWR1
	PM_SYS_POP(2, SIC_IWR1)
#endif
#ifdef SIC_IWR0
	PM_SYS_POP(1, SIC_IWR0)
#endif
#ifdef SIC_IWR
	PM_SYS_POP(1, SIC_IWR)
#endif

#ifdef SIC_IAR11
	PM_SYS_POP(0, SIC_IAR11)
#endif
	PM_POP_SYNC(13)
#ifdef SIC_IAR8
	PM_SYS_POP(13, SIC_IAR10)
	PM_SYS_POP(12, SIC_IAR9)
	PM_SYS_POP(11, SIC_IAR8)
#endif
#ifdef SIC_IAR7
	PM_SYS_POP(10, SIC_IAR7)
#endif
#ifdef SIC_IAR6
	PM_SYS_POP(9, SIC_IAR6)
	PM_SYS_POP(8, SIC_IAR5)
	PM_SYS_POP(7, SIC_IAR4)
#endif
#ifdef SIC_IAR3
	PM_SYS_POP(6, SIC_IAR3)
#endif
#ifdef SIC_IAR0
	PM_SYS_POP(5, SIC_IAR2)
	PM_SYS_POP(4, SIC_IAR1)
	PM_SYS_POP(3, SIC_IAR0)
#endif
#ifdef SIC_IMASK0
# ifdef SIC_IMASK2
	PM_SYS_POP(2, SIC_IMASK2)
# endif
	PM_SYS_POP(1, SIC_IMASK1)
	PM_SYS_POP(0, SIC_IMASK0)
#else
	PM_SYS_POP(0, SIC_IMASK)
#endif

	/* Restore Core Registers */
	RETI = [sp++];
	SEQSTAT = [sp++];
	RETX = [sp++];
	SYSCFG = [sp++];
	CYCLES2 = [sp++];
	CYCLES = [sp++];
	ASTAT = [sp++];
	RETS = [sp++];

	LB1 = [sp++];
	LB0 = [sp++];
	LT1 = [sp++];
	LT0 = [sp++];
	LC1 = [sp++];
	LC0 = [sp++];

	a1.w = [sp++];
	a1.x = [sp++];
	a0.w = [sp++];
	a0.x = [sp++];
	b3 = [sp++];
	b2 = [sp++];
	b1 = [sp++];
	b0 = [sp++];

	l3 = [sp++];
	l2 = [sp++];
	l1 = [sp++];
	l0 = [sp++];

	m3 = [sp++];
	m2 = [sp++];
	m1 = [sp++];
	m0 = [sp++];

	i3 = [sp++];
	i2 = [sp++];
	i1 = [sp++];
	i0 = [sp++];

	usp = [sp++];
	fp = [sp++];
	(R7:0, P5:0) = [sp++];

	[--sp] = RETI;	/* Clear Global Interrupt Disable */
	SP += 4;

	RTS;
ENDPROC(_do_hibernate)

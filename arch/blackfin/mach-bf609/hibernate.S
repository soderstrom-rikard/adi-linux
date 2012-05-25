#include <linux/linkage.h>
#include <asm/blackfin.h>
#include <asm/dpmc.h>

#define PM_STACK   (COREA_L1_SCRATCH_START + L1_SCRATCH_LENGTH - 12)

.section .l1.text
ENTRY(_enter_hibernate)
	/* switch stack to L1 scratch, prepare for ddr srfr */
	P0.H = HI(PM_STACK);
	P0.L = LO(PM_STACK);
	SP = P0;

	call _bf609_ddr_sr;
	call _bfin_hibernate_syscontrol;

	P0.H = HI(DPM0_RESTORE4);
	P0.L = LO(DPM0_RESTORE4);
	P1.H = _bf609_pm_data;
	P1.L = _bf609_pm_data;
	[P0] = P1;

	P0.H = HI(DPM0_CTL);
	P0.L = LO(DPM0_CTL);
	R3.H = HI(0x00000010);
	R3.L = LO(0x00000010);

	bfin_init_pm_bench_cycles;

	[P0] = R3;

	SSYNC;
ENDPROC(_enter_hibernate)


ENTRY(_dummy_deepsleep)
	cli r0;
	[--sp] = (R7:0,P5:0);
	P0.l = LO(SEC_SCI_BASE + SEC_CSID);
	P0.h = HI(SEC_SCI_BASE + SEC_CSID);
	R0 = [P0];

	P1.l = LO(SEC_END);
	P1.h = HI(SEC_END);

	[P1] = R0;

	p0.h = hi(EVT11);
	p0.l = lo(EVT11);
	p1.h = _evt_evt11;
	p1.l = _evt_evt11;

	[p0] = p1;

	(R7:0,P5:0) = [SP++];
	RTI;
ENDPROC(_dummy_deepsleep)

ENTRY(_enter_deepsleep)
	[--sp] = (R7:0,P5:0);
	p0.h = hi(EVT11);
	p0.l = lo(EVT11);
	p1.h = _dummy_deepsleep;
	p1.l = _dummy_deepsleep;

	[p0] = p1;

	P0.H = HI(PM_STACK);
	P0.L = LO(PM_STACK);

	i2 = SP;
	SP = P0;

	SSYNC;

	call _bf609_ddr_sr;

	P0.H = HI(DPM0_CTL);
	P0.L = LO(DPM0_CTL);
	R3.H = HI(0x00000008);
	R3.L = LO(0x00000008);
	[P0] = R3;

	CSYNC;

	r0.l = 0xffff;
	r0.h = 0;
	sti r0;
	SSYNC;

	idle;
	SSYNC;

	R3 = 0;
1:
	cc = R3;
	if !cc jump 1b;

	call _bf609_resume_ccbuf;

	call _bf609_ddr_sr_exit;
	SP = i2;

	(R7:0,P5:0) = [SP++];
	RTS;
ENDPROC(_enter_deepsleep)

.section .text
ENTRY(_bf609_hibernate)
	bfin_cpu_reg_save;
	bfin_core_mmr_save;

	P0.H = _bf609_pm_data;
	P0.L = _bf609_pm_data;
	R1.H = 0xDEAD;
	R1.L = 0xBEEF;
	R2.H = .Lpm_resume_here;
	R2.L = .Lpm_resume_here;
	[P0++] = R1;
	[P0++] = R2;
	[P0++] = SP;

	P1.H = _enter_hibernate;
	P1.L = _enter_hibernate;

	call (P1);
.Lpm_resume_here:

	bfin_core_mmr_restore;
	bfin_cpu_reg_restore;

	[--sp] = RETI;  /* Clear Global Interrupt Disable */
	SP += 4;

	RTS;

ENDPROC(_bf609_hibernate)


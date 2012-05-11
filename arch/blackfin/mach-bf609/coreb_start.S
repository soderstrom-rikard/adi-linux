#include <linux/linkage.h>
#include <asm/blackfin.h>

.section .l1.text.head

/* Lay the initial stack into the L1 scratch area of Core B */
#define INITIAL_STACK	(COREB_L1_SCRATCH_START + L1_SCRATCH_LENGTH - 12)

ENTRY(_coreb_trampoline_start)
	/* Set the SYSCFG register */
	R0 = 0x36;
	SYSCFG = R0; /*Enable Cycle Counter and Nesting Of Interrupts(3rd Bit)*/
	R0 = 0;

	/*Clear Out All the data and pointer  Registers*/
	R1 = R0;
	R2 = R0;
	R3 = R0;
	R4 = R0;
	R5 = R0;
	R6 = R0;
	R7 = R0;

	P0 = R0;
	P1 = R0;
	P2 = R0;
	P3 = R0;
	P4 = R0;
	P5 = R0;

	LC0 = r0;
	LC1 = r0;
	L0 = r0;
	L1 = r0;
	L2 = r0;
	L3 = r0;

	/* Clear Out All the DAG Registers*/
	B0 = r0;
	B1 = r0;
	B2 = r0;
	B3 = r0;

	I0 = r0;
	I1 = r0;
	I2 = r0;
	I3 = r0;

	M0 = r0;
	M1 = r0;
	M2 = r0;
	M3 = r0;

	/* Initialize stack pointer */
	sp.l = lo(INITIAL_STACK);
	sp.h = hi(INITIAL_STACK);
	fp = sp;
	usp = sp;

	p1.h = HI(0xff700000);
	p1.l = LO(0xff700000)
	r0.l = 0xDEAD;
	[p1] = r0;

	p0.l = lo(EVT15);
	p0.h = hi(EVT15);
	p1.l = _coreb_start;
	p1.h = _coreb_start;
	[p0] = p1;
	csync;

	p0.l = lo(IMASK);
	p0.h = hi(IMASK);
	p1.l = IMASK_IVG15;
	p1.h = 0x0;
	[p0] = p1;
	csync;

	raise 15;

	p0.l = .LWAIT_HERE;
	p0.h = .LWAIT_HERE;
	reti = p0;
	nop;nop;nop;
	rti;
.LWAIT_HERE:
	jump .LWAIT_HERE;
ENDPROC(_coreb_trampoline_start)
ENTRY(_coreb_trampoline_end)

ENTRY(_core1_evt11)
	[--sp] = SYSCFG;
	[--sp] = P0;        /*orig_p0*/
	[--sp] = R0;        /*orig_r0*/
	[--sp] = (R7:0,P5:0);
	p0.h = _core1_evt_handle;
	p0.l = _core1_evt_handle;
	call (p0);
	(R7:0,P5:0) = [SP++];
	sp += 8;        /* Skip orig_r0/orig_p0 */
	csync;
	SYSCFG = [sp++];
	csync;
	rti;
ENDPROC(_core1_evt11)

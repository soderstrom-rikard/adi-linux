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

.LWAIT_HERE:
	idle;
	jump .LWAIT_HERE;
ENDPROC(_coreb_trampoline_start)
ENTRY(_coreb_trampoline_end)

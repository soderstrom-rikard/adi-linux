/*
 * File:         arch/blackfin/kernel/kgdb.c
 * Based on:
 * Author:       Sonic Zhang
 *
 * Created:
 * Description:
 *
 * Rev:          $Id: kgdb_bfin_linux-2.6.x.patch 4934 2007-02-13 09:32:11Z sonicz $
 *
 * Modified:
 *               Copyright 2005-2006 Analog Devices Inc.
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

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/ptrace.h>		/* for linux pt_regs struct */
#include <linux/kgdb.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <asm/system.h>
#include <asm/traps.h>
#include <asm/blackfin.h>
#include <asm/dma.h>

/* Put the error code here just in case the user cares.  */
int gdb_bfin_errcode;
/* Likewise, the vector number here (since GDB only gets the signal
   number through the usual means, and that's not very specific).  */
int gdb_bfin_vector = -1;

#if KGDB_MAX_NO_CPUS != 8
#error change the definition of slavecpulocks
#endif

#ifdef CONFIG_BFIN_WDT
# error "Please unselect blackfin watchdog driver before build KGDB."
#endif

void pt_regs_to_gdb_regs(unsigned long *gdb_regs, struct pt_regs *regs)
{
	gdb_regs[BFIN_R0] = regs->r0;
	gdb_regs[BFIN_R1] = regs->r1;
	gdb_regs[BFIN_R2] = regs->r2;
	gdb_regs[BFIN_R3] = regs->r3;
	gdb_regs[BFIN_R4] = regs->r4;
	gdb_regs[BFIN_R5] = regs->r5;
	gdb_regs[BFIN_R6] = regs->r6;
	gdb_regs[BFIN_R7] = regs->r7;
	gdb_regs[BFIN_P0] = regs->p0;
	gdb_regs[BFIN_P1] = regs->p1;
	gdb_regs[BFIN_P2] = regs->p2;
	gdb_regs[BFIN_P3] = regs->p3;
	gdb_regs[BFIN_P4] = regs->p4;
	gdb_regs[BFIN_P5] = regs->p5;
	gdb_regs[BFIN_SP] = regs->reserved;
	gdb_regs[BFIN_FP] = regs->fp;
	gdb_regs[BFIN_I0] = regs->i0;
	gdb_regs[BFIN_I1] = regs->i1;
	gdb_regs[BFIN_I2] = regs->i2;
	gdb_regs[BFIN_I3] = regs->i3;
	gdb_regs[BFIN_M0] = regs->m0;
	gdb_regs[BFIN_M1] = regs->m1;
	gdb_regs[BFIN_M2] = regs->m2;
	gdb_regs[BFIN_M3] = regs->m3;
	gdb_regs[BFIN_B0] = regs->b0;
	gdb_regs[BFIN_B1] = regs->b1;
	gdb_regs[BFIN_B2] = regs->b2;
	gdb_regs[BFIN_B3] = regs->b3;
	gdb_regs[BFIN_L0] = regs->l0;
	gdb_regs[BFIN_L1] = regs->l1;
	gdb_regs[BFIN_L2] = regs->l2;
	gdb_regs[BFIN_L3] = regs->l3;
	gdb_regs[BFIN_A0_DOT_X] = regs->a0x;
	gdb_regs[BFIN_A0_DOT_W] = regs->a0w;
	gdb_regs[BFIN_A1_DOT_X] = regs->a1x;
	gdb_regs[BFIN_A1_DOT_W] = regs->a1w;
	gdb_regs[BFIN_ASTAT] = regs->astat;
	gdb_regs[BFIN_RETS] = regs->rets;
	gdb_regs[BFIN_LC0] = regs->lc0;
	gdb_regs[BFIN_LT0] = regs->lt0;
	gdb_regs[BFIN_LB0] = regs->lb0;
	gdb_regs[BFIN_LC1] = regs->lc1;
	gdb_regs[BFIN_LT1] = regs->lt1;
	gdb_regs[BFIN_LB1] = regs->lb1;
	gdb_regs[BFIN_CYCLES] = 0;
	gdb_regs[BFIN_CYCLES2] = 0;
	gdb_regs[BFIN_USP] = regs->usp;
	gdb_regs[BFIN_SEQSTAT] = regs->seqstat;
	gdb_regs[BFIN_SYSCFG] = regs->syscfg;
	gdb_regs[BFIN_RETI] = regs->pc;
	gdb_regs[BFIN_RETX] = regs->retx;
	gdb_regs[BFIN_RETN] = regs->retn;
	gdb_regs[BFIN_RETE] = regs->rete;
	gdb_regs[BFIN_PC] = regs->pc;
	gdb_regs[BFIN_CC] = 0;
	gdb_regs[BFIN_EXTRA1] = 0;
	gdb_regs[BFIN_EXTRA2] = 0;
	gdb_regs[BFIN_EXTRA3] = 0;
	gdb_regs[BFIN_IPEND] = regs->ipend;
}

/*
 * Extracts ebp, esp and eip values understandable by gdb from the values
 * saved by switch_to.
 * thread.esp points to ebp. flags and ebp are pushed in switch_to hence esp
 * prior to entering switch_to is 8 greater then the value that is saved.
 * If switch_to changes, change following code appropriately.
 */
void sleeping_thread_to_gdb_regs(unsigned long *gdb_regs, struct task_struct *p)
{
	gdb_regs[BFIN_SP] = p->thread.ksp;
	gdb_regs[BFIN_PC] = p->thread.pc;
	gdb_regs[BFIN_SEQSTAT] = p->thread.seqstat;
}

void gdb_regs_to_pt_regs(unsigned long *gdb_regs, struct pt_regs *regs)
{
	regs->r0 = gdb_regs[BFIN_R0];
	regs->r1 = gdb_regs[BFIN_R1];
	regs->r2 = gdb_regs[BFIN_R2];
	regs->r3 = gdb_regs[BFIN_R3];
	regs->r4 = gdb_regs[BFIN_R4];
	regs->r5 = gdb_regs[BFIN_R5];
	regs->r6 = gdb_regs[BFIN_R6];
	regs->r7 = gdb_regs[BFIN_R7];
	regs->p0 = gdb_regs[BFIN_P0];
	regs->p1 = gdb_regs[BFIN_P1];
	regs->p2 = gdb_regs[BFIN_P2];
	regs->p3 = gdb_regs[BFIN_P3];
	regs->p4 = gdb_regs[BFIN_P4];
	regs->p5 = gdb_regs[BFIN_P5];
	regs->fp = gdb_regs[BFIN_FP];
	regs->i0 = gdb_regs[BFIN_I0];
	regs->i1 = gdb_regs[BFIN_I1];
	regs->i2 = gdb_regs[BFIN_I2];
	regs->i3 = gdb_regs[BFIN_I3];
	regs->m0 = gdb_regs[BFIN_M0];
	regs->m1 = gdb_regs[BFIN_M1];
	regs->m2 = gdb_regs[BFIN_M2];
	regs->m3 = gdb_regs[BFIN_M3];
	regs->b0 = gdb_regs[BFIN_B0];
	regs->b1 = gdb_regs[BFIN_B1];
	regs->b2 = gdb_regs[BFIN_B2];
	regs->b3 = gdb_regs[BFIN_B3];
	regs->l0 = gdb_regs[BFIN_L0];
	regs->l1 = gdb_regs[BFIN_L1];
	regs->l2 = gdb_regs[BFIN_L2];
	regs->l3 = gdb_regs[BFIN_L3];
	regs->a0x = gdb_regs[BFIN_A0_DOT_X];
	regs->a0w = gdb_regs[BFIN_A0_DOT_W];
	regs->a1x = gdb_regs[BFIN_A1_DOT_X];
	regs->a1w = gdb_regs[BFIN_A1_DOT_W];
	regs->rets = gdb_regs[BFIN_RETS];
	regs->lc0 = gdb_regs[BFIN_LC0];
	regs->lt0 = gdb_regs[BFIN_LT0];
	regs->lb0 = gdb_regs[BFIN_LB0];
	regs->lc1 = gdb_regs[BFIN_LC1];
	regs->lt1 = gdb_regs[BFIN_LT1];
	regs->lb1 = gdb_regs[BFIN_LB1];
	regs->usp = gdb_regs[BFIN_USP];
	regs->syscfg = gdb_regs[BFIN_SYSCFG];
	regs->retx = gdb_regs[BFIN_PC];
	regs->retn = gdb_regs[BFIN_RETN];
	regs->rete = gdb_regs[BFIN_RETE];
	regs->pc = gdb_regs[BFIN_PC];

#if 0				/* can't change these */
	regs->astat = gdb_regs[BFIN_ASTAT];
	regs->seqstat = gdb_regs[BFIN_SEQSTAT];
	regs->ipend = gdb_regs[BFIN_IPEND];
#endif
}

struct hw_breakpoint {
	unsigned int occupied:1;
	unsigned int skip:1;
	unsigned int enabled:1;
	unsigned int type:1;
	unsigned int dataacc:2;
	unsigned short count;
	unsigned int addr;
} breakinfo[HW_BREAKPOINT_NUM];

int bfin_set_hw_break(unsigned long addr, int len, enum kgdb_bptype type)
{
	int breakno;

	switch (type) {
	case BP_HARDWARE_BREAKPOINT:
		for (breakno = 0; breakno < HW_BREAKPOINT_NUM; breakno++)
			if (!breakinfo[breakno].occupied) {
				breakinfo[breakno].occupied = 1;
				breakinfo[breakno].enabled = 1;
				breakinfo[breakno].type = 1;
				breakinfo[breakno].addr = addr;
				return 0;
			}
		break;
	case BP_WRITE_WATCHPOINT:
	case BP_READ_WATCHPOINT:
	case BP_ACCESS_WATCHPOINT:
		break;
	default:
		break;
	};

	return -ENOSPC;
}

int bfin_remove_hw_break(unsigned long addr, int len, enum kgdb_bptype type)
{
	int breakno;

	switch (type) {
	case BP_HARDWARE_BREAKPOINT:
		for (breakno = 0; breakno < HW_BREAKPOINT_NUM; breakno++)
			if (breakinfo[breakno].addr == addr)
				memset(&(breakinfo[breakno]), 0,
					sizeof(struct hw_breakpoint));
	case BP_WRITE_WATCHPOINT:
	case BP_READ_WATCHPOINT:
	case BP_ACCESS_WATCHPOINT:
		break;
	default:
		break;
	};

	return 0;
}

void bfin_remove_all_hw_break(void)
{
	memset(breakinfo, 0, sizeof(struct hw_breakpoint)*HW_BREAKPOINT_NUM);
}

/*
void kgdb_show_info(void)
{
	printk(KERN_DEBUG "hwd: wpia0=0x%x, wpiacnt0=%d, wpiactl=0x%x, wpstat=0x%x\n",
		bfin_read_WPIA0(), bfin_read_WPIACNT0(),
		bfin_read_WPIACTL(), bfin_read_WPSTAT());
}
*/

void bfin_correct_hw_break(void)
{
	int breakno;
	int correctit;
	uint32_t wpdactl = bfin_read_WPDACTL();

	correctit = 0;
	for (breakno = 0; breakno < HW_BREAKPOINT_NUM; breakno++) {
		if (breakinfo[breakno].type == 1) {
			switch (breakno) {
			case 0:
				if (breakinfo[breakno].enabled && !(wpdactl & WPIAEN0)) {
					correctit = 1;
					wpdactl &= ~(WPIREN01|EMUSW0);
					wpdactl |= WPIAEN0|WPICNTEN0;
					bfin_write_WPIA0(breakinfo[breakno].addr);
					bfin_write_WPIACNT0(breakinfo[breakno].skip);
				} else if (!breakinfo[breakno].enabled && (wpdactl & WPIAEN0)) {
					correctit = 1;
					wpdactl &= ~WPIAEN0;
				}
				break;

			case 1:
				if (breakinfo[breakno].enabled && !(wpdactl & WPIAEN1)) {
					correctit = 1;
					wpdactl &= ~(WPIREN01|EMUSW1);
					wpdactl |= WPIAEN1|WPICNTEN1;
					bfin_write_WPIA1(breakinfo[breakno].addr);
					bfin_write_WPIACNT1(breakinfo[breakno].skip);
				} else if (!breakinfo[breakno].enabled && (wpdactl & WPIAEN1)) {
					correctit = 1;
					wpdactl &= ~WPIAEN1;
				}
				break;

			case 2:
				if (breakinfo[breakno].enabled && !(wpdactl & WPIAEN2)) {
					correctit = 1;
					wpdactl &= ~(WPIREN23|EMUSW2);
					wpdactl |= WPIAEN2|WPICNTEN2;
					bfin_write_WPIA2(breakinfo[breakno].addr);
					bfin_write_WPIACNT2(breakinfo[breakno].skip);
				} else if (!breakinfo[breakno].enabled && (wpdactl & WPIAEN2)) {
					correctit = 1;
					wpdactl &= ~WPIAEN2;
				}
				break;

			case 3:
				if (breakinfo[breakno].enabled && !(wpdactl & WPIAEN3)) {
					correctit = 1;
					wpdactl &= ~(WPIREN23|EMUSW3);
					wpdactl |= WPIAEN3|WPICNTEN3;
					bfin_write_WPIA3(breakinfo[breakno].addr);
					bfin_write_WPIACNT3(breakinfo[breakno].skip);
				} else if (!breakinfo[breakno].enabled && (wpdactl & WPIAEN3)) {
					correctit = 1;
					wpdactl &= ~WPIAEN3;
				}
				break;
			case 4:
				if (breakinfo[breakno].enabled && !(wpdactl & WPIAEN4)) {
					correctit = 1;
					wpdactl &= ~(WPIREN45|EMUSW4);
					wpdactl |= WPIAEN4|WPICNTEN4;
					bfin_write_WPIA4(breakinfo[breakno].addr);
					bfin_write_WPIACNT4(breakinfo[breakno].skip);
				} else if (!breakinfo[breakno].enabled && (wpdactl & WPIAEN4)) {
					correctit = 1;
					wpdactl &= ~WPIAEN4;
				}
				break;
			case 5:
				if (breakinfo[breakno].enabled && !(wpdactl & WPIAEN5)) {
					correctit = 1;
					wpdactl &= ~(WPIREN45|EMUSW5);
					wpdactl |= WPIAEN5|WPICNTEN5;
					bfin_write_WPIA5(breakinfo[breakno].addr);
					bfin_write_WPIACNT5(breakinfo[breakno].skip);
				} else if (!breakinfo[breakno].enabled && (wpdactl & WPIAEN5)) {
					correctit = 1;
					wpdactl &= ~WPIAEN5;
				}
				break;
			}
		}
	}
	if (correctit) {
		wpdactl &= ~WPAND;
		wpdactl |= WPPWR;
		/*printk("correct_hw_break: wpdactl=0x%x\n", wpdactl);*/
		bfin_write_WPDACTL(wpdactl);
		CSYNC();
		/*kgdb_show_info();*/
	}
}

void kgdb_disable_hw_debug(struct pt_regs *regs)
{
	/* Disable hardware debugging while we are in kgdb */
	bfin_write_WPIACTL(bfin_read_WPIACTL() & ~0x1);
	CSYNC();
}

void kgdb_roundup_cpus(unsigned long flags)
{
}

void kgdb_post_primary_code(struct pt_regs *regs, int eVector, int err_code)
{
	/* Master processor is completely in the debugger */
	gdb_bfin_vector = eVector;
	gdb_bfin_errcode = err_code;
}

int kgdb_arch_handle_exception(int vector, int signo,
			       int err_code, char *remcom_in_buffer,
			       char *remcom_out_buffer,
			       struct pt_regs *regs)
{
	long addr;
	long breakno;
	char *ptr;
	int newPC;
	int wp_status;
	int i;

	switch (remcom_in_buffer[0]) {
	case 'c':
	case 's':
		if (kgdb_contthread && kgdb_contthread != current) {
			strcpy(remcom_out_buffer, "E00");
			break;
		}

		kgdb_contthread = NULL;

		/* try to read optional parameter, pc unchanged if no parm */
		ptr = &remcom_in_buffer[1];
		if (kgdb_hex2long(&ptr, &addr)) {
			regs->retx = addr;
		}
		newPC = regs->retx;

		/* clear the trace bit */
		regs->syscfg &= 0xfffffffe;

		/* set the trace bit if we're stepping */
		if (remcom_in_buffer[0] == 's') {
			regs->syscfg |= 0x1;
			kgdb_single_step = regs->ipend;
			kgdb_single_step >>= 6;
			for (i = 10; i > 0; i--, kgdb_single_step >>= 1)
				if (kgdb_single_step & 1)
					break;
			/* i indicate event priority of current stopped instruction
			 * user space instruction is 0, IVG15 is 1, IVTMR is 10.
			 * kgdb_single_step > 0 means in single step mode
			 */
			kgdb_single_step = i + 1;
		}

		wp_status = bfin_read_WPSTAT();
		CSYNC();

		if (vector == VEC_WATCH) {
			for (breakno = 0; breakno < 6; ++breakno) {
				if (wp_status & (1 << breakno)) {
					breakinfo->skip = 1;
					break;
				}
			}
		}
		bfin_correct_hw_break();

		bfin_write_WPSTAT(0);

		return 0;
	}			/* switch */
	return -1;		/* this means that we do not want to exit from the handler */
}

struct kgdb_arch arch_kgdb_ops = {
	.gdb_bpt_instr = {0xa1},
	.flags = KGDB_HW_BREAKPOINT,
	.set_hw_breakpoint = bfin_set_hw_break,
	.remove_hw_breakpoint = bfin_remove_hw_break,
	.remove_all_hw_break = bfin_remove_all_hw_break,
	.correct_hw_break = bfin_correct_hw_break,
};

static int hex(char ch)
{
	if ((ch >= 'a') && (ch <= 'f'))
		return ch - 'a' + 10;
	if ((ch >= '0') && (ch <= '9'))
		return ch - '0';
	if ((ch >= 'A') && (ch <= 'F'))
		return ch - 'A' + 10;
	return -1;
}

static int validate_memory_access_address(unsigned long addr, int size)
{
	if (size == 0)
		return 0;
	if (addr < (addr + size))
		return 0;
	if (addr >= 0x1000 && (addr + size) <= physical_mem_end)
		return 0;
	if (addr >= SYSMMR_BASE)
		return 0;
	if (addr >= L1_SCRATCH_START
	   && addr + size <= L1_SCRATCH_START + L1_SCRATCH_LENGTH)
		return 0;
	if (addr >= ASYNC_BANK0_BASE
	   && addr + size <= ASYNC_BANK3_BASE + ASYNC_BANK3_BASE)
		return 0;
#if L1_CODE_LENGTH != 0
	if (addr >= L1_CODE_START
	   && addr + size <= L1_CODE_START + L1_CODE_LENGTH)
		return 0;
#endif
#if L1_DATA_A_LENGTH != 0
	if (addr >= L1_DATA_A_START
	   && addr + size <= L1_DATA_A_START + L1_DATA_A_LENGTH)
		return 0;
#endif
#if L1_DATA_B_LENGTH != 0
	if (addr >= L1_DATA_B_START
	   && addr + size <= L1_DATA_B_START + L1_DATA_B_LENGTH)
		return 0;
#endif
#if L2_LENGTH != 0
	if (addr >= L2_START
	   && addr + size <= L2_START + L2_LENGTH)
		return 0;
#endif

	return EFAULT;
}

/*
 * Convert the memory pointed to by mem into hex, placing result in buf.
 * Return a pointer to the last char put in buf (null). May return an error.
 */
int kgdb_mem2hex(char *mem, char *buf, int count)
{
	char *tmp;
	int err = 0;
	unsigned char *pch;
	unsigned short mmr16;
	unsigned long mmr32;

	if (validate_memory_access_address((unsigned long)mem, count))
		return EFAULT;

	/*
	 * We use the upper half of buf as an intermediate buffer for the
	 * raw memory copy.  Hex conversion will work against this one.
	 */
	tmp = buf + count;

	if ((unsigned int)mem >= SYSMMR_BASE) { /*access MMR registers*/
		switch (count) {
		case 2:
			if ((unsigned int)mem % 2 == 0) {
				mmr16 = *(unsigned short *)mem;
				pch = (unsigned char *)&mmr16;
				*tmp++ = *pch++;
				*tmp++ = *pch++;
				tmp -= 2;
			} else
				err = EFAULT;
			break;
		case 4:
			if ((unsigned int)mem % 4 == 0) {
				mmr32 = *(unsigned long *)mem;
				pch = (unsigned char *)&mmr32;
				*tmp++ = *pch++;
				*tmp++ = *pch++;
				*tmp++ = *pch++;
				*tmp++ = *pch++;
				tmp -= 4;
			} else
				err = EFAULT;
			break;
		default:
			err = EFAULT;
		};
	} else if ((unsigned int)mem >= L1_CODE_START &&
		(unsigned int)(mem + count) <= L1_CODE_START + L1_CODE_LENGTH) {
		/* access L1 instruction SRAM*/
		if (dma_memcpy(tmp, mem, count) == NULL)
			err = EFAULT;
	} else
		err = probe_kernel_read(tmp, mem, count);

	if (!err) {
		while (count > 0) {
			buf = pack_hex_byte(buf, *tmp);
			tmp++;
			count--;
		}

		*buf = 0;
	}

	return err;
}

/*
 * Copy the binary array pointed to by buf into mem.  Fix $, #, and
 * 0x7d escaped with 0x7d.  Return a pointer to the character after
 * the last byte written.
 */
int kgdb_ebin2mem(char *buf, char *mem, int count)
{
	int err = 0;
	char c;

	if (validate_memory_access_address((unsigned long)mem, count))
		return EFAULT;

	while (count-- > 0) {
		c = *buf++;
		if (c == 0x7d)
			c = *buf++ ^ 0x20;

		if ((unsigned int)mem >= SYSMMR_BASE) {
			/*access MMR registers*/
			err = EFAULT;
		} else if ((unsigned int)mem >= L1_CODE_START &&
			(unsigned int)mem < L1_CODE_START + L1_CODE_LENGTH) {
			/* access L1 instruction SRAM */
			if (dma_memcpy(mem, &c, 1) == NULL)
				err = EFAULT;
		} else
			err = probe_kernel_write(mem, &c, 1);

		if (err)
			break;

		mem++;
	}

	return err;
}

/*
 * Convert the hex array pointed to by buf into binary to be placed in mem.
 * Return a pointer to the character AFTER the last byte written.
 * May return an error.
 */
int kgdb_hex2mem(char *buf, char *mem, int count)
{
	char *tmp_raw;
	char *tmp_hex;
	unsigned short *mmr16;
	unsigned long *mmr32;

	if (validate_memory_access_address((unsigned long)mem, count))
		return EFAULT;

	/*
	 * We use the upper half of buf as an intermediate buffer for the
	 * raw memory that is converted from hex.
	 */
	tmp_raw = buf + count * 2;

	tmp_hex = tmp_raw - 1;
	while (tmp_hex >= buf) {
		tmp_raw--;
		*tmp_raw = hex(*tmp_hex--);
		*tmp_raw |= hex(*tmp_hex--) << 4;
	}

	if ((unsigned int)mem >= SYSMMR_BASE) { /*access MMR registers*/
		switch (count) {
		case 2:
			if ((unsigned int)mem % 2 == 0) {
				mmr16 = (unsigned short *)tmp_raw;
				*(unsigned short *)mem = *mmr16;
			} else
				return EFAULT;
			break;
		case 4:
			if ((unsigned int)mem % 4 == 0) {
				mmr32 = (unsigned long *)tmp_raw;
				*(unsigned long *)mem = *mmr32;
			} else
				return EFAULT;
			break;
		default:
			return EFAULT;
		}
	} else if ((unsigned int)mem >= L1_CODE_START &&
		(unsigned int)(mem + count) <= L1_CODE_START + L1_CODE_LENGTH) {
		/* access L1 instruction SRAM */
		if (dma_memcpy(mem, tmp_raw, count) == NULL)
			return EFAULT;
	} else
		return probe_kernel_write(mem, tmp_raw, count);
	return 0;
}

int kgdb_validate_break_address(unsigned long addr)
{
	if (addr >= 0x1000 && (addr + BREAK_INSTR_SIZE) <= physical_mem_end)
		return 0;
	if (addr >= ASYNC_BANK0_BASE
	   && addr + BREAK_INSTR_SIZE <= ASYNC_BANK3_BASE + ASYNC_BANK3_BASE)
		return 0;
#if L1_CODE_LENGTH != 0
	if (addr >= L1_CODE_START
	   && addr + BREAK_INSTR_SIZE <= L1_CODE_START + L1_CODE_LENGTH)
		return 0;
#endif
#if L2_LENGTH != 0
	if (addr >= L2_START
	   && addr + BREAK_INSTR_SIZE <= L2_START + L2_LENGTH)
		return 0;
#endif

	return EFAULT;
}

int kgdb_arch_set_breakpoint(unsigned long addr, char *saved_instr)
{
	int err;

	if ((unsigned int)addr >= L1_CODE_START &&
		(unsigned int)(addr + BREAK_INSTR_SIZE) <
			L1_CODE_START + L1_CODE_LENGTH) {
		/* access L1 instruction SRAM */
		if (dma_memcpy(saved_instr, (void *)addr, BREAK_INSTR_SIZE)
			== NULL)
			return -EFAULT;

		if (dma_memcpy((void *)addr, arch_kgdb_ops.gdb_bpt_instr,
			BREAK_INSTR_SIZE) == NULL)
			return -EFAULT;

		return 0;
	} else {
		err = probe_kernel_read(saved_instr, (char *)addr,
			BREAK_INSTR_SIZE);
		if (err)
			return err;

		return probe_kernel_write((char *)addr,
			arch_kgdb_ops.gdb_bpt_instr, BREAK_INSTR_SIZE);
	}
}

int kgdb_arch_remove_breakpoint(unsigned long addr, char *bundle)
{
	if ((unsigned int)addr >= L1_CODE_START &&
		(unsigned int)(addr + BREAK_INSTR_SIZE) <
			L1_CODE_START + L1_CODE_LENGTH) {
		/* access L1 instruction SRAM */
		if (dma_memcpy((void *)addr, bundle, BREAK_INSTR_SIZE) == NULL)
			return -EFAULT;

		return 0;
	} else
		return probe_kernel_write((char *)addr,
				(char *)bundle, BREAK_INSTR_SIZE);
}

int kgdb_arch_init(void)
{
	kgdb_single_step = 0;

	bfin_remove_all_hw_break();
	return 0;
}

void kgdb_arch_exit(void)
{
}

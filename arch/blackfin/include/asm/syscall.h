/*
 * Magic syscall break down functions
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __ASM_BLACKFIN_SYSCALL_H__
#define __ASM_BLACKFIN_SYSCALL_H__

/*
 * Blackfin syscalls are simple:
 *	enter:
 *		p0: syscall number
 *		r{0,1,2,3,4,5}: syscall args 0,1,2,3,4,5
 *	exit:
 *		r0: return/error value
 */

#include <linux/err.h>
#include <linux/sched.h>
#include <asm/ptrace.h>

static inline long
syscall_get_nr(struct task_struct *task, struct pt_regs *regs)
{
	return regs->p0;
}

static inline void
syscall_rollback(struct task_struct *task, struct pt_regs *regs)
{
	/* was zu tun !? */
}

static inline long
syscall_get_error(struct task_struct *task, struct pt_regs *regs)
{
	return IS_ERR_VALUE(regs->r0) ? regs->r0 : 0;
}

static inline long
syscall_get_return_value(struct task_struct *task, struct pt_regs *regs)
{
	return regs->r0;
}

static inline void
syscall_set_return_value(struct task_struct *task, struct pt_regs *regs,
                         int error, long val)
{
	regs->r0 = error ? -error : val;
}

static inline void
syscall_get_arguments(struct task_struct *task, struct pt_regs *regs,
                      unsigned int i, unsigned int n, unsigned long *args)
{
	/* wtf is "i" ? */
	BUG_ON(i);

	switch (n) {
	case 6: args[5] = regs->r5;
	case 5: args[4] = regs->r4;
	case 4: args[3] = regs->r3;
	case 3: args[2] = regs->r2;
	case 2: args[1] = regs->r1;
	case 1: args[0] = regs->r0;
		break;
	default:
		BUG();
	}
}

static inline void
syscall_set_arguments(struct task_struct *task, struct pt_regs *regs,
                      unsigned int i, unsigned int n, const unsigned long *args)
{
	/* wtf is "i" ? */
	BUG_ON(i);

	switch (n) {
	case 6: regs->r5 = args[5];
	case 5: regs->r4 = args[4];
	case 4: regs->r3 = args[3];
	case 3: regs->r2 = args[2];
	case 2: regs->r1 = args[1];
	case 1: regs->r0 = args[0];
		break;
	default:
		BUG();
	}
}

#endif

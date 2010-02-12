/*
 * linux/kernel/ptrace.c is by Ross Biro 1/23/92, edited by Linus Torvalds
 * these modifications are Copyright 2004-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/errno.h>
#include <linux/ptrace.h>
#include <linux/user.h>
#include <linux/signal.h>
#include <linux/tracehook.h>
#include <linux/uaccess.h>

#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/processor.h>
#include <asm/asm-offsets.h>
#include <asm/dma.h>
#include <asm/fixed_code.h>
#include <asm/cacheflush.h>
#include <asm/mem_map.h>

#define TEXT_OFFSET 0
/*
 * does not yet catch signals sent when the child dies.
 * in exit.c or in signal.c.
 */

/* determines which bits in the SYSCFG reg the user has access to. */
/* 1 = access 0 = no access */
#define SYSCFG_MASK 0x0007	/* SYSCFG reg */
/* sets the trace bits. */
#define TRACE_BITS 0x0001

/*
 * Get all user integer registers.
 */
static inline int ptrace_getregs(struct task_struct *tsk, void __user *uregs)
{
	struct pt_regs regs;
	memcpy(&regs, task_pt_regs(tsk), sizeof(regs));
	regs.usp = tsk->thread.usp;
	return copy_to_user(uregs, &regs, sizeof(struct pt_regs)) ? -EFAULT : 0;
}

/* Mapping from PT_xxx to the stack offset at which the register is
 * saved.  Notice that usp has no stack-slot and needs to be treated
 * specially (see get_reg/put_reg below).
 */

/*
 * Get contents of register REGNO in task TASK.
 */
static inline long get_reg(struct task_struct *task, int regno)
{
	unsigned char *reg_ptr;
	struct pt_regs *regs = task_pt_regs(task);

	reg_ptr = (char *)regs;

	switch (regno) {
	case PT_USP:
		return task->thread.usp;
	default:
		if (regno <= 216)
			return *(long *)(reg_ptr + regno);
	}
	/* slight mystery ... never seems to come here but kernel misbehaves without this code! */

	printk(KERN_WARNING "Request to get for unknown register %d\n", regno);
	return 0;
}

/*
 * Write contents of register REGNO in task TASK.
 */
static inline int
put_reg(struct task_struct *task, int regno, unsigned long data)
{
	char *reg_ptr;
	struct pt_regs *regs = task_pt_regs(task);

	reg_ptr = (char *)regs;

	switch (regno) {
	case PT_PC:
		/*********************************************************************/
		/* At this point the kernel is most likely in exception.             */
		/* The RETX register will be used to populate the pc of the process. */
		/*********************************************************************/
		regs->retx = data;
		regs->pc = data;
		break;
	case PT_RETX:
		break;		/* regs->retx = data; break; */
	case PT_USP:
		regs->usp = data;
		task->thread.usp = data;
		break;
	default:
		if (regno <= 216)
			*(long *)(reg_ptr + regno) = data;
	}
	return 0;
}

/*
 * check that an address falls within the bounds of the target process's memory mappings
 */
static inline int is_user_addr_valid(struct task_struct *child,
				     unsigned long start, unsigned long len)
{
	struct vm_area_struct *vma;
	struct sram_list_struct *sraml;

	/* overflow */
	if (start + len < start)
		return -EIO;

	vma = find_vma(child->mm, start);
	if (vma && start >= vma->vm_start && start + len <= vma->vm_end)
			return 0;

	for (sraml = child->mm->context.sram_list; sraml; sraml = sraml->next)
		if (start >= (unsigned long)sraml->addr
		    && start + len < (unsigned long)sraml->addr + sraml->length)
			return 0;

	if (start >= FIXED_CODE_START && start + len < FIXED_CODE_END)
		return 0;

	return -EIO;
}

void user_enable_single_step(struct task_struct *child)
{
	unsigned long tmp;
	tmp = get_reg(child, PT_SYSCFG) | (TRACE_BITS);
	put_reg(child, PT_SYSCFG, tmp);
}

/*
 * Called by kernel/ptrace.c when detaching..
 */
void user_disable_single_step(struct task_struct *child)
{
	unsigned long tmp;
	/* make sure the single step bit is not set. */
	tmp = get_reg(child, PT_SYSCFG) & ~TRACE_BITS;
	put_reg(child, PT_SYSCFG, tmp);
}

long arch_ptrace(struct task_struct *child, long request, long addr, long data)
{
	int ret;
	unsigned long __user *datap = (unsigned long __user *)data;
	void *paddr = (void *)addr;

	switch (request) {
		/* when I and D space are separate, these will need to be fixed. */
	case PTRACE_PEEKDATA:
		pr_debug("ptrace: PEEKDATA\n");
		/* fall through */
	case PTRACE_PEEKTEXT:	/* read word at location addr. */
		{
			unsigned long tmp = 0;
			int copied = 0, to_copy = sizeof(tmp);

			ret = -EIO;
			pr_debug("ptrace: PEEKTEXT at addr 0x%08lx + %i\n", addr, to_copy);
			if (is_user_addr_valid(child, addr, to_copy) < 0)
				break;
			pr_debug("ptrace: user address is valid\n");

			switch (bfin_mem_access_type(addr, to_copy)) {
			case BFIN_MEM_ACCESS_CORE:
			case BFIN_MEM_ACCESS_CORE_ONLY:
				copied = access_process_vm(child, addr, &tmp,
				                           to_copy, 0);
				if (copied)
					break;

				/* hrm, why didn't that work ... maybe no mapping */
				if (addr >= FIXED_CODE_START &&
				    addr + to_copy <= FIXED_CODE_END) {
					copy_from_user_page(0, 0, 0, &tmp, paddr, to_copy);
					copied = to_copy;
				} else if (addr >= BOOT_ROM_START) {
					memcpy(&tmp, paddr, to_copy);
					copied = to_copy;
				}

				break;
			case BFIN_MEM_ACCESS_DMA:
				if (safe_dma_memcpy(&tmp, paddr, to_copy))
					copied = to_copy;
				break;
			case BFIN_MEM_ACCESS_ITEST:
				if (isram_memcpy(&tmp, paddr, to_copy))
					copied = to_copy;
				break;
			default:
				copied = 0;
				break;
			}

			pr_debug("ptrace: copied size %d [0x%08lx]\n", copied, tmp);
			if (copied == to_copy)
				ret = put_user(tmp, datap);
			break;
		}

		/* read the word at location addr in the USER area. */
	case PTRACE_PEEKUSR:
		{
			unsigned long tmp;
			ret = -EIO;
			tmp = 0;
			if ((addr & 3) || (addr > (sizeof(struct pt_regs) + 16))) {
				printk(KERN_WARNING "ptrace error : PEEKUSR : temporarily returning "
				                    "0 - %x sizeof(pt_regs) is %lx\n",
				     (int)addr, sizeof(struct pt_regs));
				break;
			}
			if (addr == sizeof(struct pt_regs)) {
				/* PT_TEXT_ADDR */
				tmp = child->mm->start_code + TEXT_OFFSET;
			} else if (addr == (sizeof(struct pt_regs) + 4)) {
				/* PT_TEXT_END_ADDR */
				tmp = child->mm->end_code;
			} else if (addr == (sizeof(struct pt_regs) + 8)) {
				/* PT_DATA_ADDR */
				tmp = child->mm->start_data;
#ifdef CONFIG_BINFMT_ELF_FDPIC	/* backwards compat */
			} else if (addr == (sizeof(struct pt_regs) + 12)) {
				request = PTRACE_GETFDPIC;
				addr = PTRACE_GETFDPIC_EXEC;
				goto case_default;
			} else if (addr == (sizeof(struct pt_regs) + 16)) {
				request = PTRACE_GETFDPIC;
				addr = PTRACE_GETFDPIC_INTERP;
				goto case_default;
#endif
			} else {
				tmp = get_reg(child, addr);
			}
			ret = put_user(tmp, datap);
			break;
		}

		/* when I and D space are separate, this will have to be fixed. */
	case PTRACE_POKEDATA:
		pr_debug("ptrace: PTRACE_PEEKDATA\n");
		/* fall through */
	case PTRACE_POKETEXT:	/* write the word at location addr. */
		{
			int copied = 0, to_copy = sizeof(data);

			ret = -EIO;
			pr_debug("ptrace: POKETEXT at addr 0x%08lx + %i bytes %lx\n",
			         addr, to_copy, data);
			if (is_user_addr_valid(child, addr, to_copy) < 0)
				break;
			pr_debug("ptrace: user address is valid\n");

			switch (bfin_mem_access_type(addr, to_copy)) {
			case BFIN_MEM_ACCESS_CORE:
			case BFIN_MEM_ACCESS_CORE_ONLY:
				copied = access_process_vm(child, addr, &data,
				                           to_copy, 1);
				break;
			case BFIN_MEM_ACCESS_DMA:
				if (safe_dma_memcpy(paddr, &data, to_copy))
					copied = to_copy;
				break;
			case BFIN_MEM_ACCESS_ITEST:
				if (isram_memcpy(paddr, &data, to_copy))
					copied = to_copy;
				break;
			default:
				copied = 0;
				break;
			}

			pr_debug("ptrace: copied size %d\n", copied);
			if (copied == to_copy)
				ret = 0;
			break;
		}

	case PTRACE_POKEUSR:	/* write the word at location addr in the USER area */
		ret = -EIO;
		if ((addr & 3) || (addr > (sizeof(struct pt_regs) + 16))) {
			printk(KERN_WARNING "ptrace error : POKEUSR: temporarily returning 0\n");
			break;
		}

		if (addr >= (sizeof(struct pt_regs))) {
			ret = 0;
			break;
		}
		if (addr == PT_SYSCFG) {
			data &= SYSCFG_MASK;
			data |= get_reg(child, PT_SYSCFG);
		}
		ret = put_reg(child, addr, data);
		break;

	case PTRACE_GETREGS:
		/* Get all gp regs from the child. */
		ret = ptrace_getregs(child, datap);
		break;

	case PTRACE_SETREGS:
		printk(KERN_WARNING "ptrace: SETREGS: **** NOT IMPLEMENTED ***\n");
		/* Set all gp regs in the child. */
		ret = 0;
		break;

	case_default:
	default:
		ret = ptrace_request(child, request, addr, data);
		break;
	}

	return ret;
}

asmlinkage int syscall_trace_enter(struct pt_regs *regs)
{
	int ret = 0;

	if (test_thread_flag(TIF_SYSCALL_TRACE))
		ret = tracehook_report_syscall_entry(regs);

	return ret;
}

asmlinkage void syscall_trace_leave(struct pt_regs *regs)
{
	if (test_thread_flag(TIF_SYSCALL_TRACE))
		tracehook_report_syscall_exit(regs, 0);
}

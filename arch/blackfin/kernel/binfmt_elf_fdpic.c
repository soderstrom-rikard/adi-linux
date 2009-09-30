/*
 * FDPIC ELF hooks
 *
 * Copyright (c) 2006-2009 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#include <linux/mm.h>
#include <linux/elf.h>
#include <linux/elf-fdpic.h>
#include <linux/kernel.h>

#include <asm/cacheflush.h>
#include <asm/dma.h>

int elf_fdpic_plat_process_phdr(struct mm_struct *mm,
                                struct elf_fdpic_params *params,
                                struct elf32_phdr *phdr,
                                unsigned long *maddr, unsigned long *disp)
{
	/* 0xfeb00000, 0xfec00000, 0xff700000, 0xff800000, 0xff900000
	 * and 0xffa00000 are also used in Dynamic linker and GNU ld.
	 * They need to be kept synchronized.
	 */
	unsigned long flag = 0;
	const char *type = NULL;

	unsigned int e_flags = params->hdr.e_flags;
	unsigned long p_vaddr = phdr->p_vaddr;
	unsigned long p_flags = phdr->p_flags;

	if (((e_flags & EF_BFIN_CODE_IN_L1) || p_vaddr == 0xffa00000) &&
	    (p_flags & (PF_W | PF_X)) == PF_X)
	{
		flag = L1_INST_SRAM;
		type = "L1 instruction";

	} else if (((e_flags & EF_BFIN_DATA_IN_L1) ||
	            p_vaddr == 0xff700000 ||
	            p_vaddr == 0xff800000 ||
	            p_vaddr == 0xff900000) &&
	           (p_flags & (PF_X | PF_W)) == PF_W)
	{
		if (p_vaddr == 0xff800000) {
			flag = L1_DATA_A_SRAM;
			type = "L1 Data A";
		} else if (p_vaddr == 0xff900000) {
			flag = L1_DATA_B_SRAM;
			type = "L1 Data B";
		} else {
			flag = L1_DATA_SRAM;
			type = "L1 Data";
		}

	} else if (p_vaddr == 0xfeb00000 || p_vaddr == 0xfec00000) {
		flag = L2_SRAM;
		type = "L2";
	}

	if (flag) {
		void *sram_addr = sram_alloc_with_lsl(phdr->p_memsz, flag);
		if (sram_addr == NULL) {
			printk(KERN_ERR "elf_fdpic: not enough %s sram\n", type);
			return -ENOMEM;
		}

		if (flag & L1_INST_SRAM)
			safe_dma_memcpy(sram_addr, (const void *)(*maddr + *disp), phdr->p_memsz);
		else
			memcpy(sram_addr, (const void *)(*maddr + *disp), phdr->p_memsz);

		down_write(&mm->mmap_sem);
		do_munmap(mm, *maddr, phdr->p_memsz + *disp);
		up_write(&mm->mmap_sem);
		*maddr = (unsigned long)sram_addr;
		*disp = 0;
	}

	return 0;
}

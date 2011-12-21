#include <linux/linkage.h>
#include <asm/blackfin.h>
#include <asm/dpmc.h>

.section .text
ENTRY(_bf609_hibernate)
	bfin_cpu_reg_save;
	idle;

.Lpm_resume_here:

	bfin_cpu_reg_restore;
ENDPROC(_bf609_hibernate)


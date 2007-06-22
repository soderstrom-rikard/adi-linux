#
# arch/blackfin/Makefile
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#


CROSS_COMPILE    ?= bfin-uclinux-
LDFLAGS_vmlinux  := -X
OBJCOPYFLAGS     := -O binary -R .note -R .comment -S
GZFLAGS          := -9

CFLAGS_MODULE    += -mlong-calls
KALLSYMS         += --symbol-prefix=_

KBUILD_DEFCONFIG := BF537-STAMP_defconfig

# setup the machine name and the machine dependent settings
machine-$(CONFIG_BF531) := bf533
machine-$(CONFIG_BF532) := bf533
machine-$(CONFIG_BF533) := bf533
machine-$(CONFIG_BF534) := bf537
machine-$(CONFIG_BF536) := bf537
machine-$(CONFIG_BF537) := bf537
machine-$(CONFIG_BF548) := bf548
machine-$(CONFIG_BF549) := bf548
machine-$(CONFIG_BF561) := bf561
MACHINE := $(machine-y)
export MACHINE

ifdef CONFIG_BF531

ifdef CONFIG_BF_REV_0_3
CFLAGS		+= -mcpu=bf531-0.3
AFLAGS		+= -mcpu=bf531-0.3
else ifdef CONFIG_BF_REV_0_4
CFLAGS		+= -mcpu=bf531-0.4
AFLAGS		+= -mcpu=bf531-0.4
else ifdef CONFIG_BF_REV_0_5
CFLAGS		+= -mcpu=bf531-0.5
AFLAGS		+= -mcpu=bf531-0.5
else ifdef CONFIG_BF_REV_NONE
CFLAGS		+= -mcpu=bf531-none
AFLAGS		+= -mcpu=bf531-none
else ifdef CONFIG_BF_REV_ANY
CFLAGS		+= -mcpu=bf531-any
AFLAGS		+= -mcpu=bf531-any
else
CFLAGS		+= -mcpu=bf531
AFLAGS		+= -mcpu=bf531
endif

else ifdef CONFIG_BF532

ifdef CONFIG_BF_REV_0_3
CFLAGS		+= -mcpu=bf532-0.3
AFLAGS		+= -mcpu=bf532-0.3
else ifdef CONFIG_BF_REV_0_4
CFLAGS		+= -mcpu=bf532-0.4
AFLAGS		+= -mcpu=bf532-0.4
else ifdef CONFIG_BF_REV_0_5
CFLAGS		+= -mcpu=bf532-0.5
AFLAGS		+= -mcpu=bf532-0.5
else ifdef CONFIG_BF_REV_NONE
CFLAGS		+= -mcpu=bf532-none
AFLAGS		+= -mcpu=bf532-none
else ifdef CONFIG_BF_REV_ANY
CFLAGS		+= -mcpu=bf532-any
AFLAGS		+= -mcpu=bf532-any
else
CFLAGS		+= -mcpu=bf532
AFLAGS		+= -mcpu=bf532
endif

else ifdef CONFIG_BF533

ifdef CONFIG_BF_REV_0_3
CFLAGS		+= -mcpu=bf533-0.3
AFLAGS		+= -mcpu=bf533-0.3
else ifdef CONFIG_BF_REV_0_4
CFLAGS		+= -mcpu=bf533-0.4
AFLAGS		+= -mcpu=bf533-0.4
else ifdef CONFIG_BF_REV_0_5
CFLAGS		+= -mcpu=bf533-0.5
AFLAGS		+= -mcpu=bf533-0.5
else ifdef CONFIG_BF_REV_NONE
CFLAGS		+= -mcpu=bf533-none
AFLAGS		+= -mcpu=bf533-none
else ifdef CONFIG_BF_REV_ANY
CFLAGS		+= -mcpu=bf533-any
AFLAGS		+= -mcpu=bf533-any
else
CFLAGS		+= -mcpu=bf533
AFLAGS		+= -mcpu=bf533
endif

else ifdef CONFIG_BF534

ifdef CONFIG_BF_REV_0_2
CFLAGS		+= -mcpu=bf534-0.2
AFLAGS		+= -mcpu=bf534-0.2
else ifdef CONFIG_BF_REV_0_3
CFLAGS		+= -mcpu=bf534-0.3
AFLAGS		+= -mcpu=bf534-0.3
else ifdef CONFIG_BF_REV_NONE
CFLAGS		+= -mcpu=bf534-none
AFLAGS		+= -mcpu=bf534-none
else ifdef CONFIG_BF_REV_ANY
CFLAGS		+= -mcpu=bf534-any
AFLAGS		+= -mcpu=bf534-any
else
CFLAGS		+= -mcpu=bf534
AFLAGS		+= -mcpu=bf534
endif

else ifdef CONFIG_BF536

ifdef CONFIG_BF_REV_0_2
CFLAGS		+= -mcpu=bf536-0.2
AFLAGS		+= -mcpu=bf536-0.2
else ifdef CONFIG_BF_REV_0_3
CFLAGS		+= -mcpu=bf536-0.3
AFLAGS		+= -mcpu=bf536-0.3
else ifdef CONFIG_BF_REV_NONE
CFLAGS		+= -mcpu=bf536-none
AFLAGS		+= -mcpu=bf536-none
else ifdef CONFIG_BF_REV_ANY
CFLAGS		+= -mcpu=bf536-any
AFLAGS		+= -mcpu=bf536-any
else
CFLAGS		+= -mcpu=bf536
AFLAGS		+= -mcpu=bf536
endif

else ifdef CONFIG_BF537

ifdef CONFIG_BF_REV_0_2
CFLAGS		+= -mcpu=bf537-0.2
AFLAGS		+= -mcpu=bf537-0.2
else ifdef CONFIG_BF_REV_0_3
CFLAGS		+= -mcpu=bf537-0.3
AFLAGS		+= -mcpu=bf537-0.3
else ifdef CONFIG_BF_REV_NONE
CFLAGS		+= -mcpu=bf537-none
AFLAGS		+= -mcpu=bf537-none
else ifdef CONFIG_BF_REV_ANY
CFLAGS		+= -mcpu=bf537-any
AFLAGS		+= -mcpu=bf537-any
else
CFLAGS		+= -mcpu=bf537
AFLAGS		+= -mcpu=bf537
endif

else ifdef CONFIG_BF549

ifdef CONFIG_BF_REV_0_0
CFLAGS		+= -mcpu=bf549-0.0
AFLAGS		+= -mcpu=bf549-0.0
else ifdef CONFIG_BF_REV_NONE
CFLAGS		+= -mcpu=bf539-none
AFLAGS		+= -mcpu=bf539-none
else ifdef CONFIG_BF_REV_ANY
CFLAGS		+= -mcpu=bf539-any
AFLAGS		+= -mcpu=bf539-any
else
CFLAGS		+= -mcpu=bf539
AFLAGS		+= -mcpu=bf539
endif

else ifdef CONFIG_BF561

ifdef CONFIG_BF_REV_0_3
CFLAGS		+= -mcpu=bf561-0.3
AFLAGS		+= -mcpu=bf561-0.3
else ifdef CONFIG_BF_REV_0_4
CFLAGS		+= -mcpu=bf561-0.4
AFLAGS		+= -mcpu=bf561-0.4
else ifdef CONFIG_BF_REV_0_5
CFLAGS		+= -mcpu=bf561-0.5
AFLAGS		+= -mcpu=bf561-0.5
else ifdef CONFIG_BF_REV_NONE
CFLAGS		+= -mcpu=bf561-none
AFLAGS		+= -mcpu=bf561-none
else ifdef CONFIG_BF_REV_ANY
CFLAGS		+= -mcpu=bf561-any
AFLAGS		+= -mcpu=bf561-any
else
CFLAGS		+= -mcpu=bf561
AFLAGS		+= -mcpu=bf561
endif

endif

head-y   := arch/$(ARCH)/mach-$(MACHINE)/head.o arch/$(ARCH)/kernel/init_task.o

core-y   += arch/$(ARCH)/kernel/ arch/$(ARCH)/mm/ arch/$(ARCH)/mach-common/

# If we have a machine-specific directory, then include it in the build.
ifneq ($(machine-y),)
core-y   += arch/$(ARCH)/mach-$(MACHINE)/
core-y   += arch/$(ARCH)/mach-$(MACHINE)/boards/
endif

libs-y   += arch/$(ARCH)/lib/

drivers-$(CONFIG_OPROFILE) += arch/$(ARCH)/oprofile/



#	Update machine arch symlinks if something which affects
#	them changed.  We use .mach to indicate when they were updated
#	last, otherwise make uses the target directory mtime.

include/asm-blackfin/.mach: $(wildcard include/config/arch/*.h) include/config/auto.conf
	@echo '  SYMLINK include/asm-$(ARCH)/mach-$(MACHINE) -> include/asm-$(ARCH)/mach'
ifneq ($(KBUILD_SRC),)
	$(Q)mkdir -p include/asm-$(ARCH)
	$(Q)ln -fsn $(srctree)/include/asm-$(ARCH)/mach-$(MACHINE) include/asm-$(ARCH)/mach
else
	$(Q)ln -fsn mach-$(MACHINE) include/asm-$(ARCH)/mach
endif
	@touch $@

CLEAN_FILES += \
	include/asm-$(ARCH)/asm-offsets.h \
	arch/$(ARCH)/kernel/asm-offsets.s \
	include/asm-$(ARCH)/mach \
	include/asm-$(ARCH)/.mach

archprepare: include/asm-blackfin/.mach
archclean:
	$(Q)$(MAKE) $(clean)=$(boot)


all: vmImage
boot := arch/$(ARCH)/boot
BOOT_TARGETS = vmImage
.PHONY: $(BOOT_TARGETS)
$(BOOT_TARGETS): vmlinux
	$(Q)$(MAKE) $(build)=$(boot) $(boot)/$@
define archhelp
  echo  '* vmImage         - Kernel-only image for U-Boot (arch/$(ARCH)/boot/vmImage)'
endef

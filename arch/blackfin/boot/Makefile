#
# arch/blackfin/boot/Makefile
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#

MKIMAGE := $(srctree)/scripts/mkuboot.sh

targets := vmImage vmImage.bz2 vmImage.gz vmImage.lzma
extra-y += vmlinux.bin vmlinux.gz

quiet_cmd_uimage = UIMAGE  $@
      cmd_uimage = $(CONFIG_SHELL) $(MKIMAGE) -A $(ARCH) -O linux -T kernel \
                   -C $(2) -n 'Linux-$(KERNELRELEASE)' -a $(CONFIG_BOOT_LOAD) \
                   -e $(shell $(NM) vmlinux | awk '$$NF == "__start" {print $$1}') \
                   -d $< $@

$(obj)/vmlinux.bin: vmlinux FORCE
	$(call if_changed,objcopy)

$(obj)/vmlinux.gz: $(obj)/vmlinux.bin FORCE
	$(call if_changed,gzip)

$(obj)/vmlinux.bz2: $(obj)/vmlinux.bin FORCE
	$(call if_changed,bzip2)

$(obj)/vmlinux.lzma: $(obj)/vmlinux.bin FORCE
	$(call if_changed,lzma)

$(obj)/vmImage.bz2: $(obj)/vmlinux.bz2
	$(call if_changed,uimage,bzip2)

$(obj)/vmImage.gz: $(obj)/vmlinux.gz
	$(call if_changed,uimage,gzip)

$(obj)/vmImage.lzma: $(obj)/vmlinux.lzma
	$(call if_changed,uimage,lzma)

$(obj)/vmImage: $(obj)/vmImage.gz
	@ln -sf $(notdir $<) $@

install:
	sh $(srctree)/$(src)/install.sh $(KERNELRELEASE) $(BOOTIMAGE) System.map "$(INSTALL_PATH)"

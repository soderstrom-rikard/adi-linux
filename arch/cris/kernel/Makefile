# $Id: Makefile 5114 2008-08-07 06:24:53Z cooloney $
#
# Makefile for the linux kernel.
#

extra-y	:= vmlinux.lds

obj-y   := process.o traps.o irq.o ptrace.o setup.o time.o sys_cris.o

obj-$(CONFIG_MODULES)    += crisksyms.o
obj-$(CONFIG_MODULES)	 += module.o
obj-$(CONFIG_SYSTEM_PROFILER)   += profile.o

clean:


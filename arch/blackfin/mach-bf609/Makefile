#
# arch/blackfin/mach-bf609/Makefile
#

obj-y := dma.o clock.o ints-priority.o
obj-$(CONFIG_PM) += pm.o dpm.o
obj-$(CONFIG_BFIN_COREB) += coreb.o coreb_start.o
obj-$(CONFIG_SCB_PRIORITY) += scb.o
obj-$(CONFIG_ICC:m=y) += icc.o

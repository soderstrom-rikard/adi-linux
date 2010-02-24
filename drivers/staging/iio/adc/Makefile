
# Makefile for industrial I/O ADC drivers
#

max1363-y := max1363_core.o
max1363-$(CONFIG_IIO_RING_BUFFER) += max1363_ring.o

obj-$(CONFIG_MAX1363) += max1363.o
obj-$(CONFIG_AD7150) += ad7150.o
obj-$(CONFIG_AD7152) += ad7152.o
obj-$(CONFIG_AD774X) += ad774x.o
obj-$(CONFIG_AD7414) += ad7414.o

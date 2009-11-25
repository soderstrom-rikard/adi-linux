/* Changed from asm-m68k version, Lineo Inc. 	May 2001	*/

#ifndef _ASM_BFIN_PCI_H
#define _ASM_BFIN_PCI_H

#include <asm/scatterlist.h>
#include <asm-generic/pci-dma-compat.h>

/*
 *
 * Written by Wout Klaren.
 */

/* Added by Chang Junxiao */
#define PCIBIOS_MIN_IO 0x00001000
#define PCIBIOS_MIN_MEM 0x10000000

#define PCI_DMA_BUS_IS_PHYS       (1)
struct pci_ops;

/*
 * Structure with hardware dependent information and functions of the
 * PCI bus.
 */
struct pci_bus_info {

	/*
	 * Resources of the PCI bus.
	 */
	struct resource mem_space;
	struct resource io_space;

	/*
	 * System dependent functions.
	 */
	struct pci_ops *bfin_pci_ops;
	void (*fixup) (int pci_modify);
	void (*conf_device) (unsigned char bus, unsigned char device_fn);
};

#define pcibios_assign_all_busses()	0
static inline void pcibios_set_master(struct pci_dev *dev)
{

	/* No special bus mastering setup handling */
}
static inline void pcibios_penalize_isa_irq(int irq)
{

	/* We don't do dynamic PCI IRQ allocation */
}
#endif				/* _ASM_BFIN_PCI_H */

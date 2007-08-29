#include "defs.h"
#include <linux/pci.h>
#ifdef CAN4LINUX_PCI
# ifndef CONFIG_PCI
#   error "trying to compile a PCI driver for a kernel without CONFIG_PCI"
# endif

#ifdef CAN4LINUX_PCI
/* reset both can controllers on the EMS-Wünsche CPC-PCI Board */
/* writing to the control range at BAR1 of the PCI board */
void reset_CPC_PCI(unsigned long address)
{
unsigned long ptr = (unsigned long)ioremap(address, 32);
    DBGin("reset_CPC_PCI");
    writeb(0x01, (void __iomem *)ptr);
}

/* check memory region if there is a CAN controller
*  assume the controller was resetted before testing 
*
*  The check for an avaliable controller is difficult !
*  After an Hardware Reset (or power on) the Conroller 
*  is in the so-called 'BasicCAN' mode.
*     we can check for: 
*         adress  name      value
*	    0x00  mode       0x21
*           0x02  status     0xc0
*           0x03  interrupt  0xe0
* Once loaded thr driver switches into 'PeliCAN' mode and things are getting
* difficult, because we now have only a 'soft reset' with not so  unique
* values. The have to be masked before comparing.
*         adress  name       mask   value
*	    0x00  mode               
*           0x01  command    0xff    0x00
*           0x02  status     0x37    0x34
*           0x03  interrupt  0xfb    0x00
*
*/
int controller_available(unsigned long address, int offset)
{
unsigned long ptr = (unsigned long)ioremap(address, 32 * offset);

    DBGin("controller_available");
    /* printk("controller_available %ld\n", address); */


    /* printk("0x%0x, ", readb(ptr + (2 * offset)) ); */
    /* printk("0x%0x\n", readb(ptr + (3 * offset)) ); */

    if ( 0x21 == readb((void __iomem *)ptr))  {
	/* compare rest values of status and interrupt register */
	if(   0x0c == readb((void __iomem *)ptr + (2 * offset))
	   && 0xe0 == readb((void __iomem *)ptr + (3 * offset)) ) {
	    return 1;
	} else {
	    return 0;
	}
    } else {
	/* may be called after a 'soft reset' in 'PeliCAN' mode */
	/*   value     address                     mask    */
	if(   0x00 ==  readb((void __iomem *)ptr + (1 * offset))
	   && 0x34 == (readb((void __iomem *)ptr + (2 * offset))    & 0x37)
	   && 0x00 == (readb((void __iomem *)ptr + (3 * offset))    & 0xfb)
	  ) {
	return 1;
    } else {
	return 0;
    }

    }
}
#endif




#define PCI_BASE_ADDRESS0(dev) (dev->resource[0].start)
#define PCI_BASE_ADDRESS1(dev) (dev->resource[1].start)
#define PCI_BASE_ADDRESS2(dev) (dev->resource[2].start)
#define PCI_BASE_ADDRESS3(dev) (dev->resource[3].start)

/* used for storing the global pci register address */
upointer_t Can_pitapci_control[MAX_CHANNELS];

# if defined(CPC_PCI)
int pcimod_scan(void)
{
struct	pci_dev *pdev = NULL;
int	candev = 0;				/* number of found devices */
unsigned long ptr;				/* ptr to PITA control */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
    /* Testing the PCI presence is no longer neccessary
     * On a non  PCI kernel, insmod will just complain
     */
    if (pci_present ()) {
#endif
	    while((pdev =
	    	pci_find_device (PCI_VENDOR_CAN_EMS, PCI_DEVICE_CAN, pdev))) {

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	    printk("  found CPC-PCI: %s\n", pdev->name);
#else
	    printk("  found CPC-PCI: %s\n", pci_pretty_name(pdev));
	    printk("               : %s\n", pci_name(pdev));
#endif
	    if (pci_enable_device(pdev)) {
		continue;
    }
	    /* printk("        using IRQ %d\n", pdev->irq); */

	    ptr = (unsigned long)ioremap(PCI_BASE_ADDRESS0(pdev), 256);
	    /* enable memory access */
	    /* printk("write to pita\n"); */
	    writel(0x04000000, (void __iomem *)ptr + 0x1c);
	    Can_pitapci_control[candev] = ptr;

	    /* printk("        pita ptr %lx\n", ptr); */
	    /* printk("---------------\n"); */
	    /* dump_CAN(PCI_BASE_ADDRESS1(pdev)+0x400, 4); */
	    /* printk("---------------\n"); */
	    /* dump_CAN(PCI_BASE_ADDRESS1(pdev)+0x600, 4); */

	    /* PCI_BASE_ADDRESS1:
	     * at address 0 are some EMS control registers
	     * at address 0x400 the first controller area 
	     * at address 0x600 the second controller area 
	     * registers are read as 32bit
	     *
	     * at adress 0 we can verify the card
	     * 0x55 0xaa 0x01 0xcb
     */
	    {
		void __iomem *sigptr; /* ptr to EMS signature  */
		unsigned long signature = 0;
	        sigptr = (void __iomem *)ioremap(PCI_BASE_ADDRESS1(pdev), 256);
	        signature =
	        	  (readb(sigptr)      << 24)
	        	+ (readb(sigptr +  4) << 16)
	        	+ (readb(sigptr +  8) <<  8)
	        	+  readb(sigptr + 12);
	    	/* printk("        signature  %lx\n", signature); */
	    	if( 0x55aa01cb != signature) {
	    	    printk(" wrong signature -- no EMS CPC-PCI board\n");
		    return -ENODEV;
	    	}
	    }
	    /* we are now sure to have the right board,
	       reset the CAN controller(s) */
	    reset_CPC_PCI(PCI_BASE_ADDRESS1(pdev) + 0x400);
	    reset_CPC_PCI(PCI_BASE_ADDRESS1(pdev) + 0x600);

	    /* enable interrupts Int_0 */
	    /* write to PITAs ICR register */
	    writel(0x00020000, (void __iomem *)Can_pitapci_control[candev] + 0x0);

	    /* look for a CAN controller at 0x400 */
	    if(controller_available(PCI_BASE_ADDRESS1(pdev) + 0x400, 4)) {
		printk(" CAN: %d. at pos 1\n", candev + 1);
		if(candev > MAX_CHANNELS) {
		    printk("CAN: only %d devices supported\n", MAX_CHANNELS);
		    break; /* the devices scan loop */
		}
		Base[candev]
		= (unsigned long)ioremap(PCI_BASE_ADDRESS1(pdev) + 0x400, 32*4);
		IOModel[candev] = 'm';
		IRQ[candev] = pdev->irq;
		candev++;
	    } else {
		/* printk(" CAN: NO at pos 1\n"); */
		;
	    }
	    /* look for a CAN controller at 0x400 */
	    if(controller_available(PCI_BASE_ADDRESS1(pdev) + 0x600, 4)) {
		printk(" CAN: %d. at pos 2\n", candev + 1);
		if(candev > MAX_CHANNELS) {
		    printk("CAN: only %d devices supported\n", MAX_CHANNELS);
		    break; /* the devices scan loop */
		}
		/* share the board control register with prev ch */
		Can_pitapci_control[candev] = 
		    Can_pitapci_control[candev - 1];
		Base[candev]
		= (unsigned long)ioremap(PCI_BASE_ADDRESS1(pdev) + 0x600, 32*4);
		IOModel[candev] = 'm';
		IRQ[candev] = pdev->irq;
		candev++;
	    } else {
		/* printk(" CAN: NO at pos 2\n"); */
		;
	    }
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
    } else {
        printk("CAN: No PCI bios present\n");
        return -ENODEV;
            }
#endif

    return 0;
}
# endif 	/* defined(CPC_PCI) */



#endif


/* Kvaser PCICan-4HS specific stuff
 * 
 * (c) 2006 oe@port.de
 */

#include <linux/pci.h>
#include "defs.h"

# ifndef CONFIG_PCI
#   error "trying to compile a PCI driver for a kernel without CONFIG_PCI"
# endif


/* used for storing the global pci register address */
/* one element more than needed for marking the end */
struct	pci_dev *Can_pcidev[MAX_CHANNELS + 1] = { 0 };


/* PCI Bridge AMCC 5920 registers */
#define S5920_OMB    0x0C
#define S5920_IMB    0x1C
#define S5920_MBEF   0x34
#define S5920_INTCSR 0x38
#define S5920_RCR    0x3C
#define S5920_PTCR   0x60

#define INTCSR_ADDON_INTENABLE_M        0x2000
#define INTCSR_INTERRUPT_ASSERTED_M     0x800000

inline void disable_pci_interrupt(unsigned int base)
{
unsigned long tmp;
    /* Disable PCI interrupts from card */
    tmp = inl(base + S5920_INTCSR);
    tmp &= ~INTCSR_ADDON_INTENABLE_M;
    outl(tmp, base + S5920_INTCSR);
}





/* reset all CAN controllers on the Kvaser-PCI Board */
void reset_KVASER_PCI(unsigned long address)
{
    DBGin("reset_KVASER_PCI");
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
    DBGin("controller_available");
    printk("controller_available 0x%lx, off %d\n", address, offset);

    printk("0x%0x, ", inb(address ) );
    printk("0x%0x, ", inb(address + (2 * offset)) );
    printk("0x%0x\n", inb(address + (3 * offset)) );

#if 0 
    /* the following does not work, unfortunately, therefor we have to use
     * inb()/outb()
     */
    ioport_map(address, 32 * offset);
    printk("0x%0x, ", ioread8(ptr ) );
    printk("0x%0x, ", ioread8(ptr + (2 * offset)) );
    printk("0x%0x\n", ioread8(ptr + (3 * offset)) );
    ioport_unmap(ptr);
#endif

    if ( 0x21 == inb(address))  {
	/* compare rest values of status and interrupt register */
	if(   0x0c == inb(address + 2)
	   && 0xe0 == inb(address + 3) ) {
	    return 1;
	} else {
	    return 0;
	}
    } else {
	/* may be called after a 'soft reset' in 'PeliCAN' mode */
	/*   value     address                     mask    */
	if(   0x00 ==  inb((address + 1))
	   && 0x34 == (inb((address + 2))    & 0x37)
	   && 0x00 == (inb((address + 3))    & 0xfb)
	  ) {
	    return 1;
        } else {
	    return 0;
        }

    }
}


int pcimod_scan(void)
{
struct	pci_dev *pdev = NULL;
int	candev = 0;			/* number of devices found so far */
int	nextcandev;
unsigned long tmp;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
    /* Testing the PCI presence is no longer neccessary
     * On a non  PCI kernel, insmod will just complain
     */
    if (pci_present ()) {
#endif

	    while((pdev =
	    	pci_find_device (PCI_VENDOR_CAN_KVASER, PCI_DEVICE_CAN_KVASER,
		    pdev))) {

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	    printk("  found KVASER-PCICAN: %s\n", pdev->name);
#else
	    printk("  found KVASER-PCICAN: %s\n", pci_pretty_name(pdev));
	    printk("                     : %s\n", pci_name(pdev));
#endif
	    if (pci_enable_device(pdev)) {
		continue;
	    }
	    printk("        using IRQ %d\n", pdev->irq);



	/* this is the pci register range S5920 */
	if ((pci_resource_flags(pdev, 0)) & IORESOURCE_IO) {
		/* printk("resource 0 IO %ld\n",  */
			/* pci_resource_len(pdev, 0) ); */
	    if(pci_request_region(pdev, 0, "kv_can_s5920") != 0)
		return -ENODEV;

	} else if((pci_resource_flags(pdev, 0)) & IORESOURCE_MEM) {
		printk("resource 0 MEM");
	}

/* printk("got PCI region\n"); */

	/* this is the CAN  I/O register range */
	if ((pci_resource_flags(pdev, 1)) & IORESOURCE_IO) {
		/* printk("resource 1 IO %ld\n",  */
			/* pci_resource_len(pdev, 1) ); */
	    if(pci_request_region(pdev, 1, "kv_can_sja1000") != 0)
		goto error_io;

	} else if((pci_resource_flags(pdev, 1)) & IORESOURCE_MEM) {
		printk("resource 1 MEM");
	}

/* printk("got CAN region\n"); */
	/* this is the Xilinx register range */
	if ((pci_resource_flags(pdev, 2)) & IORESOURCE_IO) {
		/* printk("resource 2 IO %ld\n",  */
			/* pci_resource_len(pdev, 2) ); */
	    if(pci_request_region(pdev, 2, "kv_can_xilinx") != 0)
		goto error_xilinx;

	} else if((pci_resource_flags(pdev, 2)) & IORESOURCE_MEM) {
		printk("resource 2 MEM");
	}

/* printk("got XILINX region\n"); */
	/* Assert PTADR#
	 * - we're in passive mode so the other bits are not important	*/
	outl(0x80808080L, pci_resource_start(pdev, 0) + S5920_PTCR);


	/* Read version info from xilinx chip */
	printk(KERN_INFO "Version %d\n",
		(inb(pci_resource_start(pdev, 2) + 7) >> 4));


	/* Loop through the io area 1 to see how many CAN controllers	*/
	/* are on board (1, 2 or 4)					*/
	/* be prepared that this happens for each board			*/


	nextcandev = candev + 4;   /* the PCICan has max four Controllers */
	for(; candev < nextcandev; candev++) {
	unsigned long io;
	    Can_pcidev[candev] = pdev;
	    io = pci_resource_start(pdev, 1) + (candev * 0x20);
	    /* if(controller_available(io, 1)) { */
	    if(1) {
		printk(" CAN: at pos %d\n", candev + 1);
		if(candev > MAX_CHANNELS) {
		    printk("CAN: only %d devices supported\n", MAX_CHANNELS);
		    break; /* the devices scan loop */
		}
		Base[candev] = io;
		IRQ[candev] = pdev->irq;
		IOModel[candev] = 'p';

		/* can_dump(candev); */
	    }
	}

#if 0
    {
	int i;
	for(i = 0; i < 4; i++) {
	    printk("0x%0x: ", Base[i]);
	    printk("0x%0x, ", inb(Base[i]  ));
	    printk("0x%0x, ", inb(Base[i] + 2));
	    printk("0x%0x\n", inb(Base[i] + 3));
	}
    }
#endif
	/* Disable interrupts from card */
	tmp = inl(pci_resource_start(pdev, 0) + S5920_INTCSR);
	tmp &= ~INTCSR_ADDON_INTENABLE_M;
	outl(tmp, pci_resource_start(pdev, 0) + S5920_INTCSR);
	/* Enable interrupts from card */
	tmp = inl(pci_resource_start(pdev, 0) + S5920_INTCSR);
	tmp |= INTCSR_ADDON_INTENABLE_M;
	outl(tmp, pci_resource_start(pdev, 0) + S5920_INTCSR);



	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
    } else {
        printk("CAN: No PCI bios present\n");
        return -ENODEV;
            }
#endif	
    return 0;

    error_xilinx:
	pci_release_region(pdev, 1);   /*release i/o */
    error_io:
	pci_release_region(pdev, 0);   /*release pci */
    return -ENODEV;
}




int CAN_VendorInit (int minor)
{
unsigned long tmp;

    DBGin("Kvaser CAN_VendorInit");
/* 1. Vendor specific part ------------------------------------------------ */
    can_range[minor] = CAN_RANGE;
    
/* End: 1. Vendor specific part ------------------------------------------- */


    /* Request the controllers address space */
    /* Nothing todo for the Kvaser PCICAN, we have io-addresses */
    can_base[minor] = (unsigned char *)Base[minor];

/* 2. Vendor specific part ------------------------------------------------ */


/* End: 2. Vendor specific part ------------------------------------------- */

    if( IRQ[minor] > 0 || IRQ[minor] > MAX_IRQNUMBER ){
        int err;
	/* printk("Request IRQ: %d , minor %d\n", IRQ[minor], minor); */
	err = request_irq( IRQ[minor], CAN_Interrupt, SA_SHIRQ, 
				"Can", &Can_minors[minor]);
        if( !err ){
	    DBGprint(DBG_BRANCH,("Requested IRQ: %d @ 0x%lx",
				    IRQ[minor], (unsigned long)CAN_Interrupt));
	    IRQ_requested[minor] = 1;
	} else {
	    DBGout(); return -EBUSY;
	}
    } else {
	/* Invalid IRQ number in /proc/.../IRQ */
	DBGout(); return -EBUSY;
    }

    /* Enable interrupts from card */
    /* printk(" enable PCI interrupt\n"); */
    tmp = inl(pci_resource_start(Can_pcidev[minor], 0) + S5920_INTCSR);
    tmp |= INTCSR_ADDON_INTENABLE_M;
    outl(tmp, pci_resource_start(Can_pcidev[minor], 0) + S5920_INTCSR);

    DBGout(); return 0;
}






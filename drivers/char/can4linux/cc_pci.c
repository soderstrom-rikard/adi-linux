# if defined( CC_CANPCI)
/* special handling of pci stuff of the Contemporary Controls CANPCI */
/* ================================================================= */
static char *pcican_devnames[] = {
	"PCICAN/CANopen",
	"PCICAN/DeviceNet"
};

int pcimod_scan(void)
{
struct	pci_dev *pdev = NULL;
int	candev = 0;				/* number of found devices */
unsigned long ptr;				/* ptr to PITA control */

int i;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
    /* Testing the PCI presence is no longer neccessary
     * On a non  PCI kernel, insmod will just complain
     */
    if (pci_present ()) {
#endif
	    while((pdev =
	    	pci_find_device (PCI_VENDOR_CAN_CC,
	    	                 PCI_ANY_ID, pdev))) {


	    printk("  found CC CANPCI: %x\n", pdev->vendor);
	    printk("  found CC CANPCI: %x\n", pdev->device);
	    /* we are only interseted in these types of boards */
	    if( !
	    	(pdev->device == PCI_DEVICE_CC_CANopen)
	    	||
	    	(pdev->device == PCI_DEVICE_CC_CANDnet)
	    	) continue;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	    printk("  found CC CANPCI: %s\n", pdev->name);
#else
	    printk("  found CC CANPCI: %s\n", pci_pretty_name(pdev));
	    printk("                 : %s\n", pci_name(pdev));
	    printk("                 : %s at 0x%lx\n",
		pcican_devnames[(pdev->device) - PCI_DEVICE_CC_MASK],
		pci_resource_start(pdev, 1));
#endif

	    if (pci_enable_device(pdev)) {
		printk(" pci_enable_device not succeded\n");
		continue;
	    }
	    printk("        using IRQ %d\n", pdev->irq);
/* kann weg */
	    for( i = 0; i < DEVICE_COUNT_RESOURCE; i++ ) { 
	    printk("   %d: 0x%lx\n", i, pci_resource_start(pdev, i));
	    }

	    printk(" using I/O at: 0x%lx\n", pci_resource_start(pdev, 1));
/* 0 == pci_request_regions(pdev, "CAN-IO") */
/* sonst BUSY */
	    /* look for a CAN controller at 0x00 */
	    /* if(controller_available(pci_resource_start(pdev, 1))) { */
	    if(1) {
		printk(" CAN: %d. at pos 1\n", candev + 1);
		if(candev > MAX_CHANNELS) {
		    printk("CAN: only %d devices supported\n", MAX_CHANNELS);
		    break; /* the devices scan loop */
		}

#if 0 
		if (!request_region(pci_resource_start(pdev, 1), 0x20, "CAN-IO")) {
		printk("Can: Failed to reserve IO region 0x%lx\n", pci_resource_start(pdev, 1) );
		    pci_disable_device(pdev);
		    return -EIO;
		}

		Base[candev] = pci_resource_start(pdev, 1);

		printk(" first byte %u \n", inb( Base[candev]));



		can_base[candev] =ioremap(pci_resource_start(pdev, 1), 0x20);
		if (!can_base[candev]) {
		    printk(KERN_INFO
			    "can.o: Failed to ioremap PCI memory space\n");
		    pci_disable_device(pdev);
		    release_region(Base[candev], can_range[candev] );
		    return -EIO;
		    }
#endif
		Base[candev] = pci_resource_start(pdev, 1);
		IOModel[candev] = 'p';
		IRQ[candev] = pdev->irq;
		candev++;
		    } else {
		/* printk(" CAN: NO at pos 1\n"); */
		;
		    }

#if 0
	    ptr = (unsigned long)ioremap(PCI_BASE_ADDRESS0(pdev), 256);
			/* enable memory access */
		    	/* printk("write to pita\n"); */
			writel(0x04000000, ptr + 0x1c);
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
		unsigned long sigptr; /* ptr to EMS signature  */
		unsigned long signature = 0;
	        sigptr = (unsigned long)ioremap(PCI_BASE_ADDRESS1(pdev), 256);
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
    			writel(0x00020000, Can_pitapci_control[candev] + 0x0);

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
#endif

		}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
    } else {
        printk("CAN: No PCI bios present\n");
        return -ENODEV;
    }
#endif

    return 0;
}
# endif 	/* defined( CC_CANPCI) */

/*
 * File:         drivers/char/bfin_fbdma.c
 * Based on:
 * Author:	Michael Hennerich, Analog Devices Inc.
 *
 * Created:
 * Description:
 *
 * Rev:          $Id: bfin_fbdma.c 2694 2007-01-26 15:59:03Z hennerich $
 *
 * Modified:
 *               Copyright 2004-2007 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/rtc.h>

#include <asm/blackfin.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/delay.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>


/************************************************************/

/* definitions */

#define FBDMA_MINOR         253

#define FBDMA_DEVNAME       "FBDMAFCP"
#define FBDMA_INTNAME       "FBDMA-FCP-INT"	/* Should be less than 19 chars. */

#define BITBLIT		0x1
#define FILL		0x2
#define BLOCKING 	0x40

#define DO_DMA		3
#define GET_SEMAPHORE	4

#define DRIVER_MAGIC	0xDEADBEEF

/************************************************************/

typedef struct FBDMA_Device_t {
	int opened;
	int nonblock;
	unsigned short irqnum;
	unsigned short done;
	wait_queue_head_t *avail;
	unsigned short* l1_data_dummy;
	unsigned short* user_semaphore;
} fbdma_device_t;

/************************************************************/

/* Globals */

static DECLARE_WAIT_QUEUE_HEAD(fbdma_q0);
static fbdma_device_t fbdmainfo;
static DEFINE_SPINLOCK(fbdmafcd_lock);

typedef struct memoper {
		struct dmasg srcdesc;
		struct dmasg dstdesc;
		unsigned int type;
		unsigned int color;
		unsigned int magic;
} MEMOPER;

//--------------------------------------------------------------------------//
// Function:	Enable_MemDMA1				     							//
//--------------------------------------------------------------------------//
void Enable_MemDMA1(struct dmasg *us_MemDMA1_Src_Descriptor, struct dmasg *us_MemDMA1_Dst_Descriptor )
{
    	bfin_write_MDMA_D1_START_ADDR(us_MemDMA1_Dst_Descriptor->start_addr);
	bfin_write_MDMA_D1_X_COUNT(us_MemDMA1_Dst_Descriptor->x_count);
	bfin_write_MDMA_D1_X_MODIFY(us_MemDMA1_Dst_Descriptor->x_modify);
	bfin_write_MDMA_D1_Y_COUNT(us_MemDMA1_Dst_Descriptor->y_count);
	bfin_write_MDMA_D1_Y_MODIFY(us_MemDMA1_Dst_Descriptor->y_modify);

	bfin_write_MDMA_D1_IRQ_STATUS(0x3);
	bfin_write_MDMA_S1_START_ADDR(us_MemDMA1_Src_Descriptor->start_addr);
	bfin_write_MDMA_S1_X_COUNT(us_MemDMA1_Src_Descriptor->x_count);
	bfin_write_MDMA_S1_X_MODIFY(us_MemDMA1_Src_Descriptor->x_modify);
	bfin_write_MDMA_S1_Y_COUNT(us_MemDMA1_Src_Descriptor->y_count);
	bfin_write_MDMA_S1_Y_MODIFY(us_MemDMA1_Src_Descriptor->y_modify);

	bfin_write_MDMA_S1_IRQ_STATUS(0x3);

	bfin_write_MDMA_S1_CONFIG(us_MemDMA1_Src_Descriptor->cfg);
	bfin_write_MDMA_D1_CONFIG(us_MemDMA1_Dst_Descriptor->cfg);
}


/*
 * FUNCTION NAME: fbdma_irq
 *
 * INPUTS/OUTPUTS:
 * in_irq - Interrupt vector number.
 * in_dev_id  - point to device information structure base address.
 *
 * VALUE RETURNED:
 * void
 *
 * FUNCTION(S) CALLED:
 *
 * GLOBAL VARIABLES REFERENCED: fbdmainfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: ISR of FBDMA
 *
 * CAUTION:
 */
static irqreturn_t fbdma_irq(int irq, void *dev_id)
{
	fbdma_device_t *pdev = (fbdma_device_t *) dev_id;

	pr_debug("fbdma_irq:\n");

	bfin_write_MDMA_D1_IRQ_STATUS(0x3);
	bfin_write_MDMA_S1_IRQ_STATUS(0x3);

	/* wake up */

	pdev->done = 1;
	*pdev->user_semaphore = 0;


	wake_up_interruptible(pdev->avail);

	pr_debug("fbdma_irq: return\n");

	return IRQ_HANDLED;
}

static int fbdma_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{

	fbdma_device_t *pdev = file->private_data;
	int ierr;

	MEMOPER *ptr = (MEMOPER *) arg;

	switch (cmd) {
	case DO_DMA:
	{

	if(!ptr)
		return -EINVAL;

	if(ptr->magic != DRIVER_MAGIC){
		printk("DRIVER_MAGIC = %x\n",ptr->magic);
		return -EINVAL;
	}

	if(ptr->type & FILL){
		*pdev->l1_data_dummy = ptr->color;
		ptr->srcdesc.start_addr = (unsigned long)pdev->l1_data_dummy;

#if defined(CONFIG_BFIN_DCACHE)
	if(ptr->dstdesc.start_addr < memory_end)
		blackfin_dcache_invalidate_range(ptr->dstdesc.start_addr,
	ptr->dstdesc.start_addr+(ptr->dstdesc.x_count * ptr->dstdesc.x_modify * ptr->dstdesc.y_count));
#endif

	} else {

#if defined(CONFIG_BFIN_DCACHE) && defined(CONFIG_BFIN_WB)
	blackfin_dcache_flush_range(ptr->srcdesc.start_addr,
	ptr->srcdesc.start_addr+(ptr->srcdesc.x_count * ptr->srcdesc.x_modify * ptr->srcdesc.y_count));
#endif
	}

	pdev->done = 0;
	*pdev->user_semaphore = 1;

	Enable_MemDMA1(&ptr->srcdesc, &ptr->dstdesc);


	/* Wait for data available */
	if (ptr->type & BLOCKING) {
		if (pdev->nonblock)
			return -EAGAIN;
		else {
			pr_debug("FBDMA wait_event_interruptible\n");
			ierr =
			    wait_event_interruptible(*(pdev->avail),
						     pdev->done);
			if (ierr) {
				/* waiting is broken by a signal */
				pr_debug("FBDMA wait_event_interruptible ierr\n");
				return ierr;
			}
		}
	}

			break;
	}
	case GET_SEMAPHORE:
		{

	void __user *argp = (void __user *)arg;

	unsigned long address = (unsigned long) pdev->user_semaphore;

	return copy_to_user(argp, &address, sizeof(address)) ? -EFAULT : 0;


			break;
		}

	default:
		return -EINVAL;
	}


	return 0;
}



/* We use fbdma_lock to protect against concurrent opens.*/
static int fbdma_open(struct inode *inode, struct file *file)
{

	char intname[20];
	unsigned long flags;
	int minor = MINOR(inode->i_rdev);

	pr_debug("fbdma_open:\n");

	/* FBDMA ? */
	if (minor != FBDMA_MINOR)
		return -ENXIO;

	spin_lock_irqsave(&fbdmafcd_lock, flags);

	if (fbdmainfo.opened) {
		spin_unlock_irqrestore(&fbdmafcd_lock, flags);
		return -EMFILE;
	}

	/* Clear configuration information */
	memset(&fbdmainfo, 0, sizeof(fbdma_device_t));

	if (file->f_flags & O_NONBLOCK)
		fbdmainfo.nonblock = 1;

	fbdmainfo.opened = 1;
	fbdmainfo.done = 0;

	fbdmainfo.avail = &fbdma_q0;

	strcpy(intname, FBDMA_INTNAME);


	file->private_data = &fbdmainfo;


	/* Request DMA channel, and pass the interrupt handler */

	if (request_dma(CH_MEM_STREAM1_SRC, "MEMDMA SRC") < 0) {
		panic("Unable to attach BlackFin MEMDMA DMA channel\n");
		fbdmainfo.opened = 0;
		spin_unlock_irqrestore(&fbdmafcd_lock, flags);
		return -EFAULT;
	}

	if (request_dma(CH_MEM_STREAM1_DEST, "MEMDMA DEST") < 0) {
		panic("Unable to attach BlackFin FBDMA DMA channel\n");
		fbdmainfo.opened = 0;
		free_dma(CH_MEM_STREAM1_SRC);
		spin_unlock_irqrestore(&fbdmafcd_lock, flags);
		return -EFAULT;
	} else
		set_dma_callback(CH_MEM_STREAM1_DEST, (void *)fbdma_irq,
				 file->private_data);

	spin_unlock_irqrestore(&fbdmafcd_lock, flags);


	fbdmainfo.l1_data_dummy = l1_data_A_sram_alloc(sizeof(unsigned short));
	fbdmainfo.user_semaphore = l1_data_A_sram_alloc(sizeof(unsigned short));


	pr_debug("fbdma_open: return\n");



	return 0;
}

static int fbdma_release(struct inode *inode, struct file *file)
{
	unsigned long flags;
	fbdma_device_t *pdev = file->private_data;

	pr_debug("fbdma_release: close()\n");

	spin_lock_irqsave(&fbdmafcd_lock, flags);
	/* After finish DMA, release it. */
	free_dma(CH_MEM_STREAM1_DEST);
	free_dma(CH_MEM_STREAM1_SRC);

	if (pdev->l1_data_dummy)
		l1_data_A_sram_free(pdev->l1_data_dummy);

	if (pdev->user_semaphore)
		l1_data_A_sram_free(pdev->user_semaphore);

	pdev->opened = 0;
	spin_unlock_irqrestore(&fbdmafcd_lock, flags);


	pr_debug("fbdma_release: close() return\n");
	return 0;
}

/*
 *  The various file operations we support.
 */
struct file_operations fbdma_fops = {
	owner:      THIS_MODULE,
	ioctl:      fbdma_ioctl,
	open:       fbdma_open,
	release:    fbdma_release,
};

static struct miscdevice fbdma_dev = {
	FBDMA_MINOR,
	"fbdma",
	&fbdma_fops
};

/* Init function called first time */
int __init fbdma_init(void)
{

	pr_debug("blackfin_fbdma_init\n");

	misc_register(&fbdma_dev);

	printk(KERN_INFO "FBDMA: major=%d, minor = %d\n", MISC_MAJOR, FBDMA_MINOR);
	return 0;
}

void __exit fbdma_exit (void)
{

	misc_deregister(&fbdma_dev);
}

module_init(fbdma_init);
module_exit(fbdma_exit);

MODULE_AUTHOR("Michael Hennerich");
MODULE_LICENSE("GPL");

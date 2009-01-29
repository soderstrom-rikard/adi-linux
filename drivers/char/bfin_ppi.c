/*
 * File:	 drivers/char/bfin_ppi.c
 * Based on:
 * Author:	 John DeHority <john.dehority@NOSPAM@kodak.com>
 *
 * Created:	 May 5,	2005
 * Description:	 Blackfin PPI0/1 Driver
 *
 * Modified:
 *		 Copyright (C) 2005, Eastman Kodak Company
 *		 Copyright 2005-2009 Analog Devices Inc.
 *
 * Bugs:	 Enter bugs at http://blackfin.uclinux.org/
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/blackfin.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>
#include <asm/portmux.h>

#include <asm/gptimers.h>

#include "bfin_ppi.h"

#undef	DEBUG
#define	PPI_DEVNAME	  "PPIdev"
#define	PPI_MAJOR	252

const unsigned short per_req_ppi0_7[] = {P_PPI0_CLK, P_PPI0_FS1, P_PPI0_D0, P_PPI0_D1,\
	 P_PPI0_D2, P_PPI0_D3, P_PPI0_D4, P_PPI0_D5, P_PPI0_D6, P_PPI0_D7, 0};

const unsigned short per_req_ppi8_15[] = {P_PPI0_D15, P_PPI0_D14, P_PPI0_D13, P_PPI0_D12,\
			 P_PPI0_D11, P_PPI0_D10, P_PPI0_D9, P_PPI0_D8, 0};

const unsigned short per_req_ppi_fs[] = {P_PPI0_FS2, /*P_PPI0_FS3,*/ 0};

#ifdef PPI1_CONTROL
const unsigned short per_req_ppi1_0_7[] = {P_PPI1_CLK, P_PPI1_FS1, P_PPI1_D0, P_PPI1_D1,\
	 P_PPI1_D2, P_PPI1_D3, P_PPI1_D4, P_PPI1_D5, P_PPI1_D6, P_PPI1_D7, 0};

const unsigned short per_req_ppi1_8_15[] = {P_PPI1_D15, P_PPI1_D14, P_PPI1_D13, P_PPI1_D12,\
			 P_PPI1_D11, P_PPI1_D10, P_PPI1_D9, P_PPI1_D8, 0};

const unsigned short per_req_ppi1_fs[] = {P_PPI1_FS2, /*P_PPI1_FS3,*/ 0};

static int ppi_nr_devs = 2;
#else
static int ppi_nr_devs = 1;
#endif

static int ppi_major = PPI_MAJOR;
module_param(ppi_major,	int, 0);
MODULE_PARM_DESC(ppi_major, "Major device number");

static int ppi_minor;

struct ppi_register {
	unsigned short ppi_control;
	unsigned short dummy0;
	unsigned short ppi_status;
	unsigned short dummy1;
	unsigned short ppi_count;
	unsigned short dummy2;
	unsigned short ppi_delay;
	unsigned short dummy3;
	unsigned short ppi_frame;
};

struct ppi_config {
	unsigned char opened;
	unsigned char datalen;
	unsigned char fs23;
	unsigned char timers;
	unsigned char triggeredge;
	unsigned char dimensions;
	unsigned short delay;
	unsigned short access_mode;
	unsigned short done;
	unsigned short dma_config;
	unsigned short linelen;
	unsigned short numlines;
	unsigned short ppi_control;
};

struct ppi_dev {
	struct cdev cdev;	/* Char	device structure */
	struct fasync_struct *fasyc;
	struct ppi_config conf;
	volatile struct	ppi_register *regs;
	wait_queue_head_t waitq;
	spinlock_t lock;
	int ppi_num;
	int dma_chan;
	int irq;
	int irq_error;
	int fs1_timer_id;
	int fs2_timer_id;
	unsigned short fs1_timer_bit;
	unsigned short fs2_timer_bit;
	const unsigned short *per_ppi0_7;
	const unsigned short *per_ppi8_15;
	const unsigned short *per_ppifs;
};

struct ppi_dev *ppi_devices;	/* allocated in	ppi_init_module	*/

static inline unsigned short clear_ppi_status(struct ppi_dev *dev)
{
	unsigned short stat = dev->regs->ppi_status;
	dev->regs->ppi_status =	-1;
	return stat;
}

static inline void disable_ppi_dma(struct ppi_dev *dev)
{
	if (ANOMALY_05000278) {
		/* disable DMA */
		disable_dma(dev->dma_chan);
		/* disable ppi */
		dev->regs->ppi_control &= ~PORT_EN;
	} else {
		/* disable ppi */
		dev->regs->ppi_control &= ~PORT_EN;
		/* disable DMA */
		disable_dma(dev->dma_chan);
	}
}

static void setup_timers(struct	ppi_dev	*dev, unsigned int frameSize)
{
	struct ppi_config *conf	= &dev->conf;
	short fs1_timer_cfg = 0;
	short fs2_timer_cfg = 0;
	unsigned short t_mask;
	unsigned int linePeriod;
	/*
	 ** set	timer configuration register template
	 **
	 ** see	note on	page 11-29 of BF533 HW Reference Manual
	 ** for	setting	PULSE_HI according to PPI trigger edge configuration
	 ** of PPI_FS1 and PPI_FS2
	 **
	 ** set	TOGGLE_HI so line and frame are	not asserted simultaneously
	 */
	fs1_timer_cfg =	(TIMER_CLK_SEL | TIMER_TIN_SEL |
			 TIMER_MODE_PWM	| TIMER_TOGGLE_HI);

	if (conf->triggeredge)
		fs1_timer_cfg &= ~TIMER_PULSE_HI;
	else
		fs1_timer_cfg |= TIMER_PULSE_HI;

	fs2_timer_cfg =	fs1_timer_cfg;
	fs1_timer_cfg |= TIMER_PERIOD_CNT;	/* set up line sync to be recurring */

	if (conf->dimensions ==	CFG_PPI_DIMS_2D) {	/* configure for 2D transfers */

		linePeriod = conf->linelen + conf->delay;
		frameSize = linePeriod * conf->numlines	* 2;	/* TOGGLE_HI effects */

		/*
		 ** configure 2	timers for 2D
		 ** Timer1 - hsync - line time	PPI_FS1	(Timer0	on BF537)
		 ** Timer2 - vsync - frame time	PPI_FS2	(Timer1	on BF537)
		 */
		t_mask = (dev->fs1_timer_bit | dev->fs2_timer_bit);	/* use both timers */

		set_gptimer_config(dev->fs2_timer_id, fs2_timer_cfg);
		set_gptimer_period(dev->fs2_timer_id, frameSize);
		set_gptimer_pwidth(dev->fs2_timer_id, frameSize);
		pr_debug
		    ("Timer %d:	(frame/vsync) config = %04hX, period = %d, width = %d\n",
		     dev->fs2_timer_id,	get_gptimer_config(dev->fs2_timer_id),
		     get_gptimer_period(dev->fs2_timer_id),
		     get_gptimer_pwidth(dev->fs2_timer_id));

		set_gptimer_config(dev->fs1_timer_id, fs1_timer_cfg);
		set_gptimer_period(dev->fs1_timer_id, linePeriod);
		/* divide linelen by 4 due to TOGGLE_HI	behavior */
		set_gptimer_pwidth(dev->fs1_timer_id, (conf->linelen >>	2));
		pr_debug
		    ("Timer %d:	(line/hsync) config = %04hX, period = %d, width	= %d\n",
		     dev->fs1_timer_id,	get_gptimer_config(dev->fs1_timer_id),
		     get_gptimer_period(dev->fs1_timer_id),
		     get_gptimer_pwidth(dev->fs1_timer_id));
	} else {
		t_mask = dev->fs1_timer_bit;

		/*
		 ** set	timer for frame	vsync
		 ** use fs2_timer_cfg,  'cuz it is the non-recurring config
		 */
		set_gptimer_config(dev->fs1_timer_id, fs2_timer_cfg);
		set_gptimer_period(dev->fs1_timer_id, frameSize	+ 1);
		set_gptimer_pwidth(dev->fs1_timer_id, frameSize);

		pr_debug("Timer	%d: config = %04hX, period = %d, width = %d\n",
			dev->fs1_timer_id, fs2_timer_cfg,
			get_gptimer_period(dev->fs1_timer_id),
			get_gptimer_pwidth(dev->fs1_timer_id));

	}
	pr_debug("enable_gptimers(mask=%d)\n", t_mask);
	enable_gptimers(t_mask);

}
static void disable_timer_output(struct	ppi_dev	*dev)
{
	unsigned short regdata;

	if (dev->conf.dimensions == CFG_PPI_DIMS_2D) {
		/* disable timer1 for FS2 */
		regdata	= get_gptimer_config(dev->fs2_timer_id),
		    regdata &= ~TIMER_OUT_DIS;
		set_gptimer_config(dev->fs2_timer_id, regdata);
	}

	/* disable timer0 outputs for FS1 */
	regdata	= get_gptimer_config(dev->fs1_timer_id),
	regdata	&= ~TIMER_OUT_DIS;
	set_gptimer_config(dev->fs1_timer_id, regdata);
}

/*
 * FUNCTION NAME: ppi_irq
 *
 * INPUTS/OUTPUTS:
 * in_irq - Interrupt vector number.
 * in_dev_id  - point to device information structure base address.
 * in_regs - unuse here.
 *
 * VALUE RETURNED:
 * void
 *
 * FUNCTION(S) CALLED:
 *
 * GLOBAL VARIABLES REFERENCED: ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: ISR of PPI
 *
 * CAUTION:
 */
static void ppi_reg_reset(struct ppi_dev *dev)
{
	dev->regs->ppi_control = 0;
	dev->regs->ppi_count = 0;
	dev->regs->ppi_frame = 0;
	dev->regs->ppi_delay = 0;
	clear_ppi_status(dev);
}

/*
 * FUNCTION NAME: ppi_irq
 *
 * INPUTS/OUTPUTS:
 * in_irq - Interrupt vector number.
 * in_dev_id  -	point to device	information structure base address.
 * in_regs - unuse here.
 *
 * VALUE RETURNED:
 * void
 *
 * FUNCTION(S) CALLED:
 *
 * GLOBAL VARIABLES REFERENCED:	ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION:	ISR of PPI
 *
 * CAUTION:
 */
static irqreturn_t ppi_irq(int irq, void *dev_id)
{
	struct ppi_dev *dev = (struct ppi_dev *) dev_id;
	unsigned long flags;

	if (dev->conf.timers &&	dev->conf.access_mode == PPI_WRITE)
		disable_gptimers((dev->fs1_timer_bit | dev->fs2_timer_bit));


	/* Give	a signal to user program. */
	if (dev->fasyc)
		kill_fasync(&(dev->fasyc), SIGIO, POLLIN);

	disable_ppi_dma(dev);

	clear_ppi_status(dev);
	clear_dma_irqstat(dev->dma_chan);

	spin_lock_irqsave(&dev->lock, flags);
	dev->conf.done = 1;
	spin_unlock_irqrestore(&dev->lock, flags);

	/* wake	up read/write block. */
	wake_up_interruptible(&dev->waitq);

	pr_debug("ppi_irq: return\n");

	return IRQ_HANDLED;
}

/*
 * FUNCTION NAME: ppi_irq_error
 *
 * INPUTS/OUTPUTS:
 * in_irq - Interrupt vector number.
 * in_dev_id  - point to device information structure base address.
 * in_regs - unuse here.
 *
 * VALUE RETURNED:
 * void
 *
 * FUNCTION(S) CALLED:
 *
 * GLOBAL VARIABLES REFERENCED: ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: Error ISR of PPI
 *
 * CAUTION:
 */
static irqreturn_t ppi_irq_error(int irq, void *dev_id)
{
	struct ppi_dev *dev = (struct ppi_dev *) dev_id;
	unsigned long flags;

	printk(KERN_ERR	"PPI Error: PPI	Status = 0x%X \n",
	       clear_ppi_status(dev));

	clear_dma_irqstat(dev->dma_chan);

	if (dev->conf.timers &&	dev->conf.access_mode == PPI_WRITE)
		disable_gptimers((dev->fs1_timer_bit | dev->fs2_timer_bit));

	disable_ppi_dma(dev);

	spin_lock_irqsave(&dev->lock, flags);
	/* Add some more Error Handling	Code Here */
	dev->conf.done = 1;
	spin_unlock_irqrestore(&dev->lock, flags);

	/* wake	up read/write block. */
	wake_up_interruptible(&dev->waitq);

	return IRQ_HANDLED;
}

/*
 * FUNCTION NAME: ppi_ioctl
 *
 * INPUTS/OUTPUTS:
 * in_inode - Description of openned file.
 * in_filp - Description of openned file.
 * in_cmd - Command passed into ioctl system call.
 * in/out_arg - It is parameters which is specified by last command
 *
 * RETURN:
 * 0 OK
 * -EINVAL  Invalid baudrate
 *
 * FUNCTION(S) CALLED:
 *
 * GLOBAL VARIABLES REFERENCED: ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION:
 *
 * CAUTION:
 */
static int ppi_ioctl(struct inode *inode, struct file *filp, uint cmd, unsigned long arg)
{
	unsigned short regdata;
	unsigned long flags;
	struct ppi_dev *dev = filp->private_data;
	struct ppi_config *conf	= &dev->conf;

	spin_lock_irqsave(&dev->lock, flags);
	switch (cmd) {
	case CMD_PPI_PORT_ENABLE:
		{
			pr_debug("ppi_ioctl: CMD_PPI_PORT_ENABLE\n");
			regdata	= dev->regs->ppi_control;
			if (arg)
				regdata	|= PORT_EN;
			else
				regdata	&= ~PORT_EN;
			conf->ppi_control = regdata;
			dev->regs->ppi_control = regdata;
			break;
		}
	case CMD_PPI_PORT_DIRECTION:
		{
			pr_debug("ppi_ioctl: CMD_PPI_PORT_DIRECTION\n");
			regdata	= dev->regs->ppi_control;
			if (arg)
				regdata	|= PORT_DIR;
			else
				regdata	&= ~PORT_DIR;
			conf->ppi_control = regdata;
			dev->regs->ppi_control = regdata;
			break;
		}
	case CMD_PPI_XFR_TYPE:
		{
			pr_debug("ppi_ioctl: CMD_PPI_XFR_TYPE\n");
			if (arg	< 0 || arg > 3)
				goto err_inval;
			regdata	= dev->regs->ppi_control;
			regdata	&= ~XFR_TYPE;
			regdata	|= ((unsigned short)arg	<< 2);
			conf->ppi_control = regdata;
			dev->regs->ppi_control = regdata;
			break;
		}
	case CMD_PPI_PORT_CFG:
		{
			pr_debug("ppi_ioctl: CMD_PPI_PORT_CFG\n");
			if (arg	< 0 || arg > 3)
				goto err_inval;
			regdata	= dev->regs->ppi_control;
			regdata	&= ~PORT_CFG;
			regdata	|= ((unsigned short)arg	<< 4);
			conf->ppi_control = regdata;
			dev->regs->ppi_control = regdata;

			if (arg	== CFG_PPI_PORT_CFG_SYNC23 ||
				 arg ==	CFG_PPI_PORT_CFG_XSYNC23) {
				conf->fs23 = 1;
				if (peripheral_request_list(dev->per_ppifs, PPI_DEVNAME)) {
					spin_unlock_irqrestore(&dev->lock, flags);
					printk(KERN_ERR	PPI_DEVNAME
					": Requesting Peripherals failed\n");
					return -EBUSY;
				}
				}
			break;
		}
	case CMD_PPI_FIELD_SELECT:
		{
			pr_debug("ppi_ioctl: CMD_PPI_FIELD_SELECT\n");
			regdata	= dev->regs->ppi_control;
			if (arg)
				regdata	|= FLD_SEL;
			else
				regdata	&= ~FLD_SEL;
			conf->ppi_control = regdata;
			dev->regs->ppi_control = regdata;
			break;
		}
	case CMD_PPI_PACKING:
		{
#if 0
			pr_debug("ppi_ioctl: CMD_PPI_PACKING\n");
			regdata	= dev->regs->ppi_control;
			if (arg)
				regdata	|= PACK_EN;
			else
				regdata	&= ~PACK_EN;
			conf->ppi_control = regdata;
			dev->regs->ppi_control = regdata;
#else
			printk(KERN_ERR	PPI_DEVNAME "ppi_ioctl:	CMD_PPI_PACKING\
				 Not supported\n");
#endif
			break;
		}
	case CMD_PPI_SKIPPING:
		{
			pr_debug("ppi_ioctl: CMD_PPI_SKIPPING\n");
			regdata	= dev->regs->ppi_control;
			if (arg)
				regdata	|= SKIP_EN;
			else
				regdata	&= ~SKIP_EN;
			conf->ppi_control = regdata;
			dev->regs->ppi_control = regdata;
			break;
		}
	case CMD_PPI_SKIP_ODDEVEN:
		{
			pr_debug("ppi_ioctl: CMD_PPI_SKIP_ODDEVEN\n");
			regdata	= dev->regs->ppi_control;
			if (arg)
				regdata	|= SKIP_EO;
			else
				regdata	&= ~SKIP_EO;
			conf->ppi_control = regdata;
			dev->regs->ppi_control = regdata;
			break;
		}
	case CMD_PPI_DATALEN:
		{
			pr_debug("ppi_ioctl: CMD_PPI_DATALEN\n");
			if (arg	< 0 || arg > 7)
				goto err_inval;
			conf->datalen =	(unsigned short)arg;
			regdata	= dev->regs->ppi_control;
			regdata	&= ~DLENGTH;
			regdata	|= (arg	<< 11);
			conf->ppi_control = regdata;
			dev->regs->ppi_control = regdata;

			if (peripheral_request_list(&dev->per_ppi8_15[7	- arg],
						 PPI_DEVNAME)) {
				spin_unlock_irqrestore(&dev->lock, flags);
				printk(KERN_ERR	PPI_DEVNAME
				": Requesting Peripherals failed\n");
				return -EBUSY;
			}
			break;
		}
	case CMD_PPI_CLK_EDGE:
		{
			pr_debug("ppi_ioctl: CMD_PPI_CLK_EDGE\n");
			regdata	= dev->regs->ppi_control;
			if (arg)
				regdata	|= POLC;
			else
				regdata	&= ~POLC;
			conf->ppi_control = regdata;
			dev->regs->ppi_control = regdata;
			break;
		}
	case CMD_PPI_TRIG_EDGE:
		{
			pr_debug("ppi_ioctl: CMD_PPI_TRIG_EDGE\n");
			conf->triggeredge = (unsigned short)arg;
			regdata	= dev->regs->ppi_control;
			if (arg)
				regdata	|= POLFS;
			else
				regdata	&= ~POLFS;
			conf->ppi_control = regdata;
			dev->regs->ppi_control = regdata;
			break;
		}
	case CMD_PPI_LINELEN:
		{
			pr_debug("ppi_ioctl:  CMD_PPI_LINELEN\n");
			if (arg	< 0 || arg > PPI_DMA_MAXSIZE)
				goto err_inval;
			conf->linelen =	(unsigned short)arg;
			break;
		}
	case CMD_PPI_NUMLINES:
		{
			pr_debug("ppi_ioctl:  CMD_PPI_NUMLINES\n");
			if (arg	< 0 || arg > PPI_DMA_MAXSIZE)
				goto err_inval;
			conf->numlines = (unsigned short)arg;
			break;

		}
	case CMD_PPI_SET_WRITECONTINUOUS:
		{
			printk(KERN_ERR	PPI_DEVNAME "ppi_ioctl:	 CMD_PPI_SET_WRITECONTINUOUS Not supported\n");
			break;

		}
	case CMD_PPI_SET_DIMS:
		{
			pr_debug("ppi_ioctl: CMD_PPI_SET_DIMS\n");
			conf->dimensions = (unsigned char)arg;
			break;
		}
	case CMD_PPI_DELAY:
		{
			pr_debug("ppi_ioctl: CMD_PPI_DELAY\n");
			conf->delay = (unsigned	short)arg;
			dev->regs->ppi_delay = ((unsigned short)conf->delay);
			SSYNC();
			break;
		}

#ifdef DEBUG
	case CMD_PPI_GET_ALLCONFIG:
		{
			break;
		}
#endif
	case CMD_PPI_SETGPIO:
		{
			pr_debug("ppi_ioctl: CMD_PPI_SETGPIO\n");
			break;
		}
	case CMD_PPI_GEN_FS12_TIMING_ON_WRITE:
		{
			pr_debug("ppi_ioctl: CMD_PPI_GEN_FS12_TIMING_ON_WRITE\n");
			conf->timers = (unsigned short)arg;
			break;
		}
	default:
		goto err_inval;
	}

	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;

err_inval:
	spin_unlock_irqrestore(&dev->lock, flags);
	return -EINVAL;
}

/*
 * FUNCTION NAME: ppi_fasync
 *
 * INPUTS/OUTPUTS:
 * in_fd - File descriptor of openned file.
 * in_filp - Description of openned file.
 *
 * RETURN:
 *
 * FUNCTION(S) CALLED:
 *
 * GLOBAL VARIABLES REFERENCED: ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: It is invoked when user changes status of sync
 *              it resister a hook in system. When there is
 *              data coming, user program would get a signal.
 *
 * CAUTION:
 */
static int ppi_fasync(int fd, struct file *filp, int on)
{
	struct ppi_dev *dev = filp->private_data;
	return fasync_helper(fd, filp, on, &(dev->fasyc));
}

/*
 * FUNCTION NAME: ppi_read
 *
 * INPUTS/OUTPUTS:
 * filp	- Description of openned file.
 * buf -- Pointer to buffer allocated to hold data.
 * count - how many bytes user wants to	get.
 * pos -- unused
 *
 * RETURN
 * positive number: bytes read back
 * -EINVIL When	word size is set to 16,	reading	odd bytes.
 * -EAGAIN When	reading	mode is	set to non block and there is no rx data.
 *
 * FUNCTION(S) CALLED:
 *
 * GLOBAL VARIABLES REFERENCED:	ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION:	It is invoked when user	call 'read' system call
 *		to read	from system.
 *
 * CAUTION:
 */
static ssize_t ppi_read(struct file *filp, char	*buf, size_t count, loff_t *pos)
{

	unsigned short stepSize;
	unsigned long flags;
	int ierr;
	struct ppi_dev *dev = filp->private_data;
	struct ppi_config *conf	= &dev->conf;
	unsigned int dma = dev->dma_chan;

	pr_debug("ppi_read(0x%08X, %d)\n", (int)buf, (int)count);

	if (count <= 0)
		return 0;

	spin_lock_irqsave(&dev->lock, flags);
	if (!conf->done) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return -EBUSY;
	}
	conf->done = 0;
	spin_unlock_irqrestore(&dev->lock, flags);

	conf->access_mode = PPI_READ;

	blackfin_dcache_invalidate_range((unsigned long)buf,
					 ((unsigned long)buf) +	count);

	/*
	 ** configure ppi port for DMA TIMOD RX	(receive)
	 ** Note:  the rest of PPI control register bits should	already	be set
	 ** with ioctls	before read operation
	 */

	dev->regs->ppi_control &= ~PORT_DIR;

	/*
	 ** Configure DMA Controller
	 ** WNR:  memory write
	 ** RESTART: flush DMA FIFO before beginning work unit
	 ** DI_EN: generate interrupt on completion of work unit
	 ** DMA2D: 2 dimensional buffer
	 */
	conf->dma_config |= (WNR | RESTART | DI_EN);

	if (conf->datalen > CFG_PPI_DATALEN_8) {	/* adjust transfer size	*/
		conf->dma_config |= WDSIZE_16;
		stepSize = 2;
	} else {
		conf->dma_config &= ~WDSIZE_16;
		stepSize = 1;
	}

	/*
	 ** 1D or 2D DMA
	 */
	if (conf->dimensions ==	CFG_PPI_DIMS_2D) {	/* configure for 2D transfers */
		pr_debug
		    ("PPI read -- 2D data xcount = linelen = %hd, ycount = numlines = %hd stepsize = %hd \n",
		     conf->linelen, conf->numlines, stepSize);

		set_dma_x_count(dma, conf->linelen);
		set_dma_y_count(dma, conf->numlines);
		set_dma_y_modify(dma, stepSize);
		conf->dma_config |= DMA2D;
	} else {
		if (conf->datalen > CFG_PPI_DATALEN_8)	/* adjust transfer size	*/
			set_dma_x_count(dma, count / 2);
		else
			set_dma_x_count(dma, count);

			conf->dma_config &= ~DMA2D;
		pr_debug("PPI read -- 1D data count = %d\n",
			(int)(conf->datalen ? count / 2	: count));
	}

	set_dma_config(dma, conf->dma_config);
	set_dma_start_addr(dma,	(unsigned long)buf);
	set_dma_x_modify(dma, stepSize);

	/* configure PPI registers to match DMA	registers */
	dev->regs->ppi_count = conf->linelen - 1;
	dev->regs->ppi_frame = conf->numlines;
	dev->regs->ppi_delay = conf->delay;

	if (conf->timers)
		disable_timer_output(dev);

	enable_dma(dma);

	/* enable ppi */
	dev->regs->ppi_control |= PORT_EN;
	SSYNC();

	/* Wait	for data available */
	if (1) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else {
			pr_debug("PPI wait_event_interruptible\n");
			ierr =
			    wait_event_interruptible(dev->waitq, conf->done);
			if (ierr) {
				/* waiting is broken by	a signal */
				pr_debug("PPI wait_event_interruptible ierr\n");
				disable_ppi_dma(dev);
				conf->done = 1;
				return ierr;
			}
		}
	}

	pr_debug("PPI wait_event_interruptible done\n");
	pr_debug("ppi_read: return\n");

	return count;
}

/*
 * FUNCTION NAME: ppi_write
 *
 * INPUTS/OUTPUTS:
 * in_filp - Description of openned file.
 * in_count - how many bytes user wants to send.
 * out_buf - where we get those sending data.
 *
 * RETURN
 * positive number: bytes sending out.
 * 0: There is no data send out or parameter error.
 * RETURN:
 * >0 The actual count sending out.
 * -EINVIL When word size is set to 16, writing odd bytes.
 * -EAGAIN When sending mode is set to non block and there is no tx buffer.
 *
 * FUNCTION(S) CALLED:
 *
 * GLOBAL VARIABLES REFERENCED: ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: It is invoked when user call 'read' system call
 *              to read from system.
 *
 * CAUTION:
 */
static ssize_t ppi_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	struct ppi_dev *dev = filp->private_data;
	struct ppi_config *conf	= &dev->conf;
	unsigned int dma = dev->dma_chan;
	int ierr;
	unsigned int frameSize;
	unsigned short stepSize;
	unsigned long flags;

	pr_debug("ppi_write:\n");

	if (count <= 0)
		return 0;

	spin_lock_irqsave(&dev->lock, flags);
	if (!conf->done) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return -EBUSY;
	}
	conf->done = 0;
	spin_unlock_irqrestore(&dev->lock, flags);

	conf->access_mode = PPI_WRITE;

	blackfin_dcache_invalidate_range((unsigned long)buf,
					 ((unsigned long)buf + count));

	/*
	 ** Configure DMA Controller
	 ** memory read
	 ** RESTART: flush DMA FIFO before beginning work unit
	 ** DI_EN: generate interrupt on completion of work unit
	 ** DMA2D: 2 dimensional buffer
	 */
	conf->dma_config |= (RESTART | DI_EN);

	if (conf->datalen > CFG_PPI_DATALEN_8) {	/* adjust transfer size	*/
		conf->dma_config |= WDSIZE_16;
		frameSize = count / 2;
		stepSize = 2;
	} else {
		conf->dma_config &= ~WDSIZE_16;
		frameSize = count;
		stepSize = 1;
	}

	if (conf->dimensions ==	CFG_PPI_DIMS_2D) {	/* configure for 2D transfers */
		pr_debug("PPI write -- 2D data linelen = %hd, numlines = %hd\n",
			conf->linelen, conf->numlines);
		set_dma_x_count(dma, conf->linelen);
		set_dma_x_modify(dma, stepSize);
		set_dma_y_count(dma, conf->numlines);
		set_dma_y_modify(dma, stepSize);
		conf->dma_config |= DMA2D;
	} else {
		pr_debug("PPI write -- 1D data count = %d\n", (int)count);
		set_dma_x_count(dma, frameSize);
		set_dma_x_modify(dma, stepSize);
		conf->dma_config &= ~DMA2D;
	}

	set_dma_start_addr(dma,	(unsigned long)buf);
	pr_debug("dma_config = 0x%04X\n", conf->dma_config);
	set_dma_config(dma, conf->dma_config);

	/* configure PPI registers to match DMA	registers */
	dev->regs->ppi_count = conf->linelen - 1;
	dev->regs->ppi_frame = conf->numlines;
	dev->regs->ppi_delay = conf->delay;
	SSYNC();

	enable_dma(dma);

	/* enable ppi */
	dev->regs->ppi_control |= PORT_EN;
	SSYNC();

	if (conf->timers)
		setup_timers(dev, frameSize);

	/* Wait	for DMA	to finish */

	if (1) {
		if (filp->f_flags & O_NONBLOCK)	{
			return -EAGAIN;
		} else {
			pr_debug("PPI wait_event_interruptible\n");
			ierr =
			    wait_event_interruptible(dev->waitq, conf->done);
			if (ierr) {
				/* waiting is broken by	a signal */
				pr_debug
				    ("PPI wait_event_interruptible ierr	= %d\n",
				     ierr);
				disable_ppi_dma(dev);
				conf->done = 1;
				return ierr;
			}
		}
	}

	pr_debug("ppi_write: return\n");

	return count;
}

/*
 * FUNCTION NAME: ppi_open
 *
 * INPUTS/OUTPUTS:
 * in_inode - Description of openned file.
 * in_filp - Description of openned file.
 *
 * RETURN
 * 0: Open ok.
 * -ENXIO  No such device
 *
 * FUNCTION(S) CALLED:
 *
 * GLOBAL VARIABLES REFERENCED:	ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION:	It is invoked when user	call 'open' system call
 *		to open	ppi device.
 *
 * CAUTION:
 */
static int ppi_open(struct inode *inode, struct	file *filp)
{
	struct ppi_dev *dev;	/* device information */
	unsigned long flags;
	dev = container_of(inode->i_cdev, struct ppi_dev, cdev);

	pr_debug("ppi_open:\n");

	spin_lock_irqsave(&dev->lock, flags);

	if (dev->conf.opened) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return -EMFILE;
	}

	filp->private_data = dev;	/* for other methods */

	/* Clear configuration information */
	memset(&dev->conf, 0, sizeof(struct ppi_config));
	dev->conf.opened = 1;
	dev->conf.done = 1;
	spin_unlock_irqrestore(&dev->lock, flags);

	ppi_reg_reset(dev);

	/* Request DMA0	channel, and pass the interrupt	handler	*/

	if (peripheral_request_list(dev->per_ppi0_7, PPI_DEVNAME)) {
		printk(KERN_ERR	PPI_DEVNAME
		": Requesting Peripherals failed\n");
		return -EBUSY;
	}

	if (request_dma(dev->dma_chan, "PPI_DMA") < 0) {
		printk(KERN_ERR	PPI_DEVNAME": Unable to	attach BlackFin	PPI DMA	channel\n");
		return -EFAULT;
	} else
		set_dma_callback(dev->dma_chan,	(void *)ppi_irq, dev);

	if (request_irq(dev->irq_error,	ppi_irq_error, IRQF_DISABLED,
			"PPI_ERROR", dev) < 0) {
		printk(KERN_ERR	PPI_DEVNAME": Unable to	attach BlackFin	PPI Error Interrupt\n");
		return -EFAULT;
	}

	pr_debug("ppi_open: return\n");

	return 0;
}

/*
 * FUNCTION NAME: ppi_release
 *
 * INPUTS/OUTPUTS:
 * in_inode - Description of openned file.
 * in_filp - Description of openned file.
 *
 * RETURN
 * Always 0
 *
 * FUNCTION(S) CALLED:
 *
 * GLOBAL VARIABLES REFERENCED:	ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION:	It is invoked when user	call 'close' system call
 *		to close device.
 *
 * CAUTION:
 */
static int ppi_release(struct inode *inode, struct file	*filp)
{
	struct ppi_dev *dev = filp->private_data;
	struct ppi_config *conf	= &dev->conf;
	unsigned long flags;

	pr_debug("ppi_release: close()\n");

	spin_lock_irqsave(&dev->lock, flags);
	ppi_reg_reset(dev);
	conf->opened = 0;
	spin_unlock_irqrestore(&dev->lock, flags);

	/* After finish	DMA, release it. */
	free_dma(dev->dma_chan);
	free_irq(dev->irq_error, dev);

	peripheral_free_list(dev->per_ppi0_7);

	if (conf->datalen > CFG_PPI_DATALEN_8)
		peripheral_free_list(&dev->per_ppi8_15[7 - dev->conf.datalen]);

	if (conf->fs23)
		peripheral_free_list(dev->per_ppifs);

	ppi_fasync(-1, filp, 0);

	pr_debug("ppi_release: close() return\n");
	return 0;
}

static struct file_operations ppi_fops = {
      owner:THIS_MODULE,
      read:ppi_read,
      write:ppi_write,
      ioctl:ppi_ioctl,
      open:ppi_open,
      release:ppi_release,
      fasync:ppi_fasync,
};

static ssize_t ppi_status_show(struct class *ppi_class,	char *buf)
{
	char *p;
	unsigned short i;
	p = buf;

	if (ppi_devices) {
		for (i = 0; i <	ppi_nr_devs; ++i)
			p += sprintf(p,
				     "PPI%d:\n irq %d\n	irq error %d\n dma chan	%d\n open %d\n"
				     " datalen %d\n fs23 %d\n timers %d\n triggeredge %d\n"
				     " dims=%d\n delay=%d\n mode=%d\n done=%d\n"
				     " dma config 0x%X\n linelen %d\n lines number %d\n	ppi control 0x%X\n",
					i,
					ppi_devices[i].irq,
					ppi_devices[i].irq_error,
					ppi_devices[i].dma_chan,
					ppi_devices[i].conf.opened,
					ppi_devices[i].conf.datalen,
					ppi_devices[i].conf.fs23,
					ppi_devices[i].conf.timers,
					ppi_devices[i].conf.triggeredge,
					ppi_devices[i].conf.dimensions,
					ppi_devices[i].conf.delay,
					ppi_devices[i].conf.access_mode,
					ppi_devices[i].conf.done,
					ppi_devices[i].conf.dma_config,
					ppi_devices[i].conf.linelen,
					ppi_devices[i].conf.numlines,
					ppi_devices[i].conf.ppi_control);
	}
	return p - buf;
}

static struct class *ppi_class;
static CLASS_ATTR(status, S_IRUGO, &ppi_status_show, NULL);

static void ppi_setup_cdev(struct ppi_dev *dev,	int index)
{
	int err, devno = MKDEV(ppi_major, ppi_minor + index);

	cdev_init(&dev->cdev, &ppi_fops);
	dev->cdev.owner	= THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		printk(KERN_NOTICE "Error %d adding ppi%d", err, index);
}

/*
 * FUNCTION NAME: ppi_cleanup_module
 *
 * INPUTS/OUTPUTS:
 *
 * RETURN:
 *
 * FUNCTION(S) CALLED:
 *
 * GLOBAL VARIABLES REFERENCED:
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: It will be invoked when using 'rmmod' command.
 *              or, you invoke it directly when it needs remove
 *              ppi module.
 *
 * CAUTION:
 */
static void ppi_cleanup_module(void)
{
	int i;
	dev_t devno = MKDEV(ppi_major, ppi_minor);

	if (ppi_devices) {
		for (i = 0; i <	ppi_nr_devs; ++i)
			cdev_del(&ppi_devices[i].cdev);
		kfree(ppi_devices);
	}

	unregister_chrdev_region(devno,	ppi_nr_devs);
}
module_exit(ppi_cleanup_module);

/*
 * FUNCTION NAME: ppi_init_module
 *
 * INPUTS/OUTPUTS:
 *
 * RETURN:
 * 0 if module init ok.
 * -1 init fail.
 *
 * FUNCTION(S) CALLED:
 *
 * GLOBAL VARIABLES REFERENCED:
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: It will be invoked when using 'insmod' command.
 *              or invoke it directly if ppi module is needed.
 *
 * CAUTION:
 */

static int __init ppi_init_module(void)
{
	int minor;
	int result, i;
	dev_t dev = 0;

	dev = MKDEV(ppi_major, ppi_minor);
	result = register_chrdev_region(dev, ppi_nr_devs, "ppi");
	if (result < 0)	{
		printk(KERN_WARNING "ppi: can't	get major %d\n", ppi_major);
		return result;
	}

	ppi_devices = kcalloc(ppi_nr_devs, sizeof(*ppi_devices), GFP_KERNEL);
	if (!ppi_devices) {
		ppi_cleanup_module();
		return -ENOMEM;
	}

	ppi_class = class_create(THIS_MODULE, "ppi");
	result = class_create_file(ppi_class, &class_attr_status);
	if (result) {
		ppi_cleanup_module();
		return result;
	}
	for (minor = 0;	minor <	ppi_nr_devs; minor++)
		device_create(ppi_class, NULL, MKDEV(ppi_major,	minor),
			      NULL, "ppi%d", minor);

	/* Initialize each device. */
	for (i = 0; i <	ppi_nr_devs; ++i) {
		ppi_setup_cdev(&ppi_devices[i],	i);
		ppi_devices[i].ppi_num = i;
		spin_lock_init(&ppi_devices[i].lock);
		init_waitqueue_head(&ppi_devices[i].waitq);
	}

	/* PPI0	*/

	ppi_devices[0].dma_chan	= CH_PPI;
	ppi_devices[0].irq = IRQ_PPI;
	ppi_devices[0].irq_error = IRQ_PPI_ERROR;
#ifdef FS0_1_TIMER_ID /* BF561 */
	ppi_devices[0].regs = (struct ppi_register *)PPI0_CONTROL;
	ppi_devices[0].fs1_timer_id = FS0_1_TIMER_ID;
	ppi_devices[0].fs2_timer_id = FS0_2_TIMER_ID;
	ppi_devices[0].fs1_timer_bit = FS0_1_TIMER_BIT;
	ppi_devices[0].fs2_timer_bit = FS0_2_TIMER_BIT;
#else
	ppi_devices[0].regs = (struct ppi_register *)PPI_CONTROL;
	ppi_devices[0].fs1_timer_id = FS1_TIMER_ID;
	ppi_devices[0].fs2_timer_id = FS2_TIMER_ID;
	ppi_devices[0].fs1_timer_bit = FS1_TIMER_BIT;
	ppi_devices[0].fs2_timer_bit = FS2_TIMER_BIT;
#endif
	ppi_devices[0].per_ppi0_7 = per_req_ppi0_7;
	ppi_devices[0].per_ppi8_15 = per_req_ppi8_15;
	ppi_devices[0].per_ppifs = per_req_ppi_fs;

	/* PPI1	*/
#ifdef PPI1_CONTROL
	ppi_devices[1].regs = (struct ppi_register *)PPI1_CONTROL;
	ppi_devices[1].dma_chan	= CH_PPI1;
	ppi_devices[1].irq = IRQ_PPI1;
	ppi_devices[1].irq_error = IRQ_PPI1_ERROR;
	ppi_devices[1].fs1_timer_id = FS1_1_TIMER_ID;
	ppi_devices[1].fs2_timer_id = FS1_2_TIMER_ID;
	ppi_devices[1].fs1_timer_bit = FS1_1_TIMER_BIT;
	ppi_devices[1].fs2_timer_bit = FS1_2_TIMER_BIT;
	ppi_devices[0].per_ppi0_7 = per_req_ppi1_0_7;
	ppi_devices[0].per_ppi8_15 = per_req_ppi1_8_15;
	ppi_devices[0].per_ppifs = per_req_ppi1_fs;
#endif
	return 0;
}
module_init(ppi_init_module);

MODULE_AUTHOR("John DeHority");
MODULE_LICENSE("GPL");

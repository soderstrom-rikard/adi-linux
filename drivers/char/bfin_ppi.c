/*
 * Blackfin char PPI driver
 *
 * Written by John DeHority <john.dehority@kodak.com>
 *
 * Copyright (C) 2005, Eastman Kodak Company
 * Copyright 2005-2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>

#include <asm/irq.h>
#include <asm/blackfin.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>
#include <asm/portmux.h>

#include <asm/gptimers.h>

#include "bfin_ppi.h"

#define PPI_READ 0
#define PPI_WRITE 1

static const unsigned short per_req_ppi0_7[] = {
	P_PPI0_CLK, P_PPI0_FS1,
	P_PPI0_D0, P_PPI0_D1, P_PPI0_D2, P_PPI0_D3,
	P_PPI0_D4, P_PPI0_D5, P_PPI0_D6, P_PPI0_D7,
	0
};

static const unsigned short per_req_ppi8_15[] = {
	P_PPI0_D8, P_PPI0_D9, P_PPI0_D10, P_PPI0_D11,
	P_PPI0_D12, P_PPI0_D13, P_PPI0_D14, P_PPI0_D15,
	0
};

static const unsigned short per_req_ppi_fs[] = {
	P_PPI0_FS2, /*P_PPI0_FS3,*/ 0
};

#if defined(PPI1_CONTROL) || defined(EPPI1_CONTROL)
static const unsigned short per_req_ppi1_0_7[] = {
	P_PPI1_CLK, P_PPI1_FS1,
	P_PPI1_D0, P_PPI1_D1, P_PPI1_D2, P_PPI1_D3,
	P_PPI1_D4, P_PPI1_D5, P_PPI1_D6, P_PPI1_D7,
	0
};

static const unsigned short per_req_ppi1_8_15[] = {
	P_PPI1_D8, P_PPI1_D9, P_PPI1_D10, P_PPI1_D11,
	P_PPI1_D12, P_PPI1_D13, P_PPI1_D14, P_PPI1_D15,
	0
};

static const unsigned short per_req_ppi1_fs[] = {
	P_PPI1_FS2, /*P_PPI1_FS3,*/ 0
};
static int ppi_nr_devs = 2;
#else
static int ppi_nr_devs = 1;
#endif

static int ppi_major = 252;
module_param(ppi_major, int, 0);
MODULE_PARM_DESC(ppi_major, "Major device number");

static int ppi_minor;

struct ppi_register {
#if defined(PPI_CONTROL) || defined(PPI0_CONTROL)	/* PPI */
#define PPI_COUNT_CORR_OFFSET	1
#define XFR_TYPE_SHIFT	2
#define PORT_CFG_SHIFT	4
#define PORT_DLEN_SHIFT	11
	unsigned short ppi_control;
	unsigned short dummy0;
	unsigned short ppi_status;
	unsigned short dummy1;
	unsigned short ppi_count;
	unsigned short dummy2;
	unsigned short ppi_delay;
	unsigned short dummy3;
	unsigned short ppi_frame;
#elif defined(EPPI0_CONTROL) || defined(EPPI1_CONTROL)	/* EPPI */
#define PORT_EN		EPPI_EN
#define PORT_DIR	EPPI_DIR
#define PORT_CFG	FS_CFG
#define PACK_EN		PACKEN
#define EPPI_PACK_EN	(1 << 20)
#define PPI_COUNT_CORR_OFFSET	0
#define XFR_TYPE_SHIFT	2
#define PORT_CFG_SHIFT	4  /* FS_CFG */
#define PORT_DLEN_SHIFT	15
	unsigned short ppi_status;
	unsigned short dummy0;
	unsigned short ppi_hcount;
	unsigned short dummy1;
	unsigned short ppi_delay;	/* EPPI_HDELAY */
	unsigned short dummy2;
	unsigned short ppi_vcount;
	unsigned short dummy3;
	unsigned short ppi_vdelay;
	unsigned short dummy4;
	unsigned short ppi_frame;
	unsigned short dummy5;
	unsigned short ppi_count;	/* EPPI_LINE */
	unsigned short dummy6;
	unsigned short ppi_clkdiv;
	unsigned short dummy7;
	unsigned int ppi_control;
	unsigned int ppi_fs1w_hbl;
	unsigned int ppi_fs1p_avpl;
	unsigned int ppi_fs2w_lvb;
	unsigned int ppi_fs2p_lavf;
	unsigned int ppi_clip;
#endif
};

struct ppi_config {
	unsigned char opened;
	unsigned char data_len;
	unsigned char pack_mode;
	unsigned char fs23;
	unsigned char timers;
	unsigned char trigger_edge;
	unsigned char dimensions;
	unsigned short delay;
	unsigned short access_mode;
	unsigned short done;
	unsigned short dma_config;
	unsigned short line_len;
	unsigned short num_lines;
	unsigned short fs1_blanking;
	unsigned short ppi_control;
#if defined(EPPI0_CONTROL) || defined(EPPI1_CONTROL)	/* EPPI */
	unsigned short v_delay;
	unsigned short v_count;
#endif
};

struct ppi_dev {
	struct cdev cdev;	/* Char device structure */
	struct fasync_struct *fasyc;
	struct ppi_config conf;
	volatile struct ppi_register *regs;
	wait_queue_head_t waitq;
	spinlock_t lock;
	int ppi_num;
	int dma_chan;
	int irq;
	int irq_error;
	int fs1_timer_id;
	int fs2_timer_id;
	int fs_int_timer_id;
	unsigned short fs1_timer_bit;
	unsigned short fs2_timer_bit;
	const unsigned short *per_ppi0_7;
	const unsigned short *per_ppi8_15;
	const unsigned short *per_ppifs;
};

struct ppi_dev *ppi_devices;	/* allocated in ppi_init_module */

static inline unsigned short clear_ppi_status(struct ppi_dev *dev)
{
	unsigned short stat = dev->regs->ppi_status;
	dev->regs->ppi_status = -1;
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

static int compute_data_len(int dlen)
{
	int bits;
	switch (dlen) {
	case CFG_PPI_DATALEN_8:
		bits = 8;
		break;
	case CFG_PPI_DATALEN_10:
		bits = 10;
		break;
#ifdef CFG_PPI_DATALEN_11
	case CFG_PPI_DATALEN_11:
		bits = 11;
		break;
#endif
	case CFG_PPI_DATALEN_12:
		bits = 12;
		break;
#ifdef CFG_PPI_DATALEN_13
	case CFG_PPI_DATALEN_13:
		bits = 13;
		break;
#endif
	case CFG_PPI_DATALEN_14:
		bits = 14;
		break;
#ifdef CFG_PPI_DATALEN_15
	case CFG_PPI_DATALEN_15:
		bits = 15;
		break;
#endif
	case CFG_PPI_DATALEN_16:
		bits = 16;
		break;
#ifdef CFG_PPI_DATALEN_18
	case CFG_PPI_DATALEN_18:
		bits = 18;
		break;
#endif
#ifdef CFG_PPI_DATALEN_24
	case CFG_PPI_DATALEN_24:
		bits = 24;
		break;
#endif
	default:
		bits = -1;
		break;
	}

	return bits;
}

static void setup_timers(struct ppi_dev *dev, unsigned int frame_size)
{
	struct ppi_config *conf = &dev->conf;
	short fs1_timer_cfg = 0;
	short fs2_timer_cfg = 0;
	unsigned short t_mask;
	unsigned int line_period;
	/*
	 * set timer configuration register template
	 *
	 * see note on page 11-29 of BF533 HW Reference Manual
	 * for setting PULSE_HI according to PPI trigger edge configuration
	 * of PPI_FS1 and PPI_FS2
	 */

	fs1_timer_cfg = (TIMER_CLK_SEL | TIMER_TIN_SEL | TIMER_MODE_PWM);

	if (conf->trigger_edge)
		fs1_timer_cfg &= ~TIMER_PULSE_HI;
	else
		fs1_timer_cfg |= TIMER_PULSE_HI;

	fs2_timer_cfg = fs1_timer_cfg;
	fs1_timer_cfg |= TIMER_PERIOD_CNT;	/* set up line sync to be recurring */
	fs2_timer_cfg |= TIMER_IRQ_ENA;		/* Enable IRQ on FS2 Timer */

	/*
	 * ANOMALY_05000254 - Incorrect Timer Pulse Width in Single-Shot PWM_OUT Mode with External Clock
	 */
	if (ANOMALY_05000254)
		fs2_timer_cfg |= TIMER_PERIOD_CNT;	/* set up frame sync to be recurring */


	if (conf->dimensions == CFG_PPI_DIMS_2D) {	/* configure for 2D transfers */

		line_period = conf->line_len + conf->delay + conf->fs1_blanking + 1;
		frame_size = line_period * conf->num_lines - conf->fs1_blanking - 1;

		/*
		 * configure 2 timers for 2D
		 * Timer1 - hsync - line time   PPI_FS1 (Timer0 on BF537)
		 * Timer2 - vsync - frame time  PPI_FS2 (Timer1 on BF537)
		 */
		t_mask = dev->fs1_timer_bit | dev->fs2_timer_bit;	/* use both timers */

		set_gptimer_config(dev->fs2_timer_id, fs2_timer_cfg);
		set_gptimer_period(dev->fs2_timer_id, frame_size + 2);
		set_gptimer_pwidth(dev->fs2_timer_id, frame_size);
		pr_debug
		    ("Timer %d: (frame/vsync) config = %04hX, period = %d, width = %d\n",
		     dev->fs2_timer_id, get_gptimer_config(dev->fs2_timer_id),
		     get_gptimer_period(dev->fs2_timer_id),
		     get_gptimer_pwidth(dev->fs2_timer_id));

		set_gptimer_config(dev->fs1_timer_id, fs1_timer_cfg);
		set_gptimer_period(dev->fs1_timer_id, line_period);

		set_gptimer_pwidth(dev->fs1_timer_id, (conf->line_len >> 2));
		pr_debug
		    ("Timer %d: (line/hsync) config = %04hX, period = %d, width = %d\n",
		     dev->fs1_timer_id, get_gptimer_config(dev->fs1_timer_id),
		     get_gptimer_period(dev->fs1_timer_id),
		     get_gptimer_pwidth(dev->fs1_timer_id));

		dev->fs_int_timer_id = dev->fs2_timer_id;

		enable_gptimers(t_mask);
		if (ANOMALY_05000254)
			disable_gptimers_sync(dev->fs2_timer_bit);
	} else {
		t_mask = dev->fs1_timer_bit;

		/*
		 * set timer for frame vsync
		 * use fs2_timer_cfg, 'cuz it is the non-recurring config
		 */
		set_gptimer_config(dev->fs1_timer_id, fs2_timer_cfg);
		set_gptimer_period(dev->fs1_timer_id, frame_size + 2);
		set_gptimer_pwidth(dev->fs1_timer_id, frame_size);

		pr_debug("Timer %d: config = %04hX, period = %d, width = %d\n",
			dev->fs1_timer_id, fs2_timer_cfg,
			get_gptimer_period(dev->fs1_timer_id),
			get_gptimer_pwidth(dev->fs1_timer_id));

		dev->fs_int_timer_id = dev->fs1_timer_id;
		enable_gptimers(t_mask);
		if (ANOMALY_05000254)
			disable_gptimers_sync(t_mask);
	}
	pr_debug("enable_gptimers(mask=%d)\n", t_mask);
}

static void disable_timer_output(struct ppi_dev *dev)
{
	unsigned short regdata;

	if (dev->conf.dimensions == CFG_PPI_DIMS_2D) {
		/* disable timer1 for FS2 */
		regdata = get_gptimer_config(dev->fs2_timer_id),
		    regdata &= ~TIMER_OUT_DIS;
		set_gptimer_config(dev->fs2_timer_id, regdata);
	}

	/* disable timer0 outputs for FS1 */
	regdata = get_gptimer_config(dev->fs1_timer_id),
	regdata &= ~TIMER_OUT_DIS;
	set_gptimer_config(dev->fs1_timer_id, regdata);
}

/*
 * FUNCTION NAME: ppi_irq
 *
 * INPUTS/OUTPUTS:
 * in_irq    - Interrupt vector number.
 * in_dev_id - point to device information structure base address.
 * in_regs   - unuse here.
 *
 * VALUE RETURNED:
 * void
 *
 * GLOBAL VARIABLES REFERENCED: ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: ISR of PPI
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
 * in_irq    - Interrupt vector number.
 * in_dev_id - point to device information structure base address.
 * in_regs   - unuse here.
 *
 * VALUE RETURNED:
 * void
 *
 * GLOBAL VARIABLES REFERENCED: ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: ISR of PPI
 */
static irqreturn_t ppi_irq(int irq, void *dev_id)
{
	struct ppi_dev *dev = (struct ppi_dev *) dev_id;
	unsigned long flags;

	if (dev->conf.timers && dev->conf.access_mode == PPI_WRITE) {
		while (!get_gptimer_intr(dev->fs_int_timer_id) &&
			get_gptimer_run(dev->fs_int_timer_id))
			cpu_relax();

		disable_gptimers(dev->fs1_timer_bit | dev->fs2_timer_bit);
		clear_gptimer_intr(dev->fs_int_timer_id);
	}
	/* Give a signal to user program. */
	if (dev->fasyc)
		kill_fasync(&(dev->fasyc), SIGIO, POLLIN);

	disable_ppi_dma(dev);

	clear_ppi_status(dev);
	clear_dma_irqstat(dev->dma_chan);

	spin_lock_irqsave(&dev->lock, flags);
	dev->conf.done = 1;
	spin_unlock_irqrestore(&dev->lock, flags);

	/* wake up read/write block. */
	wake_up_interruptible(&dev->waitq);
	pr_debug("ppi_irq: return\n");

	return IRQ_HANDLED;
}

/*
 * FUNCTION NAME: ppi_irq_error
 *
 * INPUTS/OUTPUTS:
 * in_irq    - Interrupt vector number.
 * in_dev_id - point to device information structure base address.
 * in_regs   - unused here.
 *
 * VALUE RETURNED:
 * void
 *
 * GLOBAL VARIABLES REFERENCED: ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: Error ISR of PPI
 */
static irqreturn_t ppi_irq_error(int irq, void *dev_id)
{
	struct ppi_dev *dev = (struct ppi_dev *) dev_id;
	unsigned long flags;

	pr_warning("PPI Error: PPI Status = %#X\n", clear_ppi_status(dev));

	if (dev->conf.timers && dev->conf.access_mode == PPI_WRITE) {
		while (!get_gptimer_intr(dev->fs_int_timer_id) &&
			get_gptimer_run(dev->fs_int_timer_id))
			cpu_relax();

		disable_gptimers(dev->fs1_timer_bit | dev->fs2_timer_bit);
		clear_gptimer_intr(dev->fs_int_timer_id);
	}
	/* Give a signal to user program. */
	if (dev->fasyc)
		kill_fasync(&(dev->fasyc), SIGIO, POLLIN);

	disable_ppi_dma(dev);

	clear_ppi_status(dev);
	clear_dma_irqstat(dev->dma_chan);

	spin_lock_irqsave(&dev->lock, flags);
	dev->conf.done = 1;
	spin_unlock_irqrestore(&dev->lock, flags);

	/* wake up read/write block. */
	wake_up_interruptible(&dev->waitq);

	return IRQ_HANDLED;
}

/*
 * FUNCTION NAME: ppi_ioctl
 *
 * INPUTS/OUTPUTS:
 * in_inode   - Description of opened file.
 * in_filp    - Description of opened file.
 * in_cmd     - Command passed into ioctl system call.
 * in/out_arg - It is parameters which is specified by last command
 *
 * RETURN:
 * 0 OK
 * -EINVAL  Invalid baudrate
 *
 * GLOBAL VARIABLES REFERENCED: ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION:
 */
static long ppi_ioctl(struct file *filp, uint cmd, unsigned long arg)
{
	unsigned long regdata;
	unsigned long flags;
	struct ppi_dev *dev = filp->private_data;
	struct ppi_config *conf = &dev->conf;

	spin_lock_irqsave(&dev->lock, flags);
	switch (cmd) {
	case CMD_PPI_XFR_TYPE:
		pr_debug("ppi_ioctl: CMD_PPI_XFR_TYPE\n");
		if (arg < 0 || arg > 3)
			goto err_inval;
		regdata = dev->regs->ppi_control;
		regdata &= ~XFR_TYPE;
		regdata |= ((unsigned short)arg << XFR_TYPE_SHIFT);
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_PORT_CFG:
		pr_debug("ppi_ioctl: CMD_PPI_PORT_CFG\n");
		if (arg < 0 || arg > 3)
			goto err_inval;
		regdata = dev->regs->ppi_control;
		regdata &= ~PORT_CFG;
		regdata |= ((unsigned short)arg << PORT_CFG_SHIFT);
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;

		if (arg == CFG_PPI_PORT_CFG_SYNC23 ||
			 arg == CFG_PPI_PORT_CFG_XSYNC23) {
			conf->fs23 = 1;
			if (peripheral_request_list(dev->per_ppifs, KBUILD_MODNAME)) {
				spin_unlock_irqrestore(&dev->lock, flags);
				pr_err("ppi_ioctl: Requesting Peripherals failed\n");
				return -EBUSY;
			}
		}
		break;

	case CMD_PPI_FIELD_SELECT:
		pr_debug("ppi_ioctl: CMD_PPI_FIELD_SELECT\n");
		regdata = dev->regs->ppi_control;
		if (arg)
			regdata |= FLD_SEL;
		else
			regdata &= ~FLD_SEL;
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_PACKING:
		pr_debug("ppi_ioctl: CMD_PPI_PACKING\n");
		regdata = dev->regs->ppi_control;
		if (arg) {
			/*
			 * The PACK_EN bit only has meaning when the PPI port width (selected by
			 * DLEN[2:0]) is 8 bits
			 */
			if (conf->data_len > CFG_PPI_DATALEN_8) {
				pr_err("ppi_ioctl: CMD_PPI_PACKING"
						"PACKING can't work while PPI port width isn't 8\n");
				goto err_inval;
			}
#if defined(PPI_CONTROL) || defined(PPI0_CONTROL)
			regdata |= PACK_EN;
#elif defined(EPPI0_CONTROL) || defined(EPPI1_CONTROL)
			regdata |= EPPI_PACK_EN;
#endif
			pr_debug("ppi_ioctl: CMD_PPI_PACKING packing enable\n");
		} else {
#if defined(PPI_CONTROL) || defined(PPI0_CONTROL)
			regdata &= ~PACK_EN;
#elif defined(EPPI0_CONTROL) || defined(EPPI1_CONTROL)
			regdata &= ~EPPI_PACK_EN;
#endif
			pr_debug("ppi_ioctl: CMD_PPI_PACKING packing disable\n");
		}
		conf->ppi_control = regdata;
		conf->pack_mode = !!arg;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_SKIPPING:
		pr_debug("ppi_ioctl: CMD_PPI_SKIPPING\n");
		regdata = dev->regs->ppi_control;
		if (arg)
			regdata |= SKIP_EN;
		else
			regdata &= ~SKIP_EN;
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_SKIP_ODDEVEN:
		pr_debug("ppi_ioctl: CMD_PPI_SKIP_ODDEVEN\n");
		regdata = dev->regs->ppi_control;
		if (arg)
			regdata |= SKIP_EO;
		else
			regdata &= ~SKIP_EO;
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_DATALEN: {
		int bits, cnt, ret;

		pr_debug("ppi_ioctl: CMD_PPI_DATALEN\n");
		bits = compute_data_len(arg);
		if (bits < 0)
			goto err_inval;
		conf->data_len = (unsigned short)arg;
		regdata = dev->regs->ppi_control;
		regdata &= ~DLENGTH;
		regdata |= (arg << PORT_DLEN_SHIFT);
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;

		for (cnt = 0; cnt < bits - 8; cnt++) {
			const unsigned short *per = dev->per_ppi8_15;
			ret = peripheral_request(per[cnt], KBUILD_MODNAME);
			if (ret < 0) {
				for ( ; cnt > 0; cnt--)
					peripheral_free(per[cnt - 1]);
				spin_unlock_irqrestore(&dev->lock, flags);
				pr_err("ppi_ioctl: Requesting \
						Peripherals failed\n");
				return ret;
			}
		}
		break;
	}

	case CMD_PPI_LINELEN:
		pr_debug("ppi_ioctl: CMD_PPI_LINELEN\n");
		if (arg < 0 || arg > PPI_DMA_MAXSIZE)
			goto err_inval;
		conf->line_len = (unsigned short)arg;
		break;

	case CMD_PPI_NUMLINES:
		pr_debug("ppi_ioctl: CMD_PPI_NUMLINES\n");
		if (arg < 0 || arg > PPI_DMA_MAXSIZE)
			goto err_inval;
		conf->num_lines = (unsigned short)arg;
		break;

	case CMD_PPI_SET_WRITECONTINUOUS:
		pr_err("ppi_ioctl: CMD_PPI_SET_WRITECONTINUOUS Not supported\n");
		goto err_inval;
		break;

	case CMD_PPI_SET_DIMS:
		pr_debug("ppi_ioctl: CMD_PPI_SET_DIMS\n");
		conf->dimensions = (unsigned char)arg;
		break;

	case CMD_PPI_DELAY:
		pr_debug("ppi_ioctl: CMD_PPI_DELAY\n");
		conf->delay = (unsigned short)arg;
		dev->regs->ppi_delay = ((unsigned short)conf->delay);
		SSYNC();
		break;

#ifdef DEBUG
	case CMD_PPI_GET_ALLCONFIG:
		break;
#endif

	case CMD_PPI_SETGPIO:
		pr_debug("ppi_ioctl: CMD_PPI_SETGPIO\n");
		break;

#if defined(PPI_CONTROL) || defined(PPI0_CONTROL)

	case CMD_PPI_GEN_FS12_TIMING_ON_WRITE:
		pr_debug("ppi_ioctl: CMD_PPI_GEN_FS12_TIMING_ON_WRITE\n");
		conf->timers = (unsigned short)arg;
		break;

	case CMD_PPI_FS1_EOL_BLANKING:
		pr_debug("ppi_ioctl: CMD_PPI_FS1_EOL_BLANKING\n");
		conf->fs1_blanking = (unsigned short)arg;
		break;

	case CMD_PPI_CLK_EDGE:
		pr_debug("ppi_ioctl: CMD_PPI_CLK_EDGE\n");
		regdata = dev->regs->ppi_control;
		if (arg)
			regdata |= POLC;
		else
			regdata &= ~POLC;
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_TRIG_EDGE:
		pr_debug("ppi_ioctl: CMD_PPI_TRIG_EDGE\n");
		conf->trigger_edge = (unsigned short)arg;
		regdata = dev->regs->ppi_control;
		if (arg)
			regdata |= POLS;
		else
			regdata &= ~POLS;
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

#else	/* EPPI */

	case CMD_PPI_CLK_EDGE:
		pr_debug("ppi_ioctl: CMD_PPI_CLK_EDGE\n");
		if (arg < 0 || arg > 3)
			goto err_inval;
		regdata = dev->regs->ppi_control;
		regdata &= ~POLC;
		regdata |= (arg << 11);
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_FSACTIVE_SELECT:
		pr_debug("ppi_ioctl: CMD_PPI_FSACTIVE_SELECT\n");
		if (arg < 0 || arg > 3)
			goto err_inval;
		conf->trigger_edge = (unsigned short)arg;
		regdata = dev->regs->ppi_control;
		regdata &= ~POLS;
		regdata |= (arg << 13);
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_BLANKGEN_SELECT:
		pr_debug("ppi_ioctl: CMD_PPI_BLANKGEN_SELECT\n");
		regdata = dev->regs->ppi_control;
		if (arg)
			regdata |= BLANKGEN;
		else
			regdata &= ~BLANKGEN;
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_CLKGEN_SELECT:
		pr_debug("ppi_ioctl: CMD_PPI_CLKGEN_SELECT\n");
		regdata = dev->regs->ppi_control;
		if (arg)
			regdata |= ICLKGEN;
		else
			regdata &= ~ICLKGEN;
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_FSGEN_SELECT:
		pr_debug("ppi_ioctl: CMD_PPI_FSGEN_SELECT\n");
		regdata = dev->regs->ppi_control;
		if (arg)
			regdata |= IFSGEN;
		else
			regdata &= ~IFSGEN;
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_SWAP:
		pr_debug("ppi_ioctl: CMD_PPI_SWAP\n");
		regdata = dev->regs->ppi_control;
		if (arg)
			regdata |= SWAPEN;
		else
			regdata &= ~SWAPEN;
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_SIGNEXT:
		pr_debug("ppi_ioctl: CMD_PPI_SIGNEXT\n");
		regdata = dev->regs->ppi_control;
		if (arg)
			regdata |= SIGN_EXT;
		else
			regdata &= ~SIGN_EXT;
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_SPLIT_EVENODD:
		pr_debug("ppi_ioctl: CMD_PPI_SPLIT_EVENODD\n");
		regdata = dev->regs->ppi_control;
		if (arg)
			regdata |= SPLT_EVEN_ODD;
		else
			regdata &= ~SPLT_EVEN_ODD;
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_SUBSPLIT_ODD:
		pr_debug("ppi_ioctl: CMD_PPI_SUBSPLIT_ODD\n");
		regdata = dev->regs->ppi_control;
		if (arg)
			regdata |= SUBSPLT_ODD;
		else
			regdata &= ~SUBSPLT_ODD;
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_RGBFORMAT:
		pr_debug("ppi_ioctl: CMD_PPI_RGBFORMAT\n");
		regdata = dev->regs->ppi_control;
		if (arg)
			regdata |= RGB_FMT_EN;
		else
			regdata &= ~RGB_FMT_EN;
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_FIFORWM:
		pr_debug("ppi_ioctl: CMD_PPI_FIFORWM\n");
		if (arg < 0 || arg > 3)
			goto err_inval;
		regdata = dev->regs->ppi_control;
		regdata &= ~FIFO_RWM;
		regdata |= (arg << 27);
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_FIFOUWM:
		pr_debug("ppi_ioctl: CMD_PPI_FIFOUWM\n");
		if (arg < 0 || arg > 3)
			goto err_inval;
		regdata = dev->regs->ppi_control;
		regdata &= ~FIFO_UWM;
		regdata |= (arg << 29);
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_ITUTYPE:
		pr_debug("ppi_ioctl: CMD_PPI_ITUTYPE\n");
		regdata = dev->regs->ppi_control;
		if (arg)
			regdata |= ITU_TYPE;
		else
			regdata &= ~ITU_TYPE;
		conf->ppi_control = regdata;
		dev->regs->ppi_control = regdata;
		break;

	case CMD_PPI_CLKDIV:
		pr_debug("ppi_ioctl: CMD_PPI_CLKDIV\n");

		dev->regs->ppi_clkdiv = (unsigned short)arg;
		break;

	case CMD_PPI_EPPI1_FS1W_HBL:
		pr_debug("ppi_ioctl: CMD_PPI_EPPI1_FS1W_HBL\n");
		dev->regs->ppi_fs1w_hbl = arg;
		break;

	case CMD_PPI_EPPI1_FS1P_AVPL:
		pr_debug("ppi_ioctl: CMD_PPI_EPPI1_FS1P_AVPL\n");
		dev->regs->ppi_fs1p_avpl = arg;
		break;

	case CMD_PPI_EPPI1_FS2W_LVB:
		pr_debug("ppi_ioctl: CMD_PPI_EPPI1_FS2W_LVB\n");
		dev->regs->ppi_fs2w_lvb = arg;
		break;

	case CMD_PPI_EPPI1_FS2P_LAVF:
		pr_debug("ppi_ioctl: CMD_PPI_EPPI1_FS2P_LAVF\n");
		dev->regs->ppi_fs2p_lavf = arg;
		break;

	case CMD_PPI_EPPI1_CLIP:
		pr_debug("ppi_ioctl: CMD_PPI_EPPI1_CLIP\n");
		dev->regs->ppi_clip = arg;
		break;

	case CMD_PPI_EPPI1_VDELAY:
		pr_debug("ppi_ioctl: CMD_PPI_EPPI1_VDELAY\n");
		conf->v_delay = (unsigned short)arg;
		dev->regs->ppi_vdelay = (unsigned short) arg;
		break;

	case CMD_PPI_EPPI1_VCOUNT:
		pr_debug("ppi_ioctl: CMD_PPI_EPPI1_VCOUNT\n");
		conf->v_count = (unsigned short)arg;
		break;

#endif

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
 * in_fd   - File descriptor of opened file.
 * in_filp - Description of opened file.
 *
 * RETURN:
 *
 * GLOBAL VARIABLES REFERENCED: ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: It is invoked when user changes status of sync
 *              it resister a hook in system. When there is
 *              data coming, user program would get a signal.
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
 * filp - Description of opened file.
 * buf -- Pointer to buffer allocated to hold data.
 * count - how many bytes user wants to get.
 * pos -- unused
 *
 * RETURN
 * positive number: bytes read back
 * -EINVAL When word size is bigger than 8, reading odd bytes
 *         When word size is bigger than 16, reading bytes which can't be divided exactly by 4
 *         For PPI PACKING mode, reading odd bytes
 *         For EPPI PAKCING mode, read bytes which can't be divided exactly by 4
 * -EAGAIN When reading mode is set to non block and there is no rx data.
 *
 * GLOBAL VARIABLES REFERENCED: ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: It is invoked when user call 'read' system call
 *  to read from system.
 */
static ssize_t ppi_read(struct file *filp, char *buf, size_t count, loff_t *pos)
{
	unsigned long flags;
	int ierr;
	struct ppi_dev *dev = filp->private_data;
	struct ppi_config *conf = &dev->conf;
	unsigned int dma = dev->dma_chan;
	unsigned int step_size, frame_size, x_count;

	pr_debug("ppi_read(0x%08X, %d)\n", (int)buf, (int)count);

#if defined(PPI_CONTROL) || defined(PPI0_CONTROL)
	if ((conf->data_len > CFG_PPI_DATALEN_8) || conf->pack_mode)
		step_size = 2;
	else
		step_size = 1;
#elif defined(EPPI0_CONTROL) || defined(EPPI1_CONTROL)
	if ((conf->data_len > CFG_PPI_DATALEN_16) || conf->pack_mode)
		step_size = 4;
	else if (conf->data_len > CFG_PPI_DATALEN_8)
		step_size = 2;
	else
		step_size = 1;
#endif

	if ((count & (step_size - 1)) | ((unsigned long)buf & (step_size - 1)))
		return -EINVAL;

	frame_size = count / step_size;

	/*
	 * calculate line len according to count, if it is not configured
	 */
	if (conf->dimensions == CFG_PPI_DIMS_2D) {
		if ((conf->line_len == 0) || (conf->num_lines == 0)) {
			pr_err("ppi_read: Devices is not configured right, 0 row or 0 column in 2D mode\n");
			return -EINVAL;
		}
	} else if (conf->line_len == 0) {
#if defined(PPI_CONTROL) || defined(PPI0_CONTROL)
		if (conf->data_len > CFG_PPI_DATALEN_8)
			conf->line_len = count / step_size;
#elif defined(EPPI0_CONTROL) || defined(EPPI1_CONTROL)
		switch (conf->data_len) {
		case CFG_PPI_DATALEN_8:
			conf->line_len = count;
			break;
		case CFG_PPI_DATALEN_10:
		case CFG_PPI_DATALEN_12:
		case CFG_PPI_DATALEN_14:
		case CFG_PPI_DATALEN_16:
			conf->line_len = count / 2;
			break;
		case CFG_PPI_DATALEN_18:
		case CFG_PPI_DATALEN_24:
			if (conf->pack_mode)
				conf->line_len = count / 3;
			else
				conf->line_len = count / 4;
			break;
		}
#endif
	}

	if (ANOMALY_05000179) {
		if (conf->line_len < 3)
			return -EFAULT;
	}

	/*
	 * calculate DMA x_count according to line_len, PPI data length, and packing mode
	 */
	if (conf->dimensions == CFG_PPI_DIMS_2D) {
#if defined(PPI_CONTROL) || defined(PPI0_CONTROL)
		if (conf->pack_mode)
			x_count = conf->line_len / 2;
		else
			x_count = conf->line_len;
#elif defined(EPPI0_CONTROL) || defined(EPPI1_CONTROL)
		if (conf->pack_mode) {
			switch (conf->data_len) {
			case CFG_PPI_DATALEN_8:
				x_count = conf->line_len / 4;
				break;
			case CFG_PPI_DATALEN_10:
			case CFG_PPI_DATALEN_12:
			case CFG_PPI_DATALEN_14:
			case CFG_PPI_DATALEN_16:
				x_count = conf->line_len / 2;
				break;
			case CFG_PPI_DATALEN_18:
			case CFG_PPI_DATALEN_24:
				/*
				 * When DLEN=18, the EPPI sign-extends or zero-fills the 18-bit data
				 * to 24 bits and packs four 24-bit words into three 32-bit words.
				 * When DLEN=24, the EPPI packs four 24-bit words into three 32-bit
				 * words.
				 */
				x_count = conf->line_len * 3 / 4;
				break;
			default:
				break;
			}
		} else
			x_count = conf->line_len;
#endif
	} else
		x_count = 0;

	spin_lock_irqsave(&dev->lock, flags);
	if (!conf->done) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return -EBUSY;
	}
	conf->done = 0;
	spin_unlock_irqrestore(&dev->lock, flags);

	conf->access_mode = PPI_READ;

	blackfin_dcache_invalidate_range((unsigned long)buf,
					 (unsigned long)buf + count);
	/*
	 * Configure DMA Controller
	 * WNR: memory write
	 * RESTART: flush DMA FIFO before beginning work unit
	 * DI_EN: generate interrupt on completion of work unit
	 * DMA2D: 2 dimensional buffer
	 */
	conf->dma_config = WNR | RESTART | DI_EN;

	switch (step_size) {
	case 4:
		conf->dma_config |= WDSIZE_32;
		conf->dma_config &= ~WDSIZE_16;
		break;
	case 2:
		conf->dma_config &= ~WDSIZE_32;
		conf->dma_config |= WDSIZE_16;
		break;
	case 1:
		conf->dma_config &= ~WDSIZE_32;
		conf->dma_config &= ~WDSIZE_16;
		break;
	default:
		break;
	}

	/*
	 * 1D or 2D DMA
	 */
	if (conf->dimensions == CFG_PPI_DIMS_2D) {	/* configure for 2D transfers */
		pr_debug("PPI read -- 2D data xcount = line_len = %hd, ycount = num_lines = %hd stepsize = %hd\n",
			conf->line_len, conf->num_lines, step_size);

		set_dma_x_count(dma, x_count);
		set_dma_x_modify(dma, step_size);
		set_dma_y_count(dma, conf->num_lines);
		set_dma_y_modify(dma, step_size);
		conf->dma_config |= DMA2D;
	} else {
		set_dma_x_count(dma, frame_size);

		pr_debug("PPI read -- 1D data count = %u\n", frame_size);
	}

	set_dma_config(dma, conf->dma_config);
	set_dma_start_addr(dma, (unsigned long)buf);
	set_dma_x_modify(dma, step_size);

	/* configure PPI registers to match DMA registers */
	dev->regs->ppi_control &= ~PORT_DIR;
	dev->regs->ppi_count = conf->line_len - PPI_COUNT_CORR_OFFSET;
	dev->regs->ppi_frame = conf->num_lines;
	dev->regs->ppi_delay = conf->delay;

	if (conf->timers)
		disable_timer_output(dev);

	enable_dma(dma);

	/* enable ppi */
	dev->regs->ppi_control |= PORT_EN;
	SSYNC();

	/* Wait for data available */
	if (1) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else {
			pr_debug("PPI wait_event_interruptible\n");
			ierr =
			    wait_event_interruptible(dev->waitq, conf->done);
			if (ierr) {
				/* waiting is broken by a signal */
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
 * in_filp  - Description of opened file.
 * in_count - how many bytes user wants to send.
 * out_buf  - where we get those sending data.
 *
 * RETURN
 * positive number: bytes sending out.
 * 0: There is no data send out or parameter error.
 * RETURN:
 * >0 The actual count sending out.
 * -EINVAL When word size is set to 16, writing odd bytes.
 *         When word size is bigger than 16, writing bytes which can't be divided exactly by 4
 *         For PPI PACKING mode, writing odd bytes
 *         For EPPI PAKCING mode, writing bytes which can't be divided exactly by 4
 * -EAGAIN When sending mode is set to non block and there is no tx buffer.
 *
 * GLOBAL VARIABLES REFERENCED: ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: It is invoked when user call 'write' system call
 *              to write to system.
 */
static ssize_t ppi_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	struct ppi_dev *dev = filp->private_data;
	struct ppi_config *conf = &dev->conf;
	unsigned int dma = dev->dma_chan;
	int ierr;
	unsigned int step_size, frame_size, x_count;
	unsigned long flags;

	pr_debug("ppi_write:\n");

#if defined(PPI_CONTROL) || defined(PPI0_CONTROL)
	if ((conf->data_len > CFG_PPI_DATALEN_8) || conf->pack_mode)
		step_size = 2;
	else
		step_size = 1;
#elif defined(EPPI0_CONTROL) || defined(EPPI1_CONTROL)
	if ((conf->data_len > CFG_PPI_DATALEN_16) || conf->pack_mode)
		step_size = 4;
	else if (conf->data_len > CFG_PPI_DATALEN_8)
		step_size = 2;
	else
		step_size = 1;
#endif

	if ((count & (step_size - 1)) | ((unsigned long)buf & (step_size - 1))) {
		pr_err("ppi_write: DMA buffer address or length not aligned with stepsize\n");
		return -EINVAL;
	}

	frame_size = count / step_size;

	/*
	 * calculate line len according to count, if it is not configured
	 */
	if (conf->dimensions == CFG_PPI_DIMS_2D) {	/* configure for 2D transfers */
		if ((conf->line_len == 0) || (conf->num_lines == 0)) {
			pr_err("ppi_write: Devices is not configured right, 0 row or 0 column in 2D mode\n");
			return -EINVAL;
		}
	} else if (conf->line_len == 0) {
#if defined(PPI_CONTROL) || defined(PPI0_CONTROL)
		if (conf->data_len > CFG_PPI_DATALEN_8)
			conf->line_len = count / step_size;
#elif defined(EPPI0_CONTROL) || defined(EPPI1_CONTROL)
		switch (conf->data_len) {
		case CFG_PPI_DATALEN_8:
			conf->line_len = count;
			break;
		case CFG_PPI_DATALEN_10:
		case CFG_PPI_DATALEN_12:
		case CFG_PPI_DATALEN_14:
		case CFG_PPI_DATALEN_16:
			conf->line_len = count / 2;
			break;
		case CFG_PPI_DATALEN_18:
		case CFG_PPI_DATALEN_24:
			if (conf->pack_mode)
				conf->line_len = count / 3;
			else
				conf->line_len = count / 4;
			break;
		}
#endif
	}

	if (ANOMALY_05000179)
		if (conf->line_len < 3)
			return -EFAULT;

	/*
	 * calculate DMA x_count according to line_len, PPI data length, and packing mode
	 */
	if (conf->dimensions == CFG_PPI_DIMS_2D) {
#if defined(PPI_CONTROL) || defined(PPI0_CONTROL)
		if (conf->pack_mode)
			x_count = conf->line_len / 2;
		else
			x_count = conf->line_len;
#elif defined(EPPI0_CONTROL) || defined(EPPI1_CONTROL)
		x_count = conf->line_len;
		if (conf->pack_mode) {
			switch (conf->data_len) {
			case CFG_PPI_DATALEN_8:
				x_count /= 4;
				break;
			case CFG_PPI_DATALEN_10:
			case CFG_PPI_DATALEN_12:
			case CFG_PPI_DATALEN_14:
			case CFG_PPI_DATALEN_16:
				x_count /= 2;
				break;
			case CFG_PPI_DATALEN_18:
			case CFG_PPI_DATALEN_24:
				/*
				 * When DLEN=18, the EPPI sign-extends or zero-fills the 18-bit data
				 * to 24 bits and packs four 24-bit words into three 32-bit words.
				 * When DLEN=24, the EPPI packs four 24-bit words into three 32-bit
				 * words.
				 */
				x_count = x_count * 3 / 4;
				break;
			}
		}
#endif
	} else
		x_count = 0;

	spin_lock_irqsave(&dev->lock, flags);
	if (!conf->done) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return -EBUSY;
	}
	conf->done = 0;
	spin_unlock_irqrestore(&dev->lock, flags);

	conf->access_mode = PPI_WRITE;

	blackfin_dcache_invalidate_range((unsigned long)buf,
					 (unsigned long)buf + count);

	/*
	 * Configure DMA Controller
	 * memory read
	 * RESTART: flush DMA FIFO before beginning work unit
	 * DI_EN: generate interrupt on completion of work unit
	 * DMA2D: 2 dimensional buffer
	 */
	conf->dma_config = RESTART | DI_EN;

	switch (step_size) {
	case 4:
		conf->dma_config |= WDSIZE_32;
		conf->dma_config &= ~WDSIZE_16;
		break;
	case 2:
		conf->dma_config &= ~WDSIZE_32;
		conf->dma_config |= WDSIZE_16;
		break;
	case 1:
		conf->dma_config &= ~WDSIZE_32;
		conf->dma_config &= ~WDSIZE_16;
		break;
	default:
		break;
	}

	if (conf->dimensions == CFG_PPI_DIMS_2D) {	/* configure for 2D transfers */
		pr_debug("PPI write -- 2D data line_len = %hd, num_lines = %hd\n",
			conf->line_len, conf->num_lines);
		set_dma_x_count(dma, x_count);
		set_dma_x_modify(dma, step_size);
		set_dma_y_count(dma, conf->num_lines);
		set_dma_y_modify(dma, step_size);
		conf->dma_config |= DMA2D;
	} else {
		pr_debug("PPI write -- 1D data count = %d\n", (int)count);
		set_dma_x_count(dma, frame_size);
		set_dma_x_modify(dma, step_size);
	}

	set_dma_start_addr(dma, (unsigned long)buf);
	pr_debug("dma_config = 0x%04X\n", conf->dma_config);
	set_dma_config(dma, conf->dma_config);

	/* configure PPI registers to match DMA registers */

	dev->regs->ppi_control |= PORT_DIR;
	dev->regs->ppi_count = conf->line_len - PPI_COUNT_CORR_OFFSET;
	dev->regs->ppi_frame = conf->num_lines;
	dev->regs->ppi_delay = conf->delay;

	enable_dma(dma);
	/* enable ppi */
	dev->regs->ppi_control |= PORT_EN;
	SSYNC();

	if (conf->timers)
		setup_timers(dev, frame_size);

	/* Wait for DMA to finish */

	if (1) {
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		} else {
			pr_debug("PPI wait_event_interruptible\n");
			ierr =
			    wait_event_interruptible(dev->waitq, conf->done);
			if (ierr) {
				/* waiting is broken by a signal */
				pr_debug
				    ("PPI wait_event_interruptible ierr = %d\n",
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
 * in_inode - Description of opened file.
 * in_filp  - Description of opened file.
 *
 * RETURN
 * 0: Open ok.
 * -ENXIO  No such device
 *
 * GLOBAL VARIABLES REFERENCED: ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: It is invoked when user call 'open' system call
 *  to open ppi device.
 */
static int ppi_open(struct inode *inode, struct file *filp)
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

	/* Request PPI DMA channel, and pass the interrupt handler */

	if (peripheral_request_list(dev->per_ppi0_7, KBUILD_MODNAME)) {
		pr_err("ppi_open: Requesting Peripherals failed\n");
		return -EBUSY;
	}

	if (request_dma(dev->dma_chan, "PPI_DMA") < 0) {
		pr_err("ppi_open: Unable to attach BlackFin PPI DMA channel\n");
		return -EFAULT;
	} else
		set_dma_callback(dev->dma_chan, ppi_irq, dev);

	if (request_irq(dev->irq_error, ppi_irq_error, 0,
			"PPI_ERROR", dev) < 0) {
		pr_err("ppi_open: Unable to attach BlackFin PPI Error Interrupt\n");
		return -EFAULT;
	}

	ppi_reg_reset(dev);
	pr_debug("ppi_open: return\n");

	return 0;
}

/*
 * FUNCTION NAME: ppi_release
 *
 * INPUTS/OUTPUTS:
 * in_inode - Description of opened file.
 * in_filp  - Description of opened file.
 *
 * RETURN
 * Always 0
 *
 * GLOBAL VARIABLES REFERENCED: ppiinfo
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: It is invoked when user call 'close' system call
 *  to close device.
 */
static int ppi_release(struct inode *inode, struct file *filp)
{
	struct ppi_dev *dev = filp->private_data;
	struct ppi_config *conf = &dev->conf;
	unsigned long flags;

	pr_debug("ppi_release: close()\n");

	spin_lock_irqsave(&dev->lock, flags);
	ppi_reg_reset(dev);
	conf->opened = 0;
	spin_unlock_irqrestore(&dev->lock, flags);

	/* After finish DMA, release it. */
	free_dma(dev->dma_chan);
	free_irq(dev->irq_error, dev);

	peripheral_free_list(dev->per_ppi0_7);

	if (conf->data_len > CFG_PPI_DATALEN_8)
		peripheral_free_list(&dev->per_ppi8_15[7 - dev->conf.data_len]);

	if (conf->fs23)
		peripheral_free_list(dev->per_ppifs);

	ppi_fasync(-1, filp, 0);

	pr_debug("ppi_release: close() return\n");
	return 0;
}

static const struct file_operations ppi_fops = {
	.owner = THIS_MODULE,
	.read = ppi_read,
	.write = ppi_write,
	.unlocked_ioctl = ppi_ioctl,
	.open = ppi_open,
	.release = ppi_release,
	.fasync = ppi_fasync,
};

static ssize_t
ppi_status_show(struct class *ppi_class, struct class_attribute *attr, char *buf)
{
	char *p;
	unsigned short i;
	p = buf;

	if (ppi_devices)
		for (i = 0; i < ppi_nr_devs; ++i)
			p += sprintf(p,
				"PPI%d:\n irq %d\n irq error %d\n dma chan %d\n open %d\n"
				" data_len %d\n fs23 %d\n timers %d\n trigger_edge %d\n"
				" dims=%d\n delay=%d\n mode=%d\n done=%d\n"
				" dma config 0x%X\n line_len %d\n lines number %d\n ppi control 0x%X\n",
				i,
				ppi_devices[i].irq,
				ppi_devices[i].irq_error,
				ppi_devices[i].dma_chan,
				ppi_devices[i].conf.opened,
				ppi_devices[i].conf.data_len,
				ppi_devices[i].conf.fs23,
				ppi_devices[i].conf.timers,
				ppi_devices[i].conf.trigger_edge,
				ppi_devices[i].conf.dimensions,
				ppi_devices[i].conf.delay,
				ppi_devices[i].conf.access_mode,
				ppi_devices[i].conf.done,
				ppi_devices[i].conf.dma_config,
				ppi_devices[i].conf.line_len,
				ppi_devices[i].conf.num_lines,
				ppi_devices[i].conf.ppi_control);

	return p - buf;
}

static struct class *ppi_class;
static CLASS_ATTR(status, S_IRUGO, &ppi_status_show, NULL);

static void ppi_setup_cdev(struct ppi_dev *dev, int index)
{
	int err, devno = MKDEV(ppi_major, ppi_minor + index);

	cdev_init(&dev->cdev, &ppi_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		pr_notice("ppi_setup_cdev: error %d adding ppi%d", err, index);
}

/*
 * FUNCTION NAME: ppi_cleanup_module
 *
 * INPUTS/OUTPUTS:
 *
 * RETURN:
 *
 * GLOBAL VARIABLES REFERENCED:
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: It will be invoked when using 'rmmod' command.
 *              or, you invoke it directly when it needs remove
 *              ppi module.
 */
static void /*__exit*/ ppi_cleanup_module(void)
{
	int i;
	dev_t devno = MKDEV(ppi_major, ppi_minor);

	if (ppi_devices) {
		for (i = 0; i < ppi_nr_devs; ++i)
			cdev_del(&ppi_devices[i].cdev);
		kfree(ppi_devices);
	}
	unregister_chrdev_region(devno, ppi_nr_devs);
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
 * GLOBAL VARIABLES REFERENCED:
 *
 * GLOBAL VARIABLES MODIFIED: NIL
 *
 * DESCRIPTION: It will be invoked when using 'insmod' command.
 *              or invoke it directly if ppi module is needed.
 */
static int __init ppi_init_module(void)
{
	int minor;
	int result, i;
	dev_t dev = 0;

	dev = MKDEV(ppi_major, ppi_minor);
	result = register_chrdev_region(dev, ppi_nr_devs, "ppi");
	if (result < 0) {
		pr_warning("ppi: can't get major %d\n", ppi_major);
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
	for (minor = 0; minor < ppi_nr_devs; minor++)
		device_create(ppi_class, NULL, MKDEV(ppi_major, minor),
			      NULL, "ppi%d", minor);

	/* Initialize each device. */
	for (i = 0; i < ppi_nr_devs; ++i) {
		ppi_setup_cdev(&ppi_devices[i], i);
		ppi_devices[i].ppi_num = i;
		spin_lock_init(&ppi_devices[i].lock);
		init_waitqueue_head(&ppi_devices[i].waitq);
	}

#if defined(PPI_CONTROL) || defined(PPI0_CONTROL)	/* PPI0 */
	ppi_devices[0].dma_chan = CH_PPI;
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

	/* PPI1 */
#ifdef PPI1_CONTROL
	ppi_devices[1].regs = (struct ppi_register *)PPI1_CONTROL;
	ppi_devices[1].dma_chan = CH_PPI1;
	ppi_devices[1].irq = IRQ_PPI1;
	ppi_devices[1].irq_error = IRQ_PPI2_ERROR;
	ppi_devices[1].fs1_timer_id = FS1_1_TIMER_ID;
	ppi_devices[1].fs2_timer_id = FS1_2_TIMER_ID;
	ppi_devices[1].fs1_timer_bit = FS1_1_TIMER_BIT;
	ppi_devices[1].fs2_timer_bit = FS1_2_TIMER_BIT;
	ppi_devices[1].per_ppi0_7 = per_req_ppi1_0_7;
	ppi_devices[1].per_ppi8_15 = per_req_ppi1_8_15;
	ppi_devices[1].per_ppifs = per_req_ppi1_fs;
#endif

#elif defined(EPPI0_CONTROL)	/* BF54x */
	ppi_devices[0].dma_chan = CH_EPPI0;
	ppi_devices[0].irq = IRQ_EPPI0;
	ppi_devices[0].irq_error = IRQ_EPPI0_ERROR;
	ppi_devices[0].regs = (struct ppi_register *)EPPI0_STATUS;
	ppi_devices[0].per_ppi0_7 = per_req_ppi0_7;
	ppi_devices[0].per_ppi8_15 = per_req_ppi8_15;
	ppi_devices[0].per_ppifs = per_req_ppi_fs;

	/* EPPI1 */
#ifdef EPPI1_CONTROL
	ppi_devices[1].regs = (struct ppi_register *)EPPI1_STATUS;
	ppi_devices[1].dma_chan = CH_EPPI1;
	ppi_devices[1].irq = IRQ_EPPI1;
	ppi_devices[1].irq_error = IRQ_EPPI1_ERROR;
	ppi_devices[1].per_ppi0_7 = per_req_ppi1_0_7;
	ppi_devices[1].per_ppi8_15 = per_req_ppi1_8_15;
	ppi_devices[1].per_ppifs = per_req_ppi1_fs;
#endif

#endif

	return 0;
}
module_init(ppi_init_module);

MODULE_AUTHOR("John DeHority");
MODULE_LICENSE("GPL");

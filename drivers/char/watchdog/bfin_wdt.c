/*
 * Blackfin On-Chip Watchdog Driver
 *  Supports BF53[123]/BF53[467]/BF561
 *
 * Originally based on softdog.c
 * Copyright 2006-2007 Analog Devices Inc.
 * Copyright 2006-2007 Michele d'Amico
 * Copyright 1996 Alan Cox <alan@redhat.com>
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

/* Notes for people customizing General Purpose Interrupt (GPI) behavior.
 *
 * Normally when the watchdog times out, you reboot the system.  In
 * some scenarios, you would rather have the machine come to a complete
 * halt.  Or perhaps you want to wake the system up from sleep.  If the
 * board provides some watchdog functions, we'll call those.  Otherwise,
 * we'll let the GPI event wake up the processor if it's put to sleep.
 *
 * The functions you should implement in your board file:
 *
 *	irqreturn_t bfin_board_watchdog_interrupt(void);
 *		called when the interrupt is fired.  perhaps you want to execute
 *		kernel_halt() or do some other crazy stuff.
 *
 *	int bfin_board_watchdog_suspend(void);
 *		called when suspending this device.  default behavior is to either
 *		mark watchdog as a wakeup source (GPI) or turn it off (RESET).
 *
 *	int bfin_board_watchdog_resume(void);
 *		called when resuming this device.  default behavior is to either
 *		unmark watchdog as a wakeup source (GPI) or turn it on (RESET).
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/blackfin.h>
#include <asm/uaccess.h>

#define stamp(fmt, args...) pr_debug("%s:%i: " fmt "\n", __func__, __LINE__, ## args)
#define stampit() stamp("here i am")

#define WATCHDOG_NAME "bfin-wdt"
#define PFX WATCHDOG_NAME ": "

/* The BF561 has two watchdogs (one per core), but since Linux
 * only runs on core A, we'll just work with that one.
 */
#ifdef BF561_FAMILY
# define bfin_read_WDOG_CTL()    bfin_read_WDOGA_CTL()
# define bfin_read_WDOG_CNT()    bfin_read_WDOGA_CNT()
# define bfin_read_WDOG_STAT()   bfin_read_WDOGA_STAT()
# define bfin_write_WDOG_CTL(x)  bfin_write_WDOGA_CTL(x)
# define bfin_write_WDOG_CNT(x)  bfin_write_WDOGA_CNT(x)
# define bfin_write_WDOG_STAT(x) bfin_write_WDOGA_STAT(x)
#endif

/* Bit in SWRST that indicates boot caused by watchdog */
#define SWRST_RESET_WDOG 0x4000

/* Bit in WDOG_CTL that indicates watchdog has expired (WDR0) */
#define WDOG_EXPIRED 0x8000

/* Masks for WDEV field in WDOG_CTL register */
#define ICTL_RESET   0x0
#define ICTL_NMI     0x2
#define ICTL_GPI     0x4
#define ICTL_NONE    0x6
#define ICTL_MASK    0x6

/* Masks for WDEN field in WDOG_CTL register */
#define WDEN_MASK    0x0FF0
#define WDEN_ENABLE  0x0000
#define WDEN_DISABLE 0x0AD0

/* some defaults */
#define WATCHDOG_TIMEOUT 20
#define WATCHDOG_ACTION 0 /* 0 = ICTL_RESET */

static unsigned int timeout = WATCHDOG_TIMEOUT;
static int nowayout = WATCHDOG_NOWAYOUT;
static unsigned int action = WATCHDOG_ACTION;
static struct watchdog_info bfin_wdt_info;
static unsigned long open_check;
static char expect_close, bfin_wdt_expired = 0;
static spinlock_t bfin_wdt_spinlock = SPIN_LOCK_UNLOCKED;

/**
 *	bfin_wdt_keepalive - Keep the Userspace Watchdog Alive
 *
 * 	The Userspace watchdog got a KeepAlive: schedule the next timeout.
 */
static int bfin_wdt_keepalive(void)
{
	stampit();
	bfin_write_WDOG_STAT(0);
	return 0;
}

/**
 *	bfin_wdt_stop - Stop the Watchdog
 *
 *	Stops the on-chip watchdog.
 */
static int bfin_wdt_stop(void)
{
	stampit();
	bfin_write_WDOG_CTL((bfin_read_WDOG_CTL() & ~WDEN_MASK) | WDEN_DISABLE);
	return 0;
}

/**
 *	bfin_wdt_start - Start the Watchdog
 *
 *	Starts the on-chip watchdog.  Automatically loads WDOG_CNT
 *	into WDOG_STAT for us.
 */
static int bfin_wdt_start(void)
{
	stampit();
	bfin_write_WDOG_CTL((bfin_read_WDOG_CTL() & ~WDEN_MASK) | WDEN_ENABLE);
	return 0;
}

/**
 *	bfin_wdt_running - Check Watchdog status
 *
 *	See if the watchdog is running.
 */
static int bfin_wdt_running(void)
{
	return ((bfin_read_WDOG_CTL() & WDEN_MASK) != WDEN_DISABLE);
}

/**
 *	bfin_wdt_set_timeout - Set the Userspace Watchdog timeout
 *	@t: new timeout value (in seconds)
 *
 *	Translate the specified timeout in seconds into System Clock
 *	terms which is what the on-chip Watchdog requires.
 */
static int bfin_wdt_set_timeout(unsigned long t)
{
	u32 cnt;
	unsigned long flags;

	stampit();

	cnt = t * get_sclk();
	if (cnt < get_sclk()) {
		printk(KERN_WARNING PFX "timeout value is too large\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&bfin_wdt_spinlock, flags);
	{
		int run = bfin_wdt_running();
		bfin_wdt_stop();
		bfin_write_WDOG_CNT(cnt);
		if (run) bfin_wdt_start();
	}
	spin_unlock_irqrestore(&bfin_wdt_spinlock, flags);

	timeout = t;

	return 0;
}

/**
 *	bfin_wdt_interrupt - General Purpose Watchdog Interrupt Handler
 *	@irq: irq # that triggered us
 *	@dev_id: this device instance
 *
 *	Call board-specific ISR if it exists, otherwise we need to stop
 *	and start the watchdog -- simply forcing a reload of the count
 *	will not reset the interrupt status.
 *
 *	Note: While we could interrogate the WDR0 bit in WDOG_CTL, that
 *	      requires us to turn off/on the watchdog to clear it.  So
 *	      we'll just suck it up and mimic the bit with bfin_wdt_expired.
 */
extern irqreturn_t bfin_board_watchdog_interrupt(void) __attribute__((weak));
static irqreturn_t bfin_wdt_interrupt(int irq, void *dev_id)
{
	bfin_wdt_expired = 1;
	if (bfin_board_watchdog_interrupt) {
		return bfin_board_watchdog_interrupt();
	} else {
		bfin_wdt_stop();
		bfin_wdt_keepalive();
		bfin_wdt_start();
		return IRQ_HANDLED;
	}
}

/**
 *	bfin_wdt_open - Open the Device
 *	@inode: inode of device
 *	@file: file handle of device
 *
 *	Watchdog device is opened and started.
 */
static int bfin_wdt_open(struct inode *inode, struct file *file)
{
	stampit();

	if (test_and_set_bit(0, &open_check))
		return -EBUSY;

	if (nowayout)
		__module_get(THIS_MODULE);

	bfin_wdt_keepalive();
	bfin_wdt_start();

	return nonseekable_open(inode, file);
}

/**
 *	bfin_wdt_close - Close the Device
 *	@inode: inode of device
 *	@file: file handle of device
 *
 *	Watchdog device is closed and stopped.
 */
static int bfin_wdt_release(struct inode *inode, struct file *file)
{
	stampit();

	if (expect_close == 42) {
		bfin_wdt_stop();
	} else {
		printk(KERN_CRIT PFX "Unexpected close, not stopping watchdog!\n");
		bfin_wdt_keepalive();
	}

	expect_close = 0;
	clear_bit(0, &open_check);

	return 0;
}

/**
 *	bfin_wdt_write - Write to Device
 *	@file: file handle of device
 *	@buf: buffer to write
 *	@count: length of buffer
 *	@ppos: offset
 *
 *	Pings the watchdog on write.
 */
static ssize_t bfin_wdt_write(struct file *file, const char __user *data,
                              size_t len, loff_t *ppos)
{
	stampit();

	if (len) {
		if (!nowayout) {
			size_t i;

			/* In case it was set long ago */
			expect_close = 0;

			for (i = 0; i != len; i++) {
				char c;
				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					expect_close = 42;
			}
		}
		bfin_wdt_keepalive();
	}

	return len;
}

/**
 *	bfin_wdt_ioctl - Query Device
 *	@inode: inode of device
 *	@file: file handle of device
 *	@cmd: watchdog command
 *	@arg: argument
 *
 *	Query basic information from the device or ping it, as outlined by the
 *	watchdog API.
 */
static int bfin_wdt_ioctl(struct inode *inode, struct file *file,
                          unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;

	stampit();

	switch (cmd) {
		default:
			return -ENOTTY;

		case WDIOC_GETSUPPORT:
			if (copy_to_user(argp, &bfin_wdt_info, sizeof(bfin_wdt_info))
				return -EFAULT;
			else
				return 0;

		case WDIOC_GETSTATUS: {
			int ret = bfin_wdt_expired;
			bfin_wdt_expired = 0;
			return put_user(ret, p);
		}

		case WDIOC_GETBOOTSTATUS:
			return put_user(!!(_bfin_swrst & SWRST_RESET_WDOG), p);

		case WDIOC_KEEPALIVE:
			bfin_wdt_keepalive();
			return 0;

		case WDIOC_SETTIMEOUT: {
			int new_timeout;

			if (get_user(new_timeout, p))
				return -EFAULT;

			if (bfin_wdt_set_timeout(new_timeout))
				return -EINVAL;
		}
			/* Fall */
		case WDIOC_GETTIMEOUT:
			return put_user(timeout, p);

		case WDIOC_SETOPTIONS: {
			unsigned long flags;
			int options, ret = -EINVAL;

			if (get_user(options, p))
				return -EFAULT;

			spin_lock_irqsave(&bfin_wdt_spinlock, flags);

			if (options & WDIOS_DISABLECARD) {
				bfin_wdt_stop();
				ret = 0;
			}

			if (options & WDIOS_ENABLECARD) {
				bfin_wdt_start();
				ret = 0;
			}

			spin_unlock_irqrestore(&bfin_wdt_spinlock, flags);

			return ret;
		}
	}
}

/**
 *	bfin_wdt_notify_sys - Notifier Handler
 *	@this: notifier block
 *	@code: notifier event
 *	@unused: unused
 *
 *	Handles specific events, such as turning off the watchdog during a
 *	shutdown event.
 */
static int bfin_wdt_notify_sys(struct notifier_block *this, unsigned long code,
                               void *unused)
{
	stampit();

	if (code == SYS_DOWN || code == SYS_HALT)
		bfin_wdt_stop();

	return NOTIFY_DONE;
}

/**
 *	bfin_wdt_probe - Init per-device settings
 *	@pdev: device being probed
 *
 *	If we are in GPI mode, grab the interrupt.
 */
static int __devinit bfin_wdt_probe(struct platform_device *pdev)
{
	int ret;

	stampit();

	if (action == ICTL_GPI) {
		ret = request_irq(IRQ_WATCH, bfin_wdt_interrupt,
		                  IRQF_DISABLED, WATCHDOG_NAME, pdev);
		if (ret) {
			printk(KERN_ERR PFX "unable to allocate watchdog IRQ %i (err=%d)\n",
			       IRQ_WATCH, ret);
			clear_bit(0, &open_check);
			return ret;
		}

		device_init_wakeup(&pdev->dev, 0);
	}

	return 0;
}

/**
 *	bfin_wdt_remove - Free per-device settings
 *	@pdev: device being removed
 *
 *	If we were in GPI mode, free the interrupt.
 */
static int __devexit bfin_wdt_remove(struct platform_device *pdev)
{
	stampit();

	if (action == ICTL_GPI) {
		device_init_wakeup(&pdev->dev, 0);
		free_irq(IRQ_WATCH, pdev);
	}

	return 0;
}

#ifdef CONFIG_PM
static int bfin_wdt_pm_state;

/**
 *	bfin_wdt_suspend - suspend the watchdog
 *	@pdev: device being suspended
 *	@state: requested suspend state
 *
 *	Remember if the watchdog was running and stop it.
 *	TODO: is this even right?  Doesn't seem to be any
 *	      standard in the watchdog world ...
 */
extern int bfin_board_watchdog_suspend(void) __attribute__((weak));
static int bfin_wdt_suspend(struct platform_device *pdev, pm_message_t state)
{
	stampit();

	if (bfin_board_watchdog_suspend) {
		return bfin_board_watchdog_suspend();
	} else if (action == ICTL_GPI) {
		if (device_may_wakeup(&pdev->dev))
			enable_irq_wake(IRQ_WATCH);
	} else {
		bfin_wdt_pm_state = bfin_wdt_running();
		bfin_wdt_stop();
	}

	return 0;
}

/**
 *	bfin_wdt_resume - resume the watchdog
 *	@pdev: device being resumed
 *
 *	If the watchdog was running, turn it back on.
 */
extern int bfin_board_watchdog_resume(void) __attribute__((weak));
static int bfin_wdt_resume(struct platform_device *pdev)
{
	stampit();

	if (bfin_board_watchdog_resume) {
		return bfin_board_watchdog_resume();
	} else if (action == ICTL_GPI) {
		if (device_may_wakeup(&pdev->dev))
			disable_irq_wake(IRQ_WATCH);
	} else if (bfin_wdt_pm_state) {
		bfin_wdt_set_timeout(timeout);
		bfin_wdt_start();
	}

	return 0;
}
#else
# define bfin_wdt_suspend NULL
# define bfin_wdt_resume NULL
#endif

static struct platform_device bfin_wdt_device = {
	.name          = WATCHDOG_NAME,
	.id            = -1,
};

static struct platform_driver bfin_wdt_driver = {
	.driver    = {
		.name  = WATCHDOG_NAME,
		.owner = THIS_MODULE,
	},
	.probe     = bfin_wdt_probe,
	.remove    = __devexit_p(bfin_wdt_remove),
	.suspend   = bfin_wdt_suspend,
	.resume    = bfin_wdt_resume,
};

static struct file_operations bfin_wdt_fops = {
	.owner    = THIS_MODULE,
	.llseek   = no_llseek,
	.write    = bfin_wdt_write,
	.ioctl    = bfin_wdt_ioctl,
	.open     = bfin_wdt_open,
	.release  = bfin_wdt_release,
};

static struct miscdevice bfin_wdt_miscdev = {
	.minor    = WATCHDOG_MINOR,
	.name     = WATCHDOG_NAME,
	.fops     = &bfin_wdt_fops,
};

static struct watchdog_info bfin_wdt_info = {
	.identity = "Blackfin Watchdog",
	.options  = WDIOF_SETTIMEOUT |
	            WDIOF_KEEPALIVEPING |
	            WDIOF_MAGICCLOSE,
};

static struct notifier_block bfin_wdt_notifier = {
	.notifier_call = bfin_wdt_notify_sys,
};

/**
 *	bfin_wdt_init - Initialize module
 *
 *	Registers the device and notifier handler. Actual device
 *	initialization is handled by bfin_wdt_open().
 */
static int __init bfin_wdt_init(void)
{
	const u16 code2action[] = { ICTL_RESET, ICTL_NMI, ICTL_GPI, ICTL_NONE };
	int ret;

	stampit();

	/* Check that the timeout value is within range */
	if (bfin_wdt_set_timeout(timeout))
		return -EINVAL;

	/* Check that the action value is within range */
	if (action >= ARRAY_SIZE(code2action)) {
		printk(KERN_ERR PFX "invalid action, must be: 0 (reboot), 1 (NMI), 2 (GPI), 3 (none)\n");
		return -EINVAL;
	} else {
		action = code2action[action];
		bfin_write_WDOG_CTL((bfin_read_WDOG_CTL() & ~ICTL_MASK) | action);
	}

	/* Since this is an on-chip device and needs no board-specific
	 * resources, we'll handle all the platform device stuff here.
	 */
	ret = platform_device_register(&bfin_wdt_device);
	if (ret)
		return ret;

	ret = platform_driver_probe(&bfin_wdt_driver, NULL);
	if (ret)
		return ret;

	ret = register_reboot_notifier(&bfin_wdt_notifier);
	if (ret) {
		printk(KERN_ERR PFX "cannot register reboot notifier (err=%d)\n", ret);
		return ret;
	}

	ret = misc_register(&bfin_wdt_miscdev);
	if (ret) {
		printk(KERN_ERR PFX "cannot register miscdev on minor=%d (err=%d)\n",
		       WATCHDOG_MINOR, ret);
		unregister_reboot_notifier(&bfin_wdt_notifier);
		return ret;
	}

	printk(KERN_INFO PFX "initialized: action=%d timeout=%d sec (nowayout=%d)\n",
	       action, timeout, nowayout);

	return 0;
}

/**
 *	bfin_wdt_exit - Deinitialize module
 *
 *	Unregisters the device and notifier handler. Actual device
 *	deinitialization is handled by bfin_wdt_close().
 */
static void __exit bfin_wdt_exit(void)
{
	misc_deregister(&bfin_wdt_miscdev);
	unregister_reboot_notifier(&bfin_wdt_notifier);
}

module_init(bfin_wdt_init);
module_exit(bfin_wdt_exit);

MODULE_AUTHOR("Michele d'Amico, Mike Frysinger <vapier@gentoo.org>");
MODULE_DESCRIPTION("Blackfin Watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);

module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds. (1<=timeout<=((2^32)/SCLK), default=" __MODULE_STRING(WATCHDOG_TIMEOUT) ")");

module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

module_param(action, uint, 0);
MODULE_PARM_DESC(action, "Watchdog timeout action: 0 for reboot, 1 for NMI, 2 for General Purpose Interrupt, 3 for none (default=" __MODULE_STRING(WATCHDOG_ACTION) ")");

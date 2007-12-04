/*
 * File:         drivers/char/bfin_pflags.c
 * Based on:
 * Author:       Bas Vermeulen, Luuk van Dijk <lvd@mndmttr.nl>
 *
 * Created:      Tue Apr 20 10:53:12 CEST 2004
 * Description:  pfbits driver for bf53x
 *
 * Rev:          $Id$
 *
 * Modified:
 *               Copyright (C) 2004 Luuk van Dijk/BuyWays B.V.
 *               Copyright 2004-2006 Analog Devices Inc.
 * Jan 10, 2005  Changed Michael Hennerich
 * Apr 20, 2005  Changed added PROC entry Michael Hennerich
 * Oct 26, 2006  Tidy up exit, dynamic major  Phil Wilshire
 * Apr 19, 2007  Add /sys/class/pflag, make PROC obsolete -- Jean-Christian de Rivaz
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

/*
   BF533 STAMP Board Connections are made as follows:
   PF2 -> GUI_LED1
   PF3 -> GUI_LED2
   PF4 -> GUI_LED3
   GUI_BUT1 -> PF5
   GUI_BUT2 -> PF6
   LAN_IRQ -> PF7
   GUI_BUT3 -> PF8

   BF537 STAMP Board Connections are made as follows:
   PF6 -> GUI_LED1
   PF7 -> GUI_LED2
   PF8 -> GUI_LED3
   PF9 -> GUI_LED4
   PF10-> GUI_LED5
   PF11-> GUI_LED6
   GUI_BUT1 -> PF2
   GUI_BUT2 -> PF3
   GUI_BUT3 -> PF4
   GUI_BUT4 -> PF5
 */

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <asm/blackfin.h>
#include <asm/gpio.h>
#if defined(CONFIG_BF5xx_PFLAGS_PROC)
#include <linux/proc_fs.h>
#endif
#include <linux/spinlock.h>
#include "bfin_pflags.h"

static int major = 0;
module_param(major, int, 0444);

static DEFINE_SPINLOCK(pflags_lock);


/* return the minor number or -ENODEV */

static int check_minor(struct inode *inode)
{
	int minor = MINOR(inode->i_rdev);

	if (minor > MAX_BLACKFIN_GPIOS)
		return -ENODEV;

	return minor;
}

/***********************************************************
*
* FUNCTION NAME :pflags_open
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
* GLOBAL VARIABLES REFERENCED:
*
* GLOBAL VARIABLES MODIFIED: NIL
*
* DESCRIPTION: It is invoked when user call 'open' system call
*              to open spi device.
*
* CAUTION:
*************************************************************
* MODIFICATION HISTORY :
**************************************************************/
static int pflags_open(struct inode *inode, struct file *filp)
{
	unsigned long flags;

	int minor = MINOR(inode->i_rdev);

	if (check_minor(inode) < 0)
		return -ENODEV;

	spin_lock_irqsave(&pflags_lock, flags);

	if(gpio_request(minor, NULL)){
		spin_unlock_irqrestore(&pflags_lock, flags);
		return -EBUSY;
	}

	spin_unlock_irqrestore(&pflags_lock, flags);

	return 0;
}

static int pflags_release(struct inode *inode, struct file *filp)
{
	unsigned long flags;

	int minor = MINOR(inode->i_rdev);

	if (check_minor(inode) < 0)
		return -ENODEV;

	spin_lock_irqsave(&pflags_lock, flags);

	gpio_free(minor);

	spin_unlock_irqrestore(&pflags_lock, flags);

	return 0;
}

/***********************************************************
*
* FUNCTION NAME :pflags_read
*
* INPUTS/OUTPUTS:
* in_filp - Description of openned file.
* in_count - how many bytes user wants to get.
* out_buf - data would be write to this address.
*
* RETURN
* positive number: bytes read back
* -ENODEV When minor not available.
* -EMSGSIZE When size more than a single ASCII digit followed by /n.
*
* FUNCTION(S) CALLED:
*
* GLOBAL VARIABLES REFERENCED:
*
* GLOBAL VARIABLES MODIFIED: NIL
*
* DESCRIPTION: It is invoked when user call 'read' system call
*              to read from system.
*
* CAUTION:
*************************************************************
* MODIFICATION HISTORY :
**************************************************************/
static ssize_t pflags_read(struct file *filp, char *buf, size_t size, loff_t * offp)
{
	const char *bit;
	int minor = check_minor(filp->f_dentry->d_inode);

	pr_debug("pfbits driver for bf53x minor = %d\n", minor);

	if (minor < 0)
		return -ENODEV;

	if (size < 2)
		return -EMSGSIZE;

	gpio_direction_input(minor);

	bit = gpio_get_value(minor) ? "1" : "0";

	return (copy_to_user(buf, bit, 2)) ? -EFAULT : 2;
}

/***********************************************************
*
* FUNCTION NAME :pflags_write
*
* INPUTS/OUTPUTS:
* in_filp - Description of openned file.
* in_count - how many bytes user wants to send.
* out_buf - where we get those sending data.
*
* RETURN
* positive number: bytes sending out.
* 0: There is no data send out or parameter error.
* RETURN
* positive number: bytes read back
* -ENODEV When minor not available.
* -EMSGSIZE When size more than a single ASCII digit followed by /n.
*
* FUNCTION(S) CALLED:
*
* GLOBAL VARIABLES REFERENCED:
*
* GLOBAL VARIABLES MODIFIED: NIL
*
* DESCRIPTION: It is invoked when user call 'Write' system call
*              to write from system.
*
* CAUTION:
*************************************************************
* MODIFICATION HISTORY :
**************************************************************/
static ssize_t pflags_write(struct file *filp, const char *buf, size_t size, loff_t * offp)
{
	int minor = check_minor(filp->f_dentry->d_inode);

	pr_debug("pfbits driver for bf53x minor = %d\n", minor);

	if (minor < 0)
		return -ENODEV;

	if (size < 2)
		return -EMSGSIZE;

	if (!buf)
		return -EFAULT;

	gpio_direction_output(minor);

	pr_debug("pfbits driver: pflag_write = %c\n", buf[0]);

		switch (*buf) {
		case '0':
			gpio_set_value(minor, 0);
			break;
		case '1':
			gpio_set_value(minor, 1);
			break;
		case 'T':
			set_gpio_toggle(minor);
			break;
		default:
			return -EINVAL;
		}

	return size;

}

/*
 *  Info exported via "/sys/class/pflag/status".
 */
static ssize_t pflag_status_show(struct class *class, char *buf)
{
	char *p;
	unsigned short i;
	p = buf;


	p += sprintf(p, "PIN\t:DATA DIR INEN EDGE BOTH POLAR MASKA MASKB\n");
	p += sprintf(p, "(1/0)\t:H/L  0/I E/D  E/L  B/S   L/H   S/C   S/C\n");

	for (i = 0; i < MAX_BLACKFIN_GPIOS; i++)
		p += sprintf(p,
			     "PF%d\t: %d....%d....%d....%d....%d....%d.....%d.....%d \n",
			     i, get_gpio_data(i), get_gpio_dir(i),
			     get_gpio_inen(i), get_gpio_edge(i),
			     get_gpio_both(i), get_gpio_polar(i),
			     get_gpio_maska(i), get_gpio_maskb(i));
	return p - buf;
}

#if defined(CONFIG_BF5xx_PFLAGS_PROC)
/*
 *  Info exported via "/proc/driver/pflags".
 *  For backward compatibility only.
 *  Use "/sys/class/pflag/status" instead.
 */
static int pflags_read_proc(char *page, char **start, off_t off,
		 int count, int *eof, void *data)
{
	int len = pflag_status_show(NULL, page);
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}
#endif

/***********************************************************
*
* FUNCTION NAME :pflags_ioctl
*
* INPUTS/OUTPUTS:
* in_inode - Description of openned file.
* in_filp - Description of openned file.
* in_cmd - Command passed into ioctl system call.
* in/out_arg - It is parameters which is specified by last command
*
* RETURN:
* 0 OK
* -EINVAL
*
* FUNCTION(S) CALLED:
*
* GLOBAL VARIABLES REFERENCED:
*
* GLOBAL VARIABLES MODIFIED: NIL
*
* DESCRIPTION:
*
* CAUTION:
*************************************************************
* MODIFICATION HISTORY :
**************************************************************/
static int pflags_ioctl(struct inode *inode, struct file *filp, uint cmd,
	     unsigned long arg)
{
	int minor = check_minor(filp->f_dentry->d_inode);

	if (minor < 0)
		return -ENODEV;

	pr_debug("%s: minor = %d\n",__FUNCTION__, minor);

	switch (cmd) {
	case SET_FIO_DIR:
		{
			pr_debug("%s: SET_FIO_DIR arg = %d\n",__FUNCTION__, (int)arg);
			set_gpio_dir(minor, arg);
			break;
		}
	case SET_FIO_POLAR:
		{
			pr_debug("%s: SET_FIO_POLAR arg = %d\n",__FUNCTION__, (int)arg);
			set_gpio_polar(minor, arg);
			break;
		}
	case SET_FIO_EDGE:
		{
			pr_debug("%s: SET_FIO_EDGE arg = %d\n",__FUNCTION__, (int)arg);
			set_gpio_edge(minor, arg);
			break;
		}
	case SET_FIO_BOTH:
		{
			pr_debug("%s: SET_FIO_BOTH arg = %d\n",__FUNCTION__, (int)arg);
			set_gpio_both(minor, arg);
			break;
		}
	case SET_FIO_INEN:
		{
			pr_debug("%s: SET_FIO_INEN arg = %d\n",__FUNCTION__, (int)arg);
			set_gpio_inen(minor, arg);
			break;
		}
	default:
		return -EINVAL;
	}

	return 0;
}


static struct file_operations pflags_fops = {
      .read    = pflags_read,
      .write   = pflags_write,
      .ioctl   = pflags_ioctl,
      .open    = pflags_open,
      .release = pflags_release,
};

static struct class *pflag_class;

static CLASS_ATTR(status, S_IRUGO, &pflag_status_show, NULL);

static int __init blackfin_pflags_init(void)
{
	int minor;
	char minor_name[8];
        int res;

	printk(KERN_INFO "pfx: pfbits driver for bf5xx\n");

	res = register_chrdev(major, "pflag", &pflags_fops);
        if ( res < 0 ) {
		printk(KERN_WARNING 
		       "unable to register major %d \n", major);
		return -EIO;
	}
        major = res;

#if defined(CONFIG_BF5xx_PFLAGS_PROC)
	if(!create_proc_read_entry("driver/pflags", 0, 0, pflags_read_proc, NULL)) {
		printk(KERN_INFO "pflag: can't create proc entry!\n");
		goto release_chrdev;
	}
#endif
        pflag_class = class_create(THIS_MODULE, "pflag");
	if (class_create_file(pflag_class, &class_attr_status)) {
		goto release_proc;
	}
	for (minor = 0; minor < MAX_BLACKFIN_GPIOS; minor++) {
		sprintf(minor_name, "pf%d", minor);
		device_create(pflag_class, NULL,
			      MKDEV(major, minor), minor_name);
	}
	

	return 0;
release_proc:
#if defined(CONFIG_BF5xx_PFLAGS_PROC)
	remove_proc_entry("driver/pflags", NULL);
release_chrdev:
#endif
	unregister_chrdev(major, "pflag");
exit:
	return -ENOMEM;
}

void __exit blackfin_pflags_exit(void)
{
#if defined(CONFIG_BF5xx_PFLAGS_PROC)
	remove_proc_entry("driver/pflags", NULL);
#endif
	unregister_chrdev(major, "pflag");
}

module_init(blackfin_pflags_init);
module_exit(blackfin_pflags_exit);

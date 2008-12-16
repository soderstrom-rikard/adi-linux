/*
 * arch/blackfin/kernel/kgdb_test.c - Blackfin kgdb tests
 *
 * Copyright 2005-2008 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>

#include <asm/current.h>
#include <asm/uaccess.h>
#include <asm/system.h>

#include <asm/blackfin.h>

static char cmdline[256];
static unsigned long len;

int kgdb_test(char *name, int len, int count, int z)
{
	printk(KERN_DEBUG "kgdb name(%d): %s, %d, %d\n", len, name, count, z);
	count = z;
	return count;
}

static int test_proc_output(char *buf)
{
	kgdb_test("hello world!", 12, 0x55, 0x10);
	return 0;
}

static int test_read_proc(char *page, char **start, off_t off,
                          int count, int *eof, void *data)
{
	int len;

	len = test_proc_output(page);
	if (len <= off+count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}

static int test_write_proc(struct file *file, const char *buffer,
                           unsigned long count, void *data)
{
	if (count >= 256)
		len = 255;
	else
		len = count;

	memcpy(cmdline, buffer, count);
	cmdline[len] = 0;

	return len;
}

static int __init cplbtest_init(void)
{
	struct proc_dir_entry *entry;

	entry = create_proc_entry("kgdbtest", 0, NULL);
	if (entry == NULL)
		return -ENOMEM;

	entry->read_proc = test_read_proc;
	entry->write_proc = test_write_proc;
	entry->data = NULL;

	return 0;
}

static void __exit cplbtest_exit(void)
{
	remove_proc_entry("kgdbtest", NULL);
}

module_init(cplbtest_init);
module_exit(cplbtest_exit);
MODULE_LICENSE("GPL");

#ifndef _BFIN_KPAD_H
#define _BFIN_KPAD_H

#include <linux/input.h>

struct bfin_kpad_platform_data {
	int rows;
	int cols;
	int *keymap;
	unsigned int keymapsize;
	u32 debounce_time;	/* in ns */
	u32 coldrive_time;	/* in ns */
	u32 keyup_test_interval; /* in ms */
};

#define KEYVAL(col, row, val) (((1 << col) << 24) | ((1 << row) << 16) | (val))

#endif

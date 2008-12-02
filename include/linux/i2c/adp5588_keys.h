/* Put one of these structures in i2c_board_info platform_data */

#ifndef _ADP5588_KPAD_H
#define _ADP5588_KPAD_H

#define ADP5588_KEYMAPSIZE	80

struct adp5588_kpad_platform_data {
	int rows;			/* Number of rows */
	int cols;			/* Number of columns */
	const unsigned short *keymap;	/* Pointer to keymap */
	unsigned short keymapsize;	/* Keymap size */
	unsigned repeat:1;		/* Enable key repeat */
	unsigned en_keylock:1;		/* Enable Key Lock feature */
	unsigned short unlock_key1;	/* Unlock Key 1 */
	unsigned short unlock_key2;	/* Unlock Key 2 */
};

#endif

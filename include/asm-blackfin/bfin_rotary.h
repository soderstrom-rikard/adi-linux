#ifndef _BFIN_ROTARY_H
#define _BFIN_ROTARY_H

#define ROT_QUAD_ENC	0x0	/* quadrature / grey code encoder mode */
#define ROT_BIN_ENC	0x100	/* binary encoder mode */
#define ROT_UD_CNT	0x200	/* rotary counter mode */
#define ROT_DIR_CNT	0x400	/* direction counter mode */

#define ROT_DEBE	0x2	/* Debounce Enable */

#define ROT_CDGINV	0x10	/* CDG Pin Polarity Invert */
#define ROT_CUDINV	0x20	/* CUD Pin Polarity Invert */
#define ROT_CZMINV	0x40	/* CZM Pin Polarity Invert */

struct bfin_rotary_platform_data {
	unsigned int rotary_up_key;
	unsigned int rotary_down_key;
	unsigned int rotary_button_key;
	unsigned int rotary_rel_code;
	unsigned short debounce;	/* 0..17 */
	unsigned short mode;
};
#endif

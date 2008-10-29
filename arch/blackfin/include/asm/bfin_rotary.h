#ifndef _BFIN_ROTARY_H
#define _BFIN_ROTARY_H

#define ROT_QUAD_ENC	CNTMODE_QUADENC	/* quadrature / grey code encoder mode */
#define ROT_BIN_ENC	CNTMODE_BINENC	/* binary encoder mode */
#define ROT_UD_CNT	CNTMODE_UDCNT	/* rotary counter mode */
#define ROT_DIR_CNT	CNTMODE_DIRCNT	/* direction counter mode */

#define ROT_DEBE	DEBE	/* Debounce Enable */

#define ROT_CDGINV	CDGINV	/* CDG Pin Polarity Invert */
#define ROT_CUDINV	CUDINV	/* CUD Pin Polarity Invert */
#define ROT_CZMINV	CZMINV	/* CZM Pin Polarity Invert */

struct bfin_rotary_platform_data {
	unsigned int rotary_up_key;
	unsigned int rotary_down_key;
	unsigned int rotary_button_key;
	unsigned int rotary_rel_code;
	unsigned short debounce;	/* 0..17 */
	unsigned short mode;
};
#endif

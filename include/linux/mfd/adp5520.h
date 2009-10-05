/*
 * Definitions and platfrom data for Analog Devices
 * ADP5520/ADP5501 MFD PMICs (Backlight, LED, GPIO and Keys)
 *
 * Copyright 2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */


#ifndef __LINUX_MFD_ADP5520_H
#define __LINUX_MFD_ADP5520_H

#define ID_ADP5520		5520
#define ID_ADP5501		5501

/*
 * ADP5520/ADP5501 Register Map
 */

#define MODE_STATUS 		0x00
#define INTERRUPT_ENABLE 	0x01
#define BL_CONTROL 		0x02
#define BL_TIME 		0x03
#define BL_FADE 		0x04
#define DAYLIGHT_MAX 		0x05
#define DAYLIGHT_DIM 		0x06
#define OFFICE_MAX 		0x07
#define OFFICE_DIM 		0x08
#define DARK_MAX 		0x09
#define DARK_DIM 		0x0A
#define BL_VALUE 		0x0B
#define ALS_CMPR_CFG 		0x0C
#define L2_TRIP 		0x0D
#define L2_HYS 			0x0E
#define L3_TRIP 		0x0F
#define L3_HYS 			0x10
#define LED_CONTROL 		0x11
#define LED_TIME 		0x12
#define LED_FADE 		0x13
#define LED1_CURRENT 		0x14
#define LED2_CURRENT 		0x15
#define LED3_CURRENT 		0x16

/*
 * ADP5520 Register Map
 */

#define GPIO_CFG_1 		0x17
#define GPIO_CFG_2 		0x18
#define GPIO_IN 		0x19
#define GPIO_OUT 		0x1A
#define GPIO_INT_EN 		0x1B
#define GPIO_INT_STAT 		0x1C
#define GPIO_INT_LVL 		0x1D
#define GPIO_DEBOUNCE 		0x1E
#define GPIO_PULLUP 		0x1F
#define KP_INT_STAT_1 		0x20
#define KP_INT_STAT_2 		0x21
#define KR_INT_STAT_1 		0x22
#define KR_INT_STAT_2 		0x23
#define KEY_STAT_1 		0x24
#define KEY_STAT_2 		0x25

/*
 * MODE_STATUS bits
 */

#define nSTNBY		(1 << 7)
#define BL_EN           (1 << 6)
#define DIM_EN          (1 << 5)
#define OVP_INT         (1 << 4)
#define CMPR_INT        (1 << 3)
#define GPI_INT         (1 << 2)
#define KR_INT          (1 << 1)
#define KP_INT          (1 << 0)

/*
 * INTERRUPT_ENABLE bits
 */

#define AUTO_LD_EN      (1 << 4)
#define CMPR_IEN        (1 << 3)
#define OVP_IEN         (1 << 2)
#define KR_IEN          (1 << 1)
#define KP_IEN          (1 << 0)

/*
 * BL_CONTROL bits
 */

#define BL_LVL          ((x) << 5)
#define BL_LAW          ((x) << 4)
#define BL_AUTO_ADJ     (1 << 3)
#define OVP_EN          (1 << 2)
#define FOVR            (1 << 1)
#define KP_BL_EN        (1 << 0)

/*
 * ALS_CMPR_CFG bits
 */

#define L3_OUT		(1 << 3)
#define L2_OUT		(1 << 2)
#define L3_EN		(1 << 1)

#define ADP5020_MAX_BRIGHTNESS	0x7F

#define FADE_VAL(in, out)	((0xF & (in)) | ((0xF & (out)) << 4))
#define BL_CTRL_VAL(law, auto)	(((1 & (auto)) << 3) | ((0x3 & (law)) << 4))
#define ALS_CMPR_CFG_VAL(filt, l3_en)	(((0x7 & filt) << 5) | l3_en)

/*
 * LEDs subdevice bits and masks
 */

#define ADP5520_01_MAXLEDS 3

#define FLAG_LED_MASK 		0x3
#define FLAG_OFFT_SHIFT 	8
#define FLAG_OFFT_MASK 		0x3

#define R3_MODE		(1 << 5)
#define C3_MODE		(1 << 4)
#define LED_LAW		(1 << 3)
#define LED3_EN		(1 << 2)
#define LED2_EN		(1 << 1)
#define LED1_EN		(1 << 0)

/*
 * GPIO subdevice bits and masks
 */

#define ADP5520_MAXGPIOS	8

#define GPIO_C3		(1 << 7)	/* LED2 or GPIO7 aka C3 */
#define GPIO_C2		(1 << 6)
#define GPIO_C1		(1 << 5)
#define GPIO_C0		(1 << 4)
#define GPIO_R3		(1 << 3)	/* LED3 or GPIO3 aka R3 */
#define GPIO_R2		(1 << 2)
#define GPIO_R1		(1 << 1)
#define GPIO_R0		(1 << 0)

struct adp5520_gpio_platfrom_data {
	unsigned gpio_start;
	u8 gpio_en_mask;
	u8 gpio_pullup_mask;
};

/*
 * Keypad subdevice bits and masks
 */

#define ADP5520_MAXKEYS	16

#define COL_C3 		(1 << 7)	/* LED2 or GPIO7 aka C3 */
#define COL_C2		(1 << 6)
#define COL_C1		(1 << 5)
#define COL_C0		(1 << 4)
#define ROW_R3		(1 << 3)	/* LED3 or GPIO3 aka R3 */
#define ROW_R2		(1 << 2)
#define ROW_R1		(1 << 1)
#define ROW_R0		(1 << 0)

#define KEY(row, col) (col + row * 4)
#define ADP5520_KEYMAPSIZE	ADP5520_MAXKEYS

struct adp5520_keys_platfrom_data {
	int rows_en_mask;		/* Number of rows */
	int cols_en_mask;		/* Number of columns */
	const unsigned short *keymap;	/* Pointer to keymap */
	unsigned short keymapsize;	/* Keymap size */
	unsigned repeat:1;		/* Enable key repeat */
};


/*
 * LEDs subdevice platfrom data
 */

#define FLAG_ID_ADP5520_LED1_ADP5501_LED0 	1	/* ADP5520 PIN ILED */
#define FLAG_ID_ADP5520_LED2_ADP5501_LED1 	2	/* ADP5520 PIN C3 */
#define FLAG_ID_ADP5520_LED3_ADP5501_LED2 	3	/* ADP5520 PIN R3 */

#define LED_DIS_BLINK	(0 << FLAG_OFFT_SHIFT)
#define LED_OFFT_600ms	(1 << FLAG_OFFT_SHIFT)
#define LED_OFFT_800ms	(2 << FLAG_OFFT_SHIFT)
#define LED_OFFT_1200ms	(3 << FLAG_OFFT_SHIFT)

#define LED_ONT_200ms	0
#define LED_ONT_600ms	1
#define LED_ONT_800ms	2
#define LED_ONT_1200ms	3

struct adp5520_leds_platfrom_data {
	int num_leds;
	struct led_info	*leds;
	u8 fade_in;		/* Backlight Fade-In Timer */
	u8 fade_out;		/* Backlight Fade-Out Timer */
	u8 led_on_time;
};

/*
 * Backlight subdevice platfrom data
 */

#define FADE_T_DIS	0	/* Fade Timer Disabled */
#define FADE_T_300ms	1	/* 0.3 Sec */
#define FADE_T_600ms	2
#define FADE_T_900ms	3
#define FADE_T_1200ms	4
#define FADE_T_1500ms	5
#define FADE_T_1800ms	6
#define FADE_T_2100ms	7
#define FADE_T_2400ms	8
#define FADE_T_2700ms	9
#define FADE_T_3000ms	10
#define FADE_T_3500ms	11
#define FADE_T_4000ms	12
#define FADE_T_4500ms	13
#define FADE_T_5000ms	14
#define FADE_T_5500ms	15	/* 5.5 Sec */

#define BL_LAW_LINEAR 	0
#define BL_LAW_SQUARE 	1
#define BL_LAW_CUBIC1 	2
#define BL_LAW_CUBIC2 	3

#define BL_AMBL_FILT_80ms 	0	/* Light sensor filter time */
#define BL_AMBL_FILT_160ms 	1
#define BL_AMBL_FILT_320ms 	2
#define BL_AMBL_FILT_640ms 	3
#define BL_AMBL_FILT_1280ms 	4
#define BL_AMBL_FILT_2560ms 	5
#define BL_AMBL_FILT_5120ms 	6
#define BL_AMBL_FILT_10240ms 	7	/* 10.24 sec */

	/*
	 * Blacklight current 0..30mA
	 */
#define BL_CUR_mA(I)		((I * 127) / 30)

	/*
	 * L2 comparator current 0..1000uA
	 */
#define L2_COMP_CURR_uA(I)	((I * 255) / 1000)

	/*
	 * L3 comparator current 0..127uA
	 */
#define L3_COMP_CURR_uA(I)	((I * 255) / 127)

struct adp5520_backlight_platfrom_data {
	u8 fade_in;		/* Backlight Fade-In Timer */
	u8 fade_out;		/* Backlight Fade-Out Timer */
	u8 fade_led_law;	/* fade-on/fade-off transfer characteristic */

	u8 en_ambl_sens;	/* 1 = enable ambient light sensor */
	u8 abml_filt;		/* Light sensor filter time */
	u8 l1_daylight_max;	/* use BL_CUR_mA(I) 0 <= I <= 30 mA */
	u8 l1_daylight_dim;	/* typ = 0, use BL_CUR_mA(I) 0 <= I <= 30 mA */
	u8 l2_office_max;	/* use BL_CUR_mA(I) 0 <= I <= 30 mA */
	u8 l2_office_dim;	/* typ = 0, use BL_CUR_mA(I) 0 <= I <= 30 mA */
	u8 l3_dark_max;		/* use BL_CUR_mA(I) 0 <= I <= 30 mA */
	u8 l3_dark_dim;		/* typ = 0, use BL_CUR_mA(I) 0 <= I <= 30 mA */
	u8 l2_trip;		/* use L2_COMP_CURR_uA(I) 0 <= I <= 1000 uA */
	u8 l2_hyst;		/* use L2_COMP_CURR_uA(I) 0 <= I <= 1000 uA */
	u8 l3_trip;		/* use L3_COMP_CURR_uA(I) 0 <= I <= 127 uA */
	u8 l3_hyst;		/* use L3_COMP_CURR_uA(I) 0 <= I <= 127 uA */
};

/*
 * MFD chip platfrom data
 */

struct adp5520_platform_data {
	struct adp5520_keys_platfrom_data *keys;
	struct adp5520_gpio_platfrom_data *gpio;
	struct adp5520_leds_platfrom_data *leds;
	struct adp5520_backlight_platfrom_data *backlight;
};

/*
 * MFD chip functions
 */

extern int adp5520_read(struct device *dev, int reg, uint8_t *val);
extern int adp5520_write(struct device *dev, int reg, u8 val);
extern int adp5520_clr_bits(struct device *dev, int reg, uint8_t bit_mask);
extern int adp5520_set_bits(struct device *dev, int reg, uint8_t bit_mask);

extern int adp5520_register_notifier(struct device *dev,
		 struct notifier_block *nb, unsigned int events);

extern int adp5520_unregister_notifier(struct device *dev,
		struct notifier_block *nb, unsigned int events);

#endif /* __LINUX_MFD_ADP5520_H */

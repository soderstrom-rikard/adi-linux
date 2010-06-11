/*
 * include/linux/input/ad7160.h
 *
 * Touchsceen characteristics are highly application specific
 * and may vary between boards and models. The platform_data for the
 * device's "struct device" holds this information.
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __LINUX_INPUT_AD7160_H__
#define __LINUX_INPUT_AD7160_H__

struct ad7160_platform_data {
	u32 sensor_x_res;
	u32 sensor_y_res;
	u32 pressure;

	/* Position filter Coefficient
	 * This option is used to define the value of the
	 * IIR filter used to smooth the response of the touch.
	 * The value must range between 1 and 9.
	 * The smaller the coefficient the bigger the averaging.
	 */

	u8 filter_coef;

	/* This option is used to determine the
	 * location of the (0,0) coordinates.
	 */
#define AD7160_ORIG_TOP_LEFT 0x8 /* (0,0) Coordinate is the TOP LEFT corner */
#define AD7160_ORIG_TOP_RIGHT 0x4 /* (0,0) Coordinate is the TOP RIGHT corner */
#define AD7160_ORIG_BOTTOM_LEFT 0x2 /* (0,0) Coordinate is the BOTTOM LEFT corner */
#define AD7160_ORIG_BOTTOM_RIGHT 0x1 /* (0,0) Coordinate is the BOTTOM RIGHT corner */

	u8 coord_pref;

	/* This option sets a window around the first touch coordinate,
	 * user needs to move outside this window to enable further press
	 * events after first touch. Typically 4-6 positions to avoid
	 * multiple press events when tapping or while touching
	 * down on the sensor.
	 */

	u8 first_touch_window;

	/* This option sets a window around the current touch coordinates
	 * so the user needs to move by a certain number of coordinates
	 * before a new press event is reported, typically -3 positions.
	 * This gives the user the flexibility to reduce host interaction
	 * and trade off linarity depending on the use case.
	 */

	u8 move_window;

	/* This will determine the minimal amount of response per finger
	 * in order to register a valid finger touch using mutual
	 * capacitance measurements. (default 0x0064)
	 */

	u16 finger_act_ctrl;

	/* Event Capabilities - MT devices may produce a lot of data.
	 * In some applications only a subset is consumed by user space.
	 * This options allows control for some optional events.
	 */

#define AD7160_TRADITIONAL_TS_EMULATION		(1 << 0)	/* EMIT: ABS_{X,Y,PRESSURE} & BTN_TOUCH */
#define AD7160_EMIT_ABS_MT_TRACKING_ID		(1 << 1)
#define AD7160_EMIT_ABS_MT_TOUCH_MAJOR		(1 << 2)
#define AD7160_EMIT_ABS_MT_TOUCH_MINOR		(1 << 3)
#define AD7160_EMIT_ABS_MT_ORIENTATION		(1 << 4)
#define AD7160_EMIT_ABS_MT_PRESSURE		(1 << 5)
#define AD7160_TRACKING_ID_ASCENDING		(1 << 6)

	u32 event_cabs;

	/*
	 * A valid BTN or KEY Code; use 0 or KEY_RESERVED to disable
	 * TAP event reporting.
	 */

	u32 ev_code_tap;

	/*
	 * A valid BTN or KEY Code; use 0 or KEY_RESERVED to disable
	 * DOUBLE TAP event reporting.
	 */

	u32 ev_code_double_tap;
};
#endif

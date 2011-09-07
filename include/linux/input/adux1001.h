/*
 * ADUX1001 Smart Electromagnetic Actuator Haptic Driver
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef __LINUX_ADUX1001_H
#define __LINUX_ADUX1001_H

#include <linux/compiler.h>
#include <linux/types.h>

/* Don't change declaration:
 * this struct matches the over-the-wire format the device expects
 */

struct adux1001_calib_data {
	unsigned char slra_cal0;
	unsigned char slra_cal1;
	unsigned char slra_cal2;
	unsigned char slra_cal3;
} __packed;

struct adux1001_reserved_data {
	unsigned char reserved0;
	unsigned char reserved1;
	unsigned char reserved2;
	unsigned char reserved3;
	unsigned char reserved4;
} __packed;

struct i2c_client; /* forward declaration */

struct adux1001_vibra_platform_data {
	/* Actuator Selection: true = ERM, false = LRA */
	bool actuator_is_erm;

	/* true = Under voltage lockout circuitry disable */
	bool uvlo_dis;

	/* true = Over current lockout circuitry disable */
	bool oclo_dis;

	/* ADUX1001_PWM_MODE_EXT_ONLY:
	 * Some other kernel module such as the Immersion TouchSense controls
	 * the main PWM, no input force feedback device is created
	 */
#define ADUX1001_PWM_MODE_EXT_ONLY	4

	/* ADUX1001_PWM_MODE_INT_AND_EXT:
	 * An input force feedback device is created and controls
	 * one system PWM, another kernel module may control the second
	 * PWM, priority can be set by pwm_mode_sel
	 */
#define ADUX1001_PWM_MODE_INT_AND_EXT	2

	/* ADUX1001_I2C_MODE:
	 * TBD, using effects.type FF_PERIODIC in combination with
	 * periodic.waveform FF_CUSTOM we could play custom waveforms
	 * using I2C streaming mode
	 */
#define ADUX1001_I2C_MODE		1

	unsigned char mode_sel;

	/* PWM Selection/Priority */
#define ADUX1001_PWMPRIO_PWM2EN_PWM1DIS	3
#define ADUX1001_PWMPRIO_PWM1EN_PWM2DIS	2
#define ADUX1001_PWMPRIO_PWM2HI_PWM1LO	1
#define ADUX1001_PWMPRIO_PWM1HI_PWM2LO	0

	unsigned char pwm_priority;		/* 0..3 */

	unsigned char loop_gain;		/* 2..15 */

	/* 0..7, from 40mA - 160mA in 10mA steps */
	unsigned char max_output_current;

	bool lra_output_unit_1ms;		/* true = 1ms, false = LRA
						 * output unit is LRA Resonance
						 * Period/2 */

	unsigned char output_rate;		/* Rate = lra_output_unit *
						 * output_rate, 1..127 */

	unsigned char period_calibration_cycles;	/* 0..7 */
	unsigned char amp_calibration_cycles;		/* 0..15 */

	struct adux1001_reserved_data *reserved_data;

	/* initial arbitrary waveform buffer */
	unsigned char *arb_wform_buffer_array;
	unsigned char arb_wform_buffer_array_size; /* 1..10 */

	/* system specific PWM identifier */
	int pwm_id;
	/* PWM period in ns, range 5000..50000 (20-200kHz) */
	unsigned short pwm_periode_ns;

	/* system specific GPIO to control the SHUTDOWN pin,
	 * if not used set to -1
	 */
	int gpio_shutdown;

	/* system specific setup callback */
	int	(*setup)(struct i2c_client *client,
			 unsigned state);
	/* get calibration callback function */
	int	(*get_calibdata)(struct i2c_client *client,
				 struct adux1001_calib_data *);
	/* store calibration callback function */
	int	(*store_calibdata)(struct i2c_client *client,
				   struct adux1001_calib_data *);
};

#endif /* __LINUX_ADUX1001_H */

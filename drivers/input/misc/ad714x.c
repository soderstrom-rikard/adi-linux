/*
 * File:         drivers/input/misc/ad714x.c
 * Based on:
 * Author:       Barry Song <Barry.Song@analog.com>
 *
 * Created:
 * Description:  AD714X CapTouch Programmable Controller driver
 *
 * Modified:
 *               Copyright 2009 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/spi/ad714x.h>

#define AD714x_SPI_ADDR        0x1C
#define AD714x_SPI_ADDR_SHFT   11
#define AD714x_SPI_READ        1
#define AD714x_SPI_READ_SHFT   10

#define AD714x_STG_CAL_EN_REG     0x1
#define AD714X_AMB_COMP_CTRL0_REG 0x2
#define AD714x_PARTID_REG         0x17
#define AD7147_PARTID             0x1470
#define AD7142_PARTID             0xE620
#define AD714x_STAGECFG_REG       0x80
#define AD714x_SYSCFG_REG         0x0

#define STG_LOW_INT_EN_REG     0x5
#define STG_HIGH_INT_EN_REG    0x6
#define STG_COM_INT_EN_REG     0x7
#define STG_LOW_INT_STA_REG    0x8
#define STG_HIGH_INT_STA_REG   0x9
#define STG_COM_INT_STA_REG    0xA

#define CDC_RESULT_S0          0xB
#define CDC_RESULT_S1          0xC
#define CDC_RESULT_S2          0xD
#define CDC_RESULT_S3          0xE
#define CDC_RESULT_S4          0xF
#define CDC_RESULT_S5          0x10
#define CDC_RESULT_S6          0x11
#define CDC_RESULT_S7          0x12
#define CDC_RESULT_S8          0x13
#define CDC_RESULT_S9          0x14
#define CDC_RESULT_S10         0x15
#define CDC_RESULT_S11         0x16

#define STAGE0_AMBIENT		0xF1
#define STAGE1_AMBIENT		0x115
#define STAGE2_AMBIENT		0x139
#define STAGE3_AMBIENT		0x15D
#define STAGE4_AMBIENT		0x181
#define STAGE5_AMBIENT		0x1A5
#define STAGE6_AMBIENT		0x1C9
#define STAGE7_AMBIENT		0x1ED
#define STAGE8_AMBIENT		0x211
#define STAGE9_AMBIENT		0x234
#define STAGE10_AMBIENT		0x259
#define STAGE11_AMBIENT		0x27D

#define PER_STAGE_REG_NUM      36
#define STAGE_NUM              12
#define STAGE_CFGREG_NUM       8
#define SYS_CFGREG_NUM         8

#if defined(CONFIG_INPUT_AD714X_SPI)
#define bus_device		struct spi_device
#elif defined(CONFIG_INPUT_AD714X_I2C)
#define bus_device		struct i2c_client
#else
#error Communication method needs to be selected (I2C or SPI)
#endif

#define AD714X_DEBUG
#ifdef AD714X_DEBUG
#define ad714x_debug(fmt, ...) \
	 printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define ad714x_debug(fmt, ...)
#endif

/*
 * driver information which will be used to maintain the software flow
 */
typedef enum {IDLE, JITTER, ACTIVE, SPACE} ad714x_device_state;

struct ad714x_slider_drv {
	int highest_stage;
	int abs_pos;
	int flt_pos;
	ad714x_device_state state;
	struct input_dev *input;
};

struct ad714x_wheel_drv {
	int abs_pos;
	int flt_pos;
	int pre_mean_value;
	int pre_highest_stage;
	int pre_mean_value_no_offset;
	int mean_value;
	int mean_value_no_offset;
	int pos_offset;
	int pos_ratio;
	int highest_stage;
	ad714x_device_state state;
	struct input_dev *input;
};

struct ad714x_touchpad_drv {
	int x_highest_stage;
	int x_flt_pos;
	int x_abs_pos;
	int y_highest_stage;
	int y_flt_pos;
	int y_abs_pos;
	int left_ep;
	int left_ep_val;
	int right_ep;
	int right_ep_val;
	int top_ep;
	int top_ep_val;
	int bottom_ep;
	int bottom_ep_val;
	ad714x_device_state state;
	struct input_dev *input;
};

struct ad714x_button_drv {
	ad714x_device_state state;
	/* Unlike slider/wheel/touchpad, all buttons point to
	 * same input_dev instance */
	struct input_dev *input;
};

struct ad714x_driver_data {
	struct ad714x_slider_drv *slider;
	struct ad714x_wheel_drv *wheel;
	struct ad714x_touchpad_drv *touchpad;
	struct ad714x_button_drv *button;
};

/* information to integrate all things which will be private data
 * of spi/i2c device */
struct ad714x_chip {
	unsigned short h_state;
	unsigned short l_state;
	unsigned short c_state;
	unsigned short adc_reg[STAGE_NUM];
	unsigned short amb_reg[STAGE_NUM];
	unsigned short sensor_val[STAGE_NUM];

	struct ad714x_platform_data *hw;
	struct ad714x_driver_data *sw;

	bus_device *bus;
	int (*read) (bus_device *, unsigned short, unsigned short *);
	int (*write) (bus_device *, unsigned short, unsigned short);

	struct mutex mutex;
};

static void stage_use_com_int(struct ad714x_chip *ad714x, int start_stage,
		int end_stage)
{
	unsigned short data;
	unsigned short mask;

	mask = ((1 << (end_stage + 1)) - 1) - (1 << start_stage);

	ad714x->read(ad714x->bus, STG_COM_INT_EN_REG, &data);
	data |= 1 << start_stage;
	ad714x->write(ad714x->bus, STG_COM_INT_EN_REG, data);

	ad714x->read(ad714x->bus, STG_HIGH_INT_EN_REG, &data);
	data &= ~mask;
	ad714x->write(ad714x->bus, STG_HIGH_INT_EN_REG, data);
}

static void stage_use_thr_int(struct ad714x_chip *ad714x, int start_stage,
		int end_stage)
{
	unsigned short data;
	unsigned short mask;

	mask = ((1 << (end_stage + 1)) - 1) - (1 << start_stage);

	ad714x->read(ad714x->bus, STG_COM_INT_EN_REG, &data);
	data &= ~(1 << start_stage);
	ad714x->write(ad714x->bus, STG_COM_INT_EN_REG, data);

	ad714x->read(ad714x->bus, STG_HIGH_INT_EN_REG, &data);
	data |= mask;
	ad714x->write(ad714x->bus, STG_HIGH_INT_EN_REG, data);
}

static int stage_cal_highest_stage(struct ad714x_chip *ad714x, int start_stage,
		int end_stage)
{
	int max_res = 0;
	int max_idx = 0;
	int i;

	for (i = start_stage; i <= end_stage; i++) {
		if (ad714x->sensor_val[i] > max_res) {
			max_res = ad714x->sensor_val[i];
			max_idx = i;
		}
	}

	return max_idx;
}

static int stage_cal_abs_pos(struct ad714x_chip *ad714x, int start_stage,
		int end_stage, int highest_stage, int max_coord)
{
	int a_param, b_param;

	if (highest_stage == start_stage) {
		a_param = ad714x->sensor_val[start_stage + 1];
		b_param = ad714x->sensor_val[start_stage] +
			ad714x->sensor_val[start_stage + 1];
	} else if (highest_stage == end_stage) {
		a_param = ad714x->sensor_val[end_stage] *
			(end_stage - start_stage) +
			ad714x->sensor_val[end_stage - 1] *
			(end_stage - start_stage - 1);
		b_param = ad714x->sensor_val[end_stage] +
			ad714x->sensor_val[end_stage - 1];
	} else {
		a_param = ad714x->sensor_val[highest_stage] *
			(highest_stage - start_stage) +
			ad714x->sensor_val[highest_stage - 1] *
			(highest_stage - start_stage - 1) +
			ad714x->sensor_val[highest_stage + 1] *
			(highest_stage - start_stage + 1);
		b_param = ad714x->sensor_val[highest_stage] +
			ad714x->sensor_val[highest_stage - 1] +
			ad714x->sensor_val[highest_stage + 1];
	}

	return (max_coord / (end_stage - start_stage)) * a_param / b_param;
}


/* One button can connect to multi postive and negative of CDCs
 * Multi-buttons can connect to same postive/negative of one CDC
 */
static void button_state_machine(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_button_plat *hw = &ad714x->hw->button[idx];
	struct ad714x_button_drv *sw = &ad714x->sw->button[idx];

	switch (sw->state) {
	case IDLE:
		if (((ad714x->h_state & hw->h_mask) == hw->h_mask) &&
			((ad714x->l_state & hw->l_mask) == hw->l_mask)) {
			ad714x_debug("button %d touched\n", idx);
			input_report_key(sw->input, hw->event, 1);
			input_sync(sw->input);
			sw->state = ACTIVE;
		}
		break;
	case ACTIVE:
		if (((ad714x->h_state & hw->h_mask) != hw->h_mask) ||
			((ad714x->l_state & hw->l_mask) != hw->l_mask)) {
			ad714x_debug("button %d released\n", idx);
			input_report_key(sw->input, hw->event, 0);
			input_sync(sw->input);
			sw->state = IDLE;
		}
		break;
	default:
		break;
	}
}

/* The response of a sensor is defined by the absolute number of codes
 * between the current CDC value and the ambient value.
 */
void slider_cal_sensor_val(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_slider_plat *hw = &ad714x->hw->slider[idx];
	int i;

	for (i = hw->start_stage; i <= hw->end_stage; i++) {
		ad714x->read(ad714x->bus, CDC_RESULT_S0 + i,
			&ad714x->adc_reg[i]);
		ad714x->read(ad714x->bus,
				STAGE0_AMBIENT + i * PER_STAGE_REG_NUM,
				&ad714x->amb_reg[i]);

		ad714x->sensor_val[i] = abs(ad714x->adc_reg[i] -
				ad714x->amb_reg[i]);
	}
}

void slider_cal_highest_stage(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_slider_plat *hw = &ad714x->hw->slider[idx];
	struct ad714x_slider_drv *sw = &ad714x->sw->slider[idx];

	sw->highest_stage = stage_cal_highest_stage(ad714x, hw->start_stage,
			hw->end_stage);

	ad714x_debug("slider %d highest_stage:%d\n", idx, sw->highest_stage);
}

/* The formulae are very straight forward. It uses the sensor with the
 * highest response and the 2 adjacent ones.
 * When Sensor 0 has the highest response, only sensor 0 and sensor 1
 * are used in the calculations. Similarly when the last sensor has the
 * highest response, only the last sensor and the second last sensors
 * are used in the calculations.
 *
 * For i= idx_of_peak_Sensor-1 to i= idx_of_peak_Sensor+1
 *         v += Sensor response(i)*i
 *         w += Sensor response(i)
 * POS=(Number_of_Positions_Wanted/(Number_of_Sensors_Used-1)) *(v/w)
 */
void slider_cal_abs_pos(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_slider_plat *hw = &ad714x->hw->slider[idx];
	struct ad714x_slider_drv *sw = &ad714x->sw->slider[idx];

	sw->abs_pos = stage_cal_abs_pos(ad714x, hw->start_stage, hw->end_stage,
		sw->highest_stage, hw->max_coord);

	ad714x_debug("slider %d absolute position:%d\n", idx, sw->abs_pos);
}

/*
 * To minimise the Impact of the noise on the algorithm, ADI developed a
 * routine that filters the CDC results after they have been read by the
 * host processor.
 * The filter used is an Infinite Input Response(IIR) filter implemented
 * in firmware and attenuates the noise on the CDC results after they've
 * been read by the host processor.
 * Filtered_CDC_result = (Filtered_CDC_result * (10 - Coefficient) +
 * 				Latest_CDC_result * Coefficient)/10
 */
void slider_cal_flt_pos(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_slider_drv *sw = &ad714x->sw->slider[idx];

	sw->flt_pos = (sw->flt_pos * (10 - 4) +
			sw->abs_pos * 4)/10;

	ad714x_debug("slider %d filter position:%d\n", idx, sw->flt_pos);
}

static void slider_use_com_int(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_slider_plat *hw = &ad714x->hw->slider[idx];
	stage_use_com_int(ad714x, hw->start_stage, hw->end_stage);
}

static void slider_use_thr_int(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_slider_plat *hw = &ad714x->hw->slider[idx];
	stage_use_thr_int(ad714x, hw->start_stage, hw->end_stage);
}

static void slider_state_machine(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_slider_plat *hw = &ad714x->hw->slider[idx];
	struct ad714x_slider_drv *sw = &ad714x->sw->slider[idx];
	unsigned short h_state, c_state;
	unsigned short mask;

	mask = ((1 << (hw->end_stage + 1)) - 1) - ((1 << hw->start_stage) - 1);

	h_state = ad714x->h_state & mask;
	c_state = ad714x->c_state & mask;

	switch (sw->state) {
	case IDLE:
		if (h_state) {
			sw->state = JITTER;
			/* In End of Conversion interrupt mode, the AD714x
			 * continuously generates hardware interrupts.
			 */
			slider_use_com_int(ad714x, idx);
			ad714x_debug("slider %d touched\n", idx);
		}
		break;
	case JITTER:
		if (c_state == mask) {
			slider_cal_sensor_val(ad714x, idx);
			slider_cal_highest_stage(ad714x, idx);
			slider_cal_abs_pos(ad714x, idx);
			sw->flt_pos = sw->abs_pos;
			sw->state = ACTIVE;
		}
		break;
	case ACTIVE:
		if (c_state == mask) {
			if (h_state) {
				slider_cal_sensor_val(ad714x, idx);
				slider_cal_highest_stage(ad714x, idx);
				slider_cal_abs_pos(ad714x, idx);
				slider_cal_flt_pos(ad714x, idx);

				input_report_abs(sw->input, ABS_X, sw->flt_pos);
				input_report_abs(sw->input, ABS_PRESSURE, 1);
			} else {
				/* When the user lifts off the sensor, configure
				 * the AD714X back to threshold interrupt mode.
				 */
				slider_use_thr_int(ad714x, idx);
				sw->state = IDLE;
				input_report_abs(sw->input, ABS_PRESSURE, 0);
				ad714x_debug("slider %d released\n", idx);
			}
			input_sync(sw->input);
		}
		break;
	default:
		break;
	}
}

/* When the scroll wheel is activated, we compute the absolute position based
 * on the sensor values. To calculate the position, we first determine the
 * sensor that has the greatest response among the 8 sensors that constitutes
 * the scrollwheel. Then we determined the 2 sensors on either sides of the
 * sensor with the highest response and we apply weights to these sensors.
 */
void wheel_cal_highest_stage(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_wheel_plat *hw = &ad714x->hw->wheel[idx];
	struct ad714x_wheel_drv *sw = &ad714x->sw->wheel[idx];

	sw->pre_highest_stage = sw->highest_stage;
	sw->highest_stage = stage_cal_highest_stage(ad714x, hw->start_stage,
			hw->end_stage);

	ad714x_debug("wheel %d highest_stage:%d\n", idx, sw->highest_stage);
}

void wheel_cal_sensor_val(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_wheel_plat *hw = &ad714x->hw->wheel[idx];
	int i;

	for (i = hw->start_stage; i <= hw->end_stage; i++) {
		ad714x->read(ad714x->bus, CDC_RESULT_S0 + i,
			&ad714x->adc_reg[i]);
		ad714x->read(ad714x->bus,
				STAGE0_AMBIENT + i * PER_STAGE_REG_NUM,
				&ad714x->amb_reg[i]);
		if (ad714x->adc_reg[i] > ad714x->amb_reg[i])
			ad714x->sensor_val[i] = ad714x->adc_reg[i] -
				ad714x->amb_reg[i];
		else
			ad714x->sensor_val[i] = 0;
	}
}

/* When the scroll wheel is activated, we compute the absolute position based
 * on the sensor values. To calculate the position, we first determine the
 * sensor that has the greatest response among the 8 sensors that constitutes
 * the scrollwheel. Then we determined the 2 sensors on either sides of the
 * sensor with the highest response and we apply weights to these sensors. The
 * result of this computation gives us the mean value which defined by the
 * following formula:
 * For i= second_before_highest_stage to i= second_after_highest_stage
 *         v += Sensor response(i)*WEIGHT*(i+3)
 *         w += Sensor response(i)
 * Mean_Value=v/w
 * pos_on_scrollwheel = (Mean_Value - position_offset) / position_ratio
 *
 */

#define WEIGHT_FACTOR 30
/* This constant prevents the "PositionOffset" from reaching a big value */
#define OFFSET_POSITION_CLAMP	120
void wheel_cal_abs_pos(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_wheel_plat *hw = &ad714x->hw->wheel[idx];
	struct ad714x_wheel_drv *sw = &ad714x->sw->wheel[idx];
	int stage_num = hw->end_stage - hw->start_stage + 1;
	int second_before, first_before, highest, first_after, second_after;
	int a_param, b_param;

	/* Calculate Mean value */

	second_before = (sw->highest_stage + stage_num - 2) % stage_num;
	first_before = (sw->highest_stage + stage_num - 1) % stage_num;
	highest = sw->highest_stage;
	first_after = (sw->highest_stage + stage_num + 1) % stage_num;
	second_after = (sw->highest_stage + stage_num + 2) % stage_num;

	if (((sw->highest_stage - hw->start_stage) > 1) &&
			((hw->end_stage - sw->highest_stage) > 1)) {
		a_param = ad714x->sensor_val[second_before] *
			(second_before - hw->start_stage + 3) +
			ad714x->sensor_val[first_before] *
			(second_before - hw->start_stage + 3) +
			ad714x->sensor_val[highest] *
			(second_before - hw->start_stage + 3) +
			ad714x->sensor_val[first_after] *
			(first_after - hw->start_stage + 3) +
			ad714x->sensor_val[second_after] *
			(second_after - hw->start_stage + 3);
	} else {
		a_param = ad714x->sensor_val[second_before] *
			(second_before - hw->start_stage + 1) +
			ad714x->sensor_val[first_before] *
			(second_before - hw->start_stage + 2) +
			ad714x->sensor_val[highest] *
			(second_before - hw->start_stage + 3) +
			ad714x->sensor_val[first_after] *
			(first_after - hw->start_stage + 4) +
			ad714x->sensor_val[second_after] *
			(second_after - hw->start_stage + 5);
	}
	a_param *= WEIGHT_FACTOR;

	b_param = ad714x->sensor_val[second_before] +
		ad714x->sensor_val[first_before] +
		ad714x->sensor_val[highest] +
		ad714x->sensor_val[first_after] +
		ad714x->sensor_val[second_after];

	sw->pre_mean_value = sw->mean_value;
	sw->mean_value = a_param / b_param;

	/* Calculate the offset */

	if ((sw->pre_highest_stage == hw->end_stage) &&
			(sw->highest_stage == hw->start_stage))
		sw->pos_offset = sw->mean_value;
	else if ((sw->pre_highest_stage == hw->start_stage) &&
			(sw->highest_stage == hw->end_stage))
		sw->pos_offset = sw->pre_mean_value;
	if (sw->pos_offset > OFFSET_POSITION_CLAMP)
		sw->pos_offset = OFFSET_POSITION_CLAMP;

	/* Calculate the mean value without the offset */

	sw->pre_mean_value_no_offset = sw->mean_value_no_offset;
	sw->mean_value_no_offset = sw->mean_value - sw->pos_offset;
	if (sw->mean_value_no_offset < 0)
		sw->mean_value_no_offset = 0;

	/* Calculate ratio to scale down to NUMBER_OF_WANTED_POSITIONS */

	if ((sw->pre_highest_stage == hw->end_stage) &&
			(sw->highest_stage == hw->start_stage))
		sw->pos_ratio = (sw->pre_mean_value_no_offset * 100) /
			hw->max_coord;
	else if ((sw->pre_highest_stage == hw->start_stage) &&
			(sw->highest_stage == hw->end_stage))
		sw->pos_ratio = (sw->mean_value_no_offset * 100) /
			hw->max_coord;
	sw->abs_pos = (sw->mean_value_no_offset * 100) / sw->pos_ratio;
	if (sw->abs_pos > hw->max_coord)
		sw->abs_pos = hw->max_coord;
}

void wheel_cal_flt_pos(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_wheel_plat *hw = &ad714x->hw->wheel[idx];
	struct ad714x_wheel_drv *sw = &ad714x->sw->wheel[idx];
	if (((sw->pre_highest_stage == hw->end_stage) &&
				(sw->highest_stage == hw->start_stage)) ||
			((sw->pre_highest_stage == hw->start_stage) &&
			 (sw->highest_stage == hw->end_stage)))
		sw->flt_pos = sw->abs_pos;
	else
		sw->flt_pos = ((sw->flt_pos * 30) + (sw->abs_pos * 71)) / 100;

	if (sw->flt_pos > hw->max_coord)
		sw->flt_pos = hw->max_coord;
}

static void wheel_use_com_int(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_wheel_plat *hw = &ad714x->hw->wheel[idx];
	stage_use_com_int(ad714x, hw->start_stage, hw->end_stage);
}

static void wheel_use_thr_int(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_wheel_plat *hw = &ad714x->hw->wheel[idx];
	stage_use_thr_int(ad714x, hw->start_stage, hw->end_stage);
}

static void wheel_state_machine(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_wheel_plat *hw = &ad714x->hw->wheel[idx];
	struct ad714x_wheel_drv *sw = &ad714x->sw->wheel[idx];
	unsigned short h_state, c_state;
	unsigned short mask;

	mask = ((1 << (hw->end_stage + 1)) - 1) - ((1 << hw->start_stage) - 1);

	h_state = ad714x->h_state & mask;
	c_state = ad714x->c_state & mask;

	switch (sw->state) {
	case IDLE:
		if (h_state) {
			sw->state = JITTER;
			/* In End of Conversion interrupt mode, the AD714x
			 * continuously generates hardware interrupts.
			 */
			wheel_use_com_int(ad714x, idx);
			ad714x_debug("wheel %d touched\n", idx);
		}
		break;
	case JITTER:
		if (c_state == mask)	{
			wheel_cal_sensor_val(ad714x, idx);
			wheel_cal_highest_stage(ad714x, idx);
			wheel_cal_abs_pos(ad714x, idx);
			sw->flt_pos = sw->abs_pos;
			sw->state = ACTIVE;
		}
		break;
	case ACTIVE:
		if (c_state == mask) {
			if (h_state) {
				wheel_cal_sensor_val(ad714x, idx);
				wheel_cal_highest_stage(ad714x, idx);
				wheel_cal_abs_pos(ad714x, idx);

				input_report_abs(sw->input, ABS_WHEEL,
					sw->abs_pos);
				input_report_abs(sw->input, ABS_PRESSURE, 1);
			} else {
				/* When the user lifts off the sensor, configure
				 * the AD714X back to threshold interrupt mode.
				 */
				wheel_use_thr_int(ad714x, idx);
				sw->state = IDLE;
				input_report_abs(sw->input, ABS_PRESSURE, 0);

				ad714x_debug("wheel %d released\n", idx);
			}
			input_sync(sw->input);
		}
		break;
	default:
		break;
	}
}

void touchpad_cal_sensor_val(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_touchpad_plat *hw = &ad714x->hw->touchpad[idx];
	int i;

	for (i = hw->x_start_stage; i <= hw->x_end_stage; i++) {
		ad714x->read(ad714x->bus, CDC_RESULT_S0 + i,
				&ad714x->adc_reg[i]);
		ad714x->read(ad714x->bus,
				STAGE0_AMBIENT + i * PER_STAGE_REG_NUM,
				&ad714x->amb_reg[i]);
		if (ad714x->adc_reg[i] > ad714x->amb_reg[i])
			ad714x->sensor_val[i] = ad714x->adc_reg[i] -
				ad714x->amb_reg[i];
		else
			ad714x->sensor_val[i] = 0;
	}
}

static void touchpad_cal_highest_stage(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_touchpad_plat *hw = &ad714x->hw->touchpad[idx];
	struct ad714x_touchpad_drv *sw = &ad714x->sw->touchpad[idx];

	sw->x_highest_stage = stage_cal_highest_stage(ad714x, hw->x_start_stage,
			hw->x_end_stage);
	sw->y_highest_stage = stage_cal_highest_stage(ad714x, hw->y_start_stage,
			hw->y_end_stage);

	ad714x_debug("touchpad %d x_highest_stage:%d, y_highest_stage:%d\n",
			idx, sw->x_highest_stage, sw->y_highest_stage);
}

/* If 2 fingers are touching the sensor then 2 peaks can be observed in the
 * distribution.
 * The arithmetic doesn't support to get absolute coordinates for multi-touch
 * yet.
 */
static int touchpad_check_second_peak(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_touchpad_plat *hw = &ad714x->hw->touchpad[idx];
	struct ad714x_touchpad_drv *sw = &ad714x->sw->touchpad[idx];
	int i;

	for (i = hw->x_start_stage; i < sw->x_highest_stage; i++) {
		if ((ad714x->sensor_val[i] - ad714x->sensor_val[i + 1])
			> (ad714x->sensor_val[i + 1] / 10))
			return 1;
	}

	for (i = sw->x_highest_stage; i < hw->x_end_stage; i++) {
		if ((ad714x->sensor_val[i + 1] - ad714x->sensor_val[i])
			> (ad714x->sensor_val[i] / 10))
			return 1;
	}

	for (i = hw->y_start_stage; i < sw->y_highest_stage; i++) {
		if ((ad714x->sensor_val[i] - ad714x->sensor_val[i + 1])
			> (ad714x->sensor_val[i + 1] / 10))
			return 1;
	}

	for (i = sw->y_highest_stage; i < hw->y_end_stage; i++) {
		if ((ad714x->sensor_val[i + 1] - ad714x->sensor_val[i])
			> (ad714x->sensor_val[i] / 10))
			return 1;
	}

	return 0;
}

/* If only one finger is used to activate the touch pad then only 1 peak will be
 * registered in the distribution. This peak and the 2 adjacent sensors will be
 * used in the calculation of the absolute position. This will prevent hand
 * shadows to affect the absolute position calculation.
 */
static void touchpad_cal_abs_pos(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_touchpad_plat *hw = &ad714x->hw->touchpad[idx];
	struct ad714x_touchpad_drv *sw = &ad714x->sw->touchpad[idx];

	sw->x_abs_pos = stage_cal_abs_pos(ad714x, hw->x_start_stage,
			hw->x_end_stage, sw->x_highest_stage, hw->x_max_coord);
	sw->y_abs_pos = stage_cal_abs_pos(ad714x, hw->y_start_stage,
			hw->y_end_stage, sw->y_highest_stage, hw->y_max_coord);

	ad714x_debug("touchpad %d absolute position:(%d, %d)\n", idx,
			sw->x_abs_pos, sw->y_abs_pos);
}

static void touchpad_cal_flt_pos(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_touchpad_drv *sw = &ad714x->sw->touchpad[idx];

	sw->x_flt_pos = (sw->x_flt_pos * (10 - 4) +
			sw->x_abs_pos * 4)/10;
	sw->y_flt_pos = (sw->y_flt_pos * (10 - 4) +
			sw->y_abs_pos * 4)/10;

	ad714x_debug("touchpad %d filter position:(%d, %d)\n",
			idx, sw->x_flt_pos, sw->y_flt_pos);
}

/* To prevent distortion from showing in the absolute position, it is
 * necessary to detect the end points. When endpoints are detected, the
 * driver stops updating the status variables with absolute positions.
 * End points are detected on the 4 edges of the touchpad sensor. The
 * method to detect them is the same for all 4.
 * To detect the end points, the firmware computes the difference in
 * percent between the sensor on the edge and the adjacent one. The
 * difference is calculated in percent in order to make the end point
 * detection independent of the pressure.
 */

#define LEFT_END_POINT_DETECTION_LEVEL			550
#define RIGHT_END_POINT_DETECTION_LEVEL			750
#define LEFT_RIGHT_END_POINT_DEAVTIVALION_LEVEL		850
#define TOP_END_POINT_DETECTION_LEVEL			550
#define BOTTOM_END_POINT_DETECTION_LEVEL		950
#define TOP_BOTTOM_END_POINT_DEAVTIVALION_LEVEL		700
static int touchpad_check_endpoint(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_touchpad_plat *hw = &ad714x->hw->touchpad[idx];
	struct ad714x_touchpad_drv *sw  = &ad714x->sw->touchpad[idx];
	int percent_sensor_diff;

	/* left endpoint detect */
	percent_sensor_diff = (ad714x->sensor_val[hw->x_start_stage] -
			ad714x->sensor_val[hw->x_start_stage + 1]) * 100 /
			ad714x->sensor_val[hw->x_start_stage + 1];
	if (!sw->left_ep) {
		if (percent_sensor_diff >= LEFT_END_POINT_DETECTION_LEVEL)  {
			sw->left_ep = 1;
			sw->left_ep_val =
				ad714x->sensor_val[hw->x_start_stage + 1];
		}
	} else {
		if ((percent_sensor_diff < LEFT_END_POINT_DETECTION_LEVEL) &&
		(ad714x->sensor_val[hw->x_start_stage + 1] >
		LEFT_RIGHT_END_POINT_DEAVTIVALION_LEVEL + sw->left_ep_val))
			sw->left_ep = 0;
	}

	/* right endpoint detect */
	percent_sensor_diff = (ad714x->sensor_val[hw->x_end_stage] -
			ad714x->sensor_val[hw->x_end_stage - 1]) * 100 /
			ad714x->sensor_val[hw->x_end_stage - 1];
	if (!sw->right_ep) {
		if (percent_sensor_diff >= RIGHT_END_POINT_DETECTION_LEVEL)  {
			sw->right_ep = 1;
			sw->right_ep_val =
				ad714x->sensor_val[hw->x_end_stage - 1];
		}
	} else {
		if ((percent_sensor_diff < RIGHT_END_POINT_DETECTION_LEVEL) &&
		(ad714x->sensor_val[hw->x_end_stage - 1] >
		LEFT_RIGHT_END_POINT_DEAVTIVALION_LEVEL + sw->right_ep_val))
			sw->right_ep = 0;
	}

	/* top endpoint detect */
	percent_sensor_diff = (ad714x->sensor_val[hw->y_start_stage] -
			ad714x->sensor_val[hw->y_start_stage + 1]) * 100 /
			ad714x->sensor_val[hw->y_start_stage + 1];
	if (!sw->top_ep) {
		if (percent_sensor_diff >= TOP_END_POINT_DETECTION_LEVEL)  {
			sw->top_ep = 1;
			sw->top_ep_val =
				ad714x->sensor_val[hw->y_start_stage + 1];
		}
	} else {
		if ((percent_sensor_diff < TOP_END_POINT_DETECTION_LEVEL) &&
		(ad714x->sensor_val[hw->y_start_stage + 1] >
		TOP_BOTTOM_END_POINT_DEAVTIVALION_LEVEL + sw->top_ep_val))
			sw->top_ep = 0;
	}

	/* bottom endpoint detect */
	percent_sensor_diff = (ad714x->sensor_val[hw->y_end_stage] -
		ad714x->sensor_val[hw->y_end_stage - 1]) * 100 /
		ad714x->sensor_val[hw->y_end_stage - 1];
	if (!sw->bottom_ep) {
		if (percent_sensor_diff >= BOTTOM_END_POINT_DETECTION_LEVEL)  {
			sw->bottom_ep = 1;
			sw->bottom_ep_val =
				ad714x->sensor_val[hw->y_end_stage - 1];
		}
	} else {
		if ((percent_sensor_diff < BOTTOM_END_POINT_DETECTION_LEVEL) &&
		(ad714x->sensor_val[hw->y_end_stage - 1] >
		 TOP_BOTTOM_END_POINT_DEAVTIVALION_LEVEL + sw->bottom_ep_val))
			sw->bottom_ep = 0;
	}

	return sw->left_ep || sw->right_ep || sw->top_ep || sw->bottom_ep;
}

static void touchpad_use_com_int(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_touchpad_plat *hw = &ad714x->hw->touchpad[idx];
	stage_use_com_int(ad714x, hw->x_start_stage, hw->x_end_stage);
}

static void touchpad_use_thr_int(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_touchpad_plat *hw = &ad714x->hw->touchpad[idx];
	stage_use_thr_int(ad714x, hw->x_start_stage, hw->x_end_stage);
	stage_use_thr_int(ad714x, hw->y_start_stage, hw->y_end_stage);
}

static void touchpad_state_machine(struct ad714x_chip *ad714x, int idx)
{
	struct ad714x_touchpad_plat *hw = &ad714x->hw->touchpad[idx];
	struct ad714x_touchpad_drv *sw = &ad714x->sw->touchpad[idx];
	unsigned short h_state, c_state;
	unsigned short mask;

	mask = (((1 << (hw->x_end_stage + 1)) - 1) -
		((1 << hw->x_start_stage) - 1)) +
		(((1 << (hw->y_end_stage + 1)) - 1) -
		((1 << hw->y_start_stage) - 1));

	h_state = ad714x->h_state & mask;
	c_state = ad714x->c_state & mask;

	switch (sw->state) {
	case IDLE:
		if (h_state) {
			sw->state = JITTER;
			/* In End of Conversion interrupt mode, the AD714x
			 * continuously generates hardware interrupts.
			 */
			touchpad_use_com_int(ad714x, idx);
			ad714x_debug("touchpad %d touched\n", idx);
		}
		break;
	case JITTER:
		if (c_state == mask) {
			touchpad_cal_sensor_val(ad714x, idx);
			touchpad_cal_highest_stage(ad714x, idx);
			if ((!touchpad_check_second_peak(ad714x, idx)) &&
				(!touchpad_check_endpoint(ad714x, idx))) {
				pr_debug("touchpad%d, 2 fingers or endpoint\n",
						idx);
				touchpad_cal_abs_pos(ad714x, idx);
				sw->x_flt_pos = sw->x_abs_pos;
				sw->y_flt_pos = sw->y_abs_pos;
				sw->state = ACTIVE;
			}
		}
		break;
	case ACTIVE:
		if (c_state == mask) {
			if (h_state) {
				touchpad_cal_sensor_val(ad714x, idx);
				touchpad_cal_highest_stage(ad714x, idx);
				if ((!touchpad_check_second_peak(ad714x, idx))
				  && (!touchpad_check_endpoint(ad714x, idx))) {
					touchpad_cal_abs_pos(ad714x, idx);
					touchpad_cal_flt_pos(ad714x, idx);
					input_report_abs(sw->input, ABS_X,
						sw->x_flt_pos);
					input_report_abs(sw->input, ABS_Y,
						sw->y_flt_pos);
					input_report_abs(sw->input,
						ABS_PRESSURE, 1);
				}
			} else {
				/* When the user lifts off the sensor, configure
				 * the AD714X back to threshold interrupt mode.
				 */
				touchpad_use_thr_int(ad714x, idx);
				sw->state = IDLE;
				input_report_abs(sw->input, ABS_PRESSURE, 0);
				ad714x_debug("touchpad %d released\n", idx);
			}
			input_sync(sw->input);
		}
		break;
	default:
		break;
	}
}

static int ad714x_hw_detect(struct ad714x_chip *ad714x)
{
	unsigned short data;

	ad714x->read(ad714x->bus, AD714x_PARTID_REG, &data);
	switch (data & 0xFFF0) {
	case AD7147_PARTID:
		dev_info(&ad714x->bus->dev, "Found AD7147 captouch, rev:%d\n",
				data & 0xF);
		return 0;
	case AD7142_PARTID:
		dev_info(&ad714x->bus->dev, "Found AD7142 captouch, rev:%d\n",
				data & 0xF);
		return 0;
	default:
		dev_err(&ad714x->bus->dev, "Fail to detect AD714x captouch,\
				read ID is %04x\n", data);
		return -ENODEV;
	}
}

static int ad714x_hw_init(struct ad714x_chip *ad714x)
{
	int i, j;
	unsigned short reg_base;
	unsigned short data;

	/* configuration CDC and interrupts*/

	for (i = 0; i < STAGE_NUM; i++) {
		reg_base = AD714x_STAGECFG_REG + i * STAGE_CFGREG_NUM;
		for (j = 0; j < STAGE_CFGREG_NUM; j++)
			ad714x->write(ad714x->bus, reg_base + j,
				ad714x->hw->stage_cfg_reg[i][j]);
	}

	for (i = 0; i < SYS_CFGREG_NUM; i++)
		ad714x->write(ad714x->bus, AD714x_SYSCFG_REG + i,
			ad714x->hw->sys_cfg_reg[i]);

	ad714x->write(ad714x->bus, AD714x_STG_CAL_EN_REG, 0xFFF);

	/* clear all interrupts */
	ad714x->read(ad714x->bus, STG_LOW_INT_STA_REG, &data);
	ad714x->read(ad714x->bus, STG_HIGH_INT_STA_REG, &data);
	ad714x->read(ad714x->bus, STG_COM_INT_STA_REG, &data);

	return 0;
}

static irqreturn_t ad714x_interrupt_thread(int irq, void *data)
{
	struct ad714x_chip *ad714x = data;
	int i;

	mutex_lock(&ad714x->mutex);

	ad714x->read(ad714x->bus, STG_LOW_INT_STA_REG, &ad714x->l_state);
	ad714x->read(ad714x->bus, STG_HIGH_INT_STA_REG, &ad714x->h_state);
	ad714x->read(ad714x->bus, STG_COM_INT_STA_REG, &ad714x->c_state);

	ad714x_debug("%s l_state:%04x h_state:%04x c_stage:%04x\n", __func__,
			ad714x->l_state, ad714x->h_state, ad714x->c_state);

	for (i = 0; i < ad714x->hw->button_num; i++)
		button_state_machine(ad714x, i);
	for (i = 0; i < ad714x->hw->slider_num; i++)
		slider_state_machine(ad714x, i);
	for (i = 0; i < ad714x->hw->wheel_num; i++)
		wheel_state_machine(ad714x, i);
	for (i = 0; i < ad714x->hw->touchpad_num; i++)
		touchpad_state_machine(ad714x, i);

	mutex_unlock(&ad714x->mutex);

	return IRQ_HANDLED;
}

static irqreturn_t ad714x_interrupt(int irq, void *data)
{
	return IRQ_WAKE_THREAD;
}

#define MAX_DEVICE_NUM 8
static int __devinit ad714x_probe(struct ad714x_chip *ad714x)
{
	int ret = 0;
	struct input_dev *input[MAX_DEVICE_NUM];

	struct ad714x_driver_data *drv_data = NULL;

	struct ad714x_button_plat *bt_plat   = ad714x->hw->button;
	struct ad714x_slider_plat *sd_plat   = ad714x->hw->slider;
	struct ad714x_wheel_plat *wl_plat    = ad714x->hw->wheel;
	struct ad714x_touchpad_plat *tp_plat = ad714x->hw->touchpad;

	struct ad714x_button_drv *bt_drv   = NULL;
	struct ad714x_slider_drv *sd_drv   = NULL;
	struct ad714x_wheel_drv *wl_drv    = NULL;
	struct ad714x_touchpad_drv *tp_drv = NULL;

	int alloc_idx = 0, reg_idx = 0;
	int i;

	ret = ad714x_hw_detect(ad714x);
	if (ret)
		goto det_err;

	/*
	 * Allocate and register AD714x input device
	 */

	drv_data = kzalloc(sizeof(struct ad714x_driver_data), GFP_KERNEL);
	if (!drv_data) {
		dev_err(&ad714x->bus->dev,
			"Can't allocate memory for ad714x driver info\n");
		ret = -ENOMEM;
		goto fail_alloc_reg;
	}
	ad714x->sw = drv_data;

	/* a slider uses one input_dev instance */
	sd_drv = kzalloc(sizeof(struct ad714x_slider_drv) *
			ad714x->hw->slider_num, GFP_KERNEL);
	if (!sd_drv) {
		dev_err(&ad714x->bus->dev,
			"Can't allocate memory for slider info\n");
		ret = -ENOMEM;
		goto fail_alloc_reg;
	}

	for (i = 0; i < ad714x->hw->slider_num; i++) {
		input[alloc_idx] = input_allocate_device();
		if (!input[alloc_idx]) {
			dev_err(&ad714x->bus->dev,
				"Can't allocate input device %d\n", alloc_idx);
			ret = -ENOMEM;
			goto fail_alloc_reg;
		}
		alloc_idx++;

		__set_bit(EV_ABS, input[alloc_idx-1]->evbit);
		__set_bit(ABS_X, input[alloc_idx-1]->absbit);
		__set_bit(ABS_PRESSURE, input[alloc_idx-1]->absbit);
		input_set_abs_params(input[alloc_idx-1], ABS_X, 0,
			sd_plat->max_coord, 0, 0);
		input_set_abs_params(input[alloc_idx-1], ABS_PRESSURE, 0, 1,
			0, 0);

		input[alloc_idx-1]->id.bustype = BUS_I2C;

		ret = input_register_device(input[reg_idx]);
		if (ret) {
			dev_err(&ad714x->bus->dev,
				"Failed to register AD714x input device!\n");
			goto fail_alloc_reg;
		}
		reg_idx++;

		sd_drv[i].input = input[alloc_idx-1];
		ad714x->sw->slider = sd_drv;
	}

	/* a wheel uses one input_dev instance */
	wl_drv = kzalloc(sizeof(struct ad714x_wheel_drv) *
			ad714x->hw->wheel_num, GFP_KERNEL);
	if (!sd_drv) {
		dev_err(&ad714x->bus->dev,
			"Can't allocate memory for wheel info\n");
		ret = -ENOMEM;
		goto fail_alloc_reg;
	}

	for (i = 0; i < ad714x->hw->wheel_num; i++) {
		input[alloc_idx] = input_allocate_device();
		if (!input[alloc_idx]) {
			dev_err(&ad714x->bus->dev,
				"Can't allocate input device %d\n", alloc_idx);
			ret = -ENOMEM;
			goto fail_alloc_reg;
		}
		alloc_idx++;

		__set_bit(EV_ABS, input[alloc_idx-1]->evbit);
		__set_bit(ABS_WHEEL, input[alloc_idx-1]->absbit);
		__set_bit(ABS_PRESSURE, input[alloc_idx-1]->absbit);
		input_set_abs_params(input[alloc_idx-1], ABS_WHEEL, 0,
					wl_plat->max_coord, 0, 0);
		input_set_abs_params(input[alloc_idx-1], ABS_PRESSURE, 0, 1,
					0, 0);

		input[alloc_idx-1]->id.bustype = BUS_I2C;

		ret = input_register_device(input[reg_idx]);
		if (ret) {
			dev_err(&ad714x->bus->dev,
				"Failed to register AD714x input device!\n");
			goto fail_alloc_reg;
		}
		reg_idx++;

		wl_drv[i].input = input[alloc_idx-1];
		ad714x->sw->wheel = wl_drv;
	}

	/* a touchpad uses one input_dev instance */
	tp_drv = kzalloc(sizeof(struct ad714x_touchpad_drv) *
			ad714x->hw->touchpad_num, GFP_KERNEL);
	if (!tp_drv) {
		dev_err(&ad714x->bus->dev,
			"Can't allocate memory for touchpad info\n");
		ret = -ENOMEM;
		goto fail_alloc_reg;
	}

	for (i = 0; i < ad714x->hw->touchpad_num; i++) {
		input[alloc_idx] = input_allocate_device();
		if (!input[alloc_idx]) {
			dev_err(&ad714x->bus->dev,
				"Can't allocate input device %d\n", alloc_idx);
			ret = -ENOMEM;
			goto fail_alloc_reg;
		}
		alloc_idx++;

		__set_bit(EV_ABS, input[alloc_idx-1]->evbit);
		__set_bit(ABS_X, input[alloc_idx-1]->absbit);
		__set_bit(ABS_Y, input[alloc_idx-1]->absbit);
		__set_bit(ABS_PRESSURE, input[alloc_idx-1]->absbit);
		input_set_abs_params(input[alloc_idx-1], ABS_X, 0,
			tp_plat->x_max_coord, 0, 0);
		input_set_abs_params(input[alloc_idx-1], ABS_Y, 0,
			tp_plat->y_max_coord, 0, 0);
		input_set_abs_params(input[alloc_idx-1], ABS_PRESSURE, 0, 1,
					0, 0);

		input[alloc_idx-1]->id.bustype = BUS_I2C;

		ret = input_register_device(input[reg_idx]);
		if (ret) {
			dev_err(&ad714x->bus->dev,
				"Failed to register AD714x input device!\n");
			goto fail_alloc_reg;
		}
		reg_idx++;

		tp_drv[i].input = input[alloc_idx-1];
		ad714x->sw->touchpad = tp_drv;
	}

	/* all buttons use one input node */
	input[alloc_idx] = input_allocate_device();
	if (!input[alloc_idx]) {
		dev_err(&ad714x->bus->dev, "Can't allocate input device %d\n",
			alloc_idx);
		ret = -ENOMEM;
		goto fail_alloc_reg;
	}
	alloc_idx++;

	bt_drv = kzalloc(sizeof(struct ad714x_button_drv) *
			ad714x->hw->button_num, GFP_KERNEL);
	if (!bt_drv) {
		dev_err(&ad714x->bus->dev,
			"Can't allocate memory for button info\n");
		ret = -ENOMEM;
		goto fail_alloc_reg;
	}

	__set_bit(EV_KEY, input[alloc_idx-1]->evbit);
	for (i = 0; i < ad714x->hw->button_num; i++)
		input[alloc_idx-1]->keybit[bt_plat[0].event] |=
			BIT_MASK(bt_plat[i].event);

	input[alloc_idx-1]->id.bustype = BUS_I2C;

	ret = input_register_device(input[reg_idx]);
	if (ret) {
		dev_err(&ad714x->bus->dev,
			"Failed to register AD714x input device!\n");
		goto fail_alloc_reg;
	}
	reg_idx++;

	for (i = 0; i < ad714x->hw->button_num; i++) {
		input[alloc_idx-1]->keybit[bt_plat[i].event] |=
			BIT_MASK(bt_plat[i].event);
		bt_drv[i].input = input[alloc_idx-1];
	}
	ad714x->sw->button = bt_drv;

	/* initilize and request sw/hw resources */

	ad714x_hw_init(ad714x);
	mutex_init(&ad714x->mutex);

	if (ad714x->bus->irq > 0) {
		ret = request_threaded_irq(ad714x->bus->irq, ad714x_interrupt,
				ad714x_interrupt_thread, IRQF_TRIGGER_FALLING,
				"ad714x_captouch", ad714x);
		if (ret) {
			dev_err(&ad714x->bus->dev, "Can't allocate irq %d\n",
					ad714x->bus->irq);
			goto fail_irq;
		}
	} else
		dev_warn(&ad714x->bus->dev, "IRQ not configured!\n");

	return 0;

fail_irq:
fail_alloc_reg:
	for (i = 0; i < reg_idx; i++)
		input_unregister_device(input[i]);
	for (i = 0; i < alloc_idx; i++)
		input_free_device(input[i]);
	kfree(bt_drv); /* kfree(NULL) is safe check is not required */
	kfree(sd_drv);
	kfree(wl_drv);
	kfree(tp_drv);
	kfree(drv_data);
det_err:
	return ret;
}

static int __devexit ad714x_remove(struct ad714x_chip *ad714x)
{
	int i;

	struct ad714x_driver_data *drv_data = ad714x->sw;
	struct ad714x_button_drv *bt_drv   = ad714x->sw->button;
	struct ad714x_slider_drv *sd_drv   = ad714x->sw->slider;
	struct ad714x_wheel_drv *wl_drv    = ad714x->sw->wheel;
	struct ad714x_touchpad_drv *tp_drv = ad714x->sw->touchpad;

	/* free irq hardware resource */

	if (ad714x->bus->irq > 0)
		free_irq(ad714x->bus->irq, ad714x);

	/* unregister and free all input devices */

	for (i = 0; i < ad714x->hw->slider_num; i++) {
		input_unregister_device(ad714x->sw->slider[i].input);
		input_free_device(ad714x->sw->slider[i].input);
	}

	for (i = 0; i < ad714x->hw->wheel_num; i++) {
		input_unregister_device(ad714x->sw->wheel[i].input);
		input_free_device(ad714x->sw->wheel[i].input);
	}

	for (i = 0; i < ad714x->hw->touchpad_num; i++) {
		input_unregister_device(ad714x->sw->touchpad[i].input);
		input_free_device(ad714x->sw->touchpad[i].input);
	}

	input_unregister_device(ad714x->sw->button[0].input);
	input_free_device(ad714x->sw->button[0].input);

	/* free all memories for software flow */
	kfree(bt_drv); /* kfree(NULL) is safe check is not required */
	kfree(sd_drv);
	kfree(wl_drv);
	kfree(tp_drv);
	kfree(drv_data);
	kfree(ad714x);

	return 0;
}

#if defined(CONFIG_INPUT_AD714X_SPI)
int ad714x_spi_read(struct spi_device *spi, unsigned short reg,
		unsigned short *data)
{
	int ret;
	unsigned short tx[2];
	unsigned short rx[2];
	struct spi_transfer t = {
		.tx_buf = tx,
		.rx_buf = rx,
		.len = 4,
	};
	struct spi_message m;

	tx[0] = (AD714x_SPI_ADDR << AD714x_SPI_ADDR_SHFT) |
		(AD714x_SPI_READ << AD714x_SPI_READ_SHFT) | reg;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(spi, &m);

	if (ret < 0) {
		dev_err(&spi->dev, "SPI read error\n");
		return ret;
	}

	*data = rx[1];
	return ret;
}

int ad714x_spi_write(struct spi_device *spi, unsigned short reg,
		unsigned short data)
{
	int ret = 0;
	unsigned short tx[2];
	struct spi_transfer t = {
		.tx_buf = tx,
		.len = 4,
	};
	struct spi_message m;

	tx[0] = (AD714x_SPI_ADDR << AD714x_SPI_ADDR_SHFT) | reg;
	tx[1] = data;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	ret = spi_sync(spi, &m);
	if (ret < 0)
		dev_err(&spi->dev, "SPI write error\n");

	return ret;
}

static int __devinit ad714x_spi_probe(struct spi_device *spi)
{
	int ret = 0;
	struct ad714x_chip *chip;

	if (spi->dev.platform_data == NULL) {
		dev_err(&spi->dev, "platform data for ad714x doesn't exist\n");
		return -ENODEV;
	}

	chip = kzalloc(sizeof(struct ad714x_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->bus = spi;
	chip->read = ad714x_spi_read;
	chip->write = ad714x_spi_write;
	chip->hw = spi->dev.platform_data;
	spi_set_drvdata(spi, chip);

	/* common probe not related with spi/i2c */
	ret = ad714x_probe(chip);
	if (ret)
		kfree(chip);

	return ret;
}

static int __devexit ad714x_spi_remove(struct spi_device *spi)
{
	struct ad714x_chip *chip = spi_get_drvdata(spi);
	ad714x_remove(chip);

	return 0;
}

static struct spi_driver ad714x_spi_driver = {
	.driver = {
		.name	= "ad714x_captouch",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= ad714x_spi_probe,
	.remove		= __devexit_p(ad714x_spi_remove),
};

static int __init ad714x_init(void)
{
	int ret;

	ret = spi_register_driver(&ad714x_spi_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register ad714x SPI driver: %d\n",
				ret);
	}

	return ret;
}

static void __exit ad714x_exit(void)
{
	spi_unregister_driver(&ad714x_spi_driver);
}
#else
static int ad714x_i2c_write(struct i2c_client *client, unsigned short reg,
		unsigned short data)
{
	int ret = 0;
	u8 tx[4];

	/* Do raw I2C, not smbus compatible */
	tx[0] = (reg & 0xFF00) >> 8;
	tx[1] = (reg & 0x00FF);
	tx[2] = (data & 0xFF00) >> 8;
	tx[3] = data & 0x00FF;

	ret = i2c_master_send(client, tx, 4);
	if (ret < 0)
		dev_err(&client->dev, "I2C write error\n");

	return ret;
}

static int ad714x_i2c_read(struct i2c_client *client, unsigned short reg,
		unsigned short *data)
{
	int ret = 0;
	u8 tx[2];
	u8 rx[2];

	/* Do raw I2C, not smbus compatible */
	tx[0] = (reg & 0xFF00) >> 8;
	tx[1] = (reg & 0x00FF);

	ret = i2c_master_send(client, tx, 2);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read error\n");
		return ret;
	}

	ret = i2c_master_recv(client, rx, 2);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read error\n");
		return ret;
	}

	*data = rx[0];
	*data = (*data << 8) | rx[1];

	return ret;
}

static int __devinit ad714x_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct ad714x_chip *chip;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev,
			"platform data for ad714x doesn't exist\n");
		return -ENODEV;
	}

	chip = kzalloc(sizeof(struct ad714x_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->bus = client;
	chip->read = ad714x_i2c_read;
	chip->write = ad714x_i2c_write;
	chip->hw = client->dev.platform_data;
	i2c_set_clientdata(client, chip);

	/* common probe not related with spi/i2c */
	ret = ad714x_probe(chip);
	if (ret)
		kfree(chip);

	return ret;
}

static int __devexit ad714x_i2c_remove(struct i2c_client *client)
{
	struct ad714x_chip *chip = i2c_get_clientdata(client);
	ad714x_remove(chip);

	return 0;
}

static const struct i2c_device_id ad714x_id[] = {
	{ "ad714x_captouch", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad714x_id);

static struct i2c_driver ad714x_i2c_driver = {
	.driver = {
		.name = "ad714x_captouch",
	},
	.probe = ad714x_i2c_probe,
	.remove = __devexit_p(ad714x_i2c_remove),
	.id_table = ad714x_id,
};

static int __init ad714x_init(void)
{
	return i2c_add_driver(&ad714x_i2c_driver);
}

static void __exit ad714x_exit(void)
{
	i2c_del_driver(&ad714x_i2c_driver);
}
#endif

module_init(ad714x_init);
module_exit(ad714x_exit);

MODULE_DESCRIPTION("ad714x captouch driver");
MODULE_AUTHOR("Barry Song <21cnbao@gmail.com>");
MODULE_LICENSE("GPL");

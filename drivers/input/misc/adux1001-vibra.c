/*
 * ADUX1001 Smart Electromagnetic Actuator Haptic Driver
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/pwm.h>

#include <linux/input/adux1001.h>

/*
 * ADUX1001 Registers
 */
#define ADUX1001_CONTROL	0x01 /* Reset and interrupt commands R/W */
#define ADUX1001_CONFIG		0x02 /* Operational mode bits and other
					configuration settings R/W */
#define ADUX1001_OUTPUT_CONFIG	0x03 /* Configure the output current and the
					ARC closed loop gain. R/W */
#define ADUX1001_OUTPUT_RATE	0x04 /* Output rate for I2C streaming and LLR or
					I2C triggered arbitrary waveform R/W */
#define ADUX1001_ARB_SEL	0x05 /* Specifies how many of the 10 ARB
					registers to use for an LLR or I2C
					arbitrary waveform R/W */
#define ADUX1001_ARB0		0x06 /* Arbitrary waveform buffer 0
					amplitude and sign R/W */
#define ADUX1001_ARB1		0x07 /* Arbitrary waveform buffer 1
					amplitude and sign R/W */
#define ADUX1001_ARB2		0x08 /* Arbitrary waveform buffer 2
					amplitude and sign R/W */
#define ADUX1001_ARB3		0x09 /* Arbitrary waveform buffer 3
					amplitude and sign R/W */
#define ADUX1001_ARB4		0x0A /* Arbitrary waveform buffer 4
					amplitude and sign R/W */
#define ADUX1001_ARB5		0x0B /* Arbitrary waveform buffer 5
					amplitude and sign R/W */
#define ADUX1001_ARB6		0x0C /* Arbitrary waveform buffer 6
					amplitude and sign R/W */
#define ADUX1001_ARB7		0x0D /* Arbitrary waveform buffer 7
					amplitude and sign R/W */
#define ADUX1001_ARB8		0x0E /* Arbitrary waveform buffer 8
					amplitude and sign R/W */
#define ADUX1001_ARB9		0x0F /* Arbitrary waveform buffer 9
					amplitude and sign R/W */
#define ADUX1001_I2C_STREAM	0x10 /* I2C streaming buffer register R/W */
#define ADUX1001_ACTIVE		0x11 /* Active commands, I2C mode output R/W */
#define ADUX1001_CALIBRATE	0x12 /* Calibration configuration and
					active command bits R/W */
#define ADUX1001_SLRA_CAL0	0x13 /* Smart LRA mode calibration results
					register 0. Maximum positive back EMF
					value R/W */
#define ADUX1001_SLRA_CAL1	0x14 /* Smart LRA mode calibration results
					register 1. Maximum positive back EMF
					value R/W */
#define ADUX1001_SLRA_CAL2	0x15 /* Smart LRA mode calibration results
					register 2. Maximum negative back EMF
					value R/W */
#define ADUX1001_SLRA_CAL3	0x16 /* Smart LRA mode calibration results
					register 3. Maximum negative back EMF
					value R/W */
#define ADUX1001_RESERVED0	0x17 /* LRA specific values (RESET: 0x63) */
#define ADUX1001_RESERVED1	0x18 /* LRA specific values (RESET: 0x10) */
#define ADUX1001_RESERVED2	0x19 /* LRA specific values (RESET: 0x41) */
#define ADUX1001_RESERVED3	0x1A /* LRA specific values (RESET: 0x5C) */
#define ADUX1001_RESERVED4	0x1B /* LRA specific values (RESET: 0x02) */

#define ADUX1001_STATUS		0x1C /* Status flags R */
#define ADUX1001_VERSION	0x1D /* ADUX1001 silicon revision R */

/*
 * ADUX1001 Bit masks
 */

/* CONTROL, Register Address 0x01, Reset 0x00 */
#define ADUX1001_RESET			(1 << 1)
#define ADUX1001_I2C_INTERRUPT		(1 << 0)

/* CONFIG, Register Address 0x02, Reset 0x64 */
#define ADUX1001_UVLO_EN(x)		((x) << 6)
#define ADUX1001_OCLO_EN(x)		((x) << 6)
#define ADUX1001_PWM_PRIORITY(x)	((x) << 3)
#define ADUX1001_INTERFACE_MODE(x)	((x) << 2)
#define ADUX1001_ACTUATOR_SEL(x)	((x) << 0)

/* OUTPUT_CONFIG, Register Address 0x03, Reset 0x44 */
#define ADUX1001_LOOP_GAIN(x)		(((x) & 0xF) << 4)
#define ADUX1001_OUTPUT_CURRENT(x)	(((x) & 0xF) << 0)

/* OUTPUT_RATE, Register Address 0x04, Reset 0x01 */
#define ADUX1001_LRA_OUTPUT_UNIT(x)	((x) << 7)
#define ADUX1001_OUTPUT_RATE_SEL(x)	(((x) & 0x7F) << 0)

/* ARB_SEL, Register Address 0x05, Reset 0x0A */
#define ADUX1001_ARB_NUM_SEL(x)		(((x) & 0xF) << 0)

/* ACTIVE, Register Address 0x11, Reset 0x00 */
#define ADUX1001_LLR_RESUME		(1 << 2)
#define ADUX1001_I2C_MODE_SEL		(1 << 1)
#define ADUX1001_I2C_MODE_ACTIVATE	(1 << 0)

/* CALIBRATE, Register Address 0x12, Reset 0x00 */
#define ADUX1001_PERIOD_CAL_CYCLES(x)	(((x) & 0x7) << 5)
#define ADUX1001_AMP_CAL_CYCLES(x)	(((x) & 0xF) << 1)
#define ADUX1001_CALIBRATE_EN		(1 << 0)

/* STATUS, Register Address 0x1C, Reset 0x20 */
#define ADUX1001_FIFO_EMPTY		(1 << 5)
#define ADUX1001_FIFO_FULL		(1 << 4)
#define ADUX1001_LRA_FAULT		(1 << 3)
#define ADUX1001_UVLO_FLAG		(1 << 2)
#define ADUX1001_OCLO_FLAG		(1 << 1)
#define ADUX1001_BUSY			(1 << 0)

/* VERSION, Register Address 0x1D, Reset 0x00 */
#define ADUX1001_VERSION_MASK		0xF

#define ADUX1001_POWERUP_DELAY_US	100 /* 100us */
#define ADUX1001_MAX_PWM_PERIODE_NS	50000
#define ADUX1001_MIN_PWM_PERIODE_NS	5000

struct adux1001_chip {
	struct i2c_client *client;
	struct adux1001_vibra_platform_data *pdata;
	struct input_dev *input;
	struct pwm_device *pwm;
	struct work_struct work;
	bool use_shutdown;
	int period;
	int duty;
};

static void adux1001_actuator_set(struct adux1001_chip *chip, unsigned duty)
{
	int ret;

	if (duty) {
		ret = pwm_config(chip->pwm,
				 chip->duty, chip->period);
		if (ret < 0)
			dev_err(&chip->client->dev,
				"pwm_config failed: duty %d period %d\n",
				chip->duty, chip->period);

		ret = pwm_enable(chip->pwm);
		if (ret < 0)
			dev_err(&chip->client->dev, "pwm_enable failed\n");
	} else {
		pwm_disable(chip->pwm);
	}
}

static int adux1001_calibrate(struct i2c_client *client)
{
	struct adux1001_chip *chip = i2c_get_clientdata(client);
	const struct adux1001_vibra_platform_data *pdata = chip->pdata;
	unsigned val, delay_ms;
	int ret;

	val = ADUX1001_PERIOD_CAL_CYCLES(pdata->period_calibration_cycles) |
		ADUX1001_AMP_CAL_CYCLES(pdata->amp_calibration_cycles) |
		ADUX1001_CALIBRATE_EN;

	ret = i2c_smbus_write_byte_data(client, ADUX1001_CALIBRATE, val);
	if (ret < 0) {
		dev_err(&client->dev, "i2c write failed\n");
		return ret;
	}
	/* assume default resonant period value (6.55ms) */
	delay_ms = pdata->amp_calibration_cycles * 7;

	do {
		msleep(delay_ms);
		val = i2c_smbus_read_byte_data(client, ADUX1001_STATUS);
		delay_ms = 4;
	} while (val & ADUX1001_BUSY);

	if (val & (ADUX1001_LRA_FAULT |
		   ADUX1001_UVLO_FLAG |
		   ADUX1001_OCLO_FLAG)) {
		dev_err(&client->dev, "LRA Fault\n");
		return -EFAULT;
	}

	return 0;
}

static int adux1001_read_calibdata(struct i2c_client *client,
				   struct adux1001_calib_data *calib_data)
{
	unsigned char *data = (unsigned char *) calib_data;
	int reg, ret;

	for (reg = ADUX1001_SLRA_CAL0; reg <= ADUX1001_SLRA_CAL3; reg++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0)
			return ret;
		data[reg - ADUX1001_SLRA_CAL0] = ret;
	 }

	return 0;
}

static int adux1001_write_calibdata(struct i2c_client *client,
				    struct adux1001_calib_data *calib_data)
{
	unsigned char *data = (unsigned char *) calib_data;
	int reg, ret;

	for (reg = ADUX1001_SLRA_CAL0; reg <= ADUX1001_SLRA_CAL3; reg++) {
		ret = i2c_smbus_write_byte_data(client, reg,
						data[reg - ADUX1001_SLRA_CAL0]);
		if (ret < 0)
			return ret;
	 }

	return 0;
}

static int adux1001_write_reserved_data(struct i2c_client *client,
				struct adux1001_reserved_data *reserved_data)
{
	unsigned char *data = (unsigned char *) reserved_data;
	int reg, ret;

	for (reg = ADUX1001_RESERVED0; reg <= ADUX1001_RESERVED4; reg++) {
		ret = i2c_smbus_write_byte_data(client, reg,
						data[reg - ADUX1001_RESERVED0]);
		if (ret < 0)
			return ret;
	 }

	return 0;
}

static int adux1001_setup(struct i2c_client *client)
{
	struct adux1001_chip *chip = i2c_get_clientdata(client);
	const struct adux1001_vibra_platform_data *pdata = chip->pdata;
	struct adux1001_calib_data calib_data;
	int ret;
	unsigned val, config;

	/* Get the adux1001 out of shutdown mode */
	if (chip->use_shutdown) {
		gpio_set_value_cansleep(chip->pdata->gpio_shutdown, 1);
		udelay(ADUX1001_POWERUP_DELAY_US);
	}

	ret = i2c_smbus_write_byte_data(client, ADUX1001_CONTROL,
					ADUX1001_RESET);
	if (ret < 0) {
		dev_err(&client->dev, "i2c write failed\n");
		return ret;
	}

	val = i2c_smbus_read_byte_data(client, ADUX1001_VERSION);
	if ((val & ADUX1001_VERSION_MASK) <= 1)
		dev_err(&client->dev, "unsupported VERSION detected\n");

	if (pdata->reserved_data)
		adux1001_write_reserved_data(client, pdata->reserved_data);

	config = ADUX1001_UVLO_EN(!pdata->uvlo_dis) |
		ADUX1001_OCLO_EN(!pdata->oclo_dis) |
		ADUX1001_PWM_PRIORITY(pdata->pwm_priority) |
		ADUX1001_INTERFACE_MODE(1) |
		ADUX1001_ACTUATOR_SEL(pdata->actuator_is_erm);

	ret = i2c_smbus_write_byte_data(client, ADUX1001_CONFIG, config);
	if (ret < 0) {
		dev_err(&client->dev, "i2c write failed\n");
		return ret;
	}

	val = ADUX1001_LOOP_GAIN(pdata->loop_gain) |
		ADUX1001_OUTPUT_CURRENT(pdata->max_output_current);

	ret = i2c_smbus_write_byte_data(client, ADUX1001_OUTPUT_CONFIG, val);
	if (ret < 0) {
		dev_err(&client->dev, "i2c write failed\n");
		return ret;
	}

	val = ADUX1001_LRA_OUTPUT_UNIT(pdata->lra_output_unit_1ms) |
		ADUX1001_OUTPUT_RATE_SEL(pdata->output_rate);

	ret = i2c_smbus_write_byte_data(client, ADUX1001_OUTPUT_RATE, val);
	if (ret < 0) {
		dev_err(&client->dev, "i2c write failed\n");
		return ret;
	}

	if (pdata->get_calibdata) {
		ret = pdata->get_calibdata(client, &calib_data);
		if (ret < 0) {
			/* First time calibration */
			adux1001_calibrate(client);

			if (pdata->store_calibdata) {
				/* Get calibration values from registers */
				adux1001_read_calibdata(client, &calib_data);

				/* Have something else take core of storing
				 * them to none-volatile memory
				 */
				ret = pdata->store_calibdata(client,
							     &calib_data);
				if (ret < 0)
					dev_warn(&client->dev,
						 "failed to store_calibdata\n");
			}
		} else {
			/* get_calibdata returned success,
			 * use saved calibration data
			 */
			ret = adux1001_write_calibdata(client, &calib_data);
			if (ret < 0) {
				dev_err(&client->dev, "i2c write failed\n");
				return ret;
			}
		}
	} else {
		adux1001_calibrate(client);
	}

	/* Populate arbitrary waveform buffers, if available.
	 * Might be used in conjunction with the
	 * Low Latency Response (LLR) feature of the chip
	 */
	if (pdata->arb_wform_buffer_array &&
		(pdata->arb_wform_buffer_array_size > 0 &&
		pdata->arb_wform_buffer_array_size <= 10)) {
		for (val = 0; val < pdata->arb_wform_buffer_array_size; val++)
			ret = i2c_smbus_write_byte_data(client,
					ADUX1001_ARB0 + val,
					pdata->arb_wform_buffer_array[val]);
				if (ret < 0) {
					dev_err(&client->dev,
						"i2c write failed\n");
					return ret;
				 }

		ret = i2c_smbus_write_byte_data(client, ADUX1001_ARB_SEL,
				pdata->arb_wform_buffer_array_size);
		if (ret < 0) {
			dev_err(&client->dev, "i2c write failed\n");
			return ret;
		}
	}

	config &= ~ADUX1001_INTERFACE_MODE(1);
	config |= ADUX1001_INTERFACE_MODE(pdata->mode_sel & 1);

	ret = i2c_smbus_write_byte_data(client, ADUX1001_CONFIG, config);
	if (ret < 0) {
		dev_err(&client->dev, "i2c write failed\n");
		return ret;
	}

	return 0;
}

static void adux1001_worker(struct work_struct *work)
{
	struct adux1001_chip *chip;

	chip = container_of(work, struct adux1001_chip, work);
	adux1001_actuator_set(chip, chip->duty);
}

static int adux1001_play_effect(struct input_dev *dev, void *data,
				struct ff_effect *effect)
{
	struct adux1001_chip *chip = input_get_drvdata(dev);

	chip->duty = (chip->period * effect->u.rumble.strong_magnitude) /
			USHRT_MAX;
	if (!chip->duty)
		chip->duty = (chip->period *
			     (effect->u.rumble.weak_magnitude >> 1)) /
			     USHRT_MAX;

	schedule_work(&chip->work);
	return 0;
}

static int adux1001_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct adux1001_chip *chip;
	int ret;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c byte data not supported\n");
		return -EIO;
	}

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "pdata is not available\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct adux1001_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->pdata = client->dev.platform_data;
	chip->client = client;
	i2c_set_clientdata(client, chip);

	if (chip->pdata->setup) {
		ret = chip->pdata->setup(client, true);
		if (ret < 0) {
			dev_err(&client->dev, "setup callback failed!\n");
			goto err_free_mem;
		}
	}

	ret = gpio_is_valid(chip->pdata->gpio_shutdown);
	if (ret) {
		ret = gpio_request(chip->pdata->gpio_shutdown, id->name);
		if (ret) {
			dev_err(&client->dev, "gpio %d request failed\n",
					chip->pdata->gpio_shutdown);
			goto err_setup;
		}
		chip->use_shutdown = true;

		ret = gpio_direction_output(chip->pdata->gpio_shutdown, 0);
		if (ret) {
			dev_err(&client->dev, "gpio %d set direction failed\n",
						chip->pdata->gpio_shutdown);
			goto err_release_gpio;
		}
	}

	ret = adux1001_setup(chip->client);
	if (ret < 0) {
		dev_err(&client->dev, "device setup failed %d\n", ret);
		goto err_release_gpio;
	}

	/* Skip input device and pwm registration. Something else, such as the
	 * Immersion TouchSense kernel module controls the ADUX1001 PWM.
	 */
	if (chip->pdata->mode_sel == ADUX1001_PWM_MODE_EXT_ONLY)
		return 0;


	chip->pwm = pwm_request(chip->pdata->pwm_id, id->name);
	if (IS_ERR(chip->pwm)) {
		dev_err(&client->dev, "pwm request failed\n");
		ret = PTR_ERR(chip->pwm);
		goto err_release_gpio;
	}

	if (chip->pdata->pwm_periode_ns > ADUX1001_MAX_PWM_PERIODE_NS ||
		chip->pdata->pwm_periode_ns < ADUX1001_MIN_PWM_PERIODE_NS) {
		dev_warn(&client->dev, "pwm_periode_ns out of spec\n");
		chip->period = 40000; /* 25kHz */
	} else {
		chip->period = chip->pdata->pwm_periode_ns;
	}

	INIT_WORK(&chip->work, adux1001_worker);

	chip->input = input_allocate_device();
	if (!chip->input) {
		dev_err(&client->dev, "input device allocation failed\n");
		ret = -ENOMEM;
		goto err_release_pwm;
	}

	input_set_drvdata(chip->input, chip);
	chip->input->name = id->name;
	chip->input->dev.parent = &client->dev;
	chip->input->id.bustype = BUS_I2C;

	input_set_capability(chip->input, EV_FF, FF_RUMBLE);

	ret = input_ff_create_memless(chip->input, NULL,
					adux1001_play_effect);
	if (ret < 0) {
		dev_err(&client->dev, "failed to create force feedback\n");
		goto err_free_idev;
	}

	ret = input_register_device(chip->input);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register input device\n");
		goto err_ff_destroy;
	}

	return 0;

err_ff_destroy:
	input_ff_destroy(chip->input);
err_free_idev:
	input_free_device(chip->input);
err_release_pwm:
	pwm_free(chip->pwm);
err_release_gpio:
	if (chip->use_shutdown)
		gpio_free(chip->pdata->gpio_shutdown);
err_setup:
	if (chip->pdata->setup)
		chip->pdata->setup(client, true);
err_free_mem:
	kfree(chip);

	return ret;
}

static int adux1001_remove(struct i2c_client *client)
{
	struct adux1001_chip *chip = i2c_get_clientdata(client);

	if (chip->pdata->mode_sel != ADUX1001_PWM_MODE_EXT_ONLY) {
		cancel_work_sync(&chip->work);
		input_unregister_device(chip->input);
		adux1001_actuator_set(chip, false);
		pwm_free(chip->pwm);
	}

	i2c_smbus_write_byte_data(client, ADUX1001_CONTROL,
				  ADUX1001_RESET);

	if (chip->use_shutdown)
		gpio_free(chip->pdata->gpio_shutdown);

	if (chip->pdata->setup)
		chip->pdata->setup(client, false);

	kfree(chip);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int adux1001_suspend(struct device *dev)
{
	struct adux1001_chip *chip = dev_get_drvdata(dev);
	int ret;

	if (chip->pdata->mode_sel != ADUX1001_PWM_MODE_EXT_ONLY) {
		cancel_work_sync(&chip->work);
		adux1001_actuator_set(chip, 0);
	}

	if (chip->use_shutdown)
		gpio_set_value_cansleep(chip->pdata->gpio_shutdown, 0);

	if (chip->pdata->setup) {
		ret = chip->pdata->setup(chip->client, false);
		if (ret) {
			dev_err(dev, "setup failed\n");
			return ret;
		}
	}
	return 0;
}

static int adux1001_resume(struct device *dev)
{
	struct adux1001_chip *chip = dev_get_drvdata(dev);
	int ret;

	if (chip->pdata->setup) {
		ret = chip->pdata->setup(chip->client, true);
		if (ret) {
			dev_err(dev, "setup failed\n");
			return ret;
		}
	}

	adux1001_setup(chip->client);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(adux1001_pm_ops, adux1001_suspend, adux1001_resume);

static const struct i2c_device_id adux1001_id[] = {
	{"adux1001", 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, adux1001_id);

static struct i2c_driver adux1001_driver = {
	.driver = {
		.name = "adux1001",
		.pm = &adux1001_pm_ops,
		.owner = THIS_MODULE,
	},
	.probe = adux1001_probe,
	.remove = adux1001_remove,
	.id_table = adux1001_id,
};

static int __init adux1001_init(void)
{
	return i2c_add_driver(&adux1001_driver);
}
module_init(adux1001_init);

static void __exit adux1001_exit(void)
{
	i2c_del_driver(&adux1001_driver);
}
module_exit(adux1001_exit);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("ADUX1001 Smart Electromagnetic Actuator Haptic Driver");
MODULE_LICENSE("GPL v2");

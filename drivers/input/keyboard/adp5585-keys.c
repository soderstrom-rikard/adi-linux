/*
 * File: drivers/input/keyboard/adp5585_keys.c
 * Description:  keypad driver for ADP5585
  *		 I2C QWERTY Keypad and IO Expander
 * Bugs: Enter bugs at http://blackfin.uclinux.org/
 *
 * Copyright (C) 2010 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include <linux/input/adp5585.h>

#define ADP5585_ID			0x00
#define ADP5585_INT_STATUS		0x01
#define ADP5585_STATUS			0x02
#define ADP5585_FIFO_1			0x03
#define ADP5585_FIFO_2			0x04
#define ADP5585_FIFO_3			0x05
#define ADP5585_FIFO_4			0x06
#define ADP5585_FIFO_5			0x07
#define ADP5585_FIFO_6			0x08
#define ADP5585_FIFO_7			0x09
#define ADP5585_FIFO_8			0x0A
#define ADP5585_FIFO_9			0x0B
#define ADP5585_FIFO_10			0x0C
#define ADP5585_FIFO_11			0x0D
#define ADP5585_FIFO_12			0x0E
#define ADP5585_FIFO_13			0x0F
#define ADP5585_FIFO_14			0x10
#define ADP5585_FIFO_15			0x11
#define ADP5585_FIFO_16			0x12
#define ADP5585_GPI_INT_STAT_A		0x13
#define ADP5585_GPI_INT_STAT_B		0x14
#define ADP5585_GPI_STATUS_A		0x15
#define ADP5585_GPI_STATUS_B		0x16
#define ADP5585_RPULL_CONFIG_A		0x17
#define ADP5585_RPULL_CONFIG_B		0x18
#define ADP5585_RPULL_CONFIG_C		0x19
#define ADP5585_RPULL_CONFIG_D		0x1A
#define ADP5585_GPI_INT_LEVEL_A		0x1B
#define ADP5585_GPI_INT_LEVEL_B		0x1C
#define ADP5585_GPI_EVENT_EN_A		0x1D
#define ADP5585_GPI_EVENT_EN_B		0x1E
#define ADP5585_GPI_INTERRUPT_EN_A	0x1F
#define ADP5585_GPI_INTERRUPT_EN_B	0x20
#define ADP5585_DEBOUNCE_DIS_A		0x21
#define ADP5585_DEBOUNCE_DIS_B		0x22
#define ADP5585_GPO_DATA_OUT_A		0x23
#define ADP5585_GPO_DATA_OUT_B		0x24
#define ADP5585_GPO_OUT_MODE_A		0x25
#define ADP5585_GPO_OUT_MODE_B		0x26
#define ADP5585_GPIO_DIRECTION_A	0x27
#define ADP5585_GPIO_DIRECTION_B	0x28
#define ADP5585_RESET1_EVENT_A		0x29
#define ADP5585_RESET1_EVENT_B		0x2A
#define ADP5585_RESET1_EVENT_C		0x2B
#define ADP5585_RESET2_EVENT_A		0x2C
#define ADP5585_RESET2_EVENT_B		0x2D
#define ADP5585_RESET_CFG		0x2E
#define ADP5585_PWM_OFFT_LOW		0x2F
#define ADP5585_PWM_OFFT_HIGH		0x30
#define ADP5585_PWM_ONT_LOW		0x31
#define ADP5585_PWM_ONT_HIGH		0x32
#define ADP5585_PWM_CFG			0x33
#define ADP5585_LOGIC_CFG		0x34
#define ADP5585_LOGIC_FF_CFG		0x35
#define ADP5585_LOGIC_INT_EVENT_EN	0x36
#define ADP5585_POLL_PTIME_CFG		0x37
#define ADP5585_PIN_CONFIG_A		0x38
#define ADP5585_PIN_CONFIG_B		0x39
#define ADP5585_PIN_CONFIG_D		0x3A
#define ADP5585_GENERAL_CFG		0x3B
#define ADP5585_INT_EN			0x3C

/* ID Register */
#define ADP5585_DEVICE_ID_MASK	0xF
#define ADP5585_MAN_ID_MASK	0xF
#define ADP5585_MAN_ID_SHIFT	4
#define ADP5585_MAN_ID		0x02

/* GENERAL_CFG Register */
#define OSC_EN		(1 << 7)
#define CORE_CLK(x)	(((x) & 0x3) << 5)
#define INT_CFG		(1 << 1)
#define RST_CFG		(1 << 0)

/* INT_EN Register */
#define LOGIC_IEN	(1 << 4)
#define OVRFLOW_IEN	(1 << 2)
#define GPI_IEN		(1 << 1)
#define EVENT_IEN	(1 << 0)

/* Interrupt Status Register */
#define LOGIC_INT	(1 << 4)
#define OVRFLOW_INT	(1 << 2)
#define GPI_INT		(1 << 1)
#define EVENT_INT	(1 << 0)

/* STATUS Register */
#define LOGIC_STAT	(1 << 6)
#define KEC		0xF

/* PIN_CONFIG_D Register */
#define C4_EXTEND_CFG	(1 << 6)	/* RESET2 */
#define R4_EXTEND_CFG	(1 << 5)	/* RESET1 */

#define PTIME_MASK	0x3

/* Key Event Register xy */
#define KEY_EV_PRESSED		(1 << 7)
#define KEY_EV_MASK		0x7F

#define KEYP_MAX_EVENT		16

#define MAXGPIO			11 /* 10 on the ADP5585-01, 11 on ADP5585-02 */
#define ADP_BANK(offs)		((offs) > ADP_MAX_ROW_NUM)
#define ADP_BIT(offs)		(offs > ADP_MAX_ROW_NUM ? \
				1u << (offs - ADP_COL_SHIFT) : 1u << (offs))
enum {
	ADP5585_01,
	ADP5585_02
};

struct adp5585_kpad {
	struct i2c_client *client;
	struct input_dev *input;
	unsigned short keycode[ADP5585_KEYMAPSIZE];
	const struct adp5585_gpi_map *gpimap;
	unsigned short gpimapsize;
	unsigned extend_cfg;
	bool support_row5;
#ifdef CONFIG_GPIOLIB
	unsigned char gpiomap[MAXGPIO];
	bool export_gpio;
	struct gpio_chip gc;
	struct mutex gpio_lock;	/* Protect cached dir, dat_out */
	u8 dat_out[2];
	u8 dir[2];
#endif
};

static int adp5585_read(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "Read Error\n");

	return ret;
}

static int adp5585_write(struct i2c_client *client, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

#ifdef CONFIG_GPIOLIB
static int adp5585_gpio_get_value(struct gpio_chip *chip, unsigned off)
{
	struct adp5585_kpad *kpad = container_of(chip, struct adp5585_kpad, gc);
	unsigned int bank = ADP_BANK(kpad->gpiomap[off]);
	unsigned int bit = ADP_BIT(kpad->gpiomap[off]);

	return !!(adp5585_read(kpad->client, ADP5585_GPI_STATUS_A + bank) &
		  bit);
}

static void adp5585_gpio_set_value(struct gpio_chip *chip,
				   unsigned off, int val)
{
	struct adp5585_kpad *kpad = container_of(chip, struct adp5585_kpad, gc);
	unsigned int bank = ADP_BANK(kpad->gpiomap[off]);
	unsigned int bit = ADP_BIT(kpad->gpiomap[off]);

	mutex_lock(&kpad->gpio_lock);

	if (val)
		kpad->dat_out[bank] |= bit;
	else
		kpad->dat_out[bank] &= ~bit;

	adp5585_write(kpad->client, ADP5585_GPO_DATA_OUT_A + bank,
		      kpad->dat_out[bank]);

	mutex_unlock(&kpad->gpio_lock);
}

static int adp5585_gpio_direction_input(struct gpio_chip *chip, unsigned off)
{
	struct adp5585_kpad *kpad = container_of(chip, struct adp5585_kpad, gc);
	unsigned int bank = ADP_BANK(kpad->gpiomap[off]);
	unsigned int bit = ADP_BIT(kpad->gpiomap[off]);
	int ret;

	mutex_lock(&kpad->gpio_lock);

	kpad->dir[bank] &= ~bit;
	ret =
	    adp5585_write(kpad->client, ADP5585_GPIO_DIRECTION_A + bank,
			  kpad->dir[bank]);

	mutex_unlock(&kpad->gpio_lock);

	return ret;
}

static int adp5585_gpio_direction_output(struct gpio_chip *chip,
					 unsigned off, int val)
{
	struct adp5585_kpad *kpad = container_of(chip, struct adp5585_kpad, gc);
	unsigned int bank = ADP_BANK(kpad->gpiomap[off]);
	unsigned int bit = ADP_BIT(kpad->gpiomap[off]);
	int ret;

	mutex_lock(&kpad->gpio_lock);

	kpad->dir[bank] |= bit;

	if (val)
		kpad->dat_out[bank] |= bit;
	else
		kpad->dat_out[bank] &= ~bit;

	ret = adp5585_write(kpad->client, ADP5585_GPO_DATA_OUT_A + bank,
			    kpad->dat_out[bank]);
	ret |= adp5585_write(kpad->client, ADP5585_GPIO_DIRECTION_A + bank,
			     kpad->dir[bank]);

	mutex_unlock(&kpad->gpio_lock);

	return ret;
}

static int  adp5585_build_gpiomap(struct adp5585_kpad *kpad,
				const struct adp5585_kpad_platform_data *pdata)
{
	bool pin_used[MAXGPIO];
	int n_unused = 0;
	int i;

	memset(pin_used, false, sizeof(pin_used));

	for (i = 0; i < MAXGPIO; i++)
		if (pdata->keypad_en_mask & (1 << i))
			pin_used[i] = true;

	for (i = 0; i < kpad->gpimapsize; i++)
		pin_used[kpad->gpimap[i].pin - ADP5585_GPI_PIN_BASE] = true;

	if (kpad->extend_cfg & R4_EXTEND_CFG)
		pin_used[4] = true;

	if (kpad->extend_cfg & C4_EXTEND_CFG)
		pin_used[10] = true;

	if (!kpad->support_row5)
		pin_used[5] = true;

	for (i = 0; i < MAXGPIO; i++)
		if (!pin_used[i])
			kpad->gpiomap[n_unused++] = i;

	return n_unused;
}

static int  adp5585_gpio_add(struct adp5585_kpad *kpad)
{
	struct device *dev = &kpad->client->dev;
	const struct adp5585_kpad_platform_data *pdata = dev->platform_data;
	const struct adp5585_gpio_platform_data *gpio_data = pdata->gpio_data;
	int i, error;

	if (!gpio_data)
		return 0;

	kpad->gc.ngpio = adp5585_build_gpiomap(kpad, pdata);
	if (kpad->gc.ngpio == 0) {
		dev_info(dev, "No unused gpios left to export\n");
		return 0;
	}

	kpad->export_gpio = true;

	kpad->gc.direction_input = adp5585_gpio_direction_input;
	kpad->gc.direction_output = adp5585_gpio_direction_output;
	kpad->gc.get = adp5585_gpio_get_value;
	kpad->gc.set = adp5585_gpio_set_value;
	kpad->gc.can_sleep = 1;

	kpad->gc.base = gpio_data->gpio_start;
	kpad->gc.label = kpad->client->name;
	kpad->gc.owner = THIS_MODULE;

	mutex_init(&kpad->gpio_lock);

	error = gpiochip_add(&kpad->gc);
	if (error) {
		dev_err(dev, "gpiochip_add failed, err: %d\n", error);
		return error;
	}

	for (i = 0; i <= ADP_BANK(MAXGPIO); i++) {
		kpad->dat_out[i] = adp5585_read(kpad->client,
						ADP5585_GPO_DATA_OUT_A + i);
		kpad->dir[i] =
		    adp5585_read(kpad->client, ADP5585_GPIO_DIRECTION_A + i);
	}

	if (gpio_data->setup) {
		error = gpio_data->setup(kpad->client,
					 kpad->gc.base, kpad->gc.ngpio,
					 gpio_data->context);
		if (error)
			dev_warn(dev, "setup failed, %d\n", error);
	}

	return 0;
}

static void  adp5585_gpio_remove(struct adp5585_kpad *kpad)
{
	struct device *dev = &kpad->client->dev;
	const struct adp5585_kpad_platform_data *pdata = dev->platform_data;
	const struct adp5585_gpio_platform_data *gpio_data = pdata->gpio_data;
	int error;

	if (!kpad->export_gpio)
		return;

	if (gpio_data->teardown) {
		error = gpio_data->teardown(kpad->client,
					    kpad->gc.base, kpad->gc.ngpio,
					    gpio_data->context);
		if (error)
			dev_warn(dev, "teardown failed %d\n", error);
	}

	error = gpiochip_remove(&kpad->gc);
	if (error)
		dev_warn(dev, "gpiochip_remove failed %d\n", error);
}
#else
static inline int adp5585_gpio_add(struct adp5585_kpad *kpad)
{
	return 0;
}

static inline void adp5585_gpio_remove(struct adp5585_kpad *kpad)
{
}
#endif

static void adp5585_report_events(struct adp5585_kpad *kpad, int ev_cnt)
{
	int i, j;

	for (i = 0; i < ev_cnt; i++) {
		int key = adp5585_read(kpad->client, ADP5585_FIFO_1 + i);
		int key_val = key & KEY_EV_MASK;

		if (key_val >= ADP5585_GPI_PIN_BASE
		    && key_val <= ADP5585_GPI_PIN_END) {
			for (j = 0; j < kpad->gpimapsize; j++) {
				if (key_val == kpad->gpimap[j].pin) {
					input_report_switch(kpad->input,
							    kpad->gpimap[j].
							    sw_evt,
							    key &
							    KEY_EV_PRESSED);
					break;
				}
			}
		} else {
			input_report_key(kpad->input,
					 kpad->keycode[key_val - 1],
					 key & KEY_EV_PRESSED);
		}
	}
}

static irqreturn_t adp5585_irq(int irq, void *handle)
{
	struct adp5585_kpad *kpad = handle;
	struct i2c_client *client = kpad->client;
	int status, ev_cnt;

	status = adp5585_read(client, ADP5585_INT_STATUS);

	if (status & OVRFLOW_INT)	/* Unlikely and should never happen */
		dev_err(&client->dev, "Event Overflow Error\n");

	if (status & EVENT_INT) {
		ev_cnt = adp5585_read(client, ADP5585_STATUS) & KEC;
		if (ev_cnt) {
			adp5585_report_events(kpad, ev_cnt);
			input_sync(kpad->input);
		}
	}
	adp5585_write(client, ADP5585_INT_STATUS, status);	/* Status is W1C */

	return IRQ_HANDLED;
}

static int  adp5585_get_evcode(struct adp5585_kpad *kpad,
	unsigned short key)
{
	int i;
	for (i = 0; i < ADP5585_KEYMAPSIZE; i++)
		if (key == kpad->keycode[i])
			return (i + 1) | KEY_EV_PRESSED;

	dev_err(&kpad->client->dev, "RESET/UNLOCK key not in keycode map\n");
	return -EINVAL;
}
static int  adp5585_setup(struct adp5585_kpad *kpad)
{
	struct i2c_client *client = kpad->client;
	const struct adp5585_kpad_platform_data *pdata =
	    client->dev.platform_data;
	int i, ret;
	unsigned char evt_mode1 = 0, evt_mode2 = 0;
	unsigned char pull_mask = 0;

	ret = adp5585_write(client, ADP5585_PIN_CONFIG_A,
			    pdata->keypad_en_mask & ADP_ROW_MASK);
	ret |= adp5585_write(client, ADP5585_PIN_CONFIG_B,
			     (pdata->keypad_en_mask >> ADP_COL_SHIFT) &
			     ADP_COL_MASK);

	for (i = 0; i < KEYP_MAX_EVENT; i++)
		ret |= adp5585_read(client, ADP5585_FIFO_1 + i);

	for (i = 0; i < pdata->gpimapsize; i++) {
		unsigned short pin = pdata->gpimap[i].pin;

		if (pin <= ADP5585_GPI_PIN_ROW_END)
			evt_mode1 |= (1 << (pin - ADP5585_GPI_PIN_ROW_BASE));
		else
			evt_mode2 |= (1 << (pin - ADP5585_GPI_PIN_COL_BASE));
	}

	if (pdata->gpimapsize) {
		ret |= adp5585_write(client, ADP5585_GPI_EVENT_EN_A, evt_mode1);
		ret |= adp5585_write(client, ADP5585_GPI_EVENT_EN_B, evt_mode2);
	}

	if (pdata->pull_dis_mask & pdata->pullup_en_100k &
	    pdata->pullup_en_300k & pdata->pulldown_en_300k)
		dev_warn(&client->dev, "Conflicting pull resistor config\n");

	for (i = 0; i <= ADP_MAX_ROW_NUM; i++) {
		unsigned val = 0;
		if (pdata->pullup_en_300k & (1 << i))
			val = 0;
		else if (pdata->pulldown_en_300k & (1 << i))
			val = 1;
		else if (pdata->pullup_en_100k & (1 << i))
			val = 2;
		else if (pdata->pull_dis_mask & (1 << i))
			val = 3;

		pull_mask |= val << (2 * (i & 0x3));

		if (i == 3 || i == ADP_MAX_ROW_NUM) {
			ret |= adp5585_write(client,
					     ADP5585_RPULL_CONFIG_A + (i >> 2),
					     pull_mask);
			pull_mask = 0;
		}
	}

	for (i = 0; i <= ADP_MAX_COL_NUM; i++) {
		unsigned val = 0;
		if (pdata->pullup_en_300k & (1 << (i + ADP_COL_SHIFT)))
			val = 0;
		else if (pdata->pulldown_en_300k & (1 << (i + ADP_COL_SHIFT)))
			val = 1;
		else if (pdata->pullup_en_100k & (1 << (i + ADP_COL_SHIFT)))
			val = 2;
		else if (pdata->pull_dis_mask & (1 << (i + ADP_COL_SHIFT)))
			val = 3;

		pull_mask |= val << (2 * (i & 0x3));

		if (i == 3 || i == ADP_MAX_COL_NUM) {
			ret |= adp5585_write(client,
					     ADP5585_RPULL_CONFIG_C + (i >> 2),
					     pull_mask);
			pull_mask = 0;
		}
	}

	if (pdata->reset1_key_1 && pdata->reset1_key_2 &&
		pdata->reset1_key_3) {
		ret |= adp5585_write(client, ADP5585_RESET1_EVENT_A,
				     adp5585_get_evcode(kpad,
				     pdata->reset1_key_1));
		ret |= adp5585_write(client, ADP5585_RESET1_EVENT_B,
				     adp5585_get_evcode(kpad,
				     pdata->reset1_key_2));
		ret |= adp5585_write(client, ADP5585_RESET1_EVENT_C,
				     adp5585_get_evcode(kpad,
				     pdata->reset1_key_3));
		kpad->extend_cfg |= R4_EXTEND_CFG;
	}

	if (pdata->reset2_key_1 && pdata->reset2_key_2) {
		ret |= adp5585_write(client, ADP5585_RESET2_EVENT_A,
				     adp5585_get_evcode(kpad,
				     pdata->reset2_key_1));
		ret |= adp5585_write(client, ADP5585_RESET2_EVENT_B,
				     adp5585_get_evcode(kpad,
				     pdata->reset2_key_2));
		kpad->extend_cfg |= C4_EXTEND_CFG;
	}

	if (kpad->extend_cfg) {
		ret |= adp5585_write(client, ADP5585_RESET_CFG,
				     pdata->reset_cfg);
		ret |= adp5585_write(client, ADP5585_PIN_CONFIG_D,
				     kpad->extend_cfg);
	}

	ret |= adp5585_write(client, ADP5585_DEBOUNCE_DIS_A,
			    pdata->debounce_dis_mask & ADP_ROW_MASK);
	ret |= adp5585_write(client, ADP5585_DEBOUNCE_DIS_B,
			     (pdata->debounce_dis_mask >> ADP_COL_SHIFT) &
			     ADP_COL_MASK);


	ret |= adp5585_write(client, ADP5585_POLL_PTIME_CFG,
			     pdata->scan_cycle_time & PTIME_MASK);
	ret |= adp5585_write(client, ADP5585_INT_STATUS, LOGIC_INT |
			     OVRFLOW_INT | GPI_INT | EVENT_INT); /* Status is W1C */

	ret |= adp5585_write(client, ADP5585_GENERAL_CFG,
			     INT_CFG | OSC_EN | CORE_CLK(3));
	ret |= adp5585_write(client, ADP5585_INT_EN,
			     OVRFLOW_IEN | GPI_IEN | EVENT_IEN);

	if (ret < 0) {
		dev_err(&client->dev, "Write Error\n");
		return ret;
	}

	return 0;
}

static void  adp5585_report_switch_state(struct adp5585_kpad *kpad)
{
	int gpi_stat1 = adp5585_read(kpad->client, ADP5585_GPI_STATUS_A);
	int gpi_stat2 = adp5585_read(kpad->client, ADP5585_GPI_STATUS_B);
	int gpi_stat_tmp, pin_loc;
	int i;

	for (i = 0; i < kpad->gpimapsize; i++) {
		unsigned short pin = kpad->gpimap[i].pin;

		if (pin <= ADP5585_GPI_PIN_ROW_END) {
			gpi_stat_tmp = gpi_stat1;
			pin_loc = pin - ADP5585_GPI_PIN_ROW_BASE;
		} else {
			gpi_stat_tmp = gpi_stat2;
			pin_loc = pin - ADP5585_GPI_PIN_COL_BASE;
		}

		if (gpi_stat_tmp < 0) {
			dev_err(&kpad->client->dev,
				"Can't read GPIO_DAT_STAT switch"
				" %d default to OFF\n", pin);
			gpi_stat_tmp = 0;
		}

		input_report_switch(kpad->input,
				    kpad->gpimap[i].sw_evt,
				    !(gpi_stat_tmp & (1 << pin_loc)));
	}

	input_sync(kpad->input);
}

static int  adp5585_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct adp5585_kpad *kpad;
	const struct adp5585_kpad_platform_data *pdata =
	    client->dev.platform_data;
	struct input_dev *input;
	unsigned int revid;
	int ret, i;
	int error;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}

	if (!pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -EINVAL;
	}

	if (!((pdata->keypad_en_mask & ADP_ROW_MASK) &&
		(pdata->keypad_en_mask >> ADP_COL_SHIFT)) || !pdata->keymap) {
		dev_err(&client->dev, "no rows, cols or keymap from pdata\n");
		return -EINVAL;
	}

	if (pdata->keymapsize != ADP5585_KEYMAPSIZE) {
		dev_err(&client->dev, "invalid keymapsize\n");
		return -EINVAL;
	}

	if (!pdata->gpimap && pdata->gpimapsize) {
		dev_err(&client->dev, "invalid gpimap from pdata\n");
		return -EINVAL;
	}

	if (pdata->gpimapsize > ADP5585_GPIMAPSIZE_MAX) {
		dev_err(&client->dev, "invalid gpimapsize\n");
		return -EINVAL;
	}

	for (i = 0; i < pdata->gpimapsize; i++) {
		unsigned short pin = pdata->gpimap[i].pin;

		if (pin < ADP5585_GPI_PIN_BASE || pin > ADP5585_GPI_PIN_END) {
			dev_err(&client->dev, "invalid gpi pin data\n");
			return -EINVAL;
		}

		if ((1 << (pin - ADP5585_GPI_PIN_ROW_BASE)) &
			pdata->keypad_en_mask) {
			dev_err(&client->dev, "invalid gpi row/col data\n");
			return -EINVAL;
		}
	}

	if (!client->irq) {
		dev_err(&client->dev, "no IRQ?\n");
		return -EINVAL;
	}

	kpad = kzalloc(sizeof(*kpad), GFP_KERNEL);
	input = input_allocate_device();
	if (!kpad || !input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	kpad->client = client;
	kpad->input = input;

	ret = adp5585_read(client, ADP5585_ID);
	if (ret < 0) {
		error = ret;
		goto err_free_mem;
	}

	revid = (u8) ret & ADP5585_DEVICE_ID_MASK;

	if (id->driver_data == ADP5585_02)
		kpad->support_row5 = true;

	input->name = client->name;
	input->phys = "adp5585-keys/input0";
	input->dev.parent = &client->dev;

	input_set_drvdata(input, kpad);

	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = revid;

	input->keycodesize = sizeof(kpad->keycode[0]);
	input->keycodemax = pdata->keymapsize;
	input->keycode = kpad->keycode;

	memcpy(kpad->keycode, pdata->keymap,
	       pdata->keymapsize * input->keycodesize);

	kpad->gpimap = pdata->gpimap;
	kpad->gpimapsize = pdata->gpimapsize;

	/* setup input device */
	__set_bit(EV_KEY, input->evbit);

	if (pdata->repeat)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < input->keycodemax; i++)
		__set_bit(kpad->keycode[i] & KEY_MAX, input->keybit);
	__clear_bit(KEY_RESERVED, input->keybit);

	if (kpad->gpimapsize)
		__set_bit(EV_SW, input->evbit);
	for (i = 0; i < kpad->gpimapsize; i++)
		__set_bit(kpad->gpimap[i].sw_evt, input->swbit);

	error = input_register_device(input);
	if (error) {
		dev_err(&client->dev, "unable to register input device\n");
		goto err_free_mem;
	}

	error = request_threaded_irq(client->irq, NULL, adp5585_irq,
				     IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				     client->dev.driver->name, kpad);
	if (error) {
		dev_err(&client->dev, "irq %d busy?\n", client->irq);
		goto err_unreg_dev;
	}

	error = adp5585_setup(kpad);
	if (error)
		goto err_free_irq;

	if (kpad->gpimapsize)
		adp5585_report_switch_state(kpad);

	error = adp5585_gpio_add(kpad);
	if (error)
		goto err_free_irq;

	device_init_wakeup(&client->dev, 1);
	i2c_set_clientdata(client, kpad);

	dev_info(&client->dev, "Rev.%d keypad, irq %d\n", revid, client->irq);
	return 0;

err_free_irq:
	free_irq(client->irq, kpad);
err_unreg_dev:
	input_unregister_device(input);
	input = NULL;
err_free_mem:
	input_free_device(input);
	kfree(kpad);

	return error;
}

static int  adp5585_remove(struct i2c_client *client)
{
	struct adp5585_kpad *kpad = i2c_get_clientdata(client);

	adp5585_write(client, ADP5585_GENERAL_CFG, 0);
	free_irq(client->irq, kpad);
	input_unregister_device(kpad->input);
	adp5585_gpio_remove(kpad);
	kfree(kpad);

	return 0;
}

#ifdef CONFIG_PM
static int adp5585_suspend(struct device *dev)
{
	struct adp5585_kpad *kpad = dev_get_drvdata(dev);
	struct i2c_client *client = kpad->client;

	disable_irq(client->irq);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int adp5585_resume(struct device *dev)
{
	struct adp5585_kpad *kpad = dev_get_drvdata(dev);
	struct i2c_client *client = kpad->client;

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	enable_irq(client->irq);

	return 0;
}

static const struct dev_pm_ops adp5585_dev_pm_ops = {
	.suspend = adp5585_suspend,
	.resume = adp5585_resume,
};
#endif

static const struct i2c_device_id adp5585_id[] = {
	{"adp5585-keys", ADP5585_01},
	{"adp5585-02-keys", ADP5585_02}, /* Adds ROW5 to ADP5585 */
	{}
};

MODULE_DEVICE_TABLE(i2c, adp5585_id);

static struct i2c_driver adp5585_driver = {
	.driver = {
		   .name = KBUILD_MODNAME,
#ifdef CONFIG_PM
		   .pm = &adp5585_dev_pm_ops,
#endif
		   },
	.probe = adp5585_probe,
	.remove = adp5585_remove,
	.id_table = adp5585_id,
};

static int __init adp5585_init(void)
{
	return i2c_add_driver(&adp5585_driver);
}

module_init(adp5585_init);

static void __exit adp5585_exit(void)
{
	i2c_del_driver(&adp5585_driver);
}

module_exit(adp5585_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("ADP5585 Keypad driver");
MODULE_ALIAS("i2c:adp5585-keys");

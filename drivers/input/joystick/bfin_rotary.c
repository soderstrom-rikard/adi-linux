/*
 * File:         drivers/input/keyboard/bfin_rotary.c
 * Based on:
 * Author:       Michael Hennerich <hennerich@blackfin.uclinux.org>
 *
 * Created:
 * Description:  Rotary counter driver for Analog Devices Blackfin Processors
 *
 *
 * Modified:
 *               Copyright 2008 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/platform_device.h>
#include <linux/input.h>

#include <asm/portmux.h>
#include <asm/bfin_rotary.h>

#define DRV_NAME	"bfin-rotary"

static const u16 per_cnt[] = {
	P_CNT_CUD,
	P_CNT_CDG,
	P_CNT_CZM,
	0
};

struct bfin_rot {
	struct input_dev *input;
	int irq;
	unsigned int rotary_up_key;
	unsigned int rotary_down_key;
	unsigned int rotary_button_key;
	unsigned int rotary_rel_code;
	unsigned short cnt_config;
	unsigned short cnt_imask;
	unsigned short cnt_debounce;
};

static inline void report_marker_event(struct bfin_rot *rotary)
{
	struct input_dev *input = rotary->input;
	int keycode = rotary->rotary_button_key;

	input_report_key(input, keycode, 1);
	input_sync(input);
	input_report_key(input, keycode, 0);
	input_sync(input);
}

static void report_rotary_event(struct bfin_rot *rotary, int delta)
{
	struct input_dev *input = rotary->input;

	if (delta == 0)
		return;

	if (rotary->rotary_up_key && rotary->rotary_down_key) {
		int keycode = (delta > 0) ? rotary->rotary_up_key:
					    rotary->rotary_down_key;

		/* simulate a press-n-release */
		input_report_key(input, keycode, 1);
		input_sync(input);
		input_report_key(input, keycode, 0);
		input_sync(input);
	} else {
		input_report_rel(input, rotary->rotary_rel_code, delta);
		input_sync(input);
	}
}

static irqreturn_t bfin_rotary_isr(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct bfin_rot *rotary = platform_get_drvdata(pdev);
	unsigned short status = bfin_read_CNT_STATUS();

	switch (status) {
	case ICII:
		break;
	case UCII:
	case DCII:
		report_rotary_event(rotary, bfin_read_CNT_COUNTER());
		break;
	case CZMII:
		report_marker_event(rotary);
		break;
	default:
		break;
	}

	bfin_write_CNT_COMMAND(W1LCNT_ZERO);	/* Clear COUNTER */
	bfin_write_CNT_STATUS(-1);	/* Clear STATUS */

	return IRQ_HANDLED;
}

static int __devinit bfin_rotary_probe(struct platform_device *pdev)
{
	struct bfin_rot *rotary;
	struct bfin_rotary_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input;
	int error;

	rotary = kzalloc(sizeof(struct bfin_rot), GFP_KERNEL);
	if (!rotary)
		return -ENOMEM;

	platform_set_drvdata(pdev, rotary);

	error = peripheral_request_list(per_cnt, DRV_NAME);
	if (error) {
		printk(KERN_ERR DRV_NAME
			": Requesting Peripherals failed\n");
		goto out1;
	}

	rotary->irq = platform_get_irq(pdev, 0);
	if (rotary->irq < 0) {
		error = rotary->irq;
		goto out2;
	}

	error = request_irq(rotary->irq, bfin_rotary_isr,
				 IRQF_SAMPLE_RANDOM, DRV_NAME, pdev);
	if (error) {
		printk(KERN_ERR DRV_NAME
			": unable to claim irq %d; error %d\n",
			rotary->irq, error);
		goto out2;
	}

	input = input_allocate_device();
	if (!input) {
		error = -ENOMEM;
		goto out3;
	}

	rotary->input = input;

	input->name = pdev->name;
	input->phys = "bfin-rotary/input0";
	input->dev.parent = &pdev->dev;

	input_set_drvdata(input, rotary);

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* setup input device */

	rotary->rotary_up_key = pdata->rotary_up_key;
	rotary->rotary_down_key = pdata->rotary_down_key;
	rotary->rotary_button_key = pdata->rotary_button_key;
	rotary->rotary_rel_code = pdata->rotary_rel_code;

	if (pdata->rotary_up_key && pdata->rotary_down_key) {
		__set_bit(EV_KEY, input->evbit);
		__set_bit(pdata->rotary_up_key, input->keybit);
		__set_bit(pdata->rotary_down_key, input->keybit);
	} else if (pdata->rotary_rel_code) {
		__set_bit(EV_REL, input->evbit);
		__set_bit(pdata->rotary_rel_code, input->relbit);
	} else {
		error = -EINVAL;
		goto out4;
	}

	if (pdata->rotary_button_key) {
		__set_bit(EV_KEY, input->evbit);
		__set_bit(pdata->rotary_button_key, input->keybit);
		bfin_write_CNT_IMASK(CZMIE);
	}

	if (pdata->mode & ROT_DEBE)
		bfin_write_CNT_DEBOUNCE(pdata->debounce & DPRESCALE);


	if (pdata->mode)
		bfin_write_CNT_CONFIG(bfin_read_CNT_CONFIG() |
					(pdata->mode & ~CNTE));


	error = input_register_device(input);
	if (error) {
		printk(KERN_ERR DRV_NAME
			": Unable to register input device (%d)\n", error);
		goto out4;
	}

	bfin_write_CNT_IMASK(bfin_read_CNT_IMASK() | UCIE | DCIE);

	bfin_write_CNT_CONFIG(bfin_read_CNT_CONFIG() | CNTE);

	device_init_wakeup(&pdev->dev, 1);

	printk(KERN_INFO DRV_NAME
		": Blackfin Rotary Driver registered IRQ %d\n", rotary->irq);

	return 0;

out4:
	input_free_device(input);
out3:
	free_irq(rotary->irq, pdev);
out2:
	peripheral_free_list(per_cnt);
out1:
	kfree(rotary);
	platform_set_drvdata(pdev, NULL);

	return error;
}

static int __devexit bfin_rotary_remove(struct platform_device *pdev)
{
	struct bfin_rot *rotary = platform_get_drvdata(pdev);

	bfin_write_CNT_CONFIG(0);
	bfin_write_CNT_IMASK(0);

	free_irq(rotary->irq, pdev);

	input_unregister_device(rotary->input);

	peripheral_free_list(per_cnt);

	kfree(rotary);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int bfin_rotary_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct bfin_rot *rotary = platform_get_drvdata(pdev);

	rotary->cnt_config = bfin_read_CNT_CONFIG();
	rotary->cnt_imask = bfin_read_CNT_IMASK();
	rotary->cnt_debounce = bfin_read_CNT_DEBOUNCE();

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(rotary->irq);

	return 0;
}

static int bfin_rotary_resume(struct platform_device *pdev)
{
	struct bfin_rot *rotary = platform_get_drvdata(pdev);

	bfin_write_CNT_DEBOUNCE(rotary->cnt_debounce);
	bfin_write_CNT_IMASK(rotary->cnt_imask);
	bfin_write_CNT_CONFIG(rotary->cnt_config & ~CNTE);

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(rotary->irq);

	if (rotary->cnt_config & CNTE)
		bfin_write_CNT_CONFIG(rotary->cnt_config);

	return 0;
}
#else
# define bfin_rotary_suspend NULL
# define bfin_rotary_resume  NULL
#endif

struct platform_driver bfin_rotary_device_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= bfin_rotary_probe,
	.remove		= __devexit_p(bfin_rotary_remove),
	.suspend	= bfin_rotary_suspend,
	.resume		= bfin_rotary_resume,
};

static int __init bfin_rotary_init(void)
{
	return platform_driver_register(&bfin_rotary_device_driver);
}

static void __exit bfin_rotary_exit(void)
{
	platform_driver_unregister(&bfin_rotary_device_driver);
}

module_init(bfin_rotary_init);
module_exit(bfin_rotary_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Rotary Counter driver for Blackfin Processors");
MODULE_ALIAS("platform:bfin-rotary");

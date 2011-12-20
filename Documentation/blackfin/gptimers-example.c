/*
 * Simple gptimers example
 *	http://docs.blackfin.uclinux.org/doku.php?id=linux-kernel:drivers:gptimers
 *
 * Copyright 2007-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/interrupt.h>
#include <linux/module.h>

#include <asm/gptimers.h>
#include <asm/portmux.h>

/* ... random driver includes ... */

#define DRIVER_NAME "gptimer_example"

#define TEST_TMR_PIN P_TMR0
#define TEST_TMR_IRQ IRQ_TIMER0
#define TEST_TMR_ID TIMER0_id
#define TEST_TMR_BIT TIMER0bit

struct gptimer_data {
	uint32_t period, width;
};
static struct gptimer_data data;

/* ... random driver state ... */

static irqreturn_t gptimer_example_irq(int irq, void *dev_id)
{
	struct gptimer_data *data = dev_id;

	/* make sure it was our timer which caused the interrupt */
	if (!get_gptimer_intr(TEST_TMR_ID))
		return IRQ_NONE;

	/* read the width/period values that were captured for the waveform */
	data->width = get_gptimer_pwidth(TEST_TMR_ID);
	data->period = get_gptimer_period(TEST_TMR_ID);

	/* acknowledge the interrupt */
	clear_gptimer_intr(TEST_TMR_ID);

	/* tell the upper layers we took care of things */
	return IRQ_HANDLED;
}

/* ... random driver code ... */

static int __init gptimer_example_init(void)
{
	int ret;
	uint16_t revid;

	/* grab the peripheral pins */
	ret = peripheral_request(TEST_TMR_PIN, DRIVER_NAME);
	if (ret) {
		printk(KERN_NOTICE DRIVER_NAME ": peripheral request failed\n");
		return ret;
	}

	/* grab the IRQ for the timer */
	ret = request_irq(TEST_TMR_IRQ, gptimer_example_irq, IRQF_SHARED, DRIVER_NAME, &data);
	if (ret) {
		printk(KERN_NOTICE DRIVER_NAME ": IRQ request failed\n");
		peripheral_free(P_TMR0);
		return ret;
	}

#ifdef CONFIG_BF60x
	revid = bfin_read16(TIMER_REVID);
	printk("gptimer revid %04x\n", revid);
	bfin_write16(TIMER_DATA_IMSK, 0);
#endif

#define TMR_MODE_PWM_ONESHOT
/* #define TMR_MODE_PWM_CONT */

#ifdef TMR_MODE_PWM_ONESHOT
	/* setup one shot pwm timer and enable it */
	set_gptimer_config(TEST_TMR_ID,  TIMER_OUT_DIS | TIMER_MODE_PWM | TIMER_PULSE_HI | TIMER_IRQ_WID_DLY);
#endif
#ifdef TMR_MODE_PWM_CONT
	/* setup continuous timer and enable it */
	set_gptimer_config(TEST_TMR_ID,  TIMER_OUT_DIS | TIMER_MODE_PWM_CONT | TIMER_PULSE_HI | TIMER_IRQ_PER);
#endif

	set_gptimer_pwidth(TEST_TMR_ID, 100);
	set_gptimer_period(TEST_TMR_ID, 10000);
#ifdef CONFIG_BF60x
	set_gptimer_delay(TEST_TMR_ID, 0);
#endif
	enable_gptimers(TEST_TMR_BIT);

	return 0;
}
module_init(gptimer_example_init);

static void __exit gptimer_example_exit(void)
{
	disable_gptimers(TEST_TMR_BIT);
	free_irq(TEST_TMR_IRQ, &data);
	peripheral_free(TEST_TMR_PIN);
}
module_exit(gptimer_example_exit);

MODULE_LICENSE("BSD");

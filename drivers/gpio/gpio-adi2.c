/*
 * ADI GPIO Abstraction Layer
 *
 * Copyright 2007-2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>
#include <linux/irq.h>

static struct gpio_port_t * const gpio_array[] = {
	(struct gpio_port_t *)PORTA_FER,
	(struct gpio_port_t *)PORTB_FER,
	(struct gpio_port_t *)PORTC_FER,
	(struct gpio_port_t *)PORTD_FER,
	(struct gpio_port_t *)PORTE_FER,
	(struct gpio_port_t *)PORTF_FER,
	(struct gpio_port_t *)PORTG_FER,
#if defined(CONFIG_BF54x)
	(struct gpio_port_t *)PORTH_FER,
	(struct gpio_port_t *)PORTI_FER,
	(struct gpio_port_t *)PORTJ_FER,
#endif
};

#define RESOURCE_LABEL_SIZE	16

static struct str_ident {
	char name[RESOURCE_LABEL_SIZE];
} str_ident[MAX_RESOURCES];

#if defined(CONFIG_PM)
static struct gpio_port_s gpio_bank_saved[GPIO_BANK_NUM];
#endif

static void gpio_error(unsigned gpio)
{
	printk(KERN_ERR "gpio-adi2: GPIO %d wasn't requested!\n", gpio);
}

static void set_label(unsigned short ident, const char *label)
{
	if (label) {
		strncpy(str_ident[ident].name, label,
			 RESOURCE_LABEL_SIZE);
		str_ident[ident].name[RESOURCE_LABEL_SIZE - 1] = 0;
	}
}

static char *get_label(unsigned short ident)
{
	return (*str_ident[ident].name ? str_ident[ident].name : "UNKNOWN");
}

static int cmp_label(unsigned short ident, const char *label)
{
	if (label == NULL) {
		dump_stack();
		printk(KERN_ERR "Please provide none-null label\n");
	}

	if (label)
		return strcmp(str_ident[ident].name, label);
	else
		return -EINVAL;
}

#define map_entry(m, i)      reserved_##m##_map[gpio_bank(i)]
#define is_reserved(m, i, e) (map_entry(m, i) & gpio_bit(i))
#define reserve(m, i)        (map_entry(m, i) |= gpio_bit(i))
#define unreserve(m, i)      (map_entry(m, i) &= ~gpio_bit(i))
#define DECLARE_RESERVED_MAP(m, c) static unsigned short reserved_##m##_map[c]

DECLARE_RESERVED_MAP(gpio, GPIO_BANK_NUM);
DECLARE_RESERVED_MAP(peri, DIV_ROUND_UP(MAX_RESOURCES, GPIO_BANKSIZE));
DECLARE_RESERVED_MAP(gpio_irq, GPIO_BANK_NUM);

static inline int check_gpio(unsigned gpio)
{
#if defined(CONFIG_BF54x)
	if (gpio == GPIO_PB15 || gpio == GPIO_PC14 || gpio == GPIO_PC15
	    || gpio == GPIO_PH14 || gpio == GPIO_PH15
	    || gpio == GPIO_PJ14 || gpio == GPIO_PJ15)
		return -EINVAL;
#endif
	if (gpio >= MAX_BLACKFIN_GPIOS)
		return -EINVAL;
	return 0;
}

static void port_setup(unsigned gpio, unsigned short usage)
{
	if (check_gpio(gpio))
		return;

	if (usage == GPIO_USAGE)
		gpio_array[gpio_bank(gpio)]->port_fer &= ~gpio_bit(gpio);
	else
		gpio_array[gpio_bank(gpio)]->port_fer |= gpio_bit(gpio);
	SSYNC();
}

static inline void portmux_setup(unsigned short per)
{
	u16 ident = P_IDENT(per);
	u16 function = P_FUNCT2MUX(per);
	u32 pmux;

	pmux = gpio_array[gpio_bank(ident)]->port_mux;

	pmux &= ~(0x3 << (2 * gpio_sub_n(ident)));
	pmux |= (function & 0x3) << (2 * gpio_sub_n(ident));

	gpio_array[gpio_bank(ident)]->port_mux = pmux;
}

static inline u16 get_portmux(unsigned short per)
{
	u16 ident = P_IDENT(per);
	u32 pmux = gpio_array[gpio_bank(ident)]->port_mux;
	return (pmux >> (2 * gpio_sub_n(ident)) & 0x3);
}

#ifdef CONFIG_PM

int adi_gpio_pm_standby_ctrl(unsigned ctrl)
{
	return 0;
}

void adi_gpio_pm_hibernate_suspend(void)
{
	int i, bank;

	for (i = 0; i < MAX_BLACKFIN_GPIOS; i += GPIO_BANKSIZE) {
		bank = gpio_bank(i);

		gpio_bank_saved[bank].fer = gpio_array[bank]->port_fer;
		gpio_bank_saved[bank].mux = gpio_array[bank]->port_mux;
		gpio_bank_saved[bank].data = gpio_array[bank]->data;
		gpio_bank_saved[bank].inen = gpio_array[bank]->inen;
		gpio_bank_saved[bank].dir = gpio_array[bank]->dir_set;
	}
}

void adi_gpio_pm_hibernate_restore(void)
{
	int i, bank;

	for (i = 0; i < MAX_BLACKFIN_GPIOS; i += GPIO_BANKSIZE) {
		bank = gpio_bank(i);

		gpio_array[bank]->port_mux = gpio_bank_saved[bank].mux;
		gpio_array[bank]->port_fer = gpio_bank_saved[bank].fer;
		gpio_array[bank]->inen = gpio_bank_saved[bank].inen;
		gpio_array[bank]->data_set = gpio_bank_saved[bank].data
						& gpio_bank_saved[bank].dir;
		gpio_array[bank]->dir_set = gpio_bank_saved[bank].dir;
	}
}
#endif

static unsigned short get_gpio_dir(unsigned gpio)
{
	return 0x01 &
		(gpio_array[gpio_bank(gpio)]->dir_clear >> gpio_sub_n(gpio));
}

/***********************************************************
*
* FUNCTIONS:	ADI Processor Peripheral Resource Allocation
*		and PortMux Setup
*
* INPUTS/OUTPUTS:
* per	Peripheral Identifier
* label	String
*
* DESCRIPTION: ADI Processor Peripheral Resource Allocation and Setup API
**************************************************************/

int peripheral_request(unsigned short per, const char *label)
{
	unsigned long flags;
	unsigned short ident = P_IDENT(per);

	/*
	 * Don't cares are pins with only one dedicated function
	 */

	if (per & P_DONTCARE)
		return 0;

	if (!(per & P_DEFINED))
		return -ENODEV;

	BUG_ON(ident >= MAX_RESOURCES);

	flags = hard_local_irq_save();

	/* If a pin can be muxed as either GPIO or peripheral, make
	 * sure it is not already a GPIO pin when we request it.
	 */
	if (unlikely(!check_gpio(ident) && is_reserved(gpio, ident, 1))) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		printk(KERN_ERR
		       "%s: Peripheral %d is already reserved as GPIO by %s !\n",
		       __func__, ident, get_label(ident));
		hard_local_irq_restore(flags);
		return -EBUSY;
	}

	if (unlikely(is_reserved(peri, ident, 1))) {

		/*
		 * Pin functions like AMC address strobes my
		 * be requested and used by several drivers
		 */

		if (!((per & P_MAYSHARE) && get_portmux(per) == P_FUNCT2MUX(per))) {
			/*
			 * Allow that the identical pin function can
			 * be requested from the same driver twice
			 */

			if (cmp_label(ident, label) == 0)
				goto anyway;

			if (system_state == SYSTEM_BOOTING)
				dump_stack();
			printk(KERN_ERR
			       "%s: Peripheral %d function %d is already reserved by %s !\n",
			       __func__, ident, P_FUNCT2MUX(per), get_label(ident));
			hard_local_irq_restore(flags);
			return -EBUSY;
		}
	}

 anyway:
	reserve(peri, ident);

	portmux_setup(per);
	port_setup(ident, PERIPHERAL_USAGE);

	hard_local_irq_restore(flags);
	set_label(ident, label);

	return 0;
}
EXPORT_SYMBOL(peripheral_request);

int peripheral_request_list(const unsigned short per[], const char *label)
{
	u16 cnt;
	int ret;

	for (cnt = 0; per[cnt] != 0; cnt++) {

		ret = peripheral_request(per[cnt], label);

		if (ret < 0) {
			for ( ; cnt > 0; cnt--)
				peripheral_free(per[cnt - 1]);

			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL(peripheral_request_list);

void peripheral_free(unsigned short per)
{
	unsigned long flags;
	unsigned short ident = P_IDENT(per);

	if (per & P_DONTCARE)
		return;

	if (!(per & P_DEFINED))
		return;

	flags = hard_local_irq_save();

	if (unlikely(!is_reserved(peri, ident, 0))) {
		hard_local_irq_restore(flags);
		return;
	}

	if (!(per & P_MAYSHARE))
		port_setup(ident, GPIO_USAGE);

	unreserve(peri, ident);

	set_label(ident, "free");

	hard_local_irq_restore(flags);
}
EXPORT_SYMBOL(peripheral_free);

void peripheral_free_list(const unsigned short per[])
{
	u16 cnt;
	for (cnt = 0; per[cnt] != 0; cnt++)
		peripheral_free(per[cnt]);
}
EXPORT_SYMBOL(peripheral_free_list);

/***********************************************************
*
* FUNCTIONS: ADI Processor GPIO Driver
*
* INPUTS/OUTPUTS:
* gpio	PIO Number between 0 and MAX_GPIOS
* label	String
*
* DESCRIPTION: ADI Processor GPIO Driver API
**************************************************************/

static int adi_gpio_request(struct gpio_chip *chip, unsigned gpio)
{
	unsigned long flags;

	if (check_gpio(gpio) < 0)
		return -EINVAL;

	flags = hard_local_irq_save();

	/*
	 * Allow that the identical GPIO can
	 * be requested from the same driver twice
	 * Do nothing and return -
	 */

	if (cmp_label(gpio, chip->label) == 0) {
		hard_local_irq_restore(flags);
		return 0;
	}

	if (unlikely(is_reserved(gpio, gpio, 1))) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		printk(KERN_ERR "gpio-adi2: GPIO %d is already reserved by %s !\n",
		       gpio, get_label(gpio));
		hard_local_irq_restore(flags);
		return -EBUSY;
	}
	if (unlikely(is_reserved(peri, gpio, 1))) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		printk(KERN_ERR
		       "gpio-adi2: GPIO %d is already reserved as Peripheral by %s !\n",
		       gpio, get_label(gpio));
		hard_local_irq_restore(flags);
		return -EBUSY;
	}
	if (unlikely(is_reserved(gpio_irq, gpio, 1))) {
		printk(KERN_NOTICE "gpio-adi2: GPIO %d is already reserved as gpio-irq!"
		       " (Documentation/blackfin/bfin-gpio-notes.txt)\n", gpio);
	}

	reserve(gpio, gpio);
	set_label(gpio, chip->label);

	hard_local_irq_restore(flags);

	port_setup(gpio, GPIO_USAGE);

	return 0;
}

static void adi_gpio_free(struct gpio_chip *chip, unsigned gpio)
{
	unsigned long flags;

	if (check_gpio(gpio) < 0)
		return;

	might_sleep();

	flags = hard_local_irq_save();

	if (unlikely(!is_reserved(gpio, gpio, 0))) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		gpio_error(gpio);
		hard_local_irq_restore(flags);
		return;
	}

	unreserve(gpio, gpio);

	set_label(gpio, "free");

	hard_local_irq_restore(flags);
}

int adi_gpio_irq_request(unsigned gpio, const char *label)
{
	unsigned long flags;

	if (check_gpio(gpio) < 0)
		return -EINVAL;

	flags = hard_local_irq_save();

	if (unlikely(is_reserved(peri, gpio, 1))) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		printk(KERN_ERR
		       "gpio-adi2: GPIO %d is already reserved as Peripheral by %s !\n",
		       gpio, get_label(gpio));
		hard_local_irq_restore(flags);
		return -EBUSY;
	}
	if (unlikely(is_reserved(gpio, gpio, 1)))
		printk(KERN_NOTICE "gpio-adi2: GPIO %d is already reserved by %s! "
		       "(Documentation/blackfin/bfin-gpio-notes.txt)\n",
		       gpio, get_label(gpio));

	reserve(gpio_irq, gpio);
	set_label(gpio, label);

	hard_local_irq_restore(flags);

	port_setup(gpio, GPIO_USAGE);

	return 0;
}

void adi_gpio_irq_free(unsigned gpio)
{
	unsigned long flags;

	if (check_gpio(gpio) < 0)
		return;

	flags = hard_local_irq_save();

	if (unlikely(!is_reserved(gpio_irq, gpio, 0))) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		gpio_error(gpio);
		hard_local_irq_restore(flags);
		return;
	}

	unreserve(gpio_irq, gpio);

	set_label(gpio, "free");

	hard_local_irq_restore(flags);
}

static inline void __adi_gpio_direction_input(unsigned gpio)
{
	gpio_array[gpio_bank(gpio)]->dir_clear = gpio_bit(gpio);
	gpio_array[gpio_bank(gpio)]->inen |= gpio_bit(gpio);
}

static int adi_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	unsigned long flags;

	if (unlikely(!is_reserved(gpio, gpio, 0))) {
		gpio_error(gpio);
		return -EINVAL;
	}

	flags = hard_local_irq_save();
	__adi_gpio_direction_input(gpio);
	hard_local_irq_restore(flags);

	return 0;
}

void adi_gpio_irq_prepare(unsigned gpio)
{
	unsigned long flags;

	port_setup(gpio, GPIO_USAGE);

	flags = hard_local_irq_save();
	__adi_gpio_direction_input(gpio);
	hard_local_irq_restore(flags);
}

static void adi_gpio_set_value(struct gpio_chip *chip, unsigned gpio, int arg)
{
	if (arg)
		gpio_array[gpio_bank(gpio)]->data_set = gpio_bit(gpio);
	else
		gpio_array[gpio_bank(gpio)]->data_clear = gpio_bit(gpio);
}

static int adi_gpio_direction_output(struct gpio_chip *chip, unsigned gpio, int value)
{
	unsigned long flags;

	if (unlikely(!is_reserved(gpio, gpio, 0))) {
		gpio_error(gpio);
		return -EINVAL;
	}

	flags = hard_local_irq_save();

	gpio_array[gpio_bank(gpio)]->inen &= ~gpio_bit(gpio);
	gpio_set_value(gpio, value);
	gpio_array[gpio_bank(gpio)]->dir_set = gpio_bit(gpio);

	hard_local_irq_restore(flags);

	return 0;
}

static int adi_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	return (1 & (gpio_array[gpio_bank(gpio)]->data >> gpio_sub_n(gpio)));
}

static int adi_gpio_to_irq(struct gpio_chip *chip, unsigned gpio)
{
	return gpio + GPIO_IRQ_BASE;
}

#if defined(CONFIG_PROC_FS)
static int gpio_proc_show(struct seq_file *m, void *v)
{
	int c, irq, gpio;

	for (c = 0; c < MAX_RESOURCES; c++) {
		irq = is_reserved(gpio_irq, c, 1);
		gpio = is_reserved(gpio, c, 1);
		if (!check_gpio(c) && (gpio || irq))
			seq_printf(m, "GPIO_%d: \t%s%s \t\tGPIO %s\n", c,
				 get_label(c), (gpio && irq) ? " *" : "",
				 get_gpio_dir(c) ? "OUTPUT" : "INPUT");
		else if (is_reserved(peri, c, 1))
			seq_printf(m, "GPIO_%d: \t%s \t\tPeripheral\n", c, get_label(c));
		else
			continue;
	}

	return 0;
}

static int gpio_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, gpio_proc_show, NULL);
}

static const struct file_operations gpio_proc_ops = {
	.open		= gpio_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static __init int gpio_register_proc(void)
{
	struct proc_dir_entry *proc_gpio;

	proc_gpio = proc_create("gpio", 0, NULL, &gpio_proc_ops);
	return proc_gpio == NULL;
}
__initcall(gpio_register_proc);
#endif

static struct gpio_chip gpio_adi2_chip = {
	.label			= "gpio-adi2",
	.direction_input	= adi_gpio_direction_input,
	.get			= adi_gpio_get_value,
	.direction_output	= adi_gpio_direction_output,
	.set			= adi_gpio_set_value,
	.request		= adi_gpio_request,
	.free			= adi_gpio_free,
	.to_irq			= adi_gpio_to_irq,
	.base			= 0,
	.ngpio			= MAX_GPIOS,
};

static int __init gpiolib_setup(void)
{
	return gpiochip_add(&gpio_adi2_chip);
}
arch_initcall(gpiolib_setup);

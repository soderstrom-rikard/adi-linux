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


#define NR_PINT_BITS		32
#define IRQ_NOT_AVAIL		0xFF

#define PINT_2_BANK(x)		((x) >> 5)
#define PINT_2_BIT(x)		((x) & 0x1F)
#define PINT_BIT(x)		(1 << (PINT_2_BIT(x)))

static unsigned char irq2pint_lut[NR_PINTS];
static unsigned char pint2irq_lut[NR_PINT_SYS_IRQS * NR_PINT_BITS];

static struct gpio_pint_regs * const pint[NR_PINT_SYS_IRQS] = {
	(struct gpio_pint_regs *)PINT0_MASK_SET,
	(struct gpio_pint_regs *)PINT1_MASK_SET,
	(struct gpio_pint_regs *)PINT2_MASK_SET,
	(struct gpio_pint_regs *)PINT3_MASK_SET,
#ifdef CONFIG_BF60x
	(struct gpio_pint_regs *)PINT4_MASK_SET,
	(struct gpio_pint_regs *)PINT5_MASK_SET,
#endif
};

inline unsigned int get_irq_base(u32 bank, u8 bmap)
{
	unsigned int irq_base;

#ifdef CONFIG_BF54x
	if (bank < 2) {		/*PA-PB */
		irq_base = IRQ_PA0 + bmap * 16;
	} else {		/*PC-PJ */
		irq_base = IRQ_PC0 + bmap * 16;
	}
#else
	irq_base = IRQ_PA0 + bank * 16 + bmap * 16;
#endif
	return irq_base;
}

	/* Whenever PINTx_ASSIGN is altered init_pint_lut() must be executed! */
static void init_pint_lut(void)
{
	u16 bank, bit, irq_base, bit_pos;
	u32 pint_assign;
	u8 bmap;

	memset(irq2pint_lut, IRQ_NOT_AVAIL, sizeof(irq2pint_lut));

	for (bank = 0; bank < NR_PINT_SYS_IRQS; bank++) {

		pint_assign = pint[bank]->assign;

		for (bit = 0; bit < NR_PINT_BITS; bit++) {

			bmap = (pint_assign >> ((bit / 8) * 8)) & 0xFF;

			irq_base = get_irq_base(bank, bmap);

			irq_base += (bit % 8) + ((bit / 8) & 1 ? 8 : 0);
			bit_pos = bit + bank * NR_PINT_BITS;

			pint2irq_lut[bit_pos] = irq_base - SYS_IRQS;
			irq2pint_lut[irq_base - SYS_IRQS] = bit_pos;
		}
	}
}

static DECLARE_BITMAP(gpio_enabled, MAX_BLACKFIN_GPIOS);

void adi_gpio_irq_prepare(unsigned gpio);
int adi_gpio_irq_request(unsigned gpio, const char *label);
void adi_gpio_irq_free(unsigned gpio);

static void adi_gpio_ack_irq(struct irq_data *d)
{
	u32 pint_val = irq2pint_lut[d->irq - SYS_IRQS];
	u32 pintbit = PINT_BIT(pint_val);
	u32 bank = PINT_2_BANK(pint_val);

	if (irqd_get_trigger_type(d) == IRQ_TYPE_EDGE_BOTH) {
		if (pint[bank]->invert_set & pintbit)
			pint[bank]->invert_clear = pintbit;
		else
			pint[bank]->invert_set = pintbit;
	}
	pint[bank]->request = pintbit;

}

static void adi_gpio_mask_ack_irq(struct irq_data *d)
{
	u32 pint_val = irq2pint_lut[d->irq - SYS_IRQS];
	u32 pintbit = PINT_BIT(pint_val);
	u32 bank = PINT_2_BANK(pint_val);

	if (irqd_get_trigger_type(d) == IRQ_TYPE_EDGE_BOTH) {
		if (pint[bank]->invert_set & pintbit)
			pint[bank]->invert_clear = pintbit;
		else
			pint[bank]->invert_set = pintbit;
	}

	pint[bank]->request = pintbit;
	pint[bank]->mask_clear = pintbit;
}

static void adi_gpio_mask_irq(struct irq_data *d)
{
	u32 pint_val = irq2pint_lut[d->irq - SYS_IRQS];

	pint[PINT_2_BANK(pint_val)]->mask_clear = PINT_BIT(pint_val);
}

static void adi_gpio_unmask_irq(struct irq_data *d)
{
	u32 pint_val = irq2pint_lut[d->irq - SYS_IRQS];
	u32 pintbit = PINT_BIT(pint_val);
	u32 bank = PINT_2_BANK(pint_val);

	pint[bank]->mask_set = pintbit;
}

static unsigned int adi_gpio_irq_startup(struct irq_data *d)
{
	unsigned int irq = d->irq;
	u32 gpionr = irq_to_gpio(irq);
	u32 pint_val = irq2pint_lut[irq - SYS_IRQS];

	if (pint_val == IRQ_NOT_AVAIL) {
		printk(KERN_ERR
		"GPIO IRQ %d :Not in PINT Assign table "
		"Reconfigure Interrupt to Port Assignemt\n", irq);
		return -ENODEV;
	}

	if (__test_and_set_bit(gpionr, gpio_enabled))
		adi_gpio_irq_prepare(gpionr);

	adi_gpio_unmask_irq(d);

	return 0;
}

static void adi_gpio_irq_shutdown(struct irq_data *d)
{
	u32 gpionr = irq_to_gpio(d->irq);

	adi_gpio_mask_irq(d);
	__clear_bit(gpionr, gpio_enabled);
	adi_gpio_irq_free(gpionr);
}

static int adi_gpio_irq_type(struct irq_data *d, unsigned int type)
{
	unsigned int irq = d->irq;
	int ret;
	char buf[16];
	u32 gpionr = irq_to_gpio(irq);
	u32 pint_val = irq2pint_lut[irq - SYS_IRQS];
	u32 pintbit = PINT_BIT(pint_val);
	u32 bank = PINT_2_BANK(pint_val);

	if (pint_val == IRQ_NOT_AVAIL)
		return -ENODEV;

	if (type == IRQ_TYPE_PROBE) {
		/* only probe unenabled GPIO interrupt lines */
		if (test_bit(gpionr, gpio_enabled))
			return 0;
		type = IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING;
	}

	if (type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING |
		    IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW)) {

		snprintf(buf, 16, "gpio-irq%d", irq);
		ret = adi_gpio_irq_request(gpionr, buf);
		if (ret)
			return ret;

		if (__test_and_set_bit(gpionr, gpio_enabled))
			adi_gpio_irq_prepare(gpionr);
	} else {
		__clear_bit(gpionr, gpio_enabled);
		return 0;
	}

	if ((type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_LEVEL_LOW)))
		pint[bank]->invert_set = pintbit;	/* low or falling edge denoted by one */
	else
		pint[bank]->invert_clear = pintbit;	/* high or rising edge denoted by zero */

	if ((type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING))
	    == (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING)) {
		if (gpio_get_value(gpionr))
			pint[bank]->invert_set = pintbit;
		else
			pint[bank]->invert_clear = pintbit;
	}

	if (type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING)) {
		pint[bank]->edge_set = pintbit;
		__irq_set_handler_locked(irq, handle_edge_irq);
	} else {
		pint[bank]->edge_clear = pintbit;
		__irq_set_handler_locked(irq, handle_level_irq);
	}

	return 0;
}

#ifdef CONFIG_PM
static struct adi_pm_pint_save save_pint_reg[NR_PINT_SYS_IRQS];

static int adi_gpio_set_wake(struct irq_data *d, unsigned int state)
{
	u32 pint_irq;
	u32 pint_val = irq2pint_lut[d->irq - SYS_IRQS];
	u32 bank = PINT_2_BANK(pint_val);

	switch (bank) {
	case 0:
		pint_irq = IRQ_PINT0;
		break;
	case 2:
		pint_irq = IRQ_PINT2;
		break;
	case 3:
		pint_irq = IRQ_PINT3;
		break;
	case 1:
		pint_irq = IRQ_PINT1;
		break;
	default:
		return -EINVAL;
	}

#ifndef SEC_GCTL
	adi_internal_set_wake(pint_irq, state);
#endif

	return 0;
}

void adi_pint_suspend(void)
{
	u32 bank;

	for (bank = 0; bank < NR_PINT_SYS_IRQS; bank++) {
		save_pint_reg[bank].mask_set = pint[bank]->mask_set;
		save_pint_reg[bank].assign = pint[bank]->assign;
		save_pint_reg[bank].edge_set = pint[bank]->edge_set;
		save_pint_reg[bank].invert_set = pint[bank]->invert_set;
	}
}

void adi_pint_resume(void)
{
	u32 bank;

	for (bank = 0; bank < NR_PINT_SYS_IRQS; bank++) {
		pint[bank]->mask_set = save_pint_reg[bank].mask_set;
		pint[bank]->assign = save_pint_reg[bank].assign;
		pint[bank]->edge_set = save_pint_reg[bank].edge_set;
		pint[bank]->invert_set = save_pint_reg[bank].invert_set;
	}
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
#else
#define adi_gpio_set_wake NULL
#endif

static void adi_demux_gpio_irq(unsigned int inta_irq,
			struct irq_desc *desc)
{
	u32 bank, pint_val;
	u32 request, irq;
	u32 level_mask;
	int umask = 0;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	if (chip->irq_mask_ack) {
		chip->irq_mask_ack(&desc->irq_data);
	} else {
		chip->irq_mask(&desc->irq_data);
		if (chip->irq_ack)
			chip->irq_ack(&desc->irq_data);
	}

	switch (inta_irq) {
	case IRQ_PINT0:
		bank = 0;
		break;
	case IRQ_PINT2:
		bank = 2;
		break;
	case IRQ_PINT3:
		bank = 3;
		break;
	case IRQ_PINT1:
		bank = 1;
		break;
#ifdef CONFIG_BF60x
	case IRQ_PINT4:
		bank = 4;
		break;
	case IRQ_PINT5:
		bank = 5;
		break;
#endif
	default:
		return;
	}

	pint_val = bank * NR_PINT_BITS;

	request = pint[bank]->request;

	level_mask = pint[bank]->edge_set & request;

	while (request) {
		if (request & 1) {
			irq = pint2irq_lut[pint_val] + SYS_IRQS;
			if (level_mask & PINT_BIT(pint_val)) {
				umask = 1;
				chip->irq_unmask(&desc->irq_data);
			}
			generic_handle_irq(irq);
		}
		pint_val++;
		request >>= 1;
	}

	if (!umask)
		chip->irq_unmask(&desc->irq_data);
}

static struct irq_chip adi_gpio_irqchip = {
	.name = "GPIO",
	.irq_ack = adi_gpio_ack_irq,
	.irq_mask = adi_gpio_mask_irq,
	.irq_mask_ack = adi_gpio_mask_ack_irq,
	.irq_unmask = adi_gpio_unmask_irq,
	.irq_disable = adi_gpio_mask_irq,
	.irq_enable = adi_gpio_unmask_irq,
	.irq_set_type = adi_gpio_irq_type,
	.irq_startup = adi_gpio_irq_startup,
	.irq_shutdown = adi_gpio_irq_shutdown,
	.irq_set_wake = adi_gpio_set_wake,
};

static int __init gpiolib_pint_setup(void)
{
	int pint_list[] = {IRQ_PINT0, IRQ_PINT1, IRQ_PINT2, IRQ_PINT3,
#ifdef COFNIG_BF60x
		IRQ_PINT4, IRQ_PINT5,
#endif
		-1
	};
	int i, irq;

# ifdef CONFIG_PINTx_REASSIGN
	pint[0]->assign = CONFIG_PINT0_ASSIGN;
	pint[1]->assign = CONFIG_PINT1_ASSIGN;
	pint[2]->assign = CONFIG_PINT2_ASSIGN;
	pint[3]->assign = CONFIG_PINT3_ASSIGN;
# ifdef COFNIG_BF60x
	pint[4]->assign = CONFIG_PINT4_ASSIGN;
	pint[5]->assign = CONFIG_PINT5_ASSIGN;
# endif
#endif
	/* Whenever PINTx_ASSIGN is altered init_pint_lut() must be executed! */
	init_pint_lut();

	for (i=0; pint_list[i]>=0; i++)
		irq_set_chained_handler(pint_list[i], adi_demux_gpio_irq);

	for (irq = GPIO_IRQ_BASE;
		irq < (GPIO_IRQ_BASE + MAX_GPIOS); irq++)
		irq_set_chip_and_handler(irq, &adi_gpio_irqchip,
					handle_level_irq);

	return 0;
}

static inline int check_gpio(unsigned gpio)
{
#ifdef CONFIG_BF54x
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
	int ret;

	if ((ret=gpiolib_pint_setup()) != 0)
		return ret;

	return gpiochip_add(&gpio_adi2_chip);
}
arch_initcall(gpiolib_setup);

/*
 * Pinctrl Driver for ADI GPIO2 controller
 *
 * Copyright 2007-2013 Analog Devices Inc.
 *
 * Licensed under the GPLv2 or later
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/platform_data/pinctrl-adi2.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>
#include <linux/syscore_ops.h>
#include <linux/gpio.h>
#include <asm/portmux.h>
#include "core.h"

static LIST_HEAD(adi_pint_list);
static LIST_HEAD(adi_pinctrl_list);

#define PERIPHERAL_USAGE 1
#define GPIO_USAGE 0

#define DRIVER_NAME "pinctrl-adi2"

#define RESOURCE_LABEL_SIZE	16
#define PINT_HI_OFFSET		16

#define RSV_NONE	0
#define RSV_GPIO	1
#define RSV_INT		2
#define RSV_PERI	3

/**
 * struct gpio_reserve_map - a GPIO map structure containing the
 * reservation status of each PIN.
 *
 * @owner: who request the reservation
 * @rsv_gpio: if this pin is reserved as GPIO
 * @rsv_int: if this pin is reserved as interrupt
 * @rsv_peri: if this pin is reserved as part of a peripheral device
 */
struct gpio_reserve_map {
	unsigned char owner[RESOURCE_LABEL_SIZE];
	bool rsv_gpio;
	bool rsv_int;
	bool rsv_peri;
};

/**
 * struct gpio_port_saved - GPIO port registers that should be saved between
 * power suspend and resume operations.
 *
 * @fer: PORTx_FER register
 * @data: PORTx_DATA register
 * @dir: PORTx_DIR register
 * @inen: PORTx_INEN register
 * @mux: PORTx_MUX register
 */
struct gpio_port_saved {
	u16 fer;
	u16 data;
	u16 dir;
	u16 inen;
	u32 mux;
};

/**
 * struct gpio_pint - GPIO interrupt controller device. Multiple ADI GPIO
 * banks can be mapped into one GPIO interrupt controller.
 *
 * @node: All gpio_pint instances are added to a global list.
 * @base: GPIO PINT device register base address
 * @irq: IRQ of the GPIO PINT device, it is the parent IRQ of all
 *       GPIO IRQs mapping to this device.
 * @domain: [0] irq domain of the gpio port, whose hardware interrupts are
 *		mapping to the low 16-bit of the pint registers.
 *          [1] irq domain of the gpio port, whose hardware interrupts are
 *		mapping to the high 16-bit of the pint registers.
 * @regs: address pointer to the GPIO PINT device
 * @map_count: No more than 2 GPIO banks can be mapped to this PINT device.
 * @lock: This lock make sure the irq_chip operations to one GPIO PINT device
 *        for different GPIO interrrupts are atomic.
 * @pint_map_port: Set up the mapping between one GPIO PINT device and
 *                 multiple GPIO banks.
 */
struct gpio_pint {
	struct list_head node;
	void __iomem *base;
	int irq;
	struct irq_domain *domain[2];
	struct gpio_pint_regs *regs;
	struct adi_pm_pint_save saved_data;
	int map_count;
	spinlock_t lock;

	int (*pint_map_port)(struct gpio_pint *pint, u8 assign,
				u8 map, struct irq_domain *domain);
};

/**
 * ADI pin controller
 *
 * @node: All adi_pmx instances are added to a global list.
 * @dev: a pointer back to containing device
 * @pctl: the pinctrl device
 * @gpio_list: the list of gpio banks owned by this pin controller.
 * @gpio_base: the base gpio number of this pin controller.
 */
struct adi_pmx {
	struct list_head node;
	struct device *dev;
	struct pinctrl_dev *pctl;
	struct list_head gpio_list;
	unsigned long gpio_base;
};

/**
 * struct gpio_pint - GPIO bank device. Multiple ADI GPIO banks can be mapped
 * into one GPIO interrupt controller.
 *
 * @node: All gpio_pint instances are added to a list.
 * @base: GPIO bank device register base address
 * @pin_base: base global GPIO pin index of the GPIO bank device
 * @irq_base: base IRQ of the GPIO bank device
 * @width: PIN number of the GPIO bank device
 * @range: The range space of the GPIO bank handled by the pin controller.
 * @regs: address pointer to the GPIO bank device
 * @saved_data: registers that should be saved between PM operations.
 * @dev: device structure of this GPIO bank
 * @pmx: the pinctrl device
 * @pint: GPIO PINT device that this GPIO bank mapped to
 * @pint_map: GIOP bank mapping code in PINT device
 * @pint_assign: The 32-bit GPIO PINT registers can be divided into 2 parts. A
 *               GPIO bank can be mapped into either low 16 bits[0] or high 16
 *               bits[1] of each PINT register.
 * @lock: This lock make sure the irq_chip operations to one GPIO PINT device
 *        for different GPIO interrrupts are atomic.
 * @chip: abstract a GPIO controller
 * @domain: The irq domain owned by the GPIO port.
 * @rsvmap: Reservation map array for each pin in the GPIO bank
 */
struct gpio_port {
	struct list_head node;
	void __iomem *base;
	unsigned int pin_base;
	unsigned int irq_base;
	unsigned int width;
	struct pinctrl_gpio_range range;
	struct gpio_port_t *regs;
	struct gpio_port_saved saved_data;
	struct device *dev;
	struct adi_pmx *pmx;

	struct gpio_pint *pint;
	u8 pint_map;
	u8 pint_assign;

	spinlock_t lock;
	struct gpio_chip chip;
	struct irq_domain *domain;

	struct gpio_reserve_map rsvmap[];
};

static inline u8 pin_to_offset(struct pinctrl_gpio_range *range, unsigned pin)
{
	return pin - range->pin_base;
}

static inline unsigned offset_to_gpio(struct gpio_port *port, u8 offset)
{
	return offset + port->chip.base;
}

static inline u8 gpio_to_offset(struct gpio_port *port, unsigned gpio)
{
	return gpio - port->chip.base;
}

static inline unsigned hwirq_to_pintbit(struct gpio_port *port, int hwirq)
{
	return (1 << hwirq) << (port->pint_assign * PINT_HI_OFFSET);
}

static void set_label(struct gpio_port *port, unsigned offset,
	const char *label)
{
	char *pch = port->rsvmap[offset].owner;

	if (label) {
		strncpy(pch, label, RESOURCE_LABEL_SIZE);
		pch[RESOURCE_LABEL_SIZE - 1] = 0;
	}
}

static char *get_label(struct gpio_port *port, unsigned offset)
{
	char *pch = port->rsvmap[offset].owner;

	return *pch ? pch : "UNKNOWN";
}

static inline unsigned int is_reserved(struct gpio_port *port, char type,
	unsigned offset)
{
	switch (type) {
	case RSV_GPIO:
		return port->rsvmap[offset].rsv_gpio == true;
	case RSV_INT:
		return port->rsvmap[offset].rsv_int == true;
	case RSV_PERI:
		return port->rsvmap[offset].rsv_peri == true;
	}

	return 0;
}

static inline void reserve(struct gpio_port *port, char type, unsigned offset)
{
	switch (type) {
	case RSV_GPIO:
		port->rsvmap[offset].rsv_gpio = true;
		break;
	case RSV_INT:
		port->rsvmap[offset].rsv_int = true;
		break;
	case RSV_PERI:
		port->rsvmap[offset].rsv_peri = true;
		break;
	}
}

static inline void unreserve(struct gpio_port *port, char type, unsigned offset)
{
	switch (type) {
	case RSV_GPIO:
		port->rsvmap[offset].rsv_gpio = false;
		break;
	case RSV_INT:
		port->rsvmap[offset].rsv_int = false;
		break;
	case RSV_PERI:
		port->rsvmap[offset].rsv_peri = false;
		break;
	}
}

static struct gpio_pint *find_gpio_pint(unsigned id)
{
	struct gpio_pint *pint;
	int i = 0;

	list_for_each_entry(pint, &adi_pint_list, node) {
		if (id == i)
			break;
		i++;
	}

	if (&pint->node != &adi_pint_list)
		return pint;
	else
		return NULL;
}

static struct adi_pmx *find_pinctrl(unsigned id)
{
	struct adi_pmx *pmx;
	int i = 0;

	list_for_each_entry(pmx, &adi_pinctrl_list, node) {
		if (id == i)
			break;
		i++;
	}

	if (&pmx->node != &adi_pinctrl_list)
		return pmx;
	else
		return NULL;
}

static struct gpio_port *find_gpio_port(unsigned pin,
	struct list_head *gpio_list)
{
	struct gpio_port *port;

	list_for_each_entry(port, gpio_list, node)
		if (pin >= port->range.pin_base &&
			pin < port->range.pin_base + port->range.npins)
			break;

	if (&port->node != gpio_list)
		return port;
	else
		return NULL;
}

static inline void port_setup(struct gpio_port *port, unsigned offset,
	unsigned short usage)
{
	struct gpio_port_t *regs = port->regs;

	if (usage == GPIO_USAGE)
		writew(readw(&regs->port_fer) & ~(1 << offset),
			&regs->port_fer);
	else
		writew(readw(&regs->port_fer) | (1 << offset), &regs->port_fer);
}

static inline void portmux_setup(struct gpio_port *port, unsigned offset,
	unsigned short function)
{
	struct gpio_port_t *regs = port->regs;
	u32 pmux;

	pmux = readl(&regs->port_mux);

	pmux &= ~(0x3 << (2 * offset));
	pmux |= (function & 0x3) << (2 * offset);

	writel(pmux, &regs->port_mux);
}

static inline u16 get_portmux(struct gpio_port *port, unsigned offset)
{
	struct gpio_port_t *regs = port->regs;
	u32 pmux = readl(&regs->port_mux);

	return pmux >> (2 * offset) & 0x3;
}


static void __adi_gpio_irq_prepare(struct gpio_port *port, unsigned offset)
{
	struct gpio_port_t *regs = port->regs;

	port_setup(port, offset, GPIO_USAGE);

	writew(1 << offset, &regs->dir_clear);
	writew(readw(&regs->inen) | (1 << offset), &regs->inen);
}

static int __adi_gpio_irq_request(struct gpio_port *port, unsigned offset,
	const char *label)
{
	if (is_reserved(port, RSV_PERI, offset)) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();

		dev_err(port->dev,
		       "GPIO %d is already reserved as Peripheral by %s !\n",
			offset_to_gpio(port, offset), get_label(port, offset));
		return -EBUSY;
	}
	if (is_reserved(port, RSV_GPIO, offset))
		dev_err(port->dev,
			"GPIO %d is already reserved by %s!\n",
			offset_to_gpio(port, offset), get_label(port, offset));

	reserve(port, RSV_INT, offset);
	set_label(port, offset, label);
	port_setup(port, offset, GPIO_USAGE);

	return 0;
}

static void __adi_gpio_irq_free(struct gpio_port *port, unsigned offset)
{
	if (!is_reserved(port, RSV_INT, offset)) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();

		dev_err(port->dev, "GPIO %d wasn't requested!\n",
			offset_to_gpio(port, offset));
		return;
	}

	unreserve(port, RSV_INT, offset);
	set_label(port, offset, "free");
}

static void adi_gpio_ack_irq(struct irq_data *d)
{
	unsigned long flags;
	struct gpio_port *port = irq_data_get_irq_chip_data(d);
	struct gpio_pint_regs *regs = port->pint->regs;
	unsigned pintbit = hwirq_to_pintbit(port, d->hwirq);

	spin_lock_irqsave(&port->lock, flags);
	spin_lock_irqsave(&port->pint->lock, flags);

	if (irqd_get_trigger_type(d) == IRQ_TYPE_EDGE_BOTH) {
		if (readl(&regs->invert_set) & pintbit)
			writel(pintbit, &regs->invert_clear);
		else
			writel(pintbit, &regs->invert_set);
	}

	writel(pintbit, &regs->request);

	spin_unlock_irqrestore(&port->pint->lock, flags);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void adi_gpio_mask_ack_irq(struct irq_data *d)
{
	unsigned long flags;
	struct gpio_port *port = irq_data_get_irq_chip_data(d);
	struct gpio_pint_regs *regs = port->pint->regs;
	unsigned pintbit = hwirq_to_pintbit(port, d->hwirq);

	spin_lock_irqsave(&port->lock, flags);
	spin_lock_irqsave(&port->pint->lock, flags);

	if (irqd_get_trigger_type(d) == IRQ_TYPE_EDGE_BOTH) {
		if (readl(&regs->invert_set) & pintbit)
			writel(pintbit, &regs->invert_clear);
		else
			writel(pintbit, &regs->invert_set);
	}

	writel(pintbit, &regs->request);
	writel(pintbit, &regs->mask_clear);

	spin_unlock_irqrestore(&port->pint->lock, flags);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void adi_gpio_mask_irq(struct irq_data *d)
{
	unsigned long flags;
	struct gpio_port *port = irq_data_get_irq_chip_data(d);
	struct gpio_pint_regs *regs = port->pint->regs;

	spin_lock_irqsave(&port->lock, flags);
	spin_lock_irqsave(&port->pint->lock, flags);

	writel(hwirq_to_pintbit(port, d->hwirq), &regs->mask_clear);

	spin_unlock_irqrestore(&port->pint->lock, flags);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void adi_gpio_unmask_irq(struct irq_data *d)
{
	unsigned long flags;
	struct gpio_port *port = irq_data_get_irq_chip_data(d);
	struct gpio_pint_regs *regs = port->pint->regs;

	spin_lock_irqsave(&port->lock, flags);
	spin_lock_irqsave(&port->pint->lock, flags);

	writel(hwirq_to_pintbit(port, d->hwirq), &regs->mask_set);

	spin_unlock_irqrestore(&port->pint->lock, flags);
	spin_unlock_irqrestore(&port->lock, flags);
}

static unsigned int adi_gpio_irq_startup(struct irq_data *d)
{
	unsigned long flags;
	struct gpio_port *port = irq_data_get_irq_chip_data(d);
	struct gpio_pint_regs *regs = port->pint->regs;

	if (!port) {
		dev_err(port->dev, "GPIO IRQ %d :Not exist\n", d->irq);
		return -ENODEV;
	}

	spin_lock_irqsave(&port->lock, flags);
	spin_lock_irqsave(&port->pint->lock, flags);

	__adi_gpio_irq_prepare(port, d->hwirq);
	writel(hwirq_to_pintbit(port, d->hwirq), &regs->mask_set);

	spin_unlock_irqrestore(&port->pint->lock, flags);
	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static void adi_gpio_irq_shutdown(struct irq_data *d)
{
	unsigned long flags;
	struct gpio_port *port = irq_data_get_irq_chip_data(d);
	struct gpio_pint_regs *regs = port->pint->regs;

	spin_lock_irqsave(&port->lock, flags);
	spin_lock_irqsave(&port->pint->lock, flags);

	writel(hwirq_to_pintbit(port, d->hwirq), &regs->mask_clear);
	__adi_gpio_irq_free(port, d->hwirq);

	spin_unlock_irqrestore(&port->pint->lock, flags);
	spin_unlock_irqrestore(&port->lock, flags);
}

static int adi_gpio_irq_type(struct irq_data *d, unsigned int type)
{
	unsigned long flags;
	struct gpio_port *port = irq_data_get_irq_chip_data(d);
	struct gpio_pint_regs *pint_regs = port->pint->regs;
	unsigned pintmask;
	unsigned int irq = d->irq;
	int ret = 0;
	char buf[16];

	if (!port) {
		dev_err(port->dev, "GPIO IRQ %d :Not exist\n", irq);
		return -ENODEV;
	}

	pintmask = hwirq_to_pintbit(port, d->hwirq);

	spin_lock_irqsave(&port->lock, flags);
	spin_lock_irqsave(&port->pint->lock, flags);

	if (type == IRQ_TYPE_PROBE)
		type = IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING;

	if (type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING |
		    IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW)) {
		snprintf(buf, 16, "gpio-irq%d", irq);
		ret = __adi_gpio_irq_request(port, d->hwirq, buf);
		if (ret)
			goto out;
	} else
		goto out;

	if ((type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_LEVEL_LOW)))
		/* low or falling edge denoted by one */
		writel(pintmask, &pint_regs->invert_set);
	else
		/* high or rising edge denoted by zero */
		writel(pintmask, &pint_regs->invert_clear);

	if ((type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH) {
		if (gpio_get_value(offset_to_gpio(port, d->hwirq)))
			writel(pintmask, &pint_regs->invert_set);
		else
			writel(pintmask, &pint_regs->invert_clear);
	}

	if (type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING)) {
		writel(pintmask, &pint_regs->edge_set);
		__irq_set_handler_locked(irq, handle_edge_irq);
	} else {
		writel(pintmask, &pint_regs->edge_clear);
		__irq_set_handler_locked(irq, handle_level_irq);
	}

out:
	spin_unlock_irqrestore(&port->pint->lock, flags);
	spin_unlock_irqrestore(&port->lock, flags);

	return ret;
}

#ifdef CONFIG_PM
static int adi_gpio_set_wake(struct irq_data *d, unsigned int state)
{
	struct gpio_port *port = irq_data_get_irq_chip_data(d);

	if (!port && !port->pint && port->pint->irq != d->irq)
		return -EINVAL;

#ifndef SEC_GCTL
	adi_internal_set_wake(port->pint->irq, state);
#endif

	return 0;
}

static int adi_pint_suspend(void)
{
	struct gpio_pint *pint;

	list_for_each_entry(pint, &adi_pint_list, node) {
		writel(0xffffffff, &pint->regs->mask_clear);
		pint->saved_data.assign = readl(&pint->regs->assign);
		pint->saved_data.edge_set = readl(&pint->regs->edge_set);
		pint->saved_data.invert_set = readl(&pint->regs->invert_set);
	}

	return 0;
}

static void adi_pint_resume(void)
{
	struct gpio_pint *pint;

	list_for_each_entry(pint, &adi_pint_list, node) {
		writel(pint->saved_data.assign, &pint->regs->assign);
		writel(pint->saved_data.edge_set, &pint->regs->edge_set);
		writel(pint->saved_data.invert_set, &pint->regs->invert_set);
	}
}

static int adi_gpio_suspend(void)
{
	struct adi_pmx *pmx;
	struct gpio_port *port;

	list_for_each_entry(pmx, &adi_pinctrl_list, node)
		list_for_each_entry(port, &pmx->gpio_list, node) {
			port->saved_data.fer = readw(&port->regs->port_fer);
			port->saved_data.mux = readl(&port->regs->port_mux);
			port->saved_data.data = readw(&port->regs->data);
			port->saved_data.inen = readw(&port->regs->inen);
			port->saved_data.dir = readw(&port->regs->dir_set);
		}

	return adi_pint_suspend();
}

static void adi_gpio_resume(void)
{
	struct adi_pmx *pmx;
	struct gpio_port *port;

	adi_pint_resume();

	list_for_each_entry(pmx, &adi_pinctrl_list, node)
		list_for_each_entry(port, &pmx->gpio_list, node) {
			writel(port->saved_data.mux, &port->regs->port_mux);
			writew(port->saved_data.fer, &port->regs->port_fer);
			writew(port->saved_data.inen, &port->regs->inen);
			writew(port->saved_data.data & port->saved_data.dir,
						&port->regs->data_set);
			writew(port->saved_data.dir, &port->regs->dir_set);
		}

}

static struct syscore_ops gpio_pm_syscore_ops = {
	.suspend = adi_gpio_suspend,
	.resume = adi_gpio_resume,
};
#else /* CONFIG_PM */
#define adi_gpio_set_wake NULL
#endif /* CONFIG_PM */

#ifdef CONFIG_IRQ_PREFLOW_FASTEOI
static inline void preflow_handler(struct irq_desc *desc)
{
	if (desc->preflow_handler)
		desc->preflow_handler(&desc->irq_data);
}
#else
static inline void preflow_handler(struct irq_desc *desc) { }
#endif

static void adi_gpio_handle_pint_irq(unsigned int inta_irq,
			struct irq_desc *desc)
{
	u32 request;
	u32 level_mask, hwirq;
	int umask = 0;
	struct gpio_pint *pint = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct gpio_pint_regs *regs = pint->regs;
	struct irq_domain *domain;

	preflow_handler(desc);
	chained_irq_enter(chip, desc);

	request = readl(&regs->request);
	level_mask = readl(&regs->edge_set) & request;

	hwirq = 0;
	domain = pint->domain[0];
	while (request) {
		if (hwirq == PINT_HI_OFFSET)
			domain = pint->domain[1];

		if (request & 1) {
			if (level_mask & (1 << hwirq)) {
				umask = 1;
				chained_irq_exit(chip, desc);
			}
			generic_handle_irq(irq_find_mapping(domain,
					hwirq % PINT_HI_OFFSET));
		}

		hwirq++;
		request >>= 1;
	}

	if (!umask)
		chained_irq_exit(chip, desc);
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


static int adi_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(adi_pin_groups);
}

static const char *adi_get_group_name(struct pinctrl_dev *pctldev,
				       unsigned selector)
{
	return adi_pin_groups[selector].name;
}

static int adi_get_group_pins(struct pinctrl_dev *pctldev, unsigned selector,
			       const unsigned **pins,
			       unsigned *num_pins)
{
	*pins = adi_pin_groups[selector].pins;
	*num_pins = adi_pin_groups[selector].num;
	return 0;
}

static void adi_pin_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
		   unsigned offset)
{
	seq_puts(s, DRIVER_NAME);
}

static struct pinctrl_ops adi_pctrl_ops = {
	.get_groups_count = adi_get_groups_count,
	.get_group_name = adi_get_group_name,
	.get_group_pins = adi_get_group_pins,
	.pin_dbg_show = adi_pin_dbg_show,
};

static int adi_pinmux_request(struct pinctrl_dev *pctldev, unsigned pin)
{
	struct adi_pmx *pmx;
	struct gpio_port *port;
	unsigned long flags;
	struct pin_desc *desc;
	u8 offset;

	pmx = pinctrl_dev_get_drvdata(pctldev);

	port = find_gpio_port(pin, &pmx->gpio_list);
	if (port == NULL) {
		dev_err(pctldev->dev,
		       "%s: Peripheral PIN %d doesn't exist!\n",
		       __func__, pin);
		return -ENODEV;
	}

	offset = pin_to_offset(&port->range, pin);
	desc = radix_tree_lookup(&pctldev->pin_desc_tree, pin);

	spin_lock_irqsave(&port->lock, flags);

	/* If a pin can be muxed as either GPIO or peripheral, make
	 * sure it is not already a GPIO pin when we request it.
	 */
	if (is_reserved(port, RSV_GPIO, offset)) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		dev_err(pctldev->dev,
		       "%s: Peripheral PIN %d is already reserved as GPIO by %s!\n",
		       __func__, pin, get_label(port, offset));
		spin_unlock_irqrestore(&port->lock, flags);
		return -EBUSY;
	}

	if (is_reserved(port, RSV_PERI, offset)) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		dev_err(pctldev->dev,
			"%s: Peripheral PIN %d is already reserved by %s!\n",
			__func__, pin, get_label(port, offset));
		spin_unlock_irqrestore(&port->lock, flags);
		return -EBUSY;
	}

	reserve(port, RSV_PERI, offset);
	set_label(port, offset, desc->mux_owner);

	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static int adi_pinmux_free(struct pinctrl_dev *pctldev, unsigned pin)
{
	struct adi_pmx *pmx;
	struct gpio_port *port;
	unsigned long flags;
	u8 offset;

	pmx = pinctrl_dev_get_drvdata(pctldev);
	port = find_gpio_port(pin, &pmx->gpio_list);
	if (port == NULL)
		return 0;

	offset = pin_to_offset(&port->range, pin);

	spin_lock_irqsave(&port->lock, flags);

	if (!is_reserved(port, RSV_PERI, offset)) {
		spin_unlock_irqrestore(&port->lock, flags);
		return 0;
	}

	unreserve(port, RSV_PERI, offset);
	set_label(port, offset, "free");

	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static int adi_pinmux_enable(struct pinctrl_dev *pctldev, unsigned selector,
	unsigned group)
{
	struct adi_pmx *pmx;
	struct gpio_port *port;
	unsigned long flags;
	unsigned short *mux = (unsigned short *)adi_pmx_functions[selector].mux;
	unsigned short gpio;

	pmx = pinctrl_dev_get_drvdata(pctldev);

	while (*mux) {
		gpio = P_IDENT(*mux);

		port = find_gpio_port(gpio - pmx->gpio_base, &pmx->gpio_list);
		if (port == NULL) /* should not happen */
			continue;

		spin_lock_irqsave(&port->lock, flags);

		portmux_setup(port, gpio_to_offset(port, gpio),
				 P_FUNCT2MUX(*mux));
		port_setup(port, gpio_to_offset(port, gpio), PERIPHERAL_USAGE);
		mux++;

		spin_unlock_irqrestore(&port->lock, flags);
	}

	return 0;
}

static void adi_pinmux_disable(struct pinctrl_dev *pctldev, unsigned selector,
	unsigned group)
{
	struct adi_pmx *pmx;
	struct gpio_port *port;
	unsigned long flags;
	unsigned short *mux = (unsigned short *)adi_pmx_functions[selector].mux;
	unsigned short gpio;

	pmx = pinctrl_dev_get_drvdata(pctldev);

	while (*mux) {
		gpio = P_IDENT(*mux);

		port = find_gpio_port(gpio - pmx->gpio_base, &pmx->gpio_list);
		if (port == NULL) /* should not happen */
			continue;

		spin_lock_irqsave(&port->lock, flags);

		port_setup(port, gpio_to_offset(port, gpio), GPIO_USAGE);
		mux++;

		spin_unlock_irqrestore(&port->lock, flags);
	}
}

static int adi_pinmux_get_funcs_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(adi_pmx_functions);
}

static const char *adi_pinmux_get_func_name(struct pinctrl_dev *pctldev,
					  unsigned selector)
{
	return adi_pmx_functions[selector].name;
}

static int adi_pinmux_get_groups(struct pinctrl_dev *pctldev, unsigned selector,
			       const char * const **groups,
			       unsigned * const num_groups)
{
	*groups = adi_pmx_functions[selector].groups;
	*num_groups = adi_pmx_functions[selector].num_groups;
	return 0;
}

static int adi_pinmux_request_gpio(struct pinctrl_dev *pctldev,
	struct pinctrl_gpio_range *range, unsigned pin)
{
	struct adi_pmx *pmx;
	struct gpio_port *port;
	unsigned long flags;
	u8 offset;

	pmx = pinctrl_dev_get_drvdata(pctldev);
	port = find_gpio_port(pin, &pmx->gpio_list);
	if (port == NULL) {
		dev_err(pctldev->dev,
		       "%s: GPIO PIN %d doesn't exist!\n",
		       __func__, pin);
		return -ENODEV;
	}

	offset = pin_to_offset(&port->range, pin);

	spin_lock_irqsave(&port->lock, flags);

	if (is_reserved(port, RSV_GPIO, offset)) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		dev_err(pctldev->dev,
			"GPIO %d is already reserved by %s !\n",
			offset_to_gpio(port, offset), get_label(port, offset));
		spin_unlock_irqrestore(&port->lock, flags);
		return -EBUSY;
	}
	if (is_reserved(port, RSV_PERI, offset)) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		dev_err(pctldev->dev,
			"GPIO %d is already reserved as peripheral by %s !\n",
			offset_to_gpio(port, offset), get_label(port, offset));
		spin_unlock_irqrestore(&port->lock, flags);
		return -EBUSY;
	}
	if (is_reserved(port, RSV_INT, offset)) {
		dev_err(pctldev->dev,
			"GPIO %d is already reserved as gpio-irq!\n",
			offset_to_gpio(port, offset));
	}

	reserve(port, RSV_GPIO, offset);
	set_label(port, offset, port->chip.label);
	port_setup(port, offset, GPIO_USAGE);

	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static void adi_pinmux_free_gpio(struct pinctrl_dev *pctldev,
	struct pinctrl_gpio_range *range, unsigned pin)
{
	struct adi_pmx *pmx;
	struct gpio_port *port;
	unsigned long flags;
	u8 offset;

	pmx = pinctrl_dev_get_drvdata(pctldev);
	port = find_gpio_port(pin, &pmx->gpio_list);
	if (port == NULL)
		return;

	offset = pin_to_offset(&port->range, pin);

	spin_lock_irqsave(&port->lock, flags);

	if (!is_reserved(port, RSV_GPIO, offset)) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();

		dev_err(pctldev->dev,
			"GPIO %d wasn't requested!\n",
			offset_to_gpio(port, offset));
		spin_unlock_irqrestore(&port->lock, flags);
		return;
	}

	unreserve(port, RSV_GPIO, offset);
	set_label(port, offset, "free");

	spin_unlock_irqrestore(&port->lock, flags);

	return;
}

static struct pinmux_ops adi_pinmux_ops = {
	.request = adi_pinmux_request,
	.free = adi_pinmux_free,
	.enable = adi_pinmux_enable,
	.disable = adi_pinmux_disable,
	.get_functions_count = adi_pinmux_get_funcs_count,
	.get_function_name = adi_pinmux_get_func_name,
	.get_function_groups = adi_pinmux_get_groups,
	.gpio_request_enable = adi_pinmux_request_gpio,
	.gpio_disable_free = adi_pinmux_free_gpio,
};


static struct pinctrl_desc adi_pinmux_desc = {
	.name = DRIVER_NAME,
	.pins = adi_pads,
	.npins = ARRAY_SIZE(adi_pads),
	.pctlops = &adi_pctrl_ops,
	.pmxops = &adi_pinmux_ops,
	.owner = THIS_MODULE,
};

static int adi_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_port *port;

	port = container_of(chip, struct gpio_port, chip);

	return pinctrl_request_gpio(offset_to_gpio(port, offset));
}

static void adi_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_port *port;

	port = container_of(chip, struct gpio_port, chip);

	pinctrl_free_gpio(offset_to_gpio(port, offset));
}

static int adi_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_port *port;
	unsigned long flags;

	port = container_of(chip, struct gpio_port, chip);

	spin_lock_irqsave(&port->lock, flags);

	if (!is_reserved(port, RSV_GPIO, offset)) {
		dev_err(port->dev, "GPIO %d wasn't requested!\n",
			offset_to_gpio(port, offset));
		spin_unlock_irqrestore(&port->lock, flags);
		return -EINVAL;
	}

	writew(1 << offset, &port->regs->dir_clear);
	writew(readw(&port->regs->inen) | (1 << offset), &port->regs->inen);

	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static void adi_gpio_set_value(struct gpio_chip *chip, unsigned offset,
	int value)
{
	struct gpio_port *port = container_of(chip, struct gpio_port, chip);
	struct gpio_port_t *regs = port->regs;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	if (value)
		writew(1 << offset, &regs->data_set);
	else
		writew(1 << offset, &regs->data_clear);

	spin_unlock_irqrestore(&port->lock, flags);
}

static int adi_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
	int value)
{
	struct gpio_port *port = container_of(chip, struct gpio_port, chip);
	struct gpio_port_t *regs = port->regs;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	if (!is_reserved(port, RSV_GPIO, offset)) {
		dev_err(port->dev, "GPIO %d wasn't requested!\n",
			offset_to_gpio(port, offset));
		spin_unlock_irqrestore(&port->lock, flags);
		return -EINVAL;
	}

	writew(readw(&regs->inen) & ~(1 << offset), &regs->inen);
	adi_gpio_set_value(chip, offset, value);
	writew(1 << offset, &regs->dir_set);

	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static int adi_gpio_get_value(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_port *port = container_of(chip, struct gpio_port, chip);
	struct gpio_port_t *regs = port->regs;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&port->lock, flags);

	ret = 1 & (readw(&regs->data) >> offset);

	spin_unlock_irqrestore(&port->lock, flags);

	return ret;
}

static int adi_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_port *port = container_of(chip, struct gpio_port, chip);

	if (port->irq_base >= 0)
		return irq_find_mapping(port->domain, offset);
	else
		return irq_create_mapping(port->domain, offset);
}

#if defined(CONFIG_PROC_FS)
static inline unsigned short get_gpio_dir(struct gpio_port *port,
	unsigned offset)
{
	struct gpio_port_t *regs = port->regs;

	return 1 & (readw(&regs->dir_clear) >> offset);
}

static int gpio_proc_show(struct seq_file *m, void *v)
{
	int offset, irq, gpio;
	struct adi_pmx *pmx;
	struct gpio_port *port;

	list_for_each_entry(pmx, &adi_pinctrl_list, node)
	list_for_each_entry(port, &pmx->gpio_list, node)
		for (offset = 0; offset < port->width; offset++) {
			irq = is_reserved(port, RSV_INT, offset);
			gpio = is_reserved(port, RSV_GPIO, offset);
			if (gpio || irq)
				seq_printf(m, "GPIO_%d: \t%s%s \t\tGPIO %s\n",
					offset_to_gpio(port, offset),
					get_label(port, offset),
					(gpio && irq) ? " *" : "",
					get_gpio_dir(port, offset) ?
					"OUTPUT" : "INPUT");
			else if (is_reserved(port, RSV_PERI, offset))
				seq_printf(m, "GPIO_%d: \t%s \t\tPeripheral\n",
					offset_to_gpio(port, offset),
					get_label(port, offset));
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
device_initcall(gpio_register_proc);
#endif

static int adi_pint_map_port(struct gpio_pint *pint, u8 assign, u8 map,
	struct irq_domain *domain)
{
	struct gpio_pint_regs *regs = pint->regs;

	if (pint->map_count > 1)
		return -EINVAL;

	if (assign > 1)
		return -EINVAL;

	pint->map_count++;

	writel((readl(&regs->assign) & (0xFFFF << !assign * PINT_HI_OFFSET)) |
		(((map << 8) | map) << assign * PINT_HI_OFFSET), &regs->assign);

	pint->domain[assign] = domain;

	return 0;
}

static int adi_gpio_pint_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct gpio_pint *pint;

	pint = devm_kzalloc(dev, sizeof(struct gpio_pint), GFP_KERNEL);
	if (!pint) {
		dev_err(dev, "Memory alloc failed\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Invalid mem resource\n");
		return -ENODEV;
	}

	if (!devm_request_mem_region(dev, res->start, resource_size(res),
				     pdev->name)) {
		dev_err(dev, "Region already claimed\n");
		return -EBUSY;
	}

	pint->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!pint->base) {
		dev_err(dev, "Could not ioremap\n");
		return -ENOMEM;
	}

	pint->regs = (struct gpio_pint_regs *)pint->base;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "Invalid IRQ resource\n");
		return -ENODEV;
	}

	spin_lock_init(&pint->lock);

	pint->irq = res->start;
	pint->pint_map_port = adi_pint_map_port;
	platform_set_drvdata(pdev, pint);

	irq_set_chained_handler(pint->irq, adi_gpio_handle_pint_irq);
	irq_set_handler_data(pint->irq, pint);

	list_add_tail(&pint->node, &adi_pint_list);

	return 0;
}

static int adi_gpio_pint_remove(struct platform_device *pdev)
{
	struct gpio_pint *pint = platform_get_drvdata(pdev);

	list_del(&pint->node);
	irq_set_handler(pint->irq, handle_simple_irq);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int adi_gpio_irq_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hwirq)
{
	struct gpio_port *port = d->host_data;

	if (!port)
		return -EINVAL;

	irq_set_chip_data(irq, port);
	irq_set_chip_and_handler(irq, &adi_gpio_irqchip,
				handle_level_irq);

	return 0;
}

const struct irq_domain_ops adi_gpio_irq_domain_ops = {
	.map = adi_gpio_irq_map,
	.xlate = irq_domain_xlate_onecell,
};

static int adi_gpio_init_int(struct gpio_port *port)
{
	struct device_node *node = port->dev->of_node;
	struct gpio_pint *pint = port->pint;
	int ret;

	port->domain = irq_domain_add_linear(node, port->width,
				&adi_gpio_irq_domain_ops, port);
	if (!port->domain) {
		dev_err(port->dev, "Failed to create irqdomain\n");
		return -ENOSYS;
	}

	ret = pint->pint_map_port(port->pint, port->pint_assign,
			port->pint_map,	port->domain);
	if (ret)
		return ret;

	if (port->irq_base >= 0) {
		ret = irq_create_strict_mappings(port->domain, port->irq_base,
					0, port->width);
		if (ret) {
			dev_err(port->dev, "Couldn't associate to domain\n");
			return ret;
		}
	}

	return 0;
}

static int adi_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct adi_pinctrl_gpio_platform_data *pdata;
	struct resource *res;
	struct gpio_port *port;
	static int gpio;
	int ret = 0;

	pdata = dev->platform_data;
	if (!pdata)
		return -EINVAL;

	port = devm_kzalloc(dev, sizeof(struct gpio_port) +
		sizeof(struct gpio_reserve_map) * pdata->port_width,
		GFP_KERNEL);
	if (!port) {
		dev_err(dev, "Memory alloc failed\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Invalid mem resource\n");
		return -ENODEV;
	}

	if (!devm_request_mem_region(dev, res->start, resource_size(res),
				     pdev->name)) {
		dev_err(dev, "Region already claimed\n");
		return -EBUSY;
	}

	port->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!port->base) {
		dev_err(dev, "Could not ioremap\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res)
		port->irq_base = -1;
	else
		port->irq_base = res->start;

	port->width = pdata->port_width;
	port->dev = dev;
	port->regs = (struct gpio_port_t *)port->base;
	port->pint_assign = !!pdata->pint_assign;
	port->pint_map = pdata->pint_map;

	port->pint = find_gpio_pint(pdata->pint_id);
	if (port->pint) {
		ret = adi_gpio_init_int(port);
		if (ret)
			return ret;
	}

	port->pmx = find_pinctrl(pdata->pinctrl_id);
	if (port->pmx == NULL) {
		dev_err(dev, "Could not find pinctrl device\n");
		return -ENODEV;
	}
	if (gpio == 0)
		gpio = port->pmx->gpio_base;

	spin_lock_init(&port->lock);

	platform_set_drvdata(pdev, port);

	port->chip.label		= "adi-gpio";
	port->chip.direction_input	= adi_gpio_direction_input;
	port->chip.get			= adi_gpio_get_value;
	port->chip.direction_output	= adi_gpio_direction_output;
	port->chip.set			= adi_gpio_set_value;
	port->chip.request		= adi_gpio_request;
	port->chip.free			= adi_gpio_free;
	port->chip.to_irq		= adi_gpio_to_irq;
	if (pdata->port_pin_base > 0)
		port->chip.base		= pdata->port_pin_base +
						port->pmx->gpio_base;
	else
		port->chip.base		= gpio;
	port->chip.ngpio		= port->width;
	gpio = port->chip.base + port->width;

	ret = gpiochip_add(&port->chip);
	if (ret)
		return ret;

	/* Set gpio range to pinctrl driver */
	port->range.name = port->chip.label;
	port->range.id = pdev->id;
	port->range.base = port->chip.base;
	port->range.pin_base = port->chip.base - port->pmx->gpio_base;
	port->range.npins = port->width;
	port->range.gc = &port->chip;
	pinctrl_add_gpio_range(port->pmx->pctl, &port->range);

	list_add_tail(&port->node, &port->pmx->gpio_list);

	return 0;
}

static int adi_gpio_remove(struct platform_device *pdev)
{
	struct gpio_port *port = platform_get_drvdata(pdev);
	int ret;
	u8 offset;

	for (offset = 0; offset < port->width; offset++)
		irq_dispose_mapping(irq_find_mapping(port->domain, offset));

	irq_domain_remove(port->domain);
	pinctrl_remove_gpio_range(port->pmx->pctl, &port->range);
	list_del(&port->node);
	ret = gpiochip_remove(&port->chip);
	platform_set_drvdata(pdev, NULL);

	return ret;
}

static int adi_pinctrl_probe(struct platform_device *pdev)
{
	struct adi_pmx *pmx;
	struct resource *res;

	pmx = devm_kzalloc(&pdev->dev, sizeof(*pmx), GFP_KERNEL);
	if (!pmx)
		return -ENOMEM;

	pmx->dev = &pdev->dev;

	/* Now register the pin controller and all pins it handles */
	pmx->pctl = pinctrl_register(&adi_pinmux_desc, &pdev->dev, pmx);
	if (!pmx->pctl) {
		dev_err(&pdev->dev, "could not register pinctrl ADI2 driver\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (res)
		pmx->gpio_base = res->start;

	INIT_LIST_HEAD(&pmx->gpio_list);

	list_add_tail(&pmx->node, &adi_pinctrl_list);

	return 0;
}

static int adi_pinctrl_remove(struct platform_device *pdev)
{
	struct adi_pmx *pmx = platform_get_drvdata(pdev);

	list_del(&pmx->node);
	pinctrl_unregister(pmx->pctl);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver adi_pinctrl_driver = {
	.probe		= adi_pinctrl_probe,
	.remove		= adi_pinctrl_remove,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static struct platform_driver adi_gpio_pint_driver = {
	.probe		= adi_gpio_pint_probe,
	.remove		= adi_gpio_pint_remove,
	.driver		= {
		.name	= "adi-gpio-pint",
	},
};

static struct platform_driver adi_gpio_driver = {
	.probe		= adi_gpio_probe,
	.remove		= adi_gpio_remove,
	.driver		= {
		.name	= "adi-gpio",
	},
};

static int __init adi_pinctrl_setup(void)
{
	int ret;

	ret = platform_driver_register(&adi_pinctrl_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&adi_gpio_pint_driver);
	if (ret)
		goto pint_error;

	ret = platform_driver_register(&adi_gpio_driver);
	if (ret)
		goto gpio_error;

#ifdef CONFIG_PM
	register_syscore_ops(&gpio_pm_syscore_ops);
#endif
	return ret;
gpio_error:
	platform_driver_unregister(&adi_gpio_pint_driver);
pint_error:
	platform_driver_unregister(&adi_pinctrl_driver);

	return ret;
}
postcore_initcall(adi_pinctrl_setup);

MODULE_AUTHOR("Sonic Zhang <sonic.zhang@analog.com>");
MODULE_DESCRIPTION("ADI gpio2 pin control driver");
MODULE_LICENSE("GPL");

/* ADI direct peripheral pin request APIs. Don't send upstream */
int pinmux_request(unsigned short fer, const char *label)
{
	struct adi_pmx *pmx;
	struct gpio_port *port;
	unsigned long flags;
	u8 offset;
	unsigned pin = P_IDENT(fer);

	pmx = list_first_entry_or_null(&adi_pinctrl_list,
			struct adi_pmx, node);
	if (pmx == NULL)
		return -ENODEV;

	port = find_gpio_port(pin, &pmx->gpio_list);
	if (port == NULL) {
		dev_err(pmx->dev,
		       "%s: Peripheral PIN %d doesn't exist!\n",
		       __func__, pin);
		return -ENODEV;
	}

	offset = pin_to_offset(&port->range, pin);

	spin_lock_irqsave(&port->lock, flags);

	/* If a pin can be muxed as either GPIO or peripheral, make
	 * sure it is not already a GPIO pin when we request it.
	 */
	if (unlikely(is_reserved(port, RSV_GPIO, offset))) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		dev_err(pmx->dev,
		       "%s: Peripheral PIN %d is already reserved as GPIO by %s!\n",
		       __func__, pin, get_label(port, offset));
		spin_unlock_irqrestore(&port->lock, flags);
		return -EBUSY;
	}

	if (unlikely(is_reserved(port, RSV_PERI, offset))) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		dev_err(pmx->dev,
			"%s: Peripheral PIN %d is already reserved by %s!\n",
			__func__, pin, get_label(port, offset));
		spin_unlock_irqrestore(&port->lock, flags);
		return -EBUSY;
	}

	reserve(port, RSV_PERI, offset);
	set_label(port, offset, label);
	portmux_setup(port, offset, P_FUNCT2MUX(fer));
	port_setup(port, offset, PERIPHERAL_USAGE);

	spin_unlock_irqrestore(&port->lock, flags);

	return 0;

}

void pinmux_free(unsigned short fer)
{
	struct adi_pmx *pmx;
	struct gpio_port *port;
	unsigned long flags;
	u8 offset;
	unsigned pin = P_IDENT(fer);

	pmx = list_first_entry_or_null(&adi_pinctrl_list,
			struct adi_pmx, node);
	if (pmx == NULL)
		return;

	port = find_gpio_port(pin, &pmx->gpio_list);
	if (port == NULL)
		return;

	offset = pin_to_offset(&port->range, pin);

	spin_lock_irqsave(&port->lock, flags);

	if (unlikely(!is_reserved(port, RSV_PERI, offset))) {
		spin_unlock_irqrestore(&port->lock, flags);
		return;
	}

	unreserve(port, RSV_PERI, offset);
	set_label(port, offset, "free");
	port_setup(port, offset, GPIO_USAGE);

	spin_unlock_irqrestore(&port->lock, flags);

	return;
}

int pinmux_request_list(const unsigned short per[], const char *label)
{
	u16 cnt;
	int ret;

	for (cnt = 0; per[cnt] != 0; cnt++) {

		ret = pinmux_request(per[cnt], label);

		if (ret < 0) {
			for ( ; cnt > 0; cnt--)
				peripheral_free(per[cnt - 1]);

			return ret;
		}
	}

	return 0;
}

void pinmux_free_list(const unsigned short per[])
{
	u16 cnt;
	for (cnt = 0; per[cnt] != 0; cnt++)
		pinmux_free(per[cnt]);
}

/*
 * GPIO Driver for ADI GPIO2 controller
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
#include <linux/platform_data/gpio-adi2.h>
#include <linux/gpio.h>
#include <linux/syscore_ops.h>

static LIST_HEAD(adi_gpio_list);
static LIST_HEAD(adi_pint_list);

#define RESOURCE_LABEL_SIZE	16

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
 * @gpio_irq_base: [0] means gpio irq base is in low 16-bit registers.
 *                 [1] means gpio irq base is in high 16-bit registers.
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
	int gpio_irq_base[2];
	struct gpio_pint_regs *regs;
	struct adi_pm_pint_save saved_data;
	int map_count;
	spinlock_t lock;

	int (*pint_map_port)(struct gpio_pint *pint, u8 assign,
				u8 map, int irq_base);
};

/**
 * struct gpio_pint - GPIO bank device. Multiple ADI GPIO banks can be mapped
 * into one GPIO interrupt controller.
 *
 * @node: All gpio_pint instances are added to a global list.
 * @base: GPIO bank device register base address
 * @pin_base: base global GPIO pin index of the GPIO bank device
 * @irq_base: base IRQ of the GPIO bank device
 * @width: PIN number of the GPIO bank device
 * @regs: address pointer to the GPIO bank device
 * @saved_data: registers that should be saved between PM operations.
 * @dev: device structure of this GPIO bank
 * @pint: GPIO PINT device that this GPIO bank mapped to
 * @pint_map: GIOP bank mapping code in PINT device
 * @pint_assign: The 32-bit GPIO PINT registers can be divided into 2 parts. A
 *               GPIO bank can be mapped into either low 16 bits[0] or high 16
 *               bits[1] of each PINT register.
 * @lock: This lock make sure the irq_chip operations to one GPIO PINT device
 *        for different GPIO interrrupts are atomic.
 * @chip: abstract a GPIO controller
 * @rsvmap: Reservation map array for each pin in the GPIO bank
 */
struct gpio_port {
	struct list_head node;
	void __iomem *base;
	unsigned int pin_base;
	unsigned int irq_base;
	unsigned int width;
	struct gpio_port_t *regs;
	struct gpio_port_saved saved_data;
	struct device *dev;

	struct gpio_pint *pint;
	u8 pint_map;
	u8 pint_assign;

	spinlock_t lock;
	struct gpio_chip chip;

	struct gpio_reserve_map rsvmap[];
};

static inline unsigned offset_to_gpio(struct gpio_port *port, u8 offset)
{
	return offset + port->chip.base;
}

static inline u8 gpio_to_offset(struct gpio_port *port, unsigned gpio)
{
	return gpio - port->chip.base;
}

static inline u8 irq_to_offset(struct gpio_port *port, int irq)
{
	int offset;

	offset = irq - port->irq_base;
	if (offset >= 0 && offset < port->width)
		return offset;
	else
		return 0;
}

static inline unsigned irq_to_pintbit(struct gpio_port *port, int irq)
{
	return (1 << irq_to_offset(port, irq)) << (port->pint_assign * 16);
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

static int cmp_label(struct gpio_port *port, unsigned offset, const char *label)
{
	if (label == NULL) {
		dump_stack();
		dev_err(port->dev, "Please provide none-null label\n");
	}

	if (label)
		return strcmp(port->rsvmap[offset].owner, label);
	else
		return -EINVAL;
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

static struct gpio_port *find_gpio_port(unsigned gpio)
{
	struct gpio_port *port;

	list_for_each_entry(port, &adi_gpio_list, node)
		if (gpio >= port->chip.base &&
			gpio < port->chip.base + port->width)
			break;

	if (&port->node != &adi_gpio_list)
		return port;
	else
		return NULL;
}

static struct gpio_pint *find_gpio_pint(unsigned pint_id)
{
	struct gpio_pint *pint;
	int i = 0;

	list_for_each_entry(pint, &adi_pint_list, node) {
		if (pint_id == i)
			break;
		i++;
	}

	if (&pint->node != &adi_pint_list)
		return pint;
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
	if (unlikely(is_reserved(port, RSV_PERI, offset))) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();

		dev_err(port->dev,
		       "gpio-adi2: GPIO %d is already reserved as Peripheral by %s !\n",
			offset_to_gpio(port, offset), get_label(port, offset));
		return -EBUSY;
	}
	if (unlikely(is_reserved(port, RSV_GPIO, offset)))
		dev_err(port->dev,
			"gpio-adi2: GPIO %d is already reserved by %s!\n",
			offset_to_gpio(port, offset), get_label(port, offset));

	reserve(port, RSV_INT, offset);
	set_label(port, offset, label);
	port_setup(port, offset, GPIO_USAGE);

	return 0;
}

static void __adi_gpio_irq_free(struct gpio_port *port, unsigned offset)
{
	if (unlikely(!is_reserved(port, RSV_INT, offset))) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();

		dev_err(port->dev, "gpio-adi2: GPIO %d wasn't requested!\n",
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
	unsigned pintbit = irq_to_pintbit(port, d->irq);

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
	unsigned pintbit = irq_to_pintbit(port, d->irq);

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

	writel(irq_to_pintbit(port, d->irq), &regs->mask_clear);

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

	writel(irq_to_pintbit(port, d->irq), &regs->mask_set);

	spin_unlock_irqrestore(&port->pint->lock, flags);
	spin_unlock_irqrestore(&port->lock, flags);
}

static unsigned int adi_gpio_irq_startup(struct irq_data *d)
{
	unsigned long flags;
	struct gpio_port *port = irq_data_get_irq_chip_data(d);
	struct gpio_pint_regs *regs = port->pint->regs;
	unsigned offset = irq_to_offset(port, d->irq);

	if (!port) {
		dev_err(port->dev, "GPIO IRQ %d :Not exist\n", d->irq);
		return -ENODEV;
	}

	spin_lock_irqsave(&port->lock, flags);
	spin_lock_irqsave(&port->pint->lock, flags);

	__adi_gpio_irq_prepare(port, offset);
	writel(irq_to_pintbit(port, d->irq), &regs->mask_set);

	spin_unlock_irqrestore(&port->pint->lock, flags);
	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static void adi_gpio_irq_shutdown(struct irq_data *d)
{
	unsigned long flags;
	struct gpio_port *port = irq_data_get_irq_chip_data(d);
	struct gpio_pint_regs *regs = port->pint->regs;
	unsigned offset = irq_to_offset(port, d->irq);

	spin_lock_irqsave(&port->lock, flags);
	spin_lock_irqsave(&port->pint->lock, flags);

	writel(irq_to_pintbit(port, d->irq), &regs->mask_clear);
	__adi_gpio_irq_free(port, offset);

	spin_unlock_irqrestore(&port->pint->lock, flags);
	spin_unlock_irqrestore(&port->lock, flags);
}

static int adi_gpio_irq_type(struct irq_data *d, unsigned int type)
{
	unsigned long flags;
	struct gpio_port *port = irq_data_get_irq_chip_data(d);
	struct gpio_pint_regs *pint_regs = port->pint->regs;
	unsigned offset, pintmask;
	unsigned int irq = d->irq;
	int ret = 0;
	char buf[16];

	if (!port) {
		dev_err(port->dev, "GPIO IRQ %d :Not exist\n", irq);
		return -ENODEV;
	}

	offset = irq_to_offset(port, irq);
	pintmask = irq_to_pintbit(port, irq);

	spin_lock_irqsave(&port->lock, flags);
	spin_lock_irqsave(&port->pint->lock, flags);

	if (type == IRQ_TYPE_PROBE)
		type = IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING;

	if (type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING |
		    IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW)) {
		snprintf(buf, 16, "gpio-irq%d", irq);
		ret = __adi_gpio_irq_request(port, offset, buf);
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
		if (gpio_get_value(offset_to_gpio(port, offset)))
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
	struct gpio_port *port;

	list_for_each_entry(port, &adi_gpio_list, node) {
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
	struct gpio_port *port;

	adi_pint_resume();

	list_for_each_entry(port, &adi_gpio_list, node) {
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

static void adi_demux_gpio_irq(unsigned int inta_irq,
			struct irq_desc *desc)
{
	u32 request, irq;
	u32 level_mask, pintbit;
	int umask = 0;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct gpio_pint *pint = irq_desc_get_chip_data(desc);
	struct gpio_pint_regs *regs = pint->regs;

	if (chip->irq_mask_ack) {
		chip->irq_mask_ack(&desc->irq_data);
	} else {
		chip->irq_mask(&desc->irq_data);
		if (chip->irq_ack)
			chip->irq_ack(&desc->irq_data);
	}

	request = readl(&regs->request);
	level_mask = readl(&regs->edge_set) & request;

	pintbit = 0;
	irq = 0;
	while (request) {
		if (!(pintbit % 16))
			irq = pint->gpio_irq_base[pintbit >> 4];

		if (request & 1) {
			if (level_mask & (1 << pintbit)) {
				umask = 1;
				chip->irq_unmask(&desc->irq_data);
			}
			generic_handle_irq(irq);
		}

		pintbit++;
		irq++;
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

int peripheral_request(unsigned short per, const char *label)
{
	unsigned long flags;
	unsigned short ident = P_IDENT(per);
	struct gpio_port *port;
	unsigned offset;

	/*
	 * Don't cares are pins with only one dedicated function
	 */

	if (per & P_DONTCARE)
		return 0;

	if (!(per & P_DEFINED))
		return -ENODEV;

	port = find_gpio_port(ident);
	if (port == NULL)
		return -ENODEV;

	spin_lock_irqsave(&port->lock, flags);

	offset = gpio_to_offset(port, ident);

	/* If a pin can be muxed as either GPIO or peripheral, make
	 * sure it is not already a GPIO pin when we request it.
	 */
	if (unlikely(is_reserved(port, RSV_GPIO, offset))) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		dev_err(port->dev,
		       "%s: Peripheral %d is already reserved as GPIO by %s !\n",
		       __func__, ident, get_label(port, offset));
		spin_unlock_irqrestore(&port->lock, flags);
		return -EBUSY;
	}

	if (unlikely(is_reserved(port, RSV_PERI, offset))) {

		/*
		 * Pin functions like AMC address strobes my
		 * be requested and used by several drivers
		 */

		if (!((per & P_MAYSHARE) && get_portmux(port, offset) ==
			P_FUNCT2MUX(per))) {
			/*
			 * Allow that the identical pin function can
			 * be requested from the same driver twice
			 */

			if (cmp_label(port, offset, label) == 0)
				goto anyway;

			if (system_state == SYSTEM_BOOTING)
				dump_stack();
			dev_err(port->dev,
				"%s: Peripheral %d function %d is already reserved by %s !\n",
				__func__, ident, P_FUNCT2MUX(per),
				get_label(port, offset));
			spin_unlock_irqrestore(&port->lock, flags);
			return -EBUSY;
		}
	}

 anyway:
	reserve(port, RSV_PERI, offset);

	portmux_setup(port, offset, P_FUNCT2MUX(per));
	port_setup(port, offset, PERIPHERAL_USAGE);

	set_label(port, offset, label);

	spin_unlock_irqrestore(&port->lock, flags);

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
	struct gpio_port *port;
	unsigned offset;

	if (per & P_DONTCARE)
		return;

	if (!(per & P_DEFINED))
		return;

	port = find_gpio_port(ident);
	if (port == NULL)
		return;

	spin_lock_irqsave(&port->lock, flags);

	offset = gpio_to_offset(port, ident);

	if (unlikely(!is_reserved(port, RSV_PERI, offset))) {
		spin_unlock_irqrestore(&port->lock, flags);
		return;
	}

	if (!(per & P_MAYSHARE))
		port_setup(port, offset, GPIO_USAGE);

	unreserve(port, RSV_PERI, offset);

	set_label(port, offset, "free");

	spin_unlock_irqrestore(&port->lock, flags);
}
EXPORT_SYMBOL(peripheral_free);

void peripheral_free_list(const unsigned short per[])
{
	u16 cnt;
	for (cnt = 0; per[cnt] != 0; cnt++)
		peripheral_free(per[cnt]);
}
EXPORT_SYMBOL(peripheral_free_list);

static int adi_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_port *port;
	unsigned long flags;

	port = container_of(chip, struct gpio_port, chip);

	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Allow that the identical GPIO can
	 * be requested from the same driver twice
	 * Do nothing and return -
	 */

	if (cmp_label(port, offset, chip->label) == 0) {
		spin_unlock_irqrestore(&port->lock, flags);
		return 0;
	}

	if (unlikely(is_reserved(port, RSV_GPIO, offset))) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		dev_err(port->dev,
			"gpio-adi2: GPIO %d is already reserved by %s !\n",
			offset_to_gpio(port, offset), get_label(port, offset));
		spin_unlock_irqrestore(&port->lock, flags);
		return -EBUSY;
	}
	if (unlikely(is_reserved(port, RSV_PERI, offset))) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();
		dev_err(port->dev,
		       "gpio-adi2: GPIO %d is already reserved as Peripheral by %s !\n",
			offset_to_gpio(port, offset), get_label(port, offset));
		spin_unlock_irqrestore(&port->lock, flags);
		return -EBUSY;
	}
	if (unlikely(is_reserved(port, RSV_INT, offset))) {
		dev_err(port->dev,
			"gpio-adi2: GPIO %d is already reserved as gpio-irq!\n",
			offset_to_gpio(port, offset));
	}

	reserve(port, RSV_GPIO, offset);
	set_label(port, offset, chip->label);

	port_setup(port, offset, GPIO_USAGE);

	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static void adi_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_port *port;
	unsigned long flags;

	port = container_of(chip, struct gpio_port, chip);

	spin_lock_irqsave(&port->lock, flags);

	if (unlikely(!is_reserved(port, RSV_GPIO, offset))) {
		if (system_state == SYSTEM_BOOTING)
			dump_stack();

		dev_err(port->dev, "gpio-adi2: GPIO %d wasn't requested!\n",
			offset_to_gpio(port, offset));
		spin_unlock_irqrestore(&port->lock, flags);
		return;
	}

	unreserve(port, RSV_GPIO, offset);

	set_label(port, offset, "free");

	spin_unlock_irqrestore(&port->lock, flags);
}

static int adi_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_port *port;
	unsigned long flags;

	port = container_of(chip, struct gpio_port, chip);

	spin_lock_irqsave(&port->lock, flags);

	if (unlikely(!is_reserved(port, RSV_GPIO, offset))) {
		dev_err(port->dev, "gpio-adi2: GPIO %d wasn't requested!\n",
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

	if (unlikely(!is_reserved(port, RSV_GPIO, offset))) {
		dev_err(port->dev, "gpio-adi2: GPIO %d wasn't requested!\n",
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

	return port->irq_base + offset;
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
	struct gpio_port *port;

	list_for_each_entry(port, &adi_gpio_list, node)
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
__initcall(gpio_register_proc);
#endif

static int adi_pint_map_port(struct gpio_pint *pint, u8 assign, u8 map,
	int irq_base)
{
	struct gpio_pint_regs *regs = pint->regs;

	if (pint->map_count > 1)
		return -EINVAL;

	if (assign > 1)
		return -EINVAL;

	pint->map_count++;

	writel((readl(&regs->assign) & (0xFFFF << !assign * 16)) |
		(((map << 8) | map) << assign * 16), &regs->assign);

	pint->gpio_irq_base[assign] = irq_base;

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
	if (unlikely(!res)) {
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
	if (unlikely(!res)) {
		dev_err(dev, "Invalid IRQ resource\n");
		return -ENODEV;
	}

	spin_lock_init(&pint->lock);

	pint->irq = res->start;
	pint->pint_map_port = adi_pint_map_port;
	platform_set_drvdata(pdev, pint);

	irq_set_chip_data(pint->irq, pint);
	irq_set_chained_handler(pint->irq, adi_demux_gpio_irq);

	list_add_tail(&pint->node, &adi_pint_list);

	return 0;
}

static int adi_gpio_init_int(struct platform_device *pdev,
	struct gpio_port *port)
{
	struct gpio_pint *pint = port->pint;
	int irq, ret;

	ret = pint->pint_map_port(port->pint, port->pint_assign,
		port->pint_map,	port->irq_base);
	if (ret)
		return ret;

	port->irq_base = irq_alloc_descs(port->irq_base, 0, port->width, 0);
	if (port->irq_base < 0) {
		dev_err(port->dev, "Couldn't allocate GPIO IRQ numbers\n");
		return -ENODEV;
	}

	for (irq = port->irq_base; irq < port->irq_base + port->width; irq++) {
		irq_set_chip_data(irq, port);
		irq_set_chip_and_handler(irq, &adi_gpio_irqchip,
					handle_level_irq);
	}

	return 0;
}

static int adi_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct adi_gpio_platform_data *pdata;
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
	if (unlikely(!res)) {
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
	if (unlikely(!res))
		port->irq_base = -1;
	else
		port->irq_base = res->start;

	port->width = pdata->port_width;
	port->dev = dev;
	port->regs = (struct gpio_port_t *)port->base;
	port->pint_assign = pdata->pint_assign;
	port->pint_map = pdata->pint_map;

	port->pint = find_gpio_pint(pdata->pint_id);
	if (port->pint) {
		ret = adi_gpio_init_int(pdev, port);
		if (ret)
			return ret;
	}

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
		port->chip.base		= pdata->port_pin_base;
	else
		port->chip.base		= gpio;
	port->chip.ngpio		= port->width;
	gpio = port->chip.base + port->width;

	ret = gpiochip_add(&port->chip);
	if (ret)
		return ret;

	list_add_tail(&port->node, &adi_gpio_list);

	return 0;
}

static struct platform_driver adi_gpio_driver = {
	.probe		= adi_gpio_probe,
	.driver		= {
		.name	= "adi-gpio",
	},
};

static struct platform_driver adi_gpio_pint_driver = {
	.probe		= adi_gpio_pint_probe,
	.driver		= {
		.name	= "adi-gpio-pint",
	},
};


static int __init adi_gpio_setup(void)
{
	int ret;

	ret = platform_driver_register(&adi_gpio_pint_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&adi_gpio_driver);
	if (ret) {
		platform_driver_unregister(&adi_gpio_pint_driver);
		return ret;
	}

#ifdef CONFIG_PM
	register_syscore_ops(&gpio_pm_syscore_ops);
#endif
	return ret;
}
postcore_initcall(adi_gpio_setup);

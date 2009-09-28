/*
 * GPIOLIB interface for BF538/9 PORT C,D and E GPIOs
 *
 * Copyright 2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <asm/blackfin.h>
#include <asm/gpio.h>
#include <asm/portmux.h>

#define DEFINE_REG(reg, off) \
static inline u16 read_##reg(void __iomem *port) \
	{ return bfin_read16(port + off); } \
static inline void write_##reg(void __iomem *port, u16 v) \
	{ bfin_write16(port + off, v); }

DEFINE_REG(PORTIO, 0x00)
DEFINE_REG(PORTIO_CLEAR, 0x10)
DEFINE_REG(PORTIO_SET, 0x20)
DEFINE_REG(PORTIO_DIR, 0x40)
DEFINE_REG(PORTIO_INEN, 0x50)

int bf538_gpio_get_value(void __iomem *port, unsigned gpio)
{
	return !!(read_PORTIO(port) & (1u << gpio));
}

void bf538_gpio_set_value(void __iomem *port, unsigned gpio, int value)
{
	if (value)
		write_PORTIO_SET(port, (1u << gpio));
	else
		write_PORTIO_CLEAR(port, (1u << gpio));
}

int bf538_gpio_direction_input(void __iomem *port, unsigned gpio)
{
	write_PORTIO_DIR(port, read_PORTIO_DIR(port) & ~(1u << gpio));
	write_PORTIO_INEN(port, read_PORTIO_INEN(port) | (1u << gpio));

	return 0;
}

int bf538_gpio_direction_output(void __iomem *port, unsigned gpio, int value)
{
	write_PORTIO_INEN(port, read_PORTIO_INEN(port) & ~(1u << gpio));
	bf538_gpio_set_value(port, gpio, value);
	write_PORTIO_DIR(port, read_PORTIO_DIR(port) | (1u << gpio));

	return 0;
}

/*
 * PORTCIO
 */

int bf538_extgpio_portc_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	return bf538_gpio_direction_input((void __iomem *)PORTCIO, gpio);
}

int bf538_extgpio_portc_direction_output(struct gpio_chip *chip, unsigned gpio,
					 int level)
{
	return bf538_gpio_direction_output((void __iomem *)PORTCIO, gpio,
					   level);
}

int bf538_extgpio_portc_get_value(struct gpio_chip *chip, unsigned gpio)
{
	return bf538_gpio_get_value((void __iomem *)PORTCIO, gpio);
}

void bf538_extgpio_portc_set_value(struct gpio_chip *chip, unsigned gpio,
				   int value)
{
	return bf538_gpio_set_value((void __iomem *)PORTCIO, gpio, value);
}

/*
 * PORTDIO
 */

int bf538_extgpio_portd_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	return bf538_gpio_direction_input((void __iomem *)PORTDIO, gpio);
}

int bf538_extgpio_portd_direction_output(struct gpio_chip *chip, unsigned gpio,
					 int level)
{
	return bf538_gpio_direction_output((void __iomem *)PORTDIO, gpio,
					   level);
}

int bf538_extgpio_portd_get_value(struct gpio_chip *chip, unsigned gpio)
{
	return bf538_gpio_get_value((void __iomem *)PORTDIO, gpio);
}

void bf538_extgpio_portd_set_value(struct gpio_chip *chip, unsigned gpio,
				   int value)
{
	return bf538_gpio_set_value((void __iomem *)PORTDIO, gpio, value);
}

/*
 * PORTEIO
 */

int bf538_extgpio_porte_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	return bf538_gpio_direction_input((void __iomem *)PORTEIO, gpio);
}

int bf538_extgpio_porte_direction_output(struct gpio_chip *chip, unsigned gpio,
					 int level)
{
	return bf538_gpio_direction_output((void __iomem *)PORTEIO, gpio,
					   level);
}

int bf538_extgpio_porte_get_value(struct gpio_chip *chip, unsigned gpio)
{
	return bf538_gpio_get_value((void __iomem *)PORTEIO, gpio);
}

void bf538_extgpio_porte_set_value(struct gpio_chip *chip, unsigned gpio,
				   int value)
{
	return bf538_gpio_set_value((void __iomem *)PORTEIO, gpio, value);
}

int bf538_extgpio_gpio_request(struct gpio_chip *chip, unsigned gpio)
{
	return bfin_special_gpio_request(chip->base + gpio, chip->label);
}

void bf538_extgpio_gpio_free(struct gpio_chip *chip, unsigned gpio)
{
	return bfin_special_gpio_free(chip->base + gpio);
}

static struct gpio_chip bf538_portc_chip = {
	.label = "GPIO-PC",
	.direction_input = bf538_extgpio_portc_direction_input,
	.get = bf538_extgpio_portc_get_value,
	.direction_output = bf538_extgpio_portc_direction_output,
	.set = bf538_extgpio_portc_set_value,
	.request = bf538_extgpio_gpio_request,
	.free = bf538_extgpio_gpio_free,
	.base = GPIO_PC0,
	.ngpio = GPIO_PC9 - GPIO_PC0 + 1,
};

static struct gpio_chip bf538_portd_chip = {
	.label = "GPIO-PD",
	.direction_input = bf538_extgpio_portd_direction_input,
	.get = bf538_extgpio_portd_get_value,
	.direction_output = bf538_extgpio_portd_direction_output,
	.set = bf538_extgpio_portd_set_value,
	.request = bf538_extgpio_gpio_request,
	.free = bf538_extgpio_gpio_free,
	.base = GPIO_PD0,
	.ngpio = GPIO_PD13 - GPIO_PD0 + 1,
};

static struct gpio_chip bf538_porte_chip = {
	.label = "GPIO-PE",
	.direction_input = bf538_extgpio_porte_direction_input,
	.get = bf538_extgpio_porte_get_value,
	.direction_output = bf538_extgpio_porte_direction_output,
	.set = bf538_extgpio_porte_set_value,
	.request = bf538_extgpio_gpio_request,
	.free = bf538_extgpio_gpio_free,
	.base = GPIO_PE0,
	.ngpio = GPIO_PE15 - GPIO_PE0 + 1,
};

static int __init bf538_extgpio_setup(void)
{
	gpiochip_add(&bf538_portc_chip);
	gpiochip_add(&bf538_portd_chip);
	gpiochip_add(&bf538_porte_chip);

	return 0;
}

arch_initcall(bf538_extgpio_setup);

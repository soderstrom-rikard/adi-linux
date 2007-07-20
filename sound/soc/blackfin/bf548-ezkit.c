/*
 * bf548_ezkit.c  --  SoC audio for bf548 ezkit
 *
 * Copyright 2007 Analog Device Inc.
 *
 * Authors: Roy Huang <roy.huang@analog.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    8th June 2007   Initial version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <asm/dma.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include <asm/gpio.h>
#include "../codecs/ad1980.h"
#include "bf5xx-sport.h"
#include "bf5xx-pcm.h"
#include "bf5xx-ac97.h"

static int	sport_num = CONFIG_SND_BF5XX_SPORT_NUM;

static struct sport_param sport_params[4] = {
	{
		.dma_rx_chan	= CH_SPORT0_RX,
		.dma_tx_chan	= CH_SPORT0_TX,
		.err_irq	= IRQ_SPORT0_ERR,
		.regs		= (struct sport_register*)SPORT0_TCR1,
	},
	{
		.dma_rx_chan	= CH_SPORT1_RX,
		.dma_tx_chan	= CH_SPORT1_TX,
		.err_irq	= IRQ_SPORT1_ERR,
		.regs		= (struct sport_register*)SPORT1_TCR1,
	},
	{
		.dma_rx_chan	= CH_SPORT2_RX,
		.dma_tx_chan	= CH_SPORT2_TX,
		.err_irq	= IRQ_SPORT2_ERR,
		.regs		= (struct sport_register*)SPORT2_TCR1,
	},
	{
		.dma_rx_chan	= CH_SPORT3_RX,
		.dma_tx_chan	= CH_SPORT3_TX,
		.err_irq	= IRQ_SPORT3_ERR,
		.regs		= (struct sport_register*)SPORT3_TCR1,
	}
};

struct sport_device *sport_handle;

static struct snd_soc_machine bf548_ezkit;

static int bf548_ezkit_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_soc_ops bf548_ezkit_ops = {
	.startup = bf548_ezkit_startup,
};

static int bf548_ezkit_ac97_init(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_dai_link bf548_ezkit_dai = {
	.name = "AC97",
	.stream_name = "AC97 HiFi",
	.cpu_dai = &bfin_ac97_dai,
	.codec_dai = &ad1980_dai,
	.init = bf548_ezkit_ac97_init,
	.ops = &bf548_ezkit_ops,
};

static int bf548_probe(struct platform_device *pdev)
{
	/* Set pin 2, 3, 4, 6, 7 of PORT C to function 00 */
	if (sport_num == 0) {
		bfin_write_PORTC_FER(bfin_read_PORTC_FER() | 0xDC);
		bfin_write_PORTC_MUX(bfin_read_PORTC_MUX() & ~(0xF3F0));
	} else {
		printk(KERN_ERR "SPORT %x not support currently\n", sport_num);
		return -ENODEV;
	}
	/* Request PB3 as reset pin */
	if (gpio_request(GPIO_PB3, NULL)) {
		printk(KERN_ERR "Failed to request PB3 for reset\n");
		return -1;
	}
	gpio_direction_output(GPIO_PB3);
	gpio_set_value(GPIO_PB3, 1);

	sport_handle = sport_init(&sport_params[sport_num], NULL);
	if (!sport_handle)
		return -ENODEV;

	sport_set_multichannel(sport_handle, 16, 0x1F, 1);
	sport_config_rx(sport_handle, IRFS, 0xF, 0, (16*16-1));
	sport_config_tx(sport_handle, ITFS, 0xF, 0, (16*16-1));

	return 0;
}

static struct snd_soc_machine bf548_ezkit = {
	.name = "bf548-ezkit",
	.probe = bf548_probe,
	.dai_link = &bf548_ezkit_dai,
	.num_links = 1,
};

static struct snd_soc_device bf548_ezkit_snd_devdata = {
	.machine = &bf548_ezkit,
	.platform = &bf5xx_soc_platform,
	.codec_dev = &soc_codec_dev_ad1980,
};

static struct platform_device *bf548_ezkit_snd_device;

static int __init bf548_ezkit_init(void)
{
	int ret;

	bf548_ezkit_snd_device = platform_device_alloc("soc-audio", -1);
	if (!bf548_ezkit_snd_device)
		return -ENOMEM;

	platform_set_drvdata(bf548_ezkit_snd_device, &bf548_ezkit_snd_devdata);
	bf548_ezkit_snd_devdata.dev = &bf548_ezkit_snd_device->dev;
	ret = platform_device_add(bf548_ezkit_snd_device);

	if (ret)
		platform_device_put(bf548_ezkit_snd_device);

	return ret;
}

static void __exit bf548_ezkit_exit(void)
{
	platform_device_unregister(bf548_ezkit_snd_device);
}

module_init(bf548_ezkit_init);
module_exit(bf548_ezkit_exit);

/* Module information */
MODULE_AUTHOR("Roy Huang");
MODULE_DESCRIPTION("ALSA SoC BF548-EZKIT");
MODULE_LICENSE("GPL");

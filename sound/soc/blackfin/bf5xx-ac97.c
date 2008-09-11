/*
 * bf5xx-ac97.c -- AC97 support for the ADI blackfin chip.
 *
 * Author:	Roy Huang
 * Created:	11th. June 2007
 * Copyright:	Analog Device Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <asm/irq.h>
#include <asm/portmux.h>
#include <linux/mutex.h>
#include <linux/gpio.h>

#include "bf5xx-sport.h"
#include "bf5xx-ac97.h"

#if defined(CONFIG_BF54x)
#define PIN_REQ_SPORT_0 {P_SPORT0_TFS, P_SPORT0_DTPRI, P_SPORT0_TSCLK, \
		P_SPORT0_RFS, P_SPORT0_DRPRI, P_SPORT0_RSCLK, 0}

#define PIN_REQ_SPORT_1 {P_SPORT1_TFS, P_SPORT1_DTPRI, P_SPORT1_TSCLK, \
		P_SPORT1_RFS, P_SPORT1_DRPRI, P_SPORT1_RSCLK, 0}

#define PIN_REQ_SPORT_2 {P_SPORT2_TFS, P_SPORT2_DTPRI, P_SPORT2_TSCLK, \
		P_SPORT2_RFS, P_SPORT2_DRPRI, P_SPORT2_RSCLK, 0}

#define PIN_REQ_SPORT_3 {P_SPORT3_TFS, P_SPORT3_DTPRI, P_SPORT3_TSCLK, \
		P_SPORT3_RFS, P_SPORT3_DRPRI, P_SPORT3_RSCLK, 0}
#else
#define PIN_REQ_SPORT_0 {P_SPORT0_DTPRI, P_SPORT0_TSCLK, P_SPORT0_RFS, \
		 P_SPORT0_DRPRI, P_SPORT0_RSCLK, 0}

#define PIN_REQ_SPORT_1 {P_SPORT1_DTPRI, P_SPORT1_TSCLK, P_SPORT1_RFS, \
		 P_SPORT1_DRPRI, P_SPORT1_RSCLK, 0}
#endif

static int *cmd_count;
static int sport_num = CONFIG_SND_BF5XX_SPORT_NUM;

static struct sport_param sport_params[4] = {
	{
		.dma_rx_chan	= CH_SPORT0_RX,
		.dma_tx_chan	= CH_SPORT0_TX,
		.err_irq	= IRQ_SPORT0_ERR,
		.regs		= (struct sport_register *)SPORT0_TCR1,
	},
#ifdef PIN_REQ_SPORT_1
	{
		.dma_rx_chan	= CH_SPORT1_RX,
		.dma_tx_chan	= CH_SPORT1_TX,
		.err_irq	= IRQ_SPORT1_ERR,
		.regs		= (struct sport_register *)SPORT1_TCR1,
	},
#endif
#ifdef PIN_REQ_SPORT_2
	{
		.dma_rx_chan	= CH_SPORT2_RX,
		.dma_tx_chan	= CH_SPORT2_TX,
		.err_irq	= IRQ_SPORT2_ERR,
		.regs		= (struct sport_register *)SPORT2_TCR1,
	},
#endif
#ifdef PIN_REQ_SPORT_1
	{
		.dma_rx_chan	= CH_SPORT3_RX,
		.dma_tx_chan	= CH_SPORT3_TX,
		.err_irq	= IRQ_SPORT3_ERR,
		.regs		= (struct sport_register *)SPORT3_TCR1,
	}
#endif
};

void bf5xx_pcm_to_ac97(struct ac97_frame *dst, const __u32 *src, \
		size_t count)
{
	while (count--) {
		dst->ac97_tag = TAG_VALID | TAG_PCM;
		(dst++)->ac97_pcm = *src++;
	}
}
EXPORT_SYMBOL(bf5xx_pcm_to_ac97);

void bf5xx_ac97_to_pcm(const struct ac97_frame *src, __u32 *dst, \
		size_t count)
{
	while (count--)
		*(dst++) = (src++)->ac97_pcm;
}
EXPORT_SYMBOL(bf5xx_ac97_to_pcm);

static unsigned int sport_tx_curr_frag(struct sport_device *sport)
{
	return sport->tx_curr_frag = sport_curr_offset_tx(sport) / \
			sport->tx_fragsize;
}

static void enqueue_cmd(struct snd_ac97 *ac97, __u16 addr, __u16 data)
{
	struct sport_device *sport = sport_handle;
	int nextfrag = sport_tx_curr_frag(sport);
	struct ac97_frame *nextwrite;

	incfrag(sport, &nextfrag, 1);
	incfrag(sport, &nextfrag, 1);

	nextwrite = (struct ac97_frame *)(sport->tx_buf + \
			nextfrag * sport->tx_fragsize);
	pr_debug("sport->tx_buf:%p, nextfrag:0x%x nextwrite:%p, cmd_count:%d\n",
		sport->tx_buf, nextfrag, nextwrite, cmd_count[nextfrag]);
	nextwrite[cmd_count[nextfrag]].ac97_tag |= TAG_CMD;
	nextwrite[cmd_count[nextfrag]].ac97_addr = addr;
	nextwrite[cmd_count[nextfrag]].ac97_data = data;
	++cmd_count[nextfrag];
	pr_debug("ac97_sport: Inserting %02x/%04x into fragment %d\n",
			addr >> 8, data, nextfrag);
}

static unsigned short bf5xx_ac97_read(struct snd_ac97 *ac97,
	unsigned short reg)
{
	struct ac97_frame out_frame[2], in_frame[2];

	pr_debug("%s enter 0x%x\n", __func__, reg);

	/* When dma descriptor is enabled, the register should not be read */
	if (sport_handle->tx_run || sport_handle->rx_run) {
		pr_err("Could you send a mail to author "
				"to report this?\n");
		return -EFAULT;
	}

	memset(&out_frame, 0, 2 * sizeof(struct ac97_frame));
	memset(&in_frame, 0, 2 * sizeof(struct ac97_frame));
	out_frame[0].ac97_tag = TAG_VALID | TAG_CMD;
	out_frame[0].ac97_addr = ((reg << 8) | 0x8000);
	sport_send_and_recv(sport_handle, (unsigned char *)&out_frame,
			(unsigned char *)&in_frame,
			2 * sizeof(struct ac97_frame));
	return in_frame[1].ac97_data;
}

void bf5xx_ac97_write(struct snd_ac97 *ac97, unsigned short reg,
	unsigned short val)
{
	pr_debug("%s enter 0x%x:0x%04x\n", __func__, reg, val);

	if (sport_handle->tx_run) {
		enqueue_cmd(ac97, (reg << 8), val); /* write */
		enqueue_cmd(ac97, (reg << 8) | 0x8000, 0); /* read back */
	} else {
		struct ac97_frame frame;
		memset(&frame, 0, sizeof(struct ac97_frame));
		frame.ac97_tag = TAG_VALID | TAG_CMD;
		frame.ac97_addr = (reg << 8);
		frame.ac97_data = val;
		sport_send_and_recv(sport_handle, (unsigned char *)&frame, \
				NULL, sizeof(struct ac97_frame));
	}
}

static void bf5xx_ac97_warm_reset(struct snd_ac97 *ac97)
{
#if defined(CONFIG_BF54x) || defined(CONFIG_BF561) || \
 (defined(BF537_FAMILY) && (CONFIG_SND_BF5XX_SPORT_NUM == 1))

#define CONCAT(a, b, c) a ## b ## c
#define BFIN_SPORT_RFS(x) CONCAT(P_SPORT, x, _RFS)

	u16 per = BFIN_SPORT_RFS(CONFIG_SND_BF5XX_SPORT_NUM);
	u16 gpio = P_IDENT(BFIN_SPORT_RFS(CONFIG_SND_BF5XX_SPORT_NUM));

	pr_debug("%s enter\n", __func__);

	peripheral_free(per);
	gpio_request(gpio, "bf5xx-ac97");
	gpio_direction_output(gpio, 1);
	udelay(2);
	gpio_set_value(gpio, 0);
	udelay(1);
	gpio_free(gpio);
	peripheral_request(per, "soc-audio");
#else
	pr_info("%s: Not implemented\n", __func__);
#endif
}

static void bf5xx_ac97_cold_reset(struct snd_ac97 *ac97)
{
#ifdef CONFIG_SND_BF5XX_HAVE_COLD_RESET
	pr_debug("%s enter\n", __func__);

	/* It is specified for bf548-ezkit */
	gpio_set_value(CONFIG_SND_BF5XX_RESET_GPIO_NUM, 0);
	/* Keep reset pin low for 1 ms */
	mdelay(1);
	gpio_set_value(CONFIG_SND_BF5XX_RESET_GPIO_NUM, 1);
	/* Wait for bit clock recover */
	mdelay(1);
#endif
}

struct snd_ac97_bus_ops soc_ac97_ops = {
	.read	= bf5xx_ac97_read,
	.write	= bf5xx_ac97_write,
	.warm_reset	= bf5xx_ac97_warm_reset,
	.reset	= bf5xx_ac97_cold_reset,
};
EXPORT_SYMBOL_GPL(soc_ac97_ops);

#ifdef CONFIG_PM
static int bf5xx_ac97_suspend(struct platform_device *pdev,
	struct snd_soc_cpu_dai *dai)
{
	struct sport_device *sport =
		(struct sport_device *)dai->private_data;

	pr_debug("%s : sport %d\n", __func__, dai->id);
	if (!dai->active)
		return 0;
	if (dai->capture.active)
		sport_rx_stop(sport);
	if (dai->playback.active)
		sport_tx_stop(sport);
	return 0;
}

static int bf5xx_ac97_resume(struct platform_device *pdev,
	struct snd_soc_cpu_dai *dai)
{
	int ret;
	struct sport_device *sport =
		(struct sport_device *)dai->private_data;

	pr_debug("%s : sport %d\n", __func__, dai->id);
	if (!dai->active)
		return 0;
	ret = sport_set_multichannel(sport_handle, 16, 0x1F, 1);

	if (ret) {
		pr_err("SPORT is busy!\n");
		return -EBUSY;
	}
	ret = sport_config_rx(sport_handle, IRFS, 0xF, 0, (16*16-1));

	if (ret) {
		pr_err("SPORT is busy!\n");
		return -EBUSY;
	}
	ret = sport_config_tx(sport_handle, ITFS, 0xF, 0, (16*16-1));

	if (ret) {
		pr_err("SPORT is busy!\n");
		return -EBUSY;
	}

	if (dai->capture.active)
		sport_rx_start(sport);
	if (dai->playback.active)
		sport_tx_start(sport);
	return 0;
}

#else
#define bf5xx_ac97_suspend	NULL
#define bf5xx_ac97_resume	NULL
#endif

static struct proc_dir_entry *ac_entry;

/* For test purpose, read a register from codec */
static int proc_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	struct ac97_frame out_frame[2], in_frame[2];
	unsigned long reg = strict_strtoul(buffer, NULL, 16);

	memset(&out_frame, 0, 2 * sizeof(struct ac97_frame));
	out_frame[0].ac97_tag = TAG_VALID | TAG_CMD;
	out_frame[0].ac97_addr = (unsigned short) ((reg << 8) | 0x8000);
	sport_send_and_recv(sport_handle, (unsigned char *)&out_frame,
				(unsigned char *)&in_frame,
				2 * sizeof(struct ac97_frame));

	pr_info("0x%x:%04x\n", out_frame[0].ac97_addr, in_frame[1].ac97_data);

	return count;
}

static int bf5xx_ac97_probe(struct platform_device *pdev)
{
	int ret;
	u16 sport_req[][7] = {
		PIN_REQ_SPORT_0,
#ifdef PIN_REQ_SPORT_1
		PIN_REQ_SPORT_1,
#endif
#ifdef PIN_REQ_SPORT_2
		PIN_REQ_SPORT_2,
#endif
#ifdef PIN_REQ_SPORT_3
		PIN_REQ_SPORT_3,
#endif
	};
	cmd_count = (int *)get_zeroed_page(GFP_KERNEL);
	if (cmd_count == NULL)
		return -ENOMEM;

	if (peripheral_request_list(&sport_req[sport_num][0], "soc-audio")) {
		pr_err("Requesting Peripherals failed\n");
		return -EFAULT;
	}

#ifdef CONFIG_SND_BF5XX_HAVE_COLD_RESET
	/* Request PB3 as reset pin */
	if (gpio_request(CONFIG_SND_BF5XX_RESET_GPIO_NUM, "SND_AD198x RESET")) {
		pr_err("Failed to request GPIO_%d for reset\n",
				CONFIG_SND_BF5XX_RESET_GPIO_NUM);
		peripheral_free_list(&sport_req[sport_num][0]);
		return -1;
	}
	gpio_direction_output(CONFIG_SND_BF5XX_RESET_GPIO_NUM, 1);
#endif
	sport_handle = sport_init(&sport_params[sport_num], 2, \
			sizeof(struct ac97_frame), NULL);
	if (!sport_handle) {
		peripheral_free_list(&sport_req[sport_num][0]);
#ifdef CONFIG_SND_BF5XX_HAVE_COLD_RESET
		gpio_free(CONFIG_SND_BF5XX_RESET_GPIO_NUM);
#endif
		return -ENODEV;
	}
	/*SPORT works in TDM mode to simulate AC97 transfers*/
	ret = sport_set_multichannel(sport_handle, 16, 0x1F, 1);

	if (ret) {
		pr_err("SPORT is busy!\n");
		return -EBUSY;
	}
	ret = sport_config_rx(sport_handle, IRFS, 0xF, 0, (16*16-1));

	if (ret) {
		pr_err("SPORT is busy!\n");
		return -EBUSY;
	}
	ret = sport_config_tx(sport_handle, ITFS, 0xF, 0, (16*16-1));

	if (ret) {
		pr_err("SPORT is busy!\n");
		return -EBUSY;
	}
	ac_entry = create_proc_entry("driver/sport_ac97", 0600, NULL);
	ac_entry->read_proc = NULL;
	ac_entry->write_proc = proc_write;
	ac_entry->data = sport_handle;
	return 0;
}

static void bf5xx_ac97_remove(struct platform_device *pdev)
{
	free_page((unsigned long)cmd_count);
	cmd_count = NULL;
	remove_proc_entry("driver/sport_ac97", NULL);
#ifdef CONFIG_SND_BF5XX_HAVE_COLD_RESET
	gpio_free(CONFIG_SND_BF5XX_RESET_GPIO_NUM);
#endif
}

struct snd_soc_cpu_dai bfin_ac97_dai = {
	.name = "bf5xx-ac97",
	.id = 0,
	.type = SND_SOC_DAI_AC97,
	.probe = bf5xx_ac97_probe,
	.remove = bf5xx_ac97_remove,
	.suspend = bf5xx_ac97_suspend,
	.resume = bf5xx_ac97_resume,
	.playback = {
		.stream_name = "AC97 Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE, },
	.capture = {
		.stream_name = "AC97 Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE, },
};
EXPORT_SYMBOL_GPL(bfin_ac97_dai);

MODULE_AUTHOR("Roy Huang");
MODULE_DESCRIPTION("AC97 driver for ADI Blackfin");
MODULE_LICENSE("GPL");

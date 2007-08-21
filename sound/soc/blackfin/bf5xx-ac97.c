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

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <asm/irq.h>
#include <asm/gpio.h>
#include <asm/portmux.h>
#include <linux/mutex.h>

#include "bf5xx-sport.h"
#include "bf5xx-ac97.h"

static int *cmd_count;

void bf5xx_ac97_pcm32_to_frame(struct ac97_frame *dst, const __u32 *src, \
		size_t count)
{
	while (count--) {
		dst->ac97_tag = TAG_VALID | TAG_PCM;
		(dst++)->ac97_pcm = *src++;
	}
}

void bf5xx_ac97_frame_to_pcm32(const struct ac97_frame *src, __u32 *dst, \
		size_t count)
{
	while (count--) {
		*(dst++) = (src++)->ac97_pcm;
	}
}

static unsigned int sport_tx_curr_frag(struct sport_device *sport)
{
	return sport->tx_curr_frag = sport_curr_offset_tx(sport) / \
			(sizeof(struct ac97_frame) * sport->tx_fragsize);
}

static void enqueue_cmd(struct snd_ac97 *ac97, __u16 addr, __u16 data)
{
	struct sport_device *sport = sport_handle;
	int nextfrag = sport_tx_curr_frag(sport);
	struct ac97_frame *nextwrite;

	pr_debug("%s enter\n", __FUNCTION__);
	incfrag(sport, &nextfrag, 1);
	incfrag(sport, &nextfrag, 1);

	pr_debug("sport->tx_buf:%p, nextfrag:0x%x\n", sport->tx_buf, nextfrag);
	nextwrite = (struct ac97_frame *)(sport->tx_buf + \
			nextfrag * sport->tx_frags);
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

	pr_debug("%s enter 0x%x\n", __FUNCTION__, reg);

	/* When dma descriptor is enabled, the register should not be read */
	if (sport_handle->tx_run || sport_handle->rx_run) {
		printk(KERN_ERR "Could you send a mail to author "
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

static void bf5xx_ac97_write(struct snd_ac97 *ac97, unsigned short reg,
	unsigned short val)
{
	pr_debug("%s enter 0x%x:0x%04x\n", __FUNCTION__, reg, val);

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

	pr_debug("%s enter\n", __FUNCTION__);

	peripheral_free(per);
	gpio_request(gpio, NULL);
	gpio_direction_output(gpio);
	gpio_set_value(gpio, 1);
	udelay(2);
	gpio_set_value(gpio, 0);
	udelay(1);
	gpio_free(gpio);
	peripheral_request(per, "soc-audio");
#else
	printk(KERN_INFO"%s: Not implemented\n", __FUNCTION__);
#endif
}

static void bf5xx_ac97_cold_reset(struct snd_ac97 *ac97)
{
#ifdef CONFIG_SND_BF5XX_HAVE_COLD_RESET
	pr_debug("%s enter\n", __FUNCTION__);

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
	return 0;
}

static int bf5xx_ac97_resume(struct platform_device *pdev,
	struct snd_soc_cpu_dai *dai)
{
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
	unsigned long reg = simple_strtoul(buffer, NULL, 16);

	memset(&out_frame, 0, 2 * sizeof(struct ac97_frame));
	out_frame[0].ac97_tag = TAG_VALID | TAG_CMD;
	out_frame[0].ac97_addr = (unsigned short) ((reg << 8) | 0x8000);
	sport_send_and_recv(sport_handle, (unsigned char *)&out_frame,
				(unsigned char *)&in_frame,
				2 * sizeof(struct ac97_frame));
	printk(KERN_INFO"0x%x:%04x\n", out_frame[0].ac97_addr, \
			in_frame[1].ac97_data);

	return count;
}

static int bf5xx_ac97_probe(struct platform_device *pdev)
{
	cmd_count = kmalloc(sport_handle->tx_frags * sizeof(int), GFP_KERNEL);
	if (cmd_count == NULL)
		return -ENOMEM;

	ac_entry = create_proc_entry("driver/sport_ac97", 0600, NULL);
	ac_entry->read_proc = NULL;
	ac_entry->write_proc = proc_write;
	ac_entry->data = sport_handle;
	return 0;
}

static void bf5xx_ac97_remove(struct platform_device *pdev)
{
	kfree(cmd_count);
	cmd_count = NULL;
	remove_proc_entry("driver/sport_ac97", NULL);
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

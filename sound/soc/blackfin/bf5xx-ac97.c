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

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <asm/irq.h>
#include <linux/mutex.h>

#include "bf5xx-sport.h"
#include "bf5xx-ac97.h"

static struct sport_ac97 ac97_dev;

#define reg_set_clean(reg)	(ac97_dev.register_dirty[(reg) >> 4] &= \
		~(1 << ((reg) & 0xf)))
#define reg_set_dirty(reg)	(ac97_dev.register_dirty[(reg) >> 4] |= \
		(1 << ((reg) & 0xf)))
#define reg_is_dirty(reg)	(ac97_dev.register_dirty[(reg) >> 4] & \
		(1 << ((reg) & 0xf)))
#define reg_any_dirty()		(ac97_dev.register_dirty[0] || \
		ac97_dev.register_dirty[1] || ac97_dev.register_dirty[2] || \
		ac97_dev.register_dirty[3] || ac97_dev.register_dirty[4] || \
		ac97_dev.register_dirty[5] || ac97_dev.register_dirty[6] || \
		ac97_dev.register_dirty[7])

void bf5xx_ac97_pcm16_to_frame(struct ac97_frame *dst, const __u16 *src, \
		size_t count)
{
	while (count--) {
		(dst++)->ac97_pcm = (*src << 16) | *src;
	}
}

void bf5xx_ac97_frame_to_pcm16(const struct ac97_frame *src, __u16 *dst, \
		size_t count)
{
	while (count--) {
		/* Left channel */
		*(dst++) = (unsigned short)((src)->ac97_pcm & 0xFFFF);
		/* Right channel */
		*(dst++) = (unsigned short)(((src)->ac97_pcm >> 16) & 0xFFFF);
	}
}
static unsigned int sport_tx_curr_frag(struct sport_device *sport)
{
	return sport->tx_curr_frag = (sport->dma_tx->curr_addr_ptr - \
			(unsigned long)sport->tx_buf) / \
			(sizeof(struct ac97_frame) * sport->tx_fragsize);
}

static unsigned int sport_rx_curr_frag(struct sport_device *sport)
{
	return sport->rx_curr_frag = (sport->dma_rx->curr_addr_ptr - \
			(unsigned long)sport->rx_buf) / \
			(sizeof(struct ac97_frame) * sport->rx_fragsize);
}


static void enqueue_cmd(struct snd_ac97 *ac97, __u16 addr, __u16 data)
{
	struct sport_device *sport = ac97->private_data;
	int nextfrag = sport_tx_curr_frag(sport);
	struct ac97_frame *nextwrite;

	incfrag(sport, &nextfrag, 1);
	incfrag(sport, &nextfrag, 1);

	nextwrite = (struct ac97_frame *)(sport->tx_buf + \
			nextfrag * sport->tx_frags);
	nextwrite[ac97_dev.cmd_count[nextfrag]].ac97_addr = addr;
	nextwrite[ac97_dev.cmd_count[nextfrag]].ac97_data = data;
	nextwrite[ac97_dev.cmd_count[nextfrag]].ac97_tag |= TAG_CMD;
	++ac97_dev.cmd_count[nextfrag];
	pr_debug("ac97_sport: Inserting %02x/%04x into fragment %d\n",
	       addr >> 8, data, nextfrag);
}

static unsigned short bf5xx_ac97_read(struct snd_ac97 *ac97,
	unsigned short reg)
{
	if ((reg > 127) || (reg & 0x1))
		return -EINVAL;

	if (reg_is_dirty(reg))
		return -EAGAIN;

	return ac97->regs[reg];
}

static void bf5xx_ac97_write(struct snd_ac97 *ac97, unsigned short reg,
	unsigned short val)
{
	if ((reg > 127) || (reg & 0x1)) {
		printk(KERN_ERR "Register out of range\n");
		return ;
	}

	enqueue_cmd(ac97, reg << 8, val); /* write */
	enqueue_cmd(ac97, (reg << 8) | 0x8000, 0); /* read back */

	reg_set_dirty(reg);
}

static void bf5xx_ac97_warm_reset(struct snd_ac97 *ac97)
{
}

static void bf5xx_ac97_cold_reset(struct snd_ac97 *ac97)
{
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

static int bf5xx_ac97_probe(struct platform_device *pdev)
{
	ac97_dev.cmd_count = kmalloc(sport_handle->tx_frags * sizeof(int), \
			GFP_KERNEL);
	if (ac97_dev.cmd_count == NULL)
		return -ENOMEM;

	return 0;
}

static void bf5xx_ac97_remove(struct platform_device *pdev)
{
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
		.channels_min = 1,
		.channels_max = 6,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.formats = SNDRV_PCM_FMTBIT_S16_LE, },
	.capture = {
		.stream_name = "AC97 Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.formats = SNDRV_PCM_FMTBIT_S16_LE, },
};
EXPORT_SYMBOL_GPL(bfin_ac97_dai);

MODULE_AUTHOR("Roy Huang");
MODULE_DESCRIPTION("AC97 driver for the ADI Blackfin chip");
MODULE_LICENSE("GPL");

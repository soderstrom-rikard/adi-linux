/*
 * File:         sound/blackfin/ad73322.c
 * Based on:	 sound/blackfin/ad73311.c
 * Author:       Cliff Cai <Cliff.Cai@analog.com>
 *
 * Created:      Tue May 06 2008
 * Description:  Driver for AD73322 sound chip connected to bf53x sport
 *
 * Rev:          $Id: ad73322.c 4104 2008-05-06 06:51:48Z cliff $
 *
 * Modified:
 *               Copyright 2008 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/* 
 *This driver supports up to 4 AD73322 connected in cascade mode. 
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <asm/blackfin.h>
#include <asm/cacheflush.h>
#include <asm/irq.h>
#include <asm/gpio.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/info.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#include "ad73311.h"
#include "bf53x_sport.h"

#ifndef CONFIG_BFIN_DMA_5XX
#error "The sound driver requires the Blackfin Simple DMA"
#endif

#ifdef CONFIG_SND_DEBUG
#define snd_printk_marker() snd_printk(KERN_INFO "%s\n", __FUNCTION__)
#else
#define snd_printk_marker()
#endif

#if CONFIG_SND_BFIN_SPORT == 0
#define SPORT_IRQ_ERR	IRQ_SPORT0_ERROR
#define SPORT_DMA_RX	CH_SPORT0_RX
#define SPORT_DMA_TX	CH_SPORT0_TX
#define bfin_write_SPORT_TCR1	bfin_write_SPORT0_TCR1
#define bfin_read_SPORT_TCR1	bfin_read_SPORT0_TCR1
#define bfin_write_SPORT_TCR2	bfin_write_SPORT0_TCR2
#define bfin_write_SPORT_TX16	bfin_write_SPORT0_TX16
#define bfin_read_SPORT_STAT	bfin_read_SPORT0_STAT
#elif CONFIG_SND_BFIN_SPORT == 1
#define SPORT_IRQ_ERR	IRQ_SPORT1_ERROR
#define SPORT_DMA_RX	CH_SPORT1_RX
#define SPORT_DMA_TX	CH_SPORT1_TX
#define bfin_write_SPORT_TCR1	bfin_write_SPORT1_TCR1
#define bfin_read_SPORT_TCR1	bfin_read_SPORT1_TCR1
#define bfin_write_SPORT_TCR2	bfin_write_SPORT1_TCR2
#define bfin_write_SPORT_TX16	bfin_write_SPORT1_TX16
#define bfin_read_SPORT_STAT	bfin_read_SPORT1_STAT
#elif CONFIG_SND_BFIN_SPORT == 2
#define HAVE_TWO_CARDS
#endif

#define GPIO_SE CONFIG_SND_BFIN_AD73322_SE
#define GPIO_RESET CONFIG_SND_BFIN_AD73322_RESET
/*8 means 4 AD73322 is connected in cascade mode,since every AD73322 has 2 
 *DAC/ADC pairs
 */
#define NUM_DEVICES_CHAIN 8

struct cascade_frame {

	short channel[NUM_DEVICES_CHAIN];
};

#undef CONFIG_SND_DEBUG_CURRPTR  /* causes output every frame! */

#define PCM_BUFFER_MAX	0x10000	/* 64KB */
#define FRAGMENT_SIZE_MIN	32
#define FRAGMENTS_MIN	2
#define FRAGMENTS_MAX	16
#define WORD_LENGTH	2

#define DRIVER_NAME "AD73322"
#define CHIP_NAME "Analog Devices AD73322"
static char *pcm_name[8] = { "AD73322PCM0", "AD73322PCM1", "AD73322PCM2", "AD73322PCM3", "AD73322PCM4", "AD73322PCM5", "AD73322PCM6", "AD73322PCM7"};

static unsigned int input_gain = 0x2;
module_param(input_gain, uint, 0);
MODULE_PARM_DESC(input_gain, "Input gain setting (0 <= input_gain <= 7)");

static unsigned int output_gain = 0x2;
module_param(output_gain, uint, 0);
MODULE_PARM_DESC(output_gain, "Output gain setting (0 <= output_gain <= 7)");

#ifdef HAVE_TWO_CARDS
static struct platform_device *device[2];
#else
static struct platform_device *device;
#endif

typedef struct snd_ad73322 {
	struct snd_card	*card;
	struct bf53x_sport	*sport;
	spinlock_t    ad73322_lock;
	struct snd_pcm	*pcm[NUM_DEVICES_CHAIN];
	struct snd_pcm_substream* rx_substream;
	struct snd_pcm_substream* tx_substream;
	int runmode;
#define RUN_RX 0x1
#define RUN_TX 0x2
} ad73322_t;

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for the AD73322 soundcard.");

static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for the AD73322 soundcard.");

static int enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable AD73322 soundcard.");


static int snd_ad73322_startup(void);
static void snd_ad73322_stop(void);

void bf5xx_pcm16_to_frame(struct cascade_frame *dst, const __u16 *src, size_t count, unsigned int dev){
	int channel_idx = NUM_DEVICES_CHAIN-(dev+1);
	while (count--)
		(dst++)->channel[channel_idx] = *src++;
}

void bf5xx_frame_to_pcm16(const struct cascade_frame *src, __u16 *dst,size_t count, unsigned int dev){
#if 0
	/*Should get channel_idx in this way*/
	int channel_idx = NUM_DEVICES_CHAIN-(dev+1);
#endif	
	/*for Valcom customer board only,since the Jacks for capture
	aren't mapped in ADCs' order*/
	int channel_idx = 0;	
	switch(dev) {
	case 0:channel_idx = 6; 
		break;
	case 1:channel_idx = 7; 
		break;
	case 2:channel_idx = 4; 
		break;
	case 3:channel_idx = 5; 
		break;
	case 4:channel_idx = 2; 
		break;
	case 5:channel_idx = 3; 
		break;
	case 6:channel_idx = 0; 
		break;
	case 7:channel_idx = 1; 
		break;
	}
	while (count--)
		*dst++ = (src++)->channel[channel_idx];
}

/*************************************************************
 *                pcm methods
 *************************************************************/

static struct snd_pcm_hardware snd_ad73322_play_hw = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_RESUME),
	.formats =          SNDRV_PCM_FMTBIT_S16_LE,
	.rates =            SNDRV_PCM_RATE_8000,
	.rate_min =         8000,
	.rate_max =         8000,
	.channels_min =     1,
	.channels_max =     1,
	.buffer_bytes_max = PCM_BUFFER_MAX,
	.period_bytes_min = FRAGMENT_SIZE_MIN,
	.period_bytes_max = PCM_BUFFER_MAX/2,
	.periods_min =      FRAGMENTS_MIN,
	.periods_max =      FRAGMENTS_MAX,
};
static struct snd_pcm_hardware snd_ad73322_cap_hw = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_RESUME),
	.formats =          SNDRV_PCM_FMTBIT_S16_LE,
	.rates =            SNDRV_PCM_RATE_8000,
	.rate_min =         8000,
	.rate_max =         8000,
	.channels_min =     1,
	.channels_max =     1,
	.buffer_bytes_max = PCM_BUFFER_MAX,
	.period_bytes_min = FRAGMENT_SIZE_MIN,
	.period_bytes_max = PCM_BUFFER_MAX/2,
	.periods_min =      FRAGMENTS_MIN,
	.periods_max =      FRAGMENTS_MAX,
};

static int snd_ad73322_play_open(struct snd_pcm_substream *substream)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	snd_printk_marker();
	substream->runtime->hw = snd_ad73322_play_hw;
	chip->tx_substream = substream;
	return 0;
}

static int snd_ad73322_cap_open(struct snd_pcm_substream *substream)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	snd_printk_marker();
	substream->runtime->hw = snd_ad73322_cap_hw;
	chip->rx_substream = substream;
	return 0;
}

static int snd_ad73322_play_close(struct snd_pcm_substream *substream)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	snd_printk_marker();
	chip->tx_substream = NULL;
	return 0;
}

static int snd_ad73322_cap_close(struct snd_pcm_substream *substream)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	snd_printk_marker();
	chip->rx_substream = NULL;
	return 0;
}

static int snd_ad73322_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params* hwparams)
{
	snd_printk_marker();
	if (snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hwparams)) < 0)
		return -ENOMEM;
	return 0;
}

static int snd_ad73322_hw_free(struct snd_pcm_substream *substream)
{
	snd_printk_marker();
	snd_pcm_lib_free_pages(substream);
	return 0;
}

static int snd_ad73322_play_pre(struct snd_pcm_substream *substream)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_assert((substream == chip->tx_substream), return -EINVAL);
	snd_printk_marker();
	snd_printd(KERN_INFO "%s channels:%d, period_bytes:0x%x, periods:%d\n",
				__FUNCTION__, runtime->channels, period_bytes,
							runtime->periods);
	return bf53x_sport_config_tx_dma(chip->sport, runtime->dma_area,\
			runtime->periods, runtime->period_size * sizeof(struct cascade_frame),\
				WORD_LENGTH);
}

static int snd_ad73322_cap_pre(struct snd_pcm_substream *substream)
{

	ad73322_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_printk_marker();
	snd_assert((substream == chip->rx_substream), return -EINVAL);
	snd_printd(KERN_INFO "%s channels:%d, period_bytes:%d, frag_count:%d\n",
				__FUNCTION__, runtime->channels, period_bytes,
							runtime->periods);
	return bf53x_sport_config_rx_dma(chip->sport, runtime->dma_area,\
			runtime->periods, runtime->period_size * sizeof(struct cascade_frame), \
				WORD_LENGTH);
}

static int snd_ad73322_play_trigger(struct snd_pcm_substream *substream, int cmd)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	spin_lock(&chip->ad73322_lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		bf53x_sport_tx_start(chip->sport);
		if (!(chip->runmode & RUN_RX))
			snd_ad73322_startup();
			chip->runmode |= RUN_TX;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		chip->runmode &= ~RUN_TX;
		bf53x_sport_tx_stop(chip->sport);
		if (!chip->runmode & RUN_RX)
			snd_ad73322_stop();
		break;
	default:
		spin_unlock(&chip->ad73322_lock);
		return -EINVAL;
	}
	spin_unlock(&chip->ad73322_lock);

	snd_printd(KERN_INFO"cmd:%s,runmode:0x%x\n", cmd?"start":"stop",
								chip->runmode);
	return 0;
}

static int snd_ad73322_cap_trigger(struct snd_pcm_substream *substream, int cmd)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);

	spin_lock(&chip->ad73322_lock);
	snd_assert(substream == chip->rx_substream, return -EINVAL);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		bf53x_sport_rx_start(chip->sport);
		if (!(chip->runmode & RUN_TX))
			snd_ad73322_startup();
		chip->runmode |= RUN_RX;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		chip->runmode &= ~RUN_RX;
		bf53x_sport_rx_stop(chip->sport);
		if (!(chip->runmode & RUN_TX))
			snd_ad73322_stop();
		break;
	default:
		spin_unlock(&chip->ad73322_lock);
		return -EINVAL;
	}
	spin_unlock(&chip->ad73322_lock);

//	printk(KERN_INFO"cmd:%s,runmode:0x%x\n", cmd?"start":"stop", chip->runmode);
	return 0;
}

static snd_pcm_uframes_t snd_ad73322_play_ptr(struct snd_pcm_substream *substream)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	unsigned long diff = bf53x_sport_curr_offset_tx(chip->sport);
	size_t frames = diff/sizeof(struct cascade_frame);
	if (frames >= substream->runtime->buffer_size)
		frames = 0;

	return frames;
}

static snd_pcm_uframes_t snd_ad73322_cap_ptr(struct snd_pcm_substream *substream)
{
	ad73322_t *chip = snd_pcm_substream_chip(substream);
	unsigned long diff = bf53x_sport_curr_offset_rx(chip->sport);
	size_t frames = diff/sizeof(struct cascade_frame);

#ifdef CONFIG_SND_DEBUG_CURRPTR
	snd_printk(KERN_INFO " cap pos: 0x%04lx / %lx\n", frames, runtime->buffer_size);
#endif
	/* the loose syncing used here is accurate enough for alsa, but
	   due to latency in the dma, the following may happen occasionally,
	   and pcm_lib shouldn't complain */
	if (frames >= substream->runtime->buffer_size)
		frames = 0;

	return frames;
}

static int snd_ad73322_play_copy(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos, void *src, snd_pcm_uframes_t count)
{
	unsigned int dev = substream->pcm->device;
	bf5xx_pcm16_to_frame((struct cascade_frame *)substream->runtime->dma_area+pos, src, count, dev);
	return 0;
}

static int snd_ad73322_cap_copy(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos, void *dst, snd_pcm_uframes_t count)
{
	unsigned int dev = substream->pcm->device;
	bf5xx_frame_to_pcm16((struct cascade_frame *)substream->runtime->dma_area+pos, dst, count, dev);
	return 0;
}

/* pcm method tables */

static struct snd_pcm_ops snd_ad73322_play_ops = {
	.open      = snd_ad73322_play_open,
	.close     = snd_ad73322_play_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.hw_params = snd_ad73322_hw_params,
	.hw_free   = snd_ad73322_hw_free,
	.prepare   = snd_ad73322_play_pre,
	.trigger   = snd_ad73322_play_trigger,
	.pointer   = snd_ad73322_play_ptr,
	.copy	   = snd_ad73322_play_copy,
};


static struct snd_pcm_ops snd_ad73322_cap_ops = {
	.open  = snd_ad73322_cap_open,
	.close = snd_ad73322_cap_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_ad73322_hw_params,
	.hw_free   = snd_ad73322_hw_free,
	.prepare   = snd_ad73322_cap_pre,
	.trigger   = snd_ad73322_cap_trigger,
	.pointer   = snd_ad73322_cap_ptr,
	.copy	   = snd_ad73322_cap_copy,
};

static void snd_ad73322_dma_rx(void *data)
{
	struct snd_ad73322 *chip = data;

	if ((chip->rx_substream) && (chip->runmode & RUN_RX))
		snd_pcm_period_elapsed(chip->rx_substream);
}

static void snd_ad73322_dma_tx(void *data)
{
	struct snd_ad73322 *chip = data;
	if ((chip->tx_substream) && (chip->runmode & RUN_TX)) {
		snd_pcm_period_elapsed(chip->tx_substream);
	}
}

static void snd_ad73322_sport_err(void *data)
{
	printk(KERN_ERR "%s: error happened on sport\n", __FUNCTION__);
}

/*************************************************************
 *      card and device
 *************************************************************/
static int snd_ad73322_startup(void)
{
	snd_printd(KERN_INFO "%s is called\n", __FUNCTION__);
	gpio_direction_output(GPIO_SE, 1);
	return 0;
}

static void snd_ad73322_stop(void)
{
	snd_printd(KERN_INFO "%s is called\n", __FUNCTION__);
	/* Pull down SE pin on AD73322 */
	gpio_direction_output(GPIO_SE, 0);
}

static void snd_ad73322_reset(void)
{
	int i;
	snd_printd(KERN_INFO "%s is called\n", __FUNCTION__);
	/* Pull down GPIO_RESET pin on AD73322 */
	gpio_direction_output(GPIO_RESET, 0);
	for (i = 0; i < 50000; i++);
	gpio_direction_output(GPIO_RESET, 1);
}

/*************************************************************
 *                 ALSA Card Level
 *************************************************************/
static int snd_ad73322_configure(int dev_id)
{
	short ctrl_regs[8];
	short dev_addr,reg_addr;
	int i,j;	
	unsigned short status = 0;
	short ctrl_buffer[NUM_DEVICES_CHAIN*8];
	short *pctrl_buffer;

	/*regs configuration */
	ctrl_regs[7] = (NUM_DEVICES_CHAIN-1)<<4 | MODE_DATA;
	ctrl_regs[0] = MCDIV(0) | SCDIV(0) | DIRATE(0);
	ctrl_regs[1] = PUDEV | PUADC | PUDAC | PUREF | REFUSE ;
	ctrl_regs[2] = 0;
	ctrl_regs[3] = 0;
	ctrl_regs[4] = 0;
	ctrl_regs[5] = 0;
	ctrl_regs[6] = 0;
	
	pctrl_buffer = &ctrl_buffer[0];
	reg_addr = 1;
	for (i = 0; i < 8; i++)
	{
		dev_addr = NUM_DEVICES_CHAIN - 1;
		for (j = 0; j < NUM_DEVICES_CHAIN; j++)
		{
			*pctrl_buffer++ = ctrl_regs[i] | (dev_addr<<11) | (reg_addr<<8) | AD_CONTROL;
			dev_addr--;
		}
	reg_addr++;
	if (reg_addr == 8)
		reg_addr = 0;

	}
	snd_ad73322_startup();
	snd_ad73322_reset();
	local_irq_disable();
	udelay(1);
#ifdef HAVE_TWO_CARDS
	bfin_write_SPORT0_TCR1(TFSR);
	bfin_write_SPORT0_TCR2(0xF);
	SSYNC();
	for (i = 0; i < 8; i++) {
		for (j = 0; j < NUM_DEVICES_CHAIN; j++)
			bfin_write_SPORT0_TX16(ctrl_buffer[8*i+j]);
		bfin_write_SPORT0_TCR1(bfin_read_SPORT0_TCR1() | TSPEN);
		status = bfin_read_SPORT0_STAT();
		while (!(status & TUVF)) {
			udelay(1);
			status = bfin_read_SPORT0_STAT();
			SSYNC();
		}
		bfin_write_SPORT0_TCR1(bfin_read_SPORT0_TCR1() & ~TSPEN);
	}
	SSYNC();
	bfin_write_SPORT1_TCR1(TFSR);
	bfin_write_SPORT1_TCR2(0xF);
	SSYNC();
	for (i = 0; i < 8; i++) {
		for (j = 0; j < NUM_DEVICES_CHAIN; j++)
			bfin_write_SPORT1_TX16(ctrl_buffer[8*i+j]);
		bfin_write_SPORT1_TCR1(bfin_read_SPORT1_TCR1() | TSPEN);
		status = bfin_read_SPORT1_STAT();
		while (!(status & TUVF)) {
			udelay(1);
			status = bfin_read_SPORT1_STAT();
			SSYNC();
		}
		bfin_write_SPORT1_TCR1(bfin_read_SPORT1_TCR1() & ~TSPEN);
	}
	SSYNC();
#else
	bfin_write_SPORT_TCR1(TFSR);
	bfin_write_SPORT_TCR2(0xF);
	SSYNC();	
	
	for (i=0; i<8; i++) {	
		for (j=0; j<NUM_DEVICES_CHAIN; j++)
			bfin_write_SPORT_TX16(ctrl_buffer[8*i+j]);
		bfin_write_SPORT_TCR1(bfin_read_SPORT_TCR1() | TSPEN);
		status = bfin_read_SPORT_STAT();
		while (!(status & TUVF)){
			udelay(1);
			status = bfin_read_SPORT_STAT();
			SSYNC();
		}
		bfin_write_SPORT_TCR1(bfin_read_SPORT_TCR1() & ~TSPEN);
	}	
	bfin_write_SPORT_TCR1(bfin_read_SPORT_TCR1() & ~TSPEN);
	SSYNC();
#endif
	local_irq_enable();
	snd_ad73322_stop();

	return 0;
}

static int __devinit snd_ad73322_pcm(struct snd_ad73322 *ad73322, int dev)
{
	int err = 0;
	struct snd_pcm *pcm;

	/* 1 playback and 1 capture substream */
	if ((err = snd_pcm_new(ad73322->card, pcm_name[dev], dev, 1, 1, &pcm))) {
		return err;
	}

	ad73322->pcm[dev] = pcm;
	strcpy(pcm->name, pcm_name[dev]);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_ad73322_play_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
			&snd_ad73322_cap_ops);

	pcm->private_data = ad73322;
	pcm->info_flags = 0;
	/* uncached DMA buffers */
	err = snd_pcm_lib_preallocate_pages_for_all(pcm,
				SNDRV_DMA_TYPE_DEV,NULL, PCM_BUFFER_MAX,
				PCM_BUFFER_MAX);
	if (err) {
		return -ENOMEM;
	}

	return 0;
}

static int __devinit snd_ad73322_probe(struct platform_device *pdev)
{
	int err;
	struct snd_card *card;
	struct snd_ad73322 *ad73322;
	struct bf53x_sport *sport = NULL;
	int i;
#if CONFIG_SND_BFIN_SPORT != 0
	unsigned short tmp_reg;
#endif
	/*
	if (device != NULL)
		return -ENOENT;
	*/
	if (pdev->id == 0) {
		if (gpio_request(GPIO_SE, "AD73322")) {
			printk(KERN_ERR "%s: Failed ro request GPIO_%d\n", __FUNCTION__, GPIO_SE);
			return -EBUSY;
		}

		if (gpio_request(GPIO_RESET, "AD73322RST")) {
			printk(KERN_ERR "%s: Failed ro request GPIO_12\n", __FUNCTION__);
			return -EBUSY;
		}
		gpio_direction_output(GPIO_SE, 0);
		gpio_direction_output(GPIO_RESET, 0);
#if CONFIG_SND_BFIN_SPORT != 0
		tmp_reg = bfin_read_PORT_MUX();
		bfin_write_PORT_MUX(tmp_reg|0x0E00);
		bfin_write_PORTG_FER(0xFFFF);
#endif
	}
	card = snd_card_new(index[pdev->id], id[pdev->id], THIS_MODULE, sizeof(struct snd_ad73322));
	if (card == NULL)
		return -ENOMEM;
	err = snd_ad73322_configure(pdev->id);
		if (err < 0)
			return -EFAULT;
	ad73322 = card->private_data;
	ad73322->card = card;
#ifdef HAVE_TWO_CARDS
	if (pdev->id == 0) {
		sport = bf53x_sport_init(0,
			CH_SPORT0_RX, snd_ad73322_dma_rx,
			CH_SPORT0_TX, snd_ad73322_dma_tx,
			IRQ_SPORT0_ERROR, snd_ad73322_sport_err, ad73322);
		if (sport == NULL) {
			err = -ENODEV;
			goto __sport_err;
		}
	} else if (pdev->id == 1) {
		sport = bf53x_sport_init(1,
			CH_SPORT1_RX, snd_ad73322_dma_rx,
			CH_SPORT1_TX, snd_ad73322_dma_tx,
			IRQ_SPORT1_ERROR, snd_ad73322_sport_err, ad73322);
		if (sport == NULL) {
			err = -ENODEV;
			goto __sport_err;
		}
	}
#else
	sport = bf53x_sport_init(CONFIG_SND_BFIN_SPORT,
		SPORT_DMA_RX, snd_ad73322_dma_rx,
		SPORT_DMA_TX, snd_ad73322_dma_tx,
		SPORT_IRQ_ERR, snd_ad73322_sport_err, ad73322);
	if (sport == NULL) {
		err = -ENODEV;
		goto __sport_err;
	}
#endif
	ad73322->sport = sport;
	for (i=0; i<NUM_DEVICES_CHAIN; i++) {
		if ((err = snd_ad73322_pcm(ad73322, i)) < 0)
			goto __nodev;
	}
	err = bf53x_sport_config_tx(sport, TFSR, 0xF, 0, 0);
	err = err || bf53x_sport_config_rx(sport, RFSR, 0xF, 0, 0);
	if (err)
		goto __nodev;
	strcpy(card->driver, DRIVER_NAME);
	strcpy(card->shortname, CHIP_NAME);
#ifdef HAVE_TWO_CARDS
	if (pdev->id == 0) {
		sprintf(card->longname, "%s at PF%d SPORT%d rx/tx dma %d/%d err irq %d",
			card->shortname,
			CONFIG_SND_BFIN_AD73322_SE,
			0,
			CH_SPORT0_RX, CH_SPORT0_TX, IRQ_SPORT0_ERROR);
	} else if (pdev->id == 1) {
		sprintf(card->longname, "%s at PF%d SPORT%d rx/tx dma %d/%d err irq %d",
			card->shortname,
			CONFIG_SND_BFIN_AD73322_SE,
			1,
			CH_SPORT1_RX, CH_SPORT1_TX, IRQ_SPORT1_ERROR);
	}
#else
	sprintf(card->longname, "%s at PF%d SPORT%d rx/tx dma %d/%d err irq %d",
	        card->shortname,
	        CONFIG_SND_BFIN_AD73322_SE,
	        CONFIG_SND_BFIN_SPORT,
	        SPORT_DMA_RX, SPORT_DMA_TX, SPORT_IRQ_ERR);
#endif
	snd_card_set_dev(card, (&pdev->dev));
	if ((err = snd_card_register(card)) < 0) {
		goto __nodev;
	}

	platform_set_drvdata(pdev, card);

	return 0;

__nodev:
	bf53x_sport_done(sport);
__sport_err:
	snd_card_free(card);
	return err;
}

static int __devexit snd_ad73322_remove(struct platform_device *pdev)
{
	struct snd_card *card;
	struct snd_ad73322 *ad73322;

	card = platform_get_drvdata(pdev);
	ad73322 = card->private_data;

	snd_ad73322_stop();
	bf53x_sport_done(ad73322->sport);
	snd_card_free(card);

	gpio_free(GPIO_SE);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int snd_ad73322_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	struct snd_ad73322 *ad73322 = card->private_data;
	int i;

	snd_power_change_state(card, SNDRV_CTL_POWER_D3hot);
	for (i=0; i<NUM_DEVICES_CHAIN; i++)	
		snd_pcm_suspend_all(ad73322->pcm[i]);

	return 0;
}
static int snd_ad73322_resume(struct platform_device *pdev)
{
	int err = 0;
	struct snd_card *card = platform_get_drvdata(pdev);
	struct snd_ad73322 *ad73322 = card->private_data;
	err = bf53x_sport_config_rx(ad73322->sport, RFSR, 0xF, 0, 0);
	err = err || bf53x_sport_config_tx(ad73322->sport, TFSR, 0xF, 0, 0);
	if (err)
		snd_printk(KERN_ERR "Unable to set sport configuration\n");
	snd_power_change_state(card, SNDRV_CTL_POWER_D0);

	return 0;
}
#endif

static struct platform_driver snd_ad73322_driver = {
	.probe		= snd_ad73322_probe,
	.remove		= __devexit_p(snd_ad73322_remove),
#ifdef CONFIG_PM
	.suspend	= snd_ad73322_suspend,
	.resume		= snd_ad73322_resume,
#endif
	.driver		= {
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
	},
};

static int __init snd_ad73322_init(void)
{
	int err;
#ifdef HAVE_TWO_CARDS
	int i;
#endif

	if (input_gain > 7) {
		printk(KERN_NOTICE DRIVER_NAME ": valid input_gain values are 0 to 7 inclusive\n");
		return -EINVAL;
	}

	if (output_gain > 7) {
		printk(KERN_NOTICE DRIVER_NAME ": valid output_gain values are 0 to 7 inclusive\n");
		return -EINVAL;
	}

	if ((err = platform_driver_register(&snd_ad73322_driver))<0)
		return err;
#ifdef HAVE_TWO_CARDS
	for (i = 0; i < 2; i++) {
		device[i] = platform_device_register_simple(DRIVER_NAME, i, NULL, 0);
		if (IS_ERR(device[i])) {
			err = PTR_ERR(device[i]);
			platform_driver_unregister(&snd_ad73322_driver);
			return err;
		}
	}
#else
	device = platform_device_register_simple(DRIVER_NAME, 0, NULL, 0);
	if (IS_ERR(device)) {
		err = PTR_ERR(device);
		platform_driver_unregister(&snd_ad73322_driver);
		return err;
	}
#endif
	return err;
}

static void __exit snd_ad73322_exit(void)
{
#ifdef HAVE_TWO_CARDS
	int i;
	for (i = 0; i < 2; i++)
		platform_device_unregister(device[i]);
#else
	platform_device_unregister(device);
#endif
	platform_driver_unregister(&snd_ad73322_driver);
}

MODULE_AUTHOR("Cliff Cai <Cliff.Cai@analog.com>");
MODULE_DESCRIPTION("Blackfin/ADI AD73322");
MODULE_LICENSE("GPL");

module_init(snd_ad73322_init);
module_exit(snd_ad73322_exit);

/*
 * Blackfin DMA Interface
 *
 * Copyright 2008-2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <asm/atomic.h>
#include <asm/blackfin.h>
#include <asm/cacheflush.h>
#include <asm/dma.h>

#define stamp(fmt, args...) pr_debug("%s:%i: " fmt "\n", __func__, __LINE__, ## args)
#define stampit() stamp("here i am")
#define pr_init(fmt, args...) ({ static const __initconst char __fmt[] = fmt; printk(__fmt, ## args); })

#define DRIVER_NAME "bfin-dma"
#define PFX DRIVER_NAME ": "

/*** User space interface: START ***/
struct user_dma_state {
	unsigned int channel;
	volatile int done;
	struct dmasg dsc_src, dsc_dst;
};

#define BF_DMA_REQUEST _IOW('D', 0x00, struct user_dma_state)
#define BF_DMA_FREE    _IOW('D', 0x01, struct user_dma_state)
#define BF_DMA_RUN     _IOW('D', 0x02, struct user_dma_state)
#define BF_DMA_ARUN    _IOW('D', 0x03, struct user_dma_state)
/*** User space interface: END ***/

struct dma_state {
	atomic_t status;
	unsigned int chan_src, chan_dst;
	struct dmasg dsc_src, dsc_dst;
	volatile int *user_done;
	struct completion c;
};

static struct dma_state all_states[] = {
#ifdef CH_MEM_STREAM0_SRC
	{
		.chan_src = CH_MEM_STREAM0_SRC,
		.chan_dst = CH_MEM_STREAM0_DEST,
	},
#endif
#ifdef CH_MEM_STREAM1_SRC
	{
		.chan_src = CH_MEM_STREAM1_SRC,
		.chan_dst = CH_MEM_STREAM1_DEST,
	},
#endif
#ifdef CH_MEM_STREAM2_SRC
	{
		.chan_src = CH_MEM_STREAM2_SRC,
		.chan_dst = CH_MEM_STREAM2_DEST,
	},
#endif
#ifdef CH_MEM_STREAM3_SRC
	{
		.chan_src = CH_MEM_STREAM3_SRC,
		.chan_dst = CH_MEM_STREAM3_DEST,
	},
#endif
};

/**
 *	dsc_start - get the start address from a DMA descriptor
 */
static inline unsigned long dsc_start(struct dmasg *sg)
{
	return sg->start_addr;
}

/**
 *	dsc_end - get the end address from a DMA descriptor
 */
static inline unsigned long dsc_end(struct dmasg *sg)
{
	return dsc_start(sg) +
		(sg->x_count * sg->x_modify) *
		(sg->cfg & DMA2D ? sg->y_count * sg->y_modify : 1);
}

/**
 *	bfin_dma_irq - handle the dma done irq
 *
 * Notify userspace in two ways: the completion unit (for synchronous
 * transfers) and the volatile done bit (for asynchronous transfers).
 */
static irqreturn_t bfin_dma_irq(int irq, void *dev_id)
{
	struct dma_state *state = dev_id;
#ifdef DMA_MMR_SIZE_32
	clear_dma_irqstat(state->chan_src);
#endif
	clear_dma_irqstat(state->chan_dst);
	put_user(1, state->user_done);
	complete(&state->c);
	return IRQ_HANDLED;
}

/**
 *	bdi_request_dma - grab the mdma channel
 */
static int bdi_request_dma(struct dma_state *state, struct user_dma_state __user *ustate)
{
	int ret;

	stampit();

	if (atomic_read(&state->status))
		return -EBUSY;

	ret = request_dma(state->chan_src, DRIVER_NAME);
	if (ret)
		goto err;
	ret = request_dma(state->chan_dst, DRIVER_NAME);
	if (ret)
		goto err_free_1;
	ret = set_dma_callback(state->chan_dst, bfin_dma_irq, state);
	if (ret)
		goto err_free_2;

	clear_dma_irqstat(state->chan_dst);
	state->user_done = &ustate->done;
	init_completion(&state->c);
	atomic_set(&state->status, 1);

	return 0;

 err_free_2:
	free_dma(state->chan_dst);
 err_free_1:
	free_dma(state->chan_src);
 err:
	return ret;
}

/**
 *	bdi_free_dma - release the mdma channel
 */
static int bdi_free_dma(struct dma_state *state)
{
	stampit();
	if (atomic_inc_return(&state->status) != 2) {
		atomic_dec(&state->status);
		return -EBUSY;
	}
	free_dma(state->chan_dst);
	free_dma(state->chan_src);
	atomic_sub(2, &state->status);
	return 0;
}

/**
 *	bdi_do_dma - actually start a DMA transfer
 *
 * Start a memory transfer using the two descriptors given to us by userspace.
 */
static int bdi_do_dma(struct dma_state *state, int async)
{
	struct dmasg *sg;

	stampit();

	if (!atomic_read(&state->status))
		return -EINVAL;

	/* Anomaly 05000301 notes:
	 * This driver (and no other) exposes DMA traffic control settings.
	 * As such, they're always disabled.  If some one does enable it
	 * though, they need to make sure all of their DMA descriptors are
	 * in external memory.  This is because we copy the first set from
	 * user space to our kernel state (all_states) and that array is in
	 * external memory.
	 */

	sg = &state->dsc_src;
	while (sg && sg->cfg & DMAEN) {
		stamp("src cfg:%lx start:%lx end:%lx", sg->cfg, dsc_start(sg), dsc_end(sg));
		flush_dcache_range(dsc_start(sg), dsc_end(sg));
		flush_dcache_range((unsigned)sg, (unsigned)(sg + sizeof(*sg)));

		if (!(sg->cfg & DMAFLOW_LARGE))
			break;
		sg = sg->next_desc_addr;
	}

	sg = &state->dsc_dst;
	while (sg && sg->cfg & DMAEN) {
		stamp("dst cfg:%lx start:%lx end:%lx", sg->cfg, dsc_start(sg), dsc_end(sg));
		invalidate_dcache_range(dsc_start(sg), dsc_end(sg));
		flush_dcache_range((unsigned)sg, (unsigned)(sg + sizeof(*sg)));

		if (!(sg->cfg & DMAFLOW_LARGE))
			break;
		sg = sg->next_desc_addr;
	}

	put_user(0, state->user_done);

	set_dma_next_desc_addr(state->chan_src, &state->dsc_src);
	set_dma_next_desc_addr(state->chan_dst, &state->dsc_dst);

#ifdef DMA_MMR_SIZE_32
	set_dma_config(state->chan_src, NDSIZE_6 |
		set_bfin_dma_config(DIR_READ, DMA_FLOW_LIST, INTR_ON_BUF,
			DIMENSION_LINEAR, DATA_SIZE_8, DMA_NOSYNC_KEEP_DMA_BUF));
	set_dma_config(state->chan_dst, NDSIZE_6 |
		set_bfin_dma_config(DIR_WRITE, DMA_FLOW_LIST, INTR_ON_BUF,
			DIMENSION_LINEAR, DATA_SIZE_8, DMA_NOSYNC_KEEP_DMA_BUF));
#else
	set_dma_config(state->chan_src, NDSIZE_9 |
		set_bfin_dma_config(DIR_READ, DMA_FLOW_LARGE, INTR_ON_BUF,
			DIMENSION_LINEAR, DATA_SIZE_8, DMA_NOSYNC_KEEP_DMA_BUF));
	set_dma_config(state->chan_dst, NDSIZE_9 |
		set_bfin_dma_config(DIR_WRITE, DMA_FLOW_LARGE, INTR_ON_BUF,
			DIMENSION_LINEAR, DATA_SIZE_8, DMA_NOSYNC_KEEP_DMA_BUF));
#endif

	enable_dma(state->chan_src);
	enable_dma(state->chan_dst);

	if (!async)
		return wait_for_completion_interruptible(&state->c);
	else
		return 0;
}

/**
 *	bfin_dma_ioctl - handle commands from user space
 *
 * Every command needs a valid struct user_dma_state as an argument.  This lets
 * us pick out the exact MDMA channel they wish to utilize.
 */
static long bfin_dma_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct user_dma_state __user *ustate = (void *)arg;
	struct dma_state *state;
	int ret, channel;

	stampit();

	ret = get_user(channel, &ustate->channel);
	if (ret)
		return ret;
	if (channel >= ARRAY_SIZE(all_states))
		return -EINVAL;

	state = &all_states[channel];

	/* Make sure we've allocated the channel before doing anything */
	if (cmd != BF_DMA_REQUEST) {
		if (!atomic_read(&state->status))
			return -EINVAL;
		if (copy_from_user(&state->dsc_src, &ustate->dsc_src, sizeof(state->dsc_src)))
			return -EFAULT;
		if (copy_from_user(&state->dsc_dst, &ustate->dsc_dst, sizeof(state->dsc_dst)))
			return -EFAULT;
	}

	/* The get/put module business is to make sure we don't let the
	 * module be unloaded while holding a dma channel.
	 */
	switch (cmd) {
	case BF_DMA_RUN:
	case BF_DMA_ARUN:
		return bdi_do_dma(state, cmd == BF_DMA_ARUN);

	case BF_DMA_REQUEST:
		if (!try_module_get(filp->f_op->owner))
			return -EAGAIN;
		ret = bdi_request_dma(state, ustate);
		if (ret)
			module_put(filp->f_op->owner);
		return ret;

	case BF_DMA_FREE:
		module_put(filp->f_op->owner);
		return bdi_free_dma(state);
	}

	return -EINVAL;
}

static const struct file_operations bfin_dma_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = bfin_dma_ioctl,
};

static struct miscdevice bfin_dma_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = DRIVER_NAME,
	.fops  = &bfin_dma_fops,
};

/**
 *	bfin_dma_init - Initialize module
 *
 * Registers the device and notifier handler. Actual device
 * initialization is handled by bfin_dma_open().
 */
static int __init bfin_dma_init(void)
{
	int ret;

	stampit();

	ret = misc_register(&bfin_dma_misc_device);
	if (ret) {
		pr_init(KERN_ERR PFX "unable to register a misc device\n");
		return ret;
	}

	pr_init(KERN_INFO PFX "initialized\n");

	return 0;
}
module_init(bfin_dma_init);

/**
 *	bfin_dma_exit - Deinitialize module
 *
 * Unregisters the device and notifier handler. Actual device
 * deinitialization is handled by bfin_dma_close().
 */
static void __exit bfin_dma_exit(void)
{
	stampit();

	misc_deregister(&bfin_dma_misc_device);
}
module_exit(bfin_dma_exit);

MODULE_AUTHOR("Mike Frysinger <vapier@gentoo.org>");
MODULE_DESCRIPTION("Blackfin DMA Interface for userspace");
MODULE_LICENSE("GPL");

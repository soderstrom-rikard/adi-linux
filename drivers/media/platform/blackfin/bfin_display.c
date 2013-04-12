/*
 * Analog Devices video display driver
 *
 * Copyright (c) 2011 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/types.h>

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include <asm/dma.h>

#include <media/blackfin/bfin_display.h>
#include <media/blackfin/ppi.h>

#define DISPLAY_DRV_NAME        "bfin_display"
#define DISP_MIN_NUM_BUF        2

struct disp_format {
	char *desc;
	u32 pixelformat;
	enum v4l2_mbus_pixelcode mbus_code;
	int bpp; /* bits per pixel */
	int dlen; /* data length for ppi in bits */
};

struct disp_buffer {
	struct vb2_buffer vb;
	struct list_head list;
};

struct disp_device {
	/* capture device instance */
	struct v4l2_device v4l2_dev;
	/* v4l2 control handler */
	struct v4l2_ctrl_handler ctrl_handler;
	/* device node data */
	struct video_device *video_dev;
	/* sub device instance */
	struct v4l2_subdev *sd;
	/* capture config */
	struct bfin_display_config *cfg;
	/* ppi interface */
	struct ppi_if *ppi;
	/* current output */
	unsigned int cur_output;
	/* current selected standard */
	v4l2_std_id std;
	/* current selected dv_timings */
	struct v4l2_dv_timings dv_timings;
	/* used to store pixel format */
	struct v4l2_pix_format fmt;
	/* bits per pixel*/
	int bpp;
	/* data length for ppi in bits */
	int dlen;
	/* used to store encoder supported format */
	struct disp_format *enc_formats;
	/* number of encoder formats array */
	int num_enc_formats;
	/* pointing to current video buffer */
	struct disp_buffer *cur_frm;
	/* buffer queue used in videobuf2 */
	struct vb2_queue buffer_queue;
	/* allocator-specific contexts for each plane */
	struct vb2_alloc_ctx *alloc_ctx;
	/* queue of filled frames */
	struct list_head dma_queue;
	/* used in videobuf2 callback */
	spinlock_t lock;
	/* used to access display device */
	struct mutex mutex;
};

struct disp_fh {
	struct v4l2_fh fh;
	/* indicates whether this file handle is doing IO */
	bool io_allowed;
};

static const struct disp_format disp_formats[] = {
	{
		.desc        = "YCbCr 4:2:2 Interleaved UYVY 8bits",
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.mbus_code   = V4L2_MBUS_FMT_UYVY8_2X8,
		.bpp         = 16,
		.dlen        = 8,
	},
	{
		.desc        = "YCbCr 4:2:2 Interleaved YUYV 8bits",
		.pixelformat = V4L2_PIX_FMT_YUYV,
		.mbus_code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.bpp         = 16,
		.dlen        = 8,
	},
	{
		.desc        = "YCbCr 4:2:2 Interleaved UYVY 16bits",
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.mbus_code   = V4L2_MBUS_FMT_UYVY8_1X16,
		.bpp         = 16,
		.dlen        = 16,
	},
	{
		.desc        = "RGB 565",
		.pixelformat = V4L2_PIX_FMT_RGB565,
		.mbus_code   = V4L2_MBUS_FMT_RGB565_2X8_LE,
		.bpp         = 16,
		.dlen        = 8,
	},
	{
		.desc        = "RGB 444",
		.pixelformat = V4L2_PIX_FMT_RGB444,
		.mbus_code   = V4L2_MBUS_FMT_RGB444_2X8_PADHI_LE,
		.bpp         = 16,
		.dlen        = 8,
	},

};
#define DISP_MAX_FMTS ARRAY_SIZE(disp_formats)

static irqreturn_t disp_isr(int irq, void *dev_id);

static struct disp_buffer *to_disp_vb(struct vb2_buffer *vb)
{
	return container_of(vb, struct disp_buffer, vb);
}

static int disp_init_encoder_formats(struct disp_device *disp)
{
	enum v4l2_mbus_pixelcode code;
	struct disp_format *df;
	unsigned int num_formats = 0;
	int i, j;

	while (!v4l2_subdev_call(disp->sd, video,
				enum_mbus_fmt, num_formats, &code))
		num_formats++;
	if (!num_formats)
		return -ENXIO;

	df = kzalloc(num_formats * sizeof(*df), GFP_KERNEL);
	if (!df)
		return -ENOMEM;

	for (i = 0; i < num_formats; i++) {
		v4l2_subdev_call(disp->sd, video,
				enum_mbus_fmt, i, &code);
		for (j = 0; j < DISP_MAX_FMTS; j++)
			if (code == disp_formats[j].mbus_code)
				break;
		if (j == DISP_MAX_FMTS) {
			/* we don't allow this encoder working with our bridge */
			kfree(df);
			return -EINVAL;
		}
		df[i] = disp_formats[j];
	}
	disp->enc_formats = df;
	disp->num_enc_formats = num_formats;
	return 0;
}

static void disp_free_encoder_formats(struct disp_device *disp)
{
	disp->num_enc_formats = 0;
	kfree(disp->enc_formats);
	disp->enc_formats = NULL;
}

static int disp_open(struct file *file)
{
	struct disp_device *disp = video_drvdata(file);
	struct video_device *vfd = disp->video_dev;
	struct disp_fh *disp_fh;

	if (!disp->sd) {
		v4l2_err(&disp->v4l2_dev, "No sub device registered\n");
		return -ENODEV;
	}

	disp_fh = kzalloc(sizeof(*disp_fh), GFP_KERNEL);
	if (!disp_fh) {
		v4l2_err(&disp->v4l2_dev,
			 "unable to allocate memory for file handle object\n");
		return -ENOMEM;
	}

	v4l2_fh_init(&disp_fh->fh, vfd);

	/* store pointer to v4l2_fh in private_data member of file */
	file->private_data = &disp_fh->fh;
	v4l2_fh_add(&disp_fh->fh);
	disp_fh->io_allowed = false;
	return 0;
}

static int disp_release(struct file *file)
{
	struct disp_device *disp = video_drvdata(file);
	struct v4l2_fh *fh = file->private_data;
	struct disp_fh *disp_fh = container_of(fh, struct disp_fh, fh);

	/* if this instance is doing IO */
	if (disp_fh->io_allowed)
		vb2_queue_release(&disp->buffer_queue);

	file->private_data = NULL;
	v4l2_fh_del(&disp_fh->fh);
	v4l2_fh_exit(&disp_fh->fh);
	kfree(disp_fh);
	return 0;
}

static int disp_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct disp_device *disp = video_drvdata(file);
	int ret;

	if (mutex_lock_interruptible(&disp->mutex))
		return -ERESTARTSYS;
	ret = vb2_mmap(&disp->buffer_queue, vma);
	mutex_unlock(&disp->mutex);
	return ret;
}

#ifndef CONFIG_MMU
static unsigned long disp_get_unmapped_area(struct file *file,
					    unsigned long addr,
					    unsigned long len,
					    unsigned long pgoff,
					    unsigned long flags)
{
	struct disp_device *disp = video_drvdata(file);
	int ret;

	if (mutex_lock_interruptible(&disp->mutex))
		return -ERESTARTSYS;
	ret = vb2_get_unmapped_area(&disp->buffer_queue,
				    addr,
				    len,
				    pgoff,
				    flags);
	mutex_unlock(&disp->mutex);
	return ret;
}
#endif

static unsigned int disp_poll(struct file *file, poll_table *wait)
{
	struct disp_device *disp = video_drvdata(file);
	int ret;

	mutex_lock(&disp->mutex);
	ret = vb2_poll(&disp->buffer_queue, file, wait);
	mutex_unlock(&disp->mutex);
	return ret;
}

static int disp_queue_setup(struct vb2_queue *vq,
				const struct v4l2_format *fmt,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct disp_device *disp = vb2_get_drv_priv(vq);

	if (*nbuffers < DISP_MIN_NUM_BUF)
		*nbuffers = DISP_MIN_NUM_BUF;

	*nplanes = 1;
	sizes[0] = disp->fmt.sizeimage;
	alloc_ctxs[0] = disp->alloc_ctx;

	return 0;
}

static int disp_buffer_init(struct vb2_buffer *vb)
{
	struct disp_buffer *buf = to_disp_vb(vb);

	INIT_LIST_HEAD(&buf->list);
	return 0;
}

static int disp_buffer_prepare(struct vb2_buffer *vb)
{
	struct disp_device *disp = vb2_get_drv_priv(vb->vb2_queue);
	struct disp_buffer *buf = to_disp_vb(vb);
	unsigned long size;

	size = disp->fmt.sizeimage;
	if (vb2_plane_size(vb, 0) < size) {
		v4l2_err(&disp->v4l2_dev, "buffer too small (%lu < %lu)\n",
				vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}
	vb2_set_plane_payload(&buf->vb, 0, size);

	return 0;
}

static void disp_buffer_queue(struct vb2_buffer *vb)
{
	struct disp_device *disp = vb2_get_drv_priv(vb->vb2_queue);
	struct disp_buffer *buf = to_disp_vb(vb);
	unsigned long flags;

	spin_lock_irqsave(&disp->lock, flags);
	list_add_tail(&buf->list, &disp->dma_queue);
	spin_unlock_irqrestore(&disp->lock, flags);
}

static void disp_buffer_cleanup(struct vb2_buffer *vb)
{
	struct disp_device *disp = vb2_get_drv_priv(vb->vb2_queue);
	struct disp_buffer *buf = to_disp_vb(vb);
	unsigned long flags;

	spin_lock_irqsave(&disp->lock, flags);
	list_del_init(&buf->list);
	spin_unlock_irqrestore(&disp->lock, flags);
}

static void disp_lock(struct vb2_queue *vq)
{
	struct disp_device *disp = vb2_get_drv_priv(vq);
	mutex_lock(&disp->mutex);
}

static void disp_unlock(struct vb2_queue *vq)
{
	struct disp_device *disp = vb2_get_drv_priv(vq);
	mutex_unlock(&disp->mutex);
}

static int disp_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct disp_device *disp = vb2_get_drv_priv(vq);
	struct ppi_if *ppi = disp->ppi;
	struct ppi_params params;
	int ret;

	/* enable streamon on the sub device */
	ret = v4l2_subdev_call(disp->sd, video, s_stream, 1);
	if (ret && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&disp->v4l2_dev, "stream on failed in subdev\n");
		return ret;
	}

	/* set ppi params */
	params.width = disp->fmt.width;
	params.height = disp->fmt.height;
	params.bpp = disp->bpp;
	params.dlen = disp->dlen;
	params.ppi_control = disp->cfg->ppi_control;
	params.int_mask = disp->cfg->int_mask;
	if (disp->cfg->outputs[disp->cur_output].capabilities
			& V4L2_IN_CAP_CUSTOM_TIMINGS) {
		struct v4l2_bt_timings *bt = &disp->dv_timings.bt;

		params.hdelay = bt->hsync + bt->hbackporch;
		params.vdelay = bt->vsync + bt->vbackporch;
		params.line = bt->hfrontporch + bt->hsync
				+ bt->hbackporch + bt->width;
		params.frame = bt->vfrontporch + bt->vsync
				+ bt->vbackporch + bt->height;
		if (bt->interlaced)
			params.frame += bt->il_vfrontporch + bt->il_vsync
					+ bt->il_vbackporch;
		params.hsync = bt->hsync;
		params.vsync = bt->vsync;
	} else if (disp->cfg->outputs[disp->cur_output].capabilities
			& V4L2_IN_CAP_STD) {
		params.hdelay = 0;
		params.vdelay = 0;
		if (disp->std & V4L2_STD_525_60) {
			params.line = 858;
			params.frame = 525;
		} else {
			params.line = 864;
			params.frame = 625;
		}
	} else {
		params.hdelay = 0;
		params.vdelay = 0;
		params.line = params.width + disp->cfg->blank_pixels;
		params.frame = params.height;
	}
	ret = ppi->ops->set_params(ppi, &params);
	if (ret < 0) {
		v4l2_err(&disp->v4l2_dev,
				"Error in setting ppi params\n");
		return ret;
	}

	/* attach ppi DMA irq handler */
	ret = ppi->ops->attach_irq(ppi, disp_isr);
	if (ret < 0) {
		v4l2_err(&disp->v4l2_dev,
				"Error in attaching interrupt handler\n");
		return ret;
	}

	return 0;
}

static int disp_stop_streaming(struct vb2_queue *vq)
{
	struct disp_device *disp = vb2_get_drv_priv(vq);
	struct ppi_if *ppi = disp->ppi;
	unsigned long flags;
	int ret;

	if (!vb2_is_streaming(vq))
		return 0;

	ppi->ops->stop(ppi);
	ppi->ops->detach_irq(ppi);
	ret = v4l2_subdev_call(disp->sd, video, s_stream, 0);
	if (ret && (ret != -ENOIOCTLCMD))
		v4l2_err(&disp->v4l2_dev,
				"stream off failed in subdev\n");

	spin_lock_irqsave(&disp->lock, flags);
	/* release all active buffers */
	while (!list_empty(&disp->dma_queue)) {
		disp->cur_frm = list_entry(disp->dma_queue.next,
						struct disp_buffer, list);
		list_del(&disp->cur_frm->list);
		vb2_buffer_done(&disp->cur_frm->vb, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&disp->lock, flags);
	return 0;
}

static struct vb2_ops disp_video_qops = {
	.queue_setup            = disp_queue_setup,
	.buf_init               = disp_buffer_init,
	.buf_prepare            = disp_buffer_prepare,
	.buf_cleanup            = disp_buffer_cleanup,
	.buf_queue              = disp_buffer_queue,
	.wait_prepare           = disp_unlock,
	.wait_finish            = disp_lock,
	.start_streaming        = disp_start_streaming,
	.stop_streaming         = disp_stop_streaming,
};

static int disp_reqbufs(struct file *file, void *priv,
			struct v4l2_requestbuffers *req_buf)
{
	struct disp_device *disp = video_drvdata(file);
	struct vb2_queue *vq = &disp->buffer_queue;
	struct v4l2_fh *fh = file->private_data;
	struct disp_fh *disp_fh = container_of(fh, struct disp_fh, fh);

	if (vb2_is_busy(vq))
		return -EBUSY;

	disp_fh->io_allowed = true;

	return vb2_reqbufs(vq, req_buf);
}

static int disp_querybuf(struct file *file, void *priv,
				struct v4l2_buffer *buf)
{
	struct disp_device *disp = video_drvdata(file);

	return vb2_querybuf(&disp->buffer_queue, buf);
}

static int disp_qbuf(struct file *file, void *priv,
			struct v4l2_buffer *buf)
{
	struct disp_device *disp = video_drvdata(file);
	struct v4l2_fh *fh = file->private_data;
	struct disp_fh *disp_fh = container_of(fh, struct disp_fh, fh);

	if (!disp_fh->io_allowed)
		return -EBUSY;

	return vb2_qbuf(&disp->buffer_queue, buf);
}

static int disp_dqbuf(struct file *file, void *priv,
			struct v4l2_buffer *buf)
{
	struct disp_device *disp = video_drvdata(file);
	struct v4l2_fh *fh = file->private_data;
	struct disp_fh *disp_fh = container_of(fh, struct disp_fh, fh);

	if (!disp_fh->io_allowed)
		return -EBUSY;

	return vb2_dqbuf(&disp->buffer_queue,
				buf, file->f_flags & O_NONBLOCK);
}

static irqreturn_t disp_isr(int irq, void *dev_id)
{
	struct ppi_if *ppi = dev_id;
	struct disp_device *disp = ppi->priv;
	struct timeval timevalue;
	struct vb2_buffer *vb = &disp->cur_frm->vb;
	dma_addr_t addr;

	spin_lock(&disp->lock);

	if (!list_empty(&disp->dma_queue)) {
		do_gettimeofday(&timevalue);
		vb->v4l2_buf.timestamp = timevalue;
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		disp->cur_frm = list_entry(disp->dma_queue.next,
				struct disp_buffer, list);
		list_del(&disp->cur_frm->list);
	}

	clear_dma_irqstat(ppi->info->dma_ch);

	addr = vb2_dma_contig_plane_dma_addr(&disp->cur_frm->vb, 0);
	ppi->ops->update_addr(ppi, (unsigned long)addr);
	ppi->ops->start(ppi);

	spin_unlock(&disp->lock);

	return IRQ_HANDLED;
}

static int disp_streamon(struct file *file, void *priv,
				enum v4l2_buf_type buf_type)
{
	struct disp_device *disp = video_drvdata(file);
	struct disp_fh *fh = file->private_data;
	struct ppi_if *ppi = disp->ppi;
	dma_addr_t addr;
	int ret;

	if (!fh->io_allowed)
		return -EBUSY;

	/* call streamon to start streaming in videobuf */
	ret = vb2_streamon(&disp->buffer_queue, buf_type);
	if (ret)
		return ret;

	/* if dma queue is empty, return error */
	if (list_empty(&disp->dma_queue)) {
		v4l2_err(&disp->v4l2_dev, "dma queue is empty\n");
		ret = -EINVAL;
		goto err;
	}

	/* get the next frame from the dma queue */
	disp->cur_frm = list_entry(disp->dma_queue.next,
					struct disp_buffer, list);
	/* remove buffer from the dma queue */
	list_del(&disp->cur_frm->list);
	addr = vb2_dma_contig_plane_dma_addr(&disp->cur_frm->vb, 0);
	/* update DMA address */
	ppi->ops->update_addr(ppi, (unsigned long)addr);
	/* enable ppi */
	ppi->ops->start(ppi);

	return 0;
err:
	vb2_streamoff(&disp->buffer_queue, buf_type);
	return ret;
}

static int disp_streamoff(struct file *file, void *priv,
				enum v4l2_buf_type buf_type)
{
	struct disp_device *disp = video_drvdata(file);
	struct disp_fh *fh = file->private_data;

	if (!fh->io_allowed)
		return -EBUSY;

	return vb2_streamoff(&disp->buffer_queue, buf_type);
}

static int disp_g_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct disp_device *disp = video_drvdata(file);

	*std = disp->std;
	return 0;
}

static int disp_s_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct disp_device *disp = video_drvdata(file);
	int ret;

	if (vb2_is_busy(&disp->buffer_queue))
		return -EBUSY;

	ret = v4l2_subdev_call(disp->sd, video, s_std_output, *std);
	if (ret < 0)
		return ret;

	disp->std = *std;
	return 0;
}

static int disp_g_dv_timings(struct file *file, void *priv,
				struct v4l2_dv_timings *timings)
{
	struct disp_device *disp = video_drvdata(file);
	int ret;

	ret = v4l2_subdev_call(disp->sd, video,
				g_dv_timings, timings);
	if (ret < 0)
		return ret;

	disp->dv_timings = *timings;
	return 0;
}

static int disp_s_dv_timings(struct file *file, void *priv,
				struct v4l2_dv_timings *timings)
{
	struct disp_device *disp = video_drvdata(file);
	int ret;
	if (vb2_is_busy(&disp->buffer_queue))
		return -EBUSY;

	ret = v4l2_subdev_call(disp->sd, video, s_dv_timings, timings);
	if (ret < 0)
		return ret;

	disp->dv_timings = *timings;
	return 0;
}

static int disp_enum_output(struct file *file, void *priv,
				struct v4l2_output *output)
{
	struct disp_device *disp = video_drvdata(file);
	struct bfin_display_config *config = disp->cfg;

	if (output->index >= config->num_outputs)
		return -EINVAL;

	*output = config->outputs[output->index];
	return 0;
}

static int disp_g_output(struct file *file, void *priv, unsigned int *index)
{
	struct disp_device *disp = video_drvdata(file);

	*index = disp->cur_output;
	return 0;
}

static int disp_s_output(struct file *file, void *priv, unsigned int index)
{
	struct disp_device *disp = video_drvdata(file);
	struct bfin_display_config *config = disp->cfg;
	struct disp_route *route;
	int ret;

	if (vb2_is_busy(&disp->buffer_queue))
		return -EBUSY;

	if (index >= config->num_outputs)
		return -EINVAL;

	route = &config->routes[index];
	ret = v4l2_subdev_call(disp->sd, video, s_routing,
				0, route->output, route->config);
	if ((ret < 0) && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&disp->v4l2_dev, "Failed to set output\n");
		return ret;
	}
	disp->cur_output = index;
	/* if this route has specific config, update ppi control */
	if (route->ppi_control)
		config->ppi_control = route->ppi_control;
	return 0;
}

static int disp_try_format(struct disp_device *disp,
				struct v4l2_pix_format *pixfmt,
				struct disp_format *disp_fmt)
{
	struct disp_format *df = disp->enc_formats;
	struct disp_format *fmt = NULL;
	struct v4l2_mbus_framefmt mbus_fmt;
	int ret, i;

	for (i = 0; i < disp->num_enc_formats; i++) {
		fmt = &df[i];
		if (pixfmt->pixelformat == fmt->pixelformat)
			break;
	}
	if (i == disp->num_enc_formats)
		fmt = &df[0];

	v4l2_fill_mbus_format(&mbus_fmt, pixfmt, fmt->mbus_code);
	ret = v4l2_subdev_call(disp->sd, video,
				try_mbus_fmt, &mbus_fmt);
	if (ret < 0)
		return ret;
	v4l2_fill_pix_format(pixfmt, &mbus_fmt);
	if (disp_fmt) {
		for (i = 0; i < disp->num_enc_formats; i++) {
			fmt = &df[i];
			if (mbus_fmt.code == fmt->mbus_code)
				break;
		}
		*disp_fmt = *fmt;
	}
	pixfmt->bytesperline = pixfmt->width * fmt->bpp / 8;
	pixfmt->sizeimage = pixfmt->bytesperline * pixfmt->height;
	return 0;
}

static int disp_enum_fmt_vid_out(struct file *file, void  *priv,
					struct v4l2_fmtdesc *fmt)
{
	struct disp_device *disp = video_drvdata(file);
	struct disp_format *df = disp->enc_formats;

	if (fmt->index >= disp->num_enc_formats)
		return -EINVAL;

	fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	strlcpy(fmt->description,
		df[fmt->index].desc,
		sizeof(fmt->description));
	fmt->pixelformat = df[fmt->index].pixelformat;
	return 0;
}

static int disp_try_fmt_vid_out(struct file *file, void *priv,
					struct v4l2_format *fmt)
{
	struct disp_device *disp = video_drvdata(file);
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;

	return disp_try_format(disp, pixfmt, NULL);
}

static int disp_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct disp_device *disp = video_drvdata(file);

	fmt->fmt.pix = disp->fmt;
	return 0;
}

static int disp_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct disp_device *disp = video_drvdata(file);
	struct v4l2_mbus_framefmt mbus_fmt;
	struct disp_format disp_fmt;
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
	int ret;

	if (vb2_is_busy(&disp->buffer_queue))
		return -EBUSY;

	/* see if format works */
	ret = disp_try_format(disp, pixfmt, &disp_fmt);
	if (ret < 0)
		return ret;

	v4l2_fill_mbus_format(&mbus_fmt, pixfmt, disp_fmt.mbus_code);
	ret = v4l2_subdev_call(disp->sd, video, s_mbus_fmt, &mbus_fmt);
	if (ret < 0)
		return ret;
	disp->fmt = *pixfmt;
	disp->bpp = disp_fmt.bpp;
	disp->dlen = disp_fmt.dlen;
	return 0;
}

static int disp_querycap(struct file *file, void  *priv,
				struct v4l2_capability *cap)
{
	struct disp_device *disp = video_drvdata(file);

	cap->capabilities = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING;
	strlcpy(cap->driver, DISPLAY_DRV_NAME, sizeof(cap->driver));
	strlcpy(cap->bus_info, "Blackfin Platform", sizeof(cap->bus_info));
	strlcpy(cap->card, disp->cfg->card_name, sizeof(cap->card));
	return 0;
}

static int disp_g_parm(struct file *file, void *fh,
				struct v4l2_streamparm *a)
{
	struct disp_device *disp = video_drvdata(file);

	return v4l2_subdev_call(disp->sd, video, g_parm, a);
}

static int disp_s_parm(struct file *file, void *fh,
				struct v4l2_streamparm *a)
{
	struct disp_device *disp = video_drvdata(file);

	return v4l2_subdev_call(disp->sd, video, s_parm, a);
}

static int disp_g_chip_ident(struct file *file, void *priv,
		struct v4l2_dbg_chip_ident *chip)
{
	struct disp_device *disp = video_drvdata(file);

	chip->ident = V4L2_IDENT_NONE;
	chip->revision = 0;
	if (chip->match.type != V4L2_CHIP_MATCH_I2C_DRIVER &&
			chip->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	return v4l2_subdev_call(disp->sd, core,
			g_chip_ident, chip);
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int disp_dbg_g_register(struct file *file, void *priv,
		struct v4l2_dbg_register *reg)
{
	struct disp_device *disp = video_drvdata(file);

	return v4l2_subdev_call(disp->sd, core,
			g_register, reg);
}

static int disp_dbg_s_register(struct file *file, void *priv,
		struct v4l2_dbg_register *reg)
{
	struct disp_device *disp = video_drvdata(file);

	return v4l2_subdev_call(disp->sd, core,
			s_register, reg);
}
#endif

static int disp_log_status(struct file *file, void *priv)
{
	struct disp_device *disp = video_drvdata(file);
	/* status for sub devices */
	v4l2_device_call_all(&disp->v4l2_dev, 0, core, log_status);
	return 0;
}

static const struct v4l2_ioctl_ops disp_ioctl_ops = {
	.vidioc_querycap         = disp_querycap,
	.vidioc_g_fmt_vid_out    = disp_g_fmt_vid_out,
	.vidioc_enum_fmt_vid_out = disp_enum_fmt_vid_out,
	.vidioc_s_fmt_vid_out    = disp_s_fmt_vid_out,
	.vidioc_try_fmt_vid_out  = disp_try_fmt_vid_out,
	.vidioc_enum_output      = disp_enum_output,
	.vidioc_g_output         = disp_g_output,
	.vidioc_s_output         = disp_s_output,
	.vidioc_s_std            = disp_s_std,
	.vidioc_g_std            = disp_g_std,
	.vidioc_s_dv_timings     = disp_s_dv_timings,
	.vidioc_g_dv_timings     = disp_g_dv_timings,
	.vidioc_reqbufs          = disp_reqbufs,
	.vidioc_querybuf         = disp_querybuf,
	.vidioc_qbuf             = disp_qbuf,
	.vidioc_dqbuf            = disp_dqbuf,
	.vidioc_streamon         = disp_streamon,
	.vidioc_streamoff        = disp_streamoff,
	.vidioc_g_parm           = disp_g_parm,
	.vidioc_s_parm           = disp_s_parm,
	.vidioc_g_chip_ident     = disp_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_register       = disp_dbg_g_register,
	.vidioc_s_register       = disp_dbg_s_register,
#endif
	.vidioc_log_status       = disp_log_status,
};

static struct v4l2_file_operations disp_fops = {
	.owner = THIS_MODULE,
	.open = disp_open,
	.release = disp_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = disp_mmap,
#ifndef CONFIG_MMU
	.get_unmapped_area = disp_get_unmapped_area,
#endif
	.poll = disp_poll
};

static int disp_probe(struct platform_device *pdev)
{
	struct disp_device *disp;
	struct video_device *vfd;
	struct i2c_adapter *i2c_adap;
	struct bfin_display_config *config;
	struct vb2_queue *q;
	struct disp_route *route;
	int ret;

	config = pdev->dev.platform_data;
	if (!config) {
		v4l2_err(pdev->dev.driver, "Unable to get board config\n");
		return -ENODEV;
	}

	disp = kzalloc(sizeof(*disp), GFP_KERNEL);
	if (!disp) {
		v4l2_err(pdev->dev.driver, "Unable to alloc disp\n");
		return -ENOMEM;
	}

	disp->cfg = config;

	disp->ppi = ppi_create_instance(config->ppi_info);
	if (!disp->ppi) {
		v4l2_err(pdev->dev.driver, "Unable to create ppi\n");
		ret = -ENODEV;
		goto err_free_dev;
	}
	disp->ppi->priv = disp;

	disp->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(disp->alloc_ctx)) {
		ret = PTR_ERR(disp->alloc_ctx);
		goto err_free_ppi;
	}

	vfd = video_device_alloc();
	if (!vfd) {
		ret = -ENOMEM;
		v4l2_err(pdev->dev.driver, "Unable to alloc video device\n");
		goto err_cleanup_ctx;
	}

	/* initialize field of video device */
	vfd->release    = video_device_release;
	vfd->fops       = &disp_fops;
	vfd->ioctl_ops  = &disp_ioctl_ops;
	vfd->tvnorms    = 0;
	vfd->v4l2_dev   = &disp->v4l2_dev;
	vfd->vfl_dir    = VFL_DIR_TX;
	set_bit(V4L2_FL_USE_FH_PRIO, &vfd->flags);
	strncpy(vfd->name, DISPLAY_DRV_NAME, sizeof(vfd->name));
	disp->video_dev = vfd;

	ret = v4l2_device_register(&pdev->dev, &disp->v4l2_dev);
	if (ret) {
		v4l2_err(pdev->dev.driver,
				"Unable to register v4l2 device\n");
		goto err_release_vdev;
	}
	v4l2_info(&disp->v4l2_dev, "v4l2 device registered\n");

	disp->v4l2_dev.ctrl_handler = &disp->ctrl_handler;
	ret = v4l2_ctrl_handler_init(&disp->ctrl_handler, 0);
	if (ret) {
		v4l2_err(&disp->v4l2_dev,
				"Unable to init control handler\n");
		goto err_unreg_v4l2;
	}

	spin_lock_init(&disp->lock);
	/* initialize queue */
	q = &disp->buffer_queue;
	q->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	q->io_modes = VB2_MMAP;
	q->drv_priv = disp;
	q->buf_struct_size = sizeof(struct disp_buffer);
	q->ops = &disp_video_qops;
	q->mem_ops = &vb2_dma_contig_memops;

	vb2_queue_init(q);

	mutex_init(&disp->mutex);

	/* init video dma queues */
	INIT_LIST_HEAD(&disp->dma_queue);

	vfd->lock = &disp->mutex;

	/* register video device */
	ret = video_register_device(disp->video_dev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(&disp->v4l2_dev,
				"Unable to register video device\n");
		goto err_free_handler;
	}
	video_set_drvdata(disp->video_dev, disp);
	v4l2_info(&disp->v4l2_dev, "video device registered as: %s\n",
			video_device_node_name(vfd));

	/* load up the subdevice */
	i2c_adap = i2c_get_adapter(config->i2c_adapter_id);
	if (!i2c_adap) {
		v4l2_err(&disp->v4l2_dev,
				"Unable to find i2c adapter\n");
		goto err_unreg_vdev;

	}
	disp->sd = v4l2_i2c_new_subdev_board(&disp->v4l2_dev,
						 i2c_adap,
						 &config->board_info,
						 NULL);
	if (disp->sd) {
		int i;
		if (!config->num_outputs) {
			v4l2_err(&disp->v4l2_dev,
					"Unable to work without output\n");
			goto err_unreg_vdev;
		}

		/* update tvnorms from the sub devices */
		for (i = 0; i < config->num_outputs; i++)
			vfd->tvnorms |= config->outputs[i].std;
	} else {
		v4l2_err(&disp->v4l2_dev,
				"Unable to register sub device\n");
		goto err_unreg_vdev;
	}

	v4l2_info(&disp->v4l2_dev, "v4l2 sub device registered\n");

	/*
	 * explicitly set output, otherwise some boards
	 * may not work at the state as we expected
	 */
	route = &config->routes[0];
	ret = v4l2_subdev_call(disp->sd, video, s_routing,
				route->output, route->output, 0);
	if ((ret < 0) && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&disp->v4l2_dev, "Failed to set output\n");
		goto err_unreg_vdev;
	}
	disp->cur_output = 0;
	/* if this route has specific config, update ppi control */
	if (route->ppi_control)
		config->ppi_control = route->ppi_control;

	/* now we can probe the default state */
	if (config->outputs[0].capabilities & V4L2_IN_CAP_STD) {
		v4l2_std_id std;
		ret = v4l2_subdev_call(disp->sd, core, g_std, &std);
		if (ret) {
			v4l2_err(&disp->v4l2_dev,
					"Unable to get std\n");
			goto err_unreg_vdev;
		}
		disp->std = std;
	}
	if (config->outputs[0].capabilities & V4L2_IN_CAP_CUSTOM_TIMINGS) {
		struct v4l2_dv_timings dv_timings;
		ret = v4l2_subdev_call(disp->sd, video,
				g_dv_timings, &dv_timings);
		if (ret) {
			v4l2_err(&disp->v4l2_dev,
					"Unable to get dv timings\n");
			goto err_unreg_vdev;
		}
		disp->dv_timings = dv_timings;
	}
	ret = disp_init_encoder_formats(disp);
	if (ret) {
		v4l2_err(&disp->v4l2_dev,
				"Unable to create encoder formats table\n");
		goto err_unreg_vdev;
	}
	return 0;
err_unreg_vdev:
	video_unregister_device(disp->video_dev);
	disp->video_dev = NULL;
err_free_handler:
	v4l2_ctrl_handler_free(&disp->ctrl_handler);
err_unreg_v4l2:
	v4l2_device_unregister(&disp->v4l2_dev);
err_release_vdev:
	if (disp->video_dev)
		video_device_release(disp->video_dev);
err_cleanup_ctx:
	vb2_dma_contig_cleanup_ctx(disp->alloc_ctx);
err_free_ppi:
	ppi_delete_instance(disp->ppi);
err_free_dev:
	kfree(disp);
	return ret;
}

static int disp_remove(struct platform_device *pdev)
{
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct disp_device *disp = container_of(v4l2_dev,
						struct disp_device, v4l2_dev);

	disp_free_encoder_formats(disp);
	video_unregister_device(disp->video_dev);
	v4l2_ctrl_handler_free(&disp->ctrl_handler);
	v4l2_device_unregister(v4l2_dev);
	vb2_dma_contig_cleanup_ctx(disp->alloc_ctx);
	ppi_delete_instance(disp->ppi);
	kfree(disp);
	return 0;
}

static struct platform_driver disp_driver = {
	.driver = {
		.name  = DISPLAY_DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = disp_probe,
	.remove = disp_remove,
};
module_platform_driver(disp_driver);

MODULE_DESCRIPTION("Analog Devices blackfin video display driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");

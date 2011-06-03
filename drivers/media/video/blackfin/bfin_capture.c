/*
 * Analog Devices video capture driver
 *
 * Copyright (c) 2011 Scott Jiang <Scott.Jiang.Linux@gmail.com>
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


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>
#include <media/videobuf-dma-contig.h>
#include <media/v4l2-chip-ident.h>

#include <asm/dma.h>

#include <media/blackfin/bfin_capture.h>
#include "ppi.h"

#define CAPTURE_DRV_NAME        "bfin_capture"
#define BCAP_MIN_NUM_BUF        3

struct bcap_format {
	u8 *desc;
	u32 pixelformat;
	enum v4l2_mbus_pixelcode mbus_code;
	int bpp; /* bytes per pixel */
};

struct bcap_device {
	/* capture device instance */
	struct v4l2_device v4l2_dev;
	/* device node data */
	struct video_device *video_dev;
	/* sub device instance */
	struct v4l2_subdev *sd;
	/* caputre config */
	struct bfin_capture_config *cfg;
	/* used to keep track of state of the priority */
	struct v4l2_prio_state prio;
	/* ppi interface */
	struct ppi_if *ppi;
	/* current input */
	unsigned int cur_input;
	/* current selected standard */
	v4l2_std_id std;
	/* used to store pixel format */
	struct v4l2_pix_format fmt;
	/* bytes per pixel*/
	int bpp;
	/* pointing to current video buffer */
	struct videobuf_buffer *cur_frm;
	/* pointing to next video buffer */
	struct videobuf_buffer *next_frm;
	/* buffer queue used in video-buf */
	struct videobuf_queue buffer_queue;
	/* queue of filled frames */
	struct list_head dma_queue;
	/* used in video-buf */
	spinlock_t irqlock;
	/* used to access capture device */
	struct mutex lock;
	/* number of users performing IO */
	u32 io_usrs;
	/* number of open instances of the device */
	u32 usrs;
	/* indicate whether streaming has started */
	u8 started;
};

struct bcap_fh {
	struct bcap_device *bcap_dev;
	/* indicates whether this file handle is doing IO */
	u8 io_allowed;
	/* used to keep track priority of this instance */
	enum v4l2_priority prio;
};

static struct bcap_format bcap_formats[] = {
	{
		.desc        = "YCbCr 4:2:2 Interleaved UYVY",
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.mbus_code   = V4L2_MBUS_FMT_UYVY8_2X8,
		.bpp         = 2,
	},
	{
		.desc        = "YCbCr 4:2:2 Interleaved YUYV",
		.pixelformat = V4L2_PIX_FMT_YUYV,
		.mbus_code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.bpp         = 2,
	},
	{
		.desc        = "RGB 565",
		.pixelformat = V4L2_PIX_FMT_RGB565,
		.mbus_code   = V4L2_MBUS_FMT_RGB565_2X8_LE,
		.bpp         = 2,
	},
	{
		.desc        = "RGB 444",
		.pixelformat = V4L2_PIX_FMT_RGB444,
		.mbus_code   = V4L2_MBUS_FMT_RGB444_2X8_PADHI_LE,
		.bpp         = 2,
	},

};
#define BCAP_MAX_FMTS ARRAY_SIZE(bcap_formats)

static int bcap_open(struct file *file)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	struct bcap_fh *fh;

	if (!bcap_dev->sd) {
		v4l2_err(&bcap_dev->v4l2_dev, "No sub device registered\n");
		return -ENODEV;
	}

	fh = kmalloc(sizeof(struct bcap_fh), GFP_KERNEL);
	if (!fh) {
		v4l2_err(&bcap_dev->v4l2_dev,
			 "unable to allocate memory for file handle object\n");
		return -ENOMEM;
	}
	/* store pointer to fh in private_data member of file */
	file->private_data = fh;
	fh->bcap_dev = bcap_dev;
	bcap_dev->usrs++;
	fh->io_allowed = 0;
	fh->prio = V4L2_PRIORITY_UNSET;
	v4l2_prio_open(&bcap_dev->prio, &fh->prio);
	return 0;
}

static int bcap_release(struct file *file)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	struct bcap_fh *fh = file->private_data;
	struct ppi_if *ppi = bcap_dev->ppi;
	int ret = 0;

	/* if this instance is doing IO */
	if (fh->io_allowed) {
		if (bcap_dev->started) {
			ppi->ops->detach_irq(ppi);
			bcap_dev->started = 0;
			ret = v4l2_subdev_call(bcap_dev->sd,
						video, s_stream, 0);
			if (ret && (ret != -ENOIOCTLCMD))
				v4l2_err(&bcap_dev->v4l2_dev,
						"stream off failed in subdev\n");
			videobuf_stop(&bcap_dev->buffer_queue);
			videobuf_mmap_free(&bcap_dev->buffer_queue);
		}
		bcap_dev->io_usrs = 0;
	}

	bcap_dev->usrs--;
	v4l2_prio_close(&bcap_dev->prio, fh->prio);
	file->private_data = NULL;
	kfree(fh);
	return 0;
}

static int bcap_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct bcap_device *bcap_dev = video_drvdata(file);

	return videobuf_mmap_mapper(&bcap_dev->buffer_queue, vma);
}

static unsigned int bcap_poll(struct file *file, poll_table *wait)
{
	struct bcap_device *bcap_dev = video_drvdata(file);

	if (bcap_dev->started)
		return videobuf_poll_stream(file,
					    &bcap_dev->buffer_queue, wait);
	return 0;
}

static int bcap_videobuf_setup(struct videobuf_queue *vq,
				unsigned int *count,
				unsigned int *size)
{
	struct bcap_fh *fh = vq->priv_data;
	struct bcap_device *bcap_dev = fh->bcap_dev;

	*size = bcap_dev->fmt.sizeimage;

	if (*count < BCAP_MIN_NUM_BUF)
		*count = BCAP_MIN_NUM_BUF;
	return 0;
}

static int bcap_videobuf_prepare(struct videobuf_queue *vq,
					struct videobuf_buffer *vb,
					enum v4l2_field field)
{
	struct bcap_fh *fh = vq->priv_data;
	struct bcap_device *bcap_dev = fh->bcap_dev;
	unsigned long addr;
	int ret;

	if (VIDEOBUF_NEEDS_INIT == vb->state) {
		vb->width = bcap_dev->fmt.width;
		vb->height = bcap_dev->fmt.height;
		vb->size = bcap_dev->fmt.sizeimage;
		vb->field = field;

		addr = videobuf_to_dma_contig(vb);
		if (!ALIGN(addr, 2))
			return -EINVAL;

		ret = videobuf_iolock(vq, vb, NULL);
		if (ret < 0)
			return ret;

		vb->state = VIDEOBUF_PREPARED;
	}

	return 0;
}

static void bcap_videobuf_queue(struct videobuf_queue *vq,
				struct videobuf_buffer *vb)
{
	struct bcap_fh *fh = vq->priv_data;
	struct bcap_device *bcap_dev = fh->bcap_dev;

	list_add_tail(&vb->queue, &bcap_dev->dma_queue);
	vb->state = VIDEOBUF_QUEUED;
}

static void bcap_videobuf_release(struct videobuf_queue *vq,
					struct videobuf_buffer *vb)
{
	videobuf_dma_contig_free(vq, vb);
	vb->state = VIDEOBUF_NEEDS_INIT;
}

static struct videobuf_queue_ops bcap_videobuf_qops = {
	.buf_setup      = bcap_videobuf_setup,
	.buf_prepare    = bcap_videobuf_prepare,
	.buf_queue      = bcap_videobuf_queue,
	.buf_release    = bcap_videobuf_release,
};

static int bcap_reqbufs(struct file *file, void *priv,
			struct v4l2_requestbuffers *req_buf)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	struct bcap_fh *fh = file->private_data;

	if ((V4L2_BUF_TYPE_VIDEO_CAPTURE != req_buf->type)
		|| (V4L2_MEMORY_MMAP != req_buf->memory))
		return -EINVAL;

	if (bcap_dev->io_usrs != 0) {
		v4l2_err(&bcap_dev->v4l2_dev, "Only one IO user allowed\n");
		return -EBUSY;
	}

	videobuf_queue_dma_contig_init(&bcap_dev->buffer_queue,
					&bcap_videobuf_qops,
					bcap_dev->v4l2_dev.dev,
					&bcap_dev->irqlock,
					req_buf->type,
					bcap_dev->fmt.field,
					sizeof(struct videobuf_buffer),
					fh, &bcap_dev->lock);
	fh->io_allowed = 1;
	bcap_dev->io_usrs = 1;
	INIT_LIST_HEAD(&bcap_dev->dma_queue);

	return videobuf_reqbufs(&bcap_dev->buffer_queue, req_buf);
}

static int bcap_querybuf(struct file *file, void *priv,
				struct v4l2_buffer *buf)
{
	struct bcap_device *bcap_dev = video_drvdata(file);

	if ((V4L2_BUF_TYPE_VIDEO_CAPTURE != buf->type)
		|| (V4L2_MEMORY_MMAP != buf->memory))
		return -EINVAL;

	return videobuf_querybuf(&bcap_dev->buffer_queue, buf);
}

static int bcap_qbuf(struct file *file, void *priv,
			struct v4l2_buffer *buf)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	struct bcap_fh *fh = file->private_data;

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != buf->type)
		return -EINVAL;

	if (!fh->io_allowed)
		return -EACCES;

	return videobuf_qbuf(&bcap_dev->buffer_queue, buf);
}

static int bcap_dqbuf(struct file *file, void *priv,
			struct v4l2_buffer *buf)
{
	struct bcap_device *bcap_dev = video_drvdata(file);

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != buf->type)
		return -EINVAL;

	return videobuf_dqbuf(&bcap_dev->buffer_queue,
				buf, file->f_flags & O_NONBLOCK);
}

static irqreturn_t bcap_isr(int irq, void *dev_id)
{
	struct ppi_if *ppi = dev_id;
	struct bcap_device *bcap_dev = ppi->priv;
	struct timeval timevalue;
	unsigned long addr;

	if (bcap_dev->cur_frm != bcap_dev->next_frm) {
		do_gettimeofday(&timevalue);
		bcap_dev->cur_frm->ts = timevalue;
		bcap_dev->cur_frm->state = VIDEOBUF_DONE;
		bcap_dev->cur_frm->size = bcap_dev->fmt.sizeimage;
		wake_up_interruptible(&bcap_dev->cur_frm->done);
		bcap_dev->cur_frm = bcap_dev->next_frm;
	}

	if (!list_empty(&bcap_dev->dma_queue)) {
		bcap_dev->next_frm = list_entry(bcap_dev->dma_queue.next,
						struct videobuf_buffer, queue);
		list_del(&bcap_dev->next_frm->queue);
		bcap_dev->next_frm->state = VIDEOBUF_ACTIVE;
		addr = videobuf_to_dma_contig(bcap_dev->next_frm);
		ppi->ops->update_addr(ppi, addr);
	}
	if (ppi->ops->clear_int)
		ppi->ops->clear_int(ppi);
	ppi->ops->start(ppi);

	return IRQ_HANDLED;
}

static int bcap_streamon(struct file *file, void *priv,
				enum v4l2_buf_type buf_type)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	struct bcap_fh *fh = file->private_data;
	struct ppi_if *ppi = bcap_dev->ppi;
	struct ppi_params params;
	unsigned long addr;
	int ret;

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != buf_type)
		return -EINVAL;

	if (!fh->io_allowed)
		return -EACCES;

	if (bcap_dev->started)
		return -EBUSY;

	/* enable streamon on the sub device */
	ret = v4l2_subdev_call(bcap_dev->sd, video, s_stream, 1);
	if (ret && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&bcap_dev->v4l2_dev, "stream on failed in subdev\n");
		return ret;
	}

	/* if buffer queue is empty, return error */
	if (list_empty(&bcap_dev->buffer_queue.stream)) {
		v4l2_err(&bcap_dev->v4l2_dev, "buffer queue is empty\n");
		return -EIO;
	}

	/* call videobuf_streamon to start streaming in videobuf */
	ret = videobuf_streamon(&bcap_dev->buffer_queue);
	if (ret)
		return ret;

	/* get the next frame from the buffer queue */
	bcap_dev->next_frm = list_entry(bcap_dev->dma_queue.next,
					struct videobuf_buffer, queue);
	bcap_dev->cur_frm = bcap_dev->next_frm;
	/* remove buffer from the buffer queue */
	list_del(&bcap_dev->cur_frm->queue);
	/* mark state of the current frame to active */
	bcap_dev->cur_frm->state = VIDEOBUF_ACTIVE;
	addr = videobuf_to_dma_contig(bcap_dev->cur_frm);

	/* attach ppi DMA irq handler */
	ret = ppi->ops->attach_irq(ppi, bcap_isr);
	if (ret < 0) {
		v4l2_err(&bcap_dev->v4l2_dev,
				"Error in attaching interrupt handler\n");
		ret = -EFAULT;
		goto err;
	}

	/* set ppi params */
	params.width = bcap_dev->fmt.width;
	params.height = bcap_dev->fmt.height;
	params.bpp = bcap_dev->bpp;
	ret = ppi->ops->set_params(ppi, &params);
	if (ret < 0) {
		v4l2_err(&bcap_dev->v4l2_dev,
				"Error in attaching interrupt handler\n");
		ret = -EINVAL;
		goto err1;
	}

	/* update DMA address */
	ppi->ops->update_addr(ppi, addr);
	/* enable ppi */
	ppi->ops->start(ppi);
	bcap_dev->started = 1;

	return 0;
err1:
	ppi->ops->detach_irq(ppi);
err:
	videobuf_streamoff(&bcap_dev->buffer_queue);
	return ret;
}

static int bcap_streamoff(struct file *file, void *priv,
				enum v4l2_buf_type buf_type)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	struct bcap_fh *fh = file->private_data;
	struct ppi_if *ppi = bcap_dev->ppi;
	int ret;

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != buf_type)
		return -EINVAL;

	if (!fh->io_allowed)
		return -EACCES;

	if (!bcap_dev->started)
		return -EINVAL;

	ppi->ops->stop(ppi);
	ppi->ops->detach_irq(ppi);
	bcap_dev->started = 0;
	ret = v4l2_subdev_call(bcap_dev->sd, video, s_stream, 0);
	if (ret && (ret != -ENOIOCTLCMD))
		v4l2_err(&bcap_dev->v4l2_dev, "stream off failed in subdev\n");
	return videobuf_streamoff(&bcap_dev->buffer_queue);
}

static int bcap_queryctrl(struct file *file, void *priv,
				struct v4l2_queryctrl *qctrl)
{
	struct bcap_device *bcap_dev = video_drvdata(file);

	return v4l2_subdev_call(bcap_dev->sd, core, queryctrl, qctrl);
}

static int bcap_g_ctrl(struct file *file, void *priv,
			struct v4l2_control *ctrl)
{
	struct bcap_device *bcap_dev = video_drvdata(file);

	return v4l2_subdev_call(bcap_dev->sd, core, g_ctrl, ctrl);
}

static int bcap_s_ctrl(struct file *file, void *priv,
			struct v4l2_control *ctrl)
{
	struct bcap_device *bcap_dev = video_drvdata(file);

	return v4l2_subdev_call(bcap_dev->sd, core, s_ctrl, ctrl);
}

static int bcap_querystd(struct file *file, void *priv, v4l2_std_id *std)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	int ret;

	ret = v4l2_subdev_call(bcap_dev->sd, video, querystd, std);
	return ret;
}

static int bcap_g_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct bcap_device *bcap_dev = video_drvdata(file);

	*std = bcap_dev->std;
	return 0;
}

static int bcap_s_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	int ret;

	/* if streaming is started, return error */
	if (bcap_dev->started) {
		v4l2_err(&bcap_dev->v4l2_dev, "Streaming is started\n");
		return -EBUSY;
	}

	ret = v4l2_subdev_call(bcap_dev->sd, core, s_std, *std);
	if (ret < 0)
		return ret;

	bcap_dev->std = *std;
	return 0;
}

static int bcap_enum_input(struct file *file, void *priv,
				struct v4l2_input *input)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	struct bfin_capture_config *config = bcap_dev->cfg;

	if (input->index >= config->num_inputs)
		return -EINVAL;

	memcpy(input, &config->inputs[input->index],
		sizeof(*input));
	return 0;
}

static int bcap_g_input(struct file *file, void *priv, unsigned int *index)
{
	struct bcap_device *bcap_dev = video_drvdata(file);

	*index = bcap_dev->cur_input;
	return 0;
}

static int bcap_s_input(struct file *file, void *priv, unsigned int index)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	struct bfin_capture_config *config = bcap_dev->cfg;
	struct bcap_route *route;
	int ret;

	/* if streaming is started, return error */
	if (bcap_dev->started) {
		v4l2_err(&bcap_dev->v4l2_dev, "Streaming is started\n");
		return -EBUSY;
	}

	if (index >= config->num_inputs)
		return -EINVAL;

	route = &config->routes[index];
	ret = v4l2_subdev_call(bcap_dev->sd, video, s_routing,
				route->input, route->output, 0);
	if ((ret < 0) && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&bcap_dev->v4l2_dev, "Failed to set input\n");
		return ret;
	}
	bcap_dev->cur_input = index;
	return 0;
}

static int bcap_try_format(struct bcap_device *bcap,
				struct v4l2_pix_format *pixfmt,
				enum v4l2_mbus_pixelcode *mbus_code,
				int *bpp)
{
	struct bcap_format *fmt;
	struct v4l2_mbus_framefmt mbus_fmt;
	int ret, i;

	for (i = 0; i < BCAP_MAX_FMTS; i++) {
		if ((pixfmt->pixelformat == bcap_formats[i].pixelformat)) {
			fmt = &bcap_formats[i];
			if (mbus_code)
				*mbus_code = fmt->mbus_code;
			if (bpp)
				*bpp = fmt->bpp;
			v4l2_fill_mbus_format(&mbus_fmt, pixfmt,
						fmt->mbus_code);
			ret = v4l2_subdev_call(bcap->sd, video,
						try_mbus_fmt, &mbus_fmt);
			if (ret < 0)
				return ret;
			v4l2_fill_pix_format(pixfmt, &mbus_fmt);
			pixfmt->bytesperline = pixfmt->width * fmt->bpp;
			pixfmt->sizeimage = pixfmt->bytesperline
						* pixfmt->height;
			return 0;
		}
	}
	return -EINVAL;
}

static int bcap_enum_fmt_vid_cap(struct file *file, void  *priv,
					struct v4l2_fmtdesc *fmt)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	enum v4l2_mbus_pixelcode mbus_code;
	u32 index = fmt->index;
	int ret, i;

	ret = v4l2_subdev_call(bcap_dev->sd, video,
				enum_mbus_fmt, index, &mbus_code);
	if (ret < 0)
		return ret;

	for (i = 0; i < BCAP_MAX_FMTS; i++) {
		if (mbus_code == bcap_formats[i].mbus_code) {
			fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			strlcpy(fmt->description,
					bcap_formats[index].desc,
					 sizeof(fmt->description));
			fmt->pixelformat = bcap_formats[index].pixelformat;
			return 0;
		}
	}
	v4l2_err(&bcap_dev->v4l2_dev,
			"subdev fmt is not supported by bcap\n");
	return -EINVAL;
}

static int bcap_try_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *fmt)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	return bcap_try_format(bcap_dev, pixfmt, NULL, NULL);
}

static int bcap_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	struct v4l2_mbus_framefmt mbus_fmt;
	struct bcap_format *bcap_fmt;
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
	int ret, i;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	ret = v4l2_subdev_call(bcap_dev->sd, video,
				g_mbus_fmt, &mbus_fmt);
	if (ret < 0)
		return ret;

	for (i = 0; i < BCAP_MAX_FMTS; i++) {
		if (mbus_fmt.code == bcap_formats[i].mbus_code) {
			bcap_fmt = &bcap_formats[i];
			v4l2_fill_pix_format(pixfmt, &mbus_fmt);
			pixfmt->bytesperline = pixfmt->width * bcap_fmt->bpp;
			pixfmt->sizeimage = pixfmt->bytesperline
						* pixfmt->height;
			return 0;
		}
	}
	v4l2_err(&bcap_dev->v4l2_dev,
			"subdev fmt is not supported by bcap\n");
	return -EINVAL;
}

static int bcap_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	struct v4l2_mbus_framefmt mbus_fmt;
	enum v4l2_mbus_pixelcode mbus_code;
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
	int ret, bpp;

	/* if streaming is started, return error */
	if (bcap_dev->started) {
		v4l2_err(&bcap_dev->v4l2_dev, "Streaming is started\n");
		return -EBUSY;
	}

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	/* see if format works */
	ret = bcap_try_format(bcap_dev, pixfmt, &mbus_code, &bpp);
	if (ret < 0)
		return ret;

	v4l2_fill_mbus_format(&mbus_fmt, pixfmt, mbus_code);
	ret = v4l2_subdev_call(bcap_dev->sd, video, s_mbus_fmt, &mbus_fmt);
	if (ret < 0)
		return ret;
	bcap_dev->fmt = *pixfmt;
	bcap_dev->bpp = bpp;
	return 0;
}

static int bcap_querycap(struct file *file, void  *priv,
				struct v4l2_capability *cap)
{
	struct bcap_device *bcap_dev = video_drvdata(file);

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	strlcpy(cap->driver, CAPTURE_DRV_NAME, sizeof(cap->driver));
	strlcpy(cap->bus_info, "Blackfin Platform", sizeof(cap->bus_info));
	strlcpy(cap->card, bcap_dev->cfg->card_name, sizeof(cap->card));
	return 0;
}

static int bcap_cropcap(struct file *file, void *priv,
			struct v4l2_cropcap *crop)
{
	struct bcap_device *bcap_dev = video_drvdata(file);

	if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	return v4l2_subdev_call(bcap_dev->sd, video, cropcap, crop);
}

static int bcap_g_chip_ident(struct file *file, void *priv,
		struct v4l2_dbg_chip_ident *chip)
{
	struct bcap_device *bcap_dev = video_drvdata(file);

	chip->ident = V4L2_IDENT_NONE;
	chip->revision = 0;
	if (chip->match.type != V4L2_CHIP_MATCH_I2C_DRIVER &&
			chip->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	return v4l2_subdev_call(bcap_dev->sd, core,
			g_chip_ident, chip);
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int bcap_dbg_g_register(struct file *file, void *priv,
		struct v4l2_dbg_register *reg)
{
	struct bcap_device *bcap_dev = video_drvdata(file);

	return v4l2_subdev_call(bcap_dev->sd, core,
			g_register, reg);
}

static int bcap_dbg_s_register(struct file *file, void *priv,
		struct v4l2_dbg_register *reg)
{
	struct bcap_device *bcap_dev = video_drvdata(file);

	return v4l2_subdev_call(bcap_dev->sd, core,
			s_register, reg);
}
#endif

static int bcap_log_status(struct file *file, void *priv)
{
	struct bcap_device *bcap_dev = video_drvdata(file);
	/* status for sub devices */
	v4l2_device_call_all(&bcap_dev->v4l2_dev, 0, core, log_status);
	return 0;
}

static const struct v4l2_ioctl_ops bcap_ioctl_ops = {
	.vidioc_querycap         = bcap_querycap,
	.vidioc_g_fmt_vid_cap    = bcap_g_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap = bcap_enum_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap    = bcap_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap  = bcap_try_fmt_vid_cap,
	.vidioc_enum_input       = bcap_enum_input,
	.vidioc_g_input          = bcap_g_input,
	.vidioc_s_input          = bcap_s_input,
	.vidioc_querystd         = bcap_querystd,
	.vidioc_s_std            = bcap_s_std,
	.vidioc_g_std            = bcap_g_std,
	.vidioc_queryctrl        = bcap_queryctrl,
	.vidioc_g_ctrl           = bcap_g_ctrl,
	.vidioc_s_ctrl           = bcap_s_ctrl,
	.vidioc_reqbufs          = bcap_reqbufs,
	.vidioc_querybuf         = bcap_querybuf,
	.vidioc_qbuf             = bcap_qbuf,
	.vidioc_dqbuf            = bcap_dqbuf,
	.vidioc_streamon         = bcap_streamon,
	.vidioc_streamoff        = bcap_streamoff,
	.vidioc_cropcap          = bcap_cropcap,
	.vidioc_g_chip_ident     = bcap_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_register       = bcap_dbg_g_register,
	.vidioc_s_register       = bcap_dbg_s_register,
#endif
	.vidioc_log_status       = bcap_log_status,
};

static struct v4l2_file_operations bcap_fops = {
	.owner = THIS_MODULE,
	.open = bcap_open,
	.release = bcap_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = bcap_mmap,
	.poll = bcap_poll
};

static int bcap_probe(struct platform_device *pdev)
{
	struct bcap_device *bcap_dev;
	struct video_device *vfd;
	struct i2c_adapter *i2c_adap;
	struct bfin_capture_config *config;
	int ret;

	bcap_dev = kzalloc(sizeof(*bcap_dev), GFP_KERNEL);
	if (!bcap_dev) {
		v4l2_err(pdev->dev.driver, "Unable to alloc bcap_dev\n");
		return -ENOMEM;
	}

	config = pdev->dev.platform_data;
	if (!config) {
		v4l2_err(pdev->dev.driver, "Unable to get board config\n");
		ret = -ENODEV;
		goto err;
	}
	bcap_dev->cfg = config;

	bcap_dev->ppi = bfin_get_ppi_if();
	if (!bcap_dev->ppi) {
		v4l2_err(pdev->dev.driver, "Unable to get ppi\n");
		ret = -ENODEV;
		goto err;
	}
	bcap_dev->ppi->priv = bcap_dev;

	vfd = video_device_alloc();
	if (!vfd) {
		ret = -ENOMEM;
		v4l2_err(pdev->dev.driver, "Unable to alloc video device\n");
		goto err;
	}

	/* initialize field of video device */
	vfd->release            = video_device_release;
	vfd->fops               = &bcap_fops;
	vfd->ioctl_ops          = &bcap_ioctl_ops;
	vfd->tvnorms            = 0;
	vfd->v4l2_dev           = &bcap_dev->v4l2_dev;
	strncpy(vfd->name, CAPTURE_DRV_NAME, sizeof(vfd->name));
	bcap_dev->video_dev     = vfd;

	ret = v4l2_device_register(&pdev->dev, &bcap_dev->v4l2_dev);
	if (ret) {
		v4l2_err(pdev->dev.driver,
				"Unable to register v4l2 device\n");
		goto err1;
	}
	v4l2_info(&bcap_dev->v4l2_dev, "v4l2 device registered\n");

	spin_lock_init(&bcap_dev->irqlock);
	mutex_init(&bcap_dev->lock);
	vfd->lock = &bcap_dev->lock;
	/* initialize prio member of device object */
	v4l2_prio_init(&bcap_dev->prio);

	/* register video device */
	ret = video_register_device(bcap_dev->video_dev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(&bcap_dev->v4l2_dev,
				"Unable to register video device\n");
		goto err2;
	}
	video_set_drvdata(bcap_dev->video_dev, bcap_dev);
	v4l2_info(&bcap_dev->v4l2_dev, "video device registered\n");

	/* load up the subdevice */
	i2c_adap = i2c_get_adapter(config->i2c_adapter_id);
	if (!i2c_adap) {
		v4l2_err(&bcap_dev->v4l2_dev,
				"Unable to find i2c adapter\n");
		goto err3;

	}
	bcap_dev->sd = v4l2_i2c_new_subdev_board(&bcap_dev->v4l2_dev,
						 i2c_adap,
						 &config->board_info,
						 NULL);
	if (bcap_dev->sd) {
		int i;
		/* update tvnorms from the sub devices */
		for (i = 0; i < config->num_inputs; i++)
			vfd->tvnorms |= config->inputs[i].std;
	} else {
		v4l2_err(&bcap_dev->v4l2_dev,
				"Unable to register sub device\n");
		goto err3;
	}
	v4l2_info(&bcap_dev->v4l2_dev, "v4l2 sub device registered\n");
	return 0;
err3:
	video_unregister_device(bcap_dev->video_dev);
err2:
	v4l2_device_unregister(&bcap_dev->v4l2_dev);
err1:
	video_device_release(bcap_dev->video_dev);
err:
	kfree(bcap_dev);
	return ret;
}

static int __devexit bcap_remove(struct platform_device *pdev)
{
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct bcap_device *bcap_dev = container_of(v4l2_dev,
						struct bcap_device, v4l2_dev);

	v4l2_device_unregister(v4l2_dev);
	video_device_release(bcap_dev->video_dev);
	kfree(bcap_dev);
	return 0;
}

static struct platform_driver bcap_driver = {
	.driver = {
		.name   = CAPTURE_DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = bcap_probe,
	.remove = __devexit_p(bcap_remove),
};

static __init int bcap_init(void)
{
	return platform_driver_register(&bcap_driver);
}

static void bcap_exit(void)
{
	platform_driver_unregister(&bcap_driver);
}

module_init(bcap_init);
module_exit(bcap_exit);

MODULE_DESCRIPTION("Analog Devices video capture driver");
MODULE_AUTHOR("Scott Jiang");
MODULE_LICENSE("GPL v2");

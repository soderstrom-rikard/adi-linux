/*
 * File:         drivers/media/video/blackfin/vs6524.c
 * Based on:
 * Author:       Michael Hennerich <hennerich@blackfin.uclinux.org>
 *
 * Created:
 * Description:  Command driver for STM VS6524 sensor
 *
 *
 * Modified:
 *               Copyright 2004-2007 Analog Devices Inc.
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

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/wait.h>
#include <media/v4l2-dev.h>

#include "vs6524.h"

static DEFINE_MUTEX(vs6524_sysfs_lock);

static int WriteByte(struct i2c_client *client,
		     unsigned short offset, unsigned char data)
{
	u8 buf[3];

	BUG_ON(client == NULL);

	buf[0] = offset >> 8;
	buf[1] = offset & 0xFF;
	buf[2] = data;

	i2c_master_send(client, buf, 3);

	return 0;
}

static int ReadByte(struct i2c_client *client, unsigned short offset,
		    unsigned char *data)
{
	u8 buf[2];

	BUG_ON(client == NULL);

	buf[0] = offset >> 8;
	buf[1] = offset & 0xFF;

	i2c_master_send(client, buf, 2);
	i2c_master_recv(client, data, 1);

	return 0;
}

static unsigned char get_reg(struct i2c_client *client, unsigned short offset)
{
	u8 buf[1];

	ReadByte(client, offset, &buf[0]);

	return buf[0];
}

static int vs_probe(struct i2c_client *client)
{

	u8 buf[2];

	ReadByte(client, DEVICEID_MSB, &buf[0]);
	ReadByte(client, DEVICEID_LSB, &buf[1]);

	if (((buf[0] << 8) | (buf[1] & 0xFF)) == VS6524_ID)
		return 0;

	return -ENODEV;

}

static int vs_set_pixfmt(struct i2c_client *client, u32 arg)
{

	switch (arg) {
	case VIDEO_PALETTE_UYVY:
		WriteByte(client, BDATAFORMAT0, 0x0);	/* YCbCr_JFIF */
		WriteByte(client, BYUVSETUP, 0x1);	/* CrYCbY */
		break;
	case VIDEO_PALETTE_RGB565:
		WriteByte(client, BDATAFORMAT0, 0x3);	/* RGB565 */
		WriteByte(client, BRGBSETUP, 0x1);	/*SWAP R-B */
		break;
	case VIDEO_PALETTE_YUV422:
		WriteByte(client, BDATAFORMAT0, 0x0);	/* YCbCr_JFIF */
		WriteByte(client, BYUVSETUP, 0x3);	/* YCbYCr */
		break;
	default:

		return -ENOIOCTLCMD;
	}

	return 0;

}

static int vs_set_framerate(struct i2c_client *client, u32 arg)
{

	if (arg <= MAX_FRAMERATE) {

		if (arg == 0)
			arg = MAX_FRAMERATE;	/* 0 widely means max fps */

		WriteByte(client, UWDESIREDFRAMERATE_NUM_MSB, MSB(arg));
		WriteByte(client, UWDESIREDFRAMERATE_NUM_LSB, LSB(arg));
		WriteByte(client, BDESIREDFRAMERATE_DEN, 0x1);

		return 0;
	}

	return -EPERM;

}

static int vs_get_framerate(struct i2c_client *client, u32 arg)
{

	u8 buf[3];
	int ret;

	ReadByte(client, UWDESIREDFRAMERATE_NUM_MSB, &buf[0]);
	ReadByte(client, UWDESIREDFRAMERATE_NUM_LSB, &buf[1]);
	ReadByte(client, BDESIREDFRAMERATE_DEN, &buf[2]);

	ret = (((buf[0] << 8) | buf[1]) / buf[2]);

	return ret;

}

static int vs_set_window(struct i2c_client *client, u32 res)
{
	WriteByte(client, BIMAGESIZE0, 0x4);

	WriteByte(client, BCROPHSTARTMSB0,
		  MSB((MAX_FRAME_WIDTH - X_RES(res)) / 2));
	WriteByte(client, BCROPHSTARTLSB0,
		  LSB((MAX_FRAME_WIDTH - X_RES(res)) / 2));

	WriteByte(client, BCROPVSTARTMSB0,
		  MSB((MAX_FRAME_HEIGHT - Y_RES(res)) / 2));
	WriteByte(client, BCROPVSTARTLSB0,
		  LSB((MAX_FRAME_HEIGHT - Y_RES(res)) / 2));

	WriteByte(client, BCROPHSIZEMSB0, MSB(X_RES(res)));
	WriteByte(client, BCROPHSIZELSB0, LSB(X_RES(res)));

	WriteByte(client, BCROPVSIZEMSB0, MSB(Y_RES(res)));
	WriteByte(client, BCROPVSIZELSB0, LSB(Y_RES(res)));

	WriteByte(client, FENABLECROP0, 0x1);

	return 0;
}

static int vs_set_resolution(struct i2c_client *client, u32 res)
{

	switch (res) {
	case RES_VGA:
		WriteByte(client, FENABLECROP0, 0x0);
		WriteByte(client, BSUBSAMPLE0, 0x1);
		WriteByte(client, BIMAGESIZE0, 0x1);
		break;
	case RES_QVGA:
		WriteByte(client, FENABLECROP0, 0x0);
		WriteByte(client, BSUBSAMPLE0, 0x1);
		WriteByte(client, BIMAGESIZE0, 0x2);
		break;
	case RES_QQVGA:
		WriteByte(client, FENABLECROP0, 0x0);
		WriteByte(client, BSUBSAMPLE0, 0x1);
		WriteByte(client, BIMAGESIZE0, 0x3);
		break;
	case RES_CIF:
		vs_set_window(client, RES_CIF);
		WriteByte(client, BSUBSAMPLE0, 0x1);
		break;
	case RES_QCIF:
		vs_set_window(client, RES_CIF);
		WriteByte(client, BSUBSAMPLE0, 0x2);
		break;
	case RES_SQCIF:
		vs_set_window(client, RES_CIF);
		WriteByte(client, BSUBSAMPLE0, 0x4);
		break;
	default:
		vs_set_window(client, res);
		WriteByte(client, BSUBSAMPLE0, 0x1);
	}

	return 0;
}

static int vs_init(struct i2c_client *client, u32 arg)
{

	WriteByte(client, MICROENABLE, 0x6);
	mdelay(1);

	WriteByte(client, ENABLE_IO, 0x1);

#ifdef USE_ITU656
	WriteByte(client, BCODECHECKEN, 0x7);	/* allow all */
	WriteByte(client, BDATAFORMAT0, 0x1);	/* REC601 */

	WriteByte(client, BSYNCCODESETUP, 0x19);	/* ITU656 TOGGLE 1,2 */
	WriteByte(client, BSYNCCODESETUP, 0x9);	/* ITU656 TOGGLE 1,2 */

#else
	WriteByte(client, BCODECHECKEN, 0x0);	/* allow all */
	WriteByte(client, BRGBSETUP, 0x1);	/*SWAP R-B */
	WriteByte(client, BDATAFORMAT0, 0x3);	/* RGB565 */
	WriteByte(client, BSYNCCODESETUP, 0x1);	/* SYNC */

	WriteByte(client, BHSYNCSETUP, 0xF);	/* Active lines only, Automatic */

#endif

	WriteByte(client, BUSERCOMMAND, 0x2);	/* RUN */

	vs_set_framerate(client, MAX_FRAMERATE);	/* set max frame rate */

	if (vs_probe(client))
		return -ENODEV;

	printk(KERN_INFO "%s: Firmware Version %d.%d \n",
	       SENSOR_NAME, get_reg(client, BFIRMWAREVSNMAJOR),
	       get_reg(client, BFIRMWAREVSNMINOR));
	printk(KERN_INFO "%s: Patch Version %d.%d \n",
	       SENSOR_NAME, get_reg(client, BPATCHVSNMAJOR),
	       get_reg(client, BPATCHVSNMINOR));

	return 0;

}

static int vs_exit(struct i2c_client *client, u32 arg)
{

	WriteByte(client, BUSERCOMMAND, 0x4);	/* STOP */
	WriteByte(client, ENABLE_IO, 0x0);
	WriteByte(client, MICROENABLE, 0x0);

	return 0;

}

int cam_control(struct i2c_client *client, u32 cmd, u32 arg)
{
	switch (cmd) {
	case CAM_CMD_INIT:
		return vs_init(client, arg);
	case CAM_CMD_SET_RESOLUTION:
		return vs_set_resolution(client, arg);
	case CAM_CMD_SET_FRAMERATE:
		return vs_set_framerate(client, arg);
	case CAM_CMD_SET_PIXFMT:
		return vs_set_pixfmt(client, arg);
	case CAM_CMD_EXIT:
		return vs_exit(client, arg);
	case CAM_CMD_SET_CONTRAST:
		WriteByte(client, BCONTRAST0, arg & 0xFF);
		break;
	case CAM_CMD_SET_SATURATION:
		WriteByte(client, BCOLOURSATURATION0, arg & 0xFF);
		break;
	case CAM_CMD_SET_HOR_MIRROR:
		WriteByte(client, FHORIZONTALMIRROR0, arg & 0x1);
		break;
	case CAM_CMD_SET_VERT_MIRROR:
		WriteByte(client, FVERTICALFLIP0, arg & 0x1);
		break;
	case CAM_CMD_SET_FLICKER_FREQ:
		WriteByte(client, BLIGHTINGFREQUENCYHZ, 2 * arg);
		break;
	case CAM_CMD_GET_FRAMERATE:
		(*(unsigned char *)(arg)) = vs_get_framerate(client, 0);
		break;
	case CAM_CMD_GET_FLICKER_FREQ:
		(*(unsigned char *)(arg)) =
		    get_reg(client, BLIGHTINGFREQUENCYHZ) / 2;
		break;
	case CAM_CMD_GET_VERT_MIRROR:
		(*(unsigned char *)(arg)) =
		    get_reg(client, FVERTICALFLIP0) & 0x1;
		break;
	case CAM_CMD_GET_HOR_MIRROR:
		(*(unsigned char *)(arg)) =
		    get_reg(client, FHORIZONTALMIRROR0) & 0x1;
		break;
	case CAM_CMD_GET_SATURATION:
		(*(unsigned char *)(arg)) = get_reg(client, BCOLOURSATURATION0);
		break;
	case CAM_CMD_GET_CONTRAST:
		(*(unsigned char *)(arg)) = get_reg(client, BCONTRAST0);
		break;
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

/****************************************************************************
 *  sysfs
 ***************************************************************************/

static u8 sysfs_strtou8(const char *buff, size_t len, ssize_t * count)
{
	char str[5];
	char *endp;
	unsigned long val;

	if (len < 4) {
		strncpy(str, buff, len);
		str[len + 1] = '\0';
	} else {
		strncpy(str, buff, 4);
		str[4] = '\0';
	}

	val = simple_strtoul(str, &endp, 0);

	*count = 0;
	if (val <= 0xff)
		*count = (ssize_t) (endp - str);
	if ((*count) && (len == *count + 1) && (buff[*count] == '\n'))
		*count += 1;

	return (u8) val;
}

static ssize_t sysfs_sysfs_show_val(struct class_device *cd, char *buf, int cmd)
{
	struct bcap_device_t *cam;
	ssize_t count;
	u8 val[1];

	if (mutex_lock_interruptible(&vs6524_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));
	if (!cam) {
		mutex_unlock(&vs6524_sysfs_lock);
		return -ENODEV;
	}

	if (cam_control(cam->client, cmd, (u32) val) < 0) {
		mutex_unlock(&vs6524_sysfs_lock);
		return -EIO;
	}

	count = sprintf(buf, "%d\n", val[0]);

	mutex_unlock(&vs6524_sysfs_lock);

	return count;
}

static ssize_t
sysfs_sysfs_store_val(struct class_device *cd, const char *buf, size_t len,
		      int cmd)
{
	struct bcap_device_t *cam;
	u8 value;
	ssize_t count;
	int err;

	if (mutex_lock_interruptible(&vs6524_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));

	if (!cam) {
		mutex_unlock(&vs6524_sysfs_lock);
		return -ENODEV;
	}

	value = sysfs_strtou8(buf, len, &count);

	if (!count) {
		mutex_unlock(&vs6524_sysfs_lock);
		return -EINVAL;
	}

	err = cam_control(cam->client, cmd, value);

	if (err) {
		mutex_unlock(&vs6524_sysfs_lock);
		return -EIO;
	}

	mutex_unlock(&vs6524_sysfs_lock);

	return count;
}

static ssize_t sysfs_fps_show(struct class_device *cd, char *buf)
{

	return sysfs_sysfs_show_val(cd, buf, CAM_CMD_GET_FRAMERATE);
}

static ssize_t
sysfs_fps_store(struct class_device *cd, const char *buf, size_t len)
{
	return sysfs_sysfs_store_val(cd, buf, len, CAM_CMD_SET_FRAMERATE);
}

static CLASS_DEVICE_ATTR(fps, S_IRUGO | S_IWUSR,
			 sysfs_fps_show, sysfs_fps_store);

static ssize_t sysfs_flicker_show(struct class_device *cd, char *buf)
{
	return sysfs_sysfs_show_val(cd, buf, CAM_CMD_GET_FLICKER_FREQ);
}

static ssize_t
sysfs_flicker_store(struct class_device *cd, const char *buf, size_t len)
{
	return sysfs_sysfs_store_val(cd, buf, len, CAM_CMD_SET_FLICKER_FREQ);
}

static CLASS_DEVICE_ATTR(flicker, S_IRUGO | S_IWUSR,
			 sysfs_flicker_show, sysfs_flicker_store);

static ssize_t sysfs_h_mirror_show(struct class_device *cd, char *buf)
{
	return sysfs_sysfs_show_val(cd, buf, CAM_CMD_GET_HOR_MIRROR);
}

static ssize_t
sysfs_h_mirror_store(struct class_device *cd, const char *buf, size_t len)
{
	return sysfs_sysfs_store_val(cd, buf, len, CAM_CMD_SET_HOR_MIRROR);
}

static CLASS_DEVICE_ATTR(h_mirror, S_IRUGO | S_IWUSR,
			 sysfs_h_mirror_show, sysfs_h_mirror_store);

static ssize_t sysfs_v_mirror_show(struct class_device *cd, char *buf)
{
	return sysfs_sysfs_show_val(cd, buf, CAM_CMD_GET_VERT_MIRROR);
}

static ssize_t
sysfs_v_mirror_store(struct class_device *cd, const char *buf, size_t len)
{
	return sysfs_sysfs_store_val(cd, buf, len, CAM_CMD_SET_VERT_MIRROR);
}

static CLASS_DEVICE_ATTR(v_mirror, S_IRUGO | S_IWUSR,
			 sysfs_v_mirror_show, sysfs_v_mirror_store);

static int vs6524_create_sysfs(struct video_device *v4ldev)
{
	int rc;

	rc = video_device_create_file(v4ldev, &class_device_attr_fps);
	if (rc)
		goto err;
	rc = video_device_create_file(v4ldev, &class_device_attr_flicker);
	if (rc)
		goto err_flicker;
	rc = video_device_create_file(v4ldev, &class_device_attr_v_mirror);
	if (rc)
		goto err_v_mirror;
	rc = video_device_create_file(v4ldev, &class_device_attr_h_mirror);
	if (rc)
		goto err_h_mirror;

	return 0;

err_h_mirror:
	video_device_remove_file(v4ldev, &class_device_attr_v_mirror);
err_v_mirror:
	video_device_remove_file(v4ldev, &class_device_attr_flicker);
err_flicker:
	video_device_remove_file(v4ldev, &class_device_attr_fps);
err:
	return rc;
}

static struct bcap_camera_ops vs6524_ops = {
	cam_control,
	vs6524_create_sysfs,
	NULL,
};

struct bcap_camera_ops *get_camops(void)
{
	return (&vs6524_ops);

}
EXPORT_SYMBOL(get_camops);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");

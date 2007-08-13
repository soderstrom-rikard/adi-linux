/*
 * File:         drivers/media/video/blackfin/mt9v022.c
 * Based on:
 * Author:       Michael Hennerich <hennerich@blackfin.uclinux.org>
 *
 * Created:
 * Description:  Command driver for Micron MT9V022 sensor
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

#include "mt9v022.h"

static DEFINE_MUTEX(mt9v022_sysfs_lock);


static int ReadWord(struct i2c_client *client, unsigned char offset,
				u16 * data)
{
	u8 buf[2];

	BUG_ON(client == NULL);

	i2c_smbus_write_byte(client, offset);
	i2c_master_recv(client, buf, 2);

	*data = buf[0] << 8 | buf[1];

	return 0;
}

static int WriteWord(struct i2c_client *client,
				 unsigned char offset, unsigned short data)
{
	u8 buf[3];

	BUG_ON(client == NULL);

	buf[0] = offset;
	buf[1] = data >> 8;
	buf[2] = data & 0xFF;

	i2c_master_send(client, buf, 3);

	return 0;
}

static unsigned short get_reg(struct i2c_client *client, unsigned char offset)
{
	u16 buf[1];

	ReadWord(client, offset, &buf[0]);

	return buf[0];
}

static int mt_probe(struct i2c_client *client)
{

/*
	u8 buf[2];

	ReadWord(client, DEVICEID_MSB, &buf[0]);

	if (((buf[0] << 8) | (buf[1] & 0xFF)) == MT9V022_ID)
		return 0;

	return -ENODEV;
*/
	return 0;

}

static int mt_set_pixfmt(struct i2c_client *client, u32 arg)
{
	return 0;
}

static int mt_set_framerate(struct i2c_client *client, u32 arg)
{

	return -EPERM;

}

static int mt_get_framerate(struct i2c_client *client, u32 arg)
{

	return -EPERM;

}

static int mt_set_window(struct i2c_client *client, u32 res)
{
	WriteWord(client, 0x01,
		       COL_OFF_MIN + (MAX_FRAME_HEIGHT - Y_RES(res)) / 2);

	WriteWord(client, 0x02,
		       ROW_OFF_MIN + (MAX_FRAME_WIDTH - X_RES(res)) / 2);

	return 0;
}

static int mt_set_resolution(struct i2c_client *client, u32 res)
{

	switch (res) {

	default:
		mt_set_window(client, res);

	}

	return 0;
}

static int mt_init(struct i2c_client *client, u32 arg)
{


	if (mt_probe(client))
		return -ENODEV;


	return 0;

}

static int mt_exit(struct i2c_client *client, u32 arg)
{
	return 0;
}

int cam_control(struct i2c_client *client, u32 cmd, u32 arg)
{
	switch (cmd) {
	case CAM_CMD_INIT:
		return mt_init(client, arg);
	case CAM_CMD_SET_RESOLUTION:
		return mt_set_resolution(client, arg);
	case CAM_CMD_SET_FRAMERATE:
		return mt_set_framerate(client, arg);
	case CAM_CMD_SET_PIXFMT:
		return mt_set_pixfmt(client, arg);
	case CAM_CMD_EXIT:
		return mt_exit(client, arg);
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

	if (mutex_lock_interruptible(&mt9v022_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));
	if (!cam) {
		mutex_unlock(&mt9v022_sysfs_lock);
		return -ENODEV;
	}

	if (cam_control(cam->client, cmd, (u32) val) < 0) {
		mutex_unlock(&mt9v022_sysfs_lock);
		return -EIO;
	}

	count = sprintf(buf, "%d\n", val[0]);

	mutex_unlock(&mt9v022_sysfs_lock);

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

	if (mutex_lock_interruptible(&mt9v022_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));

	if (!cam) {
		mutex_unlock(&mt9v022_sysfs_lock);
		return -ENODEV;
	}

	value = sysfs_strtou8(buf, len, &count);

	if (!count) {
		mutex_unlock(&mt9v022_sysfs_lock);
		return -EINVAL;
	}

	err = cam_control(cam->client, cmd, value);

	if (err) {
		mutex_unlock(&mt9v022_sysfs_lock);
		return -EIO;
	}

	mutex_unlock(&mt9v022_sysfs_lock);

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

static int mt9v022_create_sysfs(struct video_device *v4ldev)
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

static struct bcap_camera_ops mt9v022_ops = {
	cam_control,
	mt9v022_create_sysfs,
	NULL,
};

struct bcap_camera_ops *get_camops(void)
{
	return (&mt9v022_ops);

}
EXPORT_SYMBOL(get_camops);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");

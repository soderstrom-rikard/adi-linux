/*
 * File:         drivers/media/video/blackfin/st_sensor.c
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

static int vs_probe(struct i2c_client *client)
{

	u8 buf[2];	

	ReadByte(client, DEVICEID_MSB, &buf[0]);
	ReadByte(client, DEVICEID_LSB, &buf[1]);

//	printk(" %x %x \n",buf[0],buf[1]);

	if(((buf[0] << 8) | (buf[1] & 0xFF)) == VS6524_ID)
		return 0;

	return -ENODEV;

}

static int vs_init(struct i2c_client *client, u32 arg)
{


	WriteByte(client, MICROENABLE, 0x6);
	mdelay(1);

	WriteByte(client, ENABLE_IO, 0x1);

	WriteByte(client, BUSERCOMMAND, 0x2); /* RUN */
	WriteByte(client, BDATAFORMAT0, 0x3); /* RGB565 */

	WriteByte(client, BCODECHECKEN, 0x0); /* allow all */
	WriteByte(client, BSYNCCODESETUP, 0x1); /* mode 2 */
	WriteByte(client, BRGBSETUP, 0x1); /*SWAP R-B*/

	if (vs_probe(client))
		return -ENODEV;

	return 0;

}

static int vs_exit(struct i2c_client *client, u32 arg)
{

	WriteByte(client, BUSERCOMMAND, 0x4); /* STOP */
	WriteByte(client, ENABLE_IO, 0x0);
	WriteByte(client, MICROENABLE, 0x0);
	
	return 0;

}

static int vs_set_pixfmt(struct i2c_client *client, u32 arg)
{

	switch (arg) {
	case VIDEO_PALETTE_UYVY:
		WriteByte(client, BDATAFORMAT0, 0x0); /* YCbCr_JFIF */
		WriteByte(client, BYUVSETUP, 0x1); /* CrYCbY */
		break;
	case VIDEO_PALETTE_RGB565:
		WriteByte(client, BDATAFORMAT0, 0x3); /* RGB565 */
		WriteByte(client, BRGBSETUP, 0x1); /*SWAP R-B*/
		break;
	case VIDEO_PALETTE_YUV422:
		WriteByte(client, BDATAFORMAT0, 0x0); /* YCbCr_JFIF */
		WriteByte(client, BYUVSETUP, 0x3); /* YCbYCr */
		break;
	default:

		return -ENOIOCTLCMD;
	}

	return 0;

}

static int vs_set_framerate(struct i2c_client *client, u32 arg)
{
	
	WriteByte(client, UWDESIREDFRAMERATE_NUM_MSB,  MSB(arg));
	WriteByte(client, UWDESIREDFRAMERATE_NUM_LSB, LSB(arg));
	WriteByte(client, BDESIREDFRAMERATE_DEN, 0x1);		

	return 0;

}

static int vs_set_window(struct i2c_client *client, u32 res)
{
	WriteByte(client, BIMAGESIZE0, 0x4);
		
	WriteByte(client, BCROPHSTARTMSB0, MSB((MAX_FRAME_WIDTH - X_RES(res))/2));
	WriteByte(client, BCROPHSTARTLSB0, LSB((MAX_FRAME_WIDTH - X_RES(res))/2));

	WriteByte(client, BCROPVSTARTMSB0, MSB((MAX_FRAME_HEIGHT - Y_RES(res))/2));
	WriteByte(client, BCROPVSTARTLSB0, LSB((MAX_FRAME_HEIGHT - Y_RES(res))/2));

	WriteByte(client, BCROPHSIZEMSB0, MSB(X_RES(res)));
	WriteByte(client, BCROPHSIZELSB0, LSB(X_RES(res)));
		
	WriteByte(client, BCROPVSIZEMSB0, MSB(Y_RES(res)));
	WriteByte(client, BCROPVSIZELSB0, LSB(Y_RES(res)));
		
	WriteByte(client, FENABLECROP0, 0x1);

	return 0;
}

static int  vs_set_resolution(struct i2c_client *client, u32 res)
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
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}
EXPORT_SYMBOL(cam_control);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");

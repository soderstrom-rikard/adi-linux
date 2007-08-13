/*
 * File:         drivers/media/video/blackfin/blackfin_cam.h
 * Based on:
 * Author:       Michael Hennerich <hennerich@blackfin.uclinux.org>
 *
 * Created:
 * Description:  V4L driver for Blackfin
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

#ifndef BLACKFIN_CAM_H
#define BLACKFIN_CAM_H

#undef USE_ITU656
#undef USE_2ND_BUF_IN_CACHED_MEM
#define USE_PPI_ERROR
#undef USE_GPIO

#if defined(CONFIG_BF537)
# define  bcap_STANDBY  GPIO_PG11
# define  bcap_LEDS     GPIO_PG8
# define  bcap_TRIGGER  GPIO_PG13
#endif

#if defined(CONFIG_BF533)
# define  bcap_STANDBY  GPIO_8
# define  bcap_LEDS     GPIO_11
# define  bcap_TRIGGER  GPIO_6
# define  bcap_FS3      GPIO_3
#endif

#define DRV_NAME	"blackfin-cam"

#define PPI0_8 {P_PPI0_CLK, P_PPI0_D0, P_PPI0_D1, P_PPI0_D2, P_PPI0_D3, \
 P_PPI0_D4, P_PPI0_D5, P_PPI0_D6, P_PPI0_D7, P_PPI0_FS1, P_PPI0_FS2, 0}

#define BCAP_NUM_BUFS 2
#define VID_HARDWARE_BCAP  13	/* experimental */
#define I2C_DRIVERID_BCAP  81	/* experimental (next avail. in i2c-id.h) */
#define NO_TRIGGER  16

#define X_RES(x) (x >> 16)
#define Y_RES(x) (x & 0xFFFF)
#define MSB(x)(x >> 8)
#define LSB(x)(x & 0xFF)

#define RES(x, y)	(x << 16) | (y & 0xFFFF)

#define RES_VGA		RES(640, 480)
#define RES_QVGA	RES(320, 240)
#define RES_QQVGA	RES(160, 120)

#define RES_CIF		RES(352, 288)
#define RES_QCIF	RES(176, 144)
#define RES_SQCIF	RES(128, 96)

/* Controls */
enum {
	CAM_CMD_INIT,
	CAM_CMD_SET_RESOLUTION,
	CAM_CMD_SET_FRAMERATE,
	CAM_CMD_SET_PIXFMT,
	CAM_CMD_EXIT,
	CAM_CMD_SET_CONTRAST,
	CAM_CMD_SET_SATURATION,
	CAM_CMD_SET_HOR_MIRROR,
	CAM_CMD_SET_VERT_MIRROR,
	CAM_CMD_SET_FLICKER_FREQ,
	CAM_CMD_GET_FRAMERATE,
	CAM_CMD_GET_FLICKER_FREQ,
	CAM_CMD_GET_VERT_MIRROR,
	CAM_CMD_GET_HOR_MIRROR,
	CAM_CMD_GET_SATURATION,
	CAM_CMD_GET_CONTRAST,
};

struct ppi_device_t {
	struct bcap_device_t *bcap_dev;
	int opened;
	unsigned short irqnum;
	unsigned short done;
	unsigned short dma_config;
	unsigned short pixel_per_line;
	unsigned short lines_per_frame;
	unsigned short bpp;
	unsigned short ppi_control;
	unsigned short ppi_status;
	unsigned short ppi_delay;
	unsigned short ppi_trigger_gpio;
	wait_queue_head_t *rx_avail;
};

struct bcap_buffer {
	unsigned char *data;
	wait_queue_head_t wq;
	volatile int state;
	unsigned int scyc;	/* < start cycle for frame  */
	unsigned int ecyc;	/* < end cycle for frame    */
	unsigned long stime;	/* < start time for frame   */
	unsigned long etime;	/* < end time for frame     */
};

/*
 * States for each frame buffer.
 */
enum {
	FRAME_UNUSED = 0,	/* < Unused                           */
	FRAME_READY = 1,	/* < Ready to start grabbing          */
	FRAME_GRABBING = 2,	/* < Grabbing the frame               */
	FRAME_DONE = 3,		/* < Grabbing done, frame not synced  */
	FRAME_ERROR = 4,	/* < Error                            */
};

struct bcap_device_t {
	int frame_count;
	struct video_device *videodev;
	struct ppi_device_t *ppidev;
	struct i2c_client *client;
	struct bcap_camera_ops *cam_ops;
	int user;
	char name[32];
	int width;
	int height;
	size_t size;
	struct timeval *stv;
	struct timeval *etv;
	struct bcap_buffer buffer[BCAP_NUM_BUFS];
	struct bcap_buffer *next_buf;
	struct bcap_buffer *dma_buf;
	struct bcap_buffer *ready_buf;
	spinlock_t lock;
};

struct sensor_data {
	struct i2c_client client;
	struct bcap_device_t *bcap_dev;
	struct bcap_camera_ops *cam_ops;
};

struct bcap_camera_ops {
	int (*cam_control) (struct i2c_client * client, u32 cmd, u32 arg);
	int (*create_sysfs) (struct video_device * v4ldev);
	int (*power) (u32 arg);
};

#endif				/* BLACKFIN_CAM_H */

/*
 * File:         drivers/media/video/blackfin/mt9v022.h
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

#ifndef MT9V022_H
#define MT9V022_H

#include "blackfin_cam.h"

/*0 = VIDEO_PALETTE_GREY
  1 = VIDEO_PALETTE_RGB565
  2 = VIDEO_PALETTE_YUV422
  3 = VIDEO_PALETTE_UYVY */

#define DEFAULT_FORMAT		0

# define POL_C              	0x0000
# define POL_S              	0x0000
# define PIXEL_PER_LINE     	720
# define LINES_PER_FRAME    	488
# define CFG_GP_Input_3Syncs	0x0020
# define GP_Input_Mode      	0x000C
# define PPI_DATA_LEN       	DLEN_8
# define PPI_PACKING        	PACK_EN
# define DMA_FLOW_MODE      	0x0000	/* STOPMODE */
# define DMA_WDSIZE_16      	WDSIZE_16


#define I2C_SENSOR_ID		(0x5C << 1)
#define MAX_FRAME_WIDTH		752
#define MAX_FRAME_HEIGHT	480
#define MIN_FRAME_WIDTH		80
#define MIN_FRAME_HEIGHT	60
#define DEFAULT_DEPTH		8
#define CORR_VAL		0

#define ROW_OFF_MIN		4
#define COL_OFF_MIN		1


#define SENSOR_NAME 		"MT9V022"

struct bcap_camera_ops *get_camops(void);

#define MAX_FRAMERATE		30


#endif				/* MT9V022_H */

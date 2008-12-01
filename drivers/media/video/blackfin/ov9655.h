/*
 * File:         drivers/media/video/blackfin/ov9655.h
 * Based on:
 * Author:       Martin Strubel <hackfin@section5.ch>
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

#ifndef OV9655_H
#define OV9655_H

#include "blackfin_cam.h"

/*0 = VIDEO_PALETTE_GREY
  1 = VIDEO_PALETTE_RGB565
  2 = VIDEO_PALETTE_YUV422
  3 = VIDEO_PALETTE_UYVY */

#define DEFAULT_FORMAT		3

# define POL_C              	0x0000
# define POL_S              	0x0000
# define PIXEL_PER_LINE     	1280
# define LINES_PER_FRAME    	1024
# define CFG_GP_Input_3Syncs	0x0020
# define GP_Input_Mode      	0x000C
# define PPI_DATA_LEN       	DLEN_8
# define PPI_PACKING        	PACK_EN
# define DMA_FLOW_MODE      	0x0000	/* STOPMODE */
# define DMA_WDSIZE_16      	WDSIZE_16


#define I2C_SENSOR_ID		(0x30 << 1)
#define MAX_FRAME_WIDTH		1280
#define MAX_FRAME_HEIGHT	1024
#define MIN_FRAME_WIDTH		80
#define MIN_FRAME_HEIGHT	60
#define DEFAULT_DEPTH		16
#define CORR_VAL		0

#define ROW_OFF_MIN		4
#define COL_OFF_MIN		1

#define USE_2ND_BUF_IN_CACHED_MEM

#define SENSOR_NAME 		"OV9655"

struct bcap_camera_ops *get_camops(void);

#define MAX_FRAMERATE		30


#endif				/* OV9655_H */

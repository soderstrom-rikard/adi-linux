/*
 * File:         drivers/media/video/blackfin/adv7183b.h
 * Based on:     drivers/media/video/blackfin/mt9v022.h
 * Author:       Taha Iali
 *
 * Created:      may 2008
 * Description:  Command driver for Analog Devices ADV7183B sensor
 *
 *
 * Modified:
 *               Copyright 2008 Taha Iali
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

#ifndef ADV7183B_H
#define ADV7183B_H

#include "blackfin_cam.h"

/*0 = VIDEO_PALETTE_GREY
  1 = VIDEO_PALETTE_RGB565
  2 = VIDEO_PALETTE_YUV422
  3 = VIDEO_PALETTE_UYVY */

/* This should go into the platform resources when the Blackfin V4L framwork will use it. */
#ifdef CONFIG_BFIN561_EZKIT
# define ADV7183B_GPIO_RESET     GPIO_PF13
# define ADV7183B_GPIO_OE        GPIO_PF2
# undef  ADV7183B_28MHZ
# undef  ADV7183B_STRONG
#endif
#ifdef CONFIG_PRESTO_VIDEO
# undef  ADV7183B_GPIO_RESET
# define ADV7183B_GPIO_OE        GPIO_PF8
# define ADV7183B_28MHZ
# define ADV7183B_STRONG
#else
# ifdef CONFIG_BFIN533_EZKIT
#  undef  ADV7183B_GPIO_RESET
#  define ADV7183B_GPIO_OE        GPIO_PF2
#  undef  ADV7183B_28MHZ
#  undef  ADV7183B_STRONG
# endif
#endif

#define USE_ITU656
#define DEFAULT_FORMAT		3

# define POL_C              	0x0000
# define POL_S              	0x0000
# define PIXEL_PER_LINE     	720
# define LINES_PER_FRAME    	576
# define CFG_GP_Input_3Syncs	0x0000
# define GP_Input_Mode      	0x0000 /*ITU-R 656 Active Field Only*/
# define PPI_DATA_LEN       	DLEN_8
# define DMA_FLOW_MODE      	0x0000	/* STOPMODE */
#ifdef CONFIG_BF561
# define DMA_WDSIZE_16      	WDSIZE_32
# define PPI_PACKING        	(PACK_EN | DMA32)
#else
# define DMA_WDSIZE_16      	WDSIZE_16
# define PPI_PACKING        	PACK_EN
#endif


#define I2C_SENSOR_ID		0x40
#define MAX_FRAME_WIDTH		720
#define MAX_FRAME_HEIGHT	576
#define MIN_FRAME_WIDTH		80
#define MIN_FRAME_HEIGHT	60
#define DEFAULT_DEPTH		16
#define CORR_VAL		0

#define ROW_OFF_MIN		4
#define COL_OFF_MIN		1


#define SENSOR_NAME 		"ADV7183B"

struct bcap_camera_ops *get_camops(void);

#define MAX_FRAMERATE		25


#endif				/* ADV7183B_H */

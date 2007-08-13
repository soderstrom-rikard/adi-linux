/*
 * File:         drivers/media/video/blackfin/vs6524.h
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

#ifndef VS6524_H
#define VS6524_H

#include "blackfin_cam.h"

/*0 = VIDEO_PALETTE_GREY
  1 = VIDEO_PALETTE_RGB565
  2 = VIDEO_PALETTE_YUV422
  3 = VIDEO_PALETTE_UYVY */

#define DEFAULT_FORMAT 	3

#define POL_C              	0x0000
#define POL_S              	0x0000
#define PIXEL_PER_LINE     	640
#define LINES_PER_FRAME    	480
#define PPI_DATA_LEN       	DLEN_8
#define PPI_PACKING        	PACK_EN
#define DMA_FLOW_MODE      	0x0000	/* STOPMODE */
#define DMA_WDSIZE_16      	WDSIZE_16

#ifdef USE_ITU656
# define CFG_GP_Input_3Syncs	0x0000
# define GP_Input_Mode      	0x0040
#else
# define CFG_GP_Input_3Syncs	0x0020
# define GP_Input_Mode      	0x000C
#endif

#define I2C_SENSOR_ID  (0x20)
#define DEFAULT_DEPTH 16

#define SENSOR_NAME "VS6524"

struct bcap_camera_ops *get_camops(void);

#define MAX_FRAME_WIDTH  640
#define MAX_FRAME_HEIGHT 480
#define MIN_FRAME_WIDTH    80
#define MIN_FRAME_HEIGHT   60

#define MAX_FRAMERATE  30

#define VS6524_ID 				 524

#define MICROENABLE				 0xC003
#define ENABLE_IO				 0xC034

#define DEVICEID_MSB				 0x0001
#define DEVICEID_LSB				 0x0002
#define BFIRMWAREVSNMAJOR			 0x0004
#define BFIRMWAREVSNMINOR			 0x0006
#define BPATCHVSNMAJOR				 0x0008
#define BPATCHVSNMINOR				 0x000a

#define BUSERCOMMAND				 0x0180

#define BSTATE				 	 0x0202
#define FMETERINGON				 0x0280
#define FEXITONSTABLE				 0x0282
#define UWEXTERNALCLOCKFREQUENCYMHZNUMERATOR	 0x060b
#define BEXTERNALCLOCKFREQUENCYMHZDENOMINATOR	 0x060e
#define BENABLEGLOBALSYSTEMCLOCKDIVISION	 0x0610

#define BTIMETOPOWERDOWN			 0x0580

#define UWDESIREDFRAMERATE_NUM_MSB		 0x0d81
#define UWDESIREDFRAMERATE_NUM_LSB		 0x0d82
#define BDESIREDFRAMERATE_DEN			 0x0d84

#define BIMAGESIZE0				 0x0380
#define BSUBSAMPLE0				 0x0382
#define FENABLECROP0				 0x0384
#define UWCROPHSTART0				 0x0387
#define UWCROPVSTART0				 0x038f
#define UWCROPHSIZE0				 0x038b
#define UWCROPVSIZE0				 0x0393
#define BCROPHSTARTMSB0				 0x0387
#define BCROPHSTARTLSB0				 0x0388
#define BCROPVSTARTMSB0				 0x038f
#define BCROPVSTARTLSB0				 0x0390
#define BCROPHSIZEMSB0				 0x038b
#define BCROPHSIZELSB0				 0x038c
#define BCROPVSIZEMSB0				 0x0393
#define BCROPVSIZELSB0				 0x0394
#define BDATAFORMAT0				 0x0396
#define BBAYEROUTPUTALIGNMENT0			 0x0398
#define BCONTRAST0				 0x039a
#define BCOLOURSATURATION0			 0x039c
#define BGAMMA0				 	 0x039e
#define FHORIZONTALMIRROR0			 0x03a0
#define FVERTICALFLIP0				 0x03a2

#define BIMAGESIZE1				 0x0400
#define BSUBSAMPLE1				 0x0402
#define FENABLECROP1				 0x0404
#define UWCROPHSTART1				 0x0407
#define UWCROPVSTART1				 0x040f
#define UWCROPHSIZE1				 0x040b
#define UWCROPVSIZE1				 0x0413
#define BCROPHSTARTMSB1				 0x0407
#define BCROPHSTARTLSB1				 0x0408
#define BCROPVSTARTMSB1				 0x040f
#define BCROPVSTARTLSB1				 0x0410
#define BCROPHSIZEMSB1				 0x040b
#define BCROPHSIZELSB1				 0x040c
#define BCROPVSIZEMSB1				 0x0413
#define BCROPVSIZELSB1				 0x0414
#define BDATAFORMAT1				 0x0416
#define BBAYEROUTPUTALIGNMENT1			 0x0418
#define BCONTRAST1				 0x041a
#define BCOLOURSATURATION1			 0x041c
#define BGAMMA1				 	 0x041e
#define FHORIZONTALMIRROR1			 0x0420
#define FVERTICALFLIP1				 0x0422

#define FENABLE				 	 0x0480
#define BINITIALPIPESETUPBANK			 0x0482

#define BWHITEBALANCEMODE			 0x1380
#define BMANUALREDGAIN				 0x1382
#define BMANUALGREENGAIN			 0x1384
#define BMANUALBLUEGAIN				 0x1386
#define FPREDGAINFORFLASHGUN			 0x138b
#define FPGREENGAINFORFLASHGUN			 0x138f
#define FPBLUEGAINFORFLASHGUN			 0x1393

#define BEXPOSUREMODE				 0x1080
#define BEXPOSUREMETERING			 0x1082
#define BMANUALEXPOSURETIME_NUM			 0x1084
#define BMANUALEXPOSURETIME_DEN			 0x1086
#define IEXPOSURECOMPENSATION			 0x1090
#define FFREEZEAUTOEXPOSURE			 0x10b4
#define FPUSERMAXIMUMINTEGRATIONTIME		 0x10b7

#define FPDESIREDEXPOSURETIME_US		 0x1219

#define BLEAKSHIFT				 0x113c

#define BFLASHMODE				 0x1780
#define UWFLASHOFFLINE				 0x1783

#define BANTIFLICKERMODE			 0x10c0
#define BLIGHTINGFREQUENCYHZ			 0x0c80
#define FFLICKERCOMPATIBLEFRAMELENGTH		 0x0c82

#define FPFLICKERFREQUENCY			 0x1901
#define FPFLICKERFREQUENCY			 0x1901

#define FDISABLESCYTHEFILTER			 0x1a80
#define FDISABLEJACKFILTER			 0x1b00

#define BUSERPEAKGAIN				 0x1d80
#define BUSERPEAKLOTHRESH			 0x1d90

#define FDISABLE				 0x2000
#define FPBLACKVALUE				 0x2003
#define FPDAMPERLOWTHRESHOLD			 0x2007
#define FPDAMPERHIGHTHRESHOLD			 0x200b
#define FPDAMPEROUTPUT				 0x200f
#define FPBLACKVALUE				 0x2003
#define FPDAMPERLOWTHRESHOLD			 0x2007
#define FPDAMPERHIGHTHRESHOLD			 0x200b
#define FPDAMPEROUTPUT				 0x200f

#define BDITHERBLOCKCONTROL			 0x2080

#define BCODECHECKEN				 0x2100
#define BBLANKFORMAT				 0x2102
#define BSYNCCODESETUP				 0x2104

#define BHSYNCSETUP				 0x2106

#define BVSYNCSETUP				 0x2108

#define BPCLKSETUP				 0x210a

#define FPCLKEN				 	 0x210c
#define BOPFSPSETUP				 0x210e
#define BBLANKDATA_MSB				 0x2110
#define BBLANKDATA_LSB				 0x2112
#define BRGBSETUP				 0x2114

#define BYUVSETUP				 0x2116

#define BVSYNCRISINGLINEH			 0x2118
#define BVSYNCRISINGLINEL			 0x211a
#define BVSYNCRISINGPIXELH			 0x211c
#define BVSYNCRISINGPIXELL			 0x211e
#define BVSYNCFALLINGLINEH			 0x2120
#define BVSYNCFALLINGLINEL			 0x2122
#define BVSYNCFALLINGPIXELH			 0x2124
#define BVSYNCFALLINGPIXELL			 0x2126
#define BHSYNCRISINGH				 0x2128
#define BHSYNCRISINGL				 0x212a
#define BHSYNCFALLINGH				 0x212c
#define BHSYNCFALLINGL				 0x212e

#define BBLACKCORRECTIONOFFSET			 0x1690

#define IR2COEFFICIENT				 0x1a08

/*#define FDISABLE                               0x1c80*/
#define FPLOWTHRESHOLD				 0x1c83
#define FPLOWTHRESHOLDLSB			 0x1c84
#define FPLOWTHRESHOLDMSB			 0x1c83
#define FPHIGHTHRESHOLD				 0x1c87
#define FPHIGHTHRESHOLDLSB			 0x1c88
#define FPHIGHTHRESHOLDMSB			 0x1c87
#define FPMINIMUMOUTPUT				 0x1c8b
#define FPMINIMUMOUTPUTLSB			 0x1c8c
#define FPMINIMUMOUTPUTMSB			 0x1c8b
#define FPDESIREDEXPOSURETIME_US		 0x1219

#define BUSERPEAKGAIN				 0x1d80
#define BUSERPEAKLOTHRESH			 0x1d90
#define FPDAMPERLOWTHRESHOLD_CORING		 0x1d97
#define FPDAMPERHIGHTHRESHOLD_CORING		 0x1d9b
#define FPMINIMUMDAMPEROUTPUT_CORING		 0x1d9f
#define FPMINIMUMDAMPEROUTPUT_CORINGLSB		 0x1da0
#define FPMINIMUMDAMPEROUTPUT_CORINGMSB		 0x1d9f
#define FPDESIREDEXPOSURETIME_US		 0x1219

#define FDISABLEFRAMERATEDAMPER			 0x128c
#define FPDAMPERLOWTHRESHOLD_FRAMERATE		 0x1281
#define FPDAMPERHIGHTHRESHOLD_FRAMERATE		 0x1285
#define MINIMUMDAMPEROUTPUT			 0x80e7
#define MINIMUMDAMPEROUTPUT_MSB			 0x80e7
#define MINIMUMDAMPEROUTPUT_LSB			 0x80e8
#define UNCLIPEDCOMPLIEDTIME			 0x1289

#endif				/* VS6524_H */

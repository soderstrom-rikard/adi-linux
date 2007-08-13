/*
 * File:         drivers/media/video/blackfin/vs6624.h
 * Based on:
 * Author:       Michael Hennerich <hennerich@blackfin.uclinux.org>
 *
 * Created:
 * Description:  Command driver for STM VS6624 sensor
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

#ifndef VS6624_H
#define VS6624_H

#include "blackfin_cam.h"

/*0 = VIDEO_PALETTE_GREY
  1 = VIDEO_PALETTE_RGB565
  2 = VIDEO_PALETTE_YUV422
  3 = VIDEO_PALETTE_UYVY */

#define DEFAULT_FORMAT 	3

#define VS6624_PWDN		GPIO_PF10

#undef USE_ITU656
#undef USE_2ND_BUF_IN_CACHED_MEM
#define USE_PPI_ERROR
#undef USE_PROC
#undef USE_GPIO

# define POL_C              	0x0000
# define POL_S              	0x0000
# define PIXEL_PER_LINE     	640
# define LINES_PER_FRAME    	480
# define PPI_DATA_LEN       	DLEN_8
# define PPI_PACKING        	PACK_EN
# define DMA_FLOW_MODE      	0x0000	/* STOPMODE */
# define DMA_WDSIZE_16      	WDSIZE_16

#ifdef USE_ITU656
# define CFG_GP_Input_3Syncs	0x0000
# define GP_Input_Mode      	0x0040
#else
# define CFG_GP_Input_3Syncs	0x0020
# define GP_Input_Mode      	0x000C
#endif

#define I2C_SENSOR_ID  (0x20)
#define DEFAULT_DEPTH 16

#define SENSOR_NAME "VS6624"

struct bcap_camera_ops *get_camops(void);

#define MAX_FRAME_WIDTH  640
#define MAX_FRAME_HEIGHT 480
#define MIN_FRAME_WIDTH    80
#define MIN_FRAME_HEIGHT   60

#define MAX_FRAMERATE  30

#define VS6624_ID 				 624
#define PWR_MAN_SETUP_MODE_SELECT		0xc003	/* (7:0) */
#define PWR_MAN_DIO_ENABLE			0xc044	/* (7:0) */
#define uwDeviceId				0x0001	/* (7:0) IndexLo 0x0002 */
#define DeviceID_MSB				0x0001	/* (7:0) */
#define DeviceID_LSB				0x0002	/* (7:0) */
#define bFirmwareVsnMajor			0x0004	/* (7:0) */
#define bFirmwareVsnMinor			0x0006	/* (7:0) */
#define bPatchVsnMajor				0x0008	/* (7:0) */
#define bPatchVsnMinor				0x000a	/* (7:0) */

#define bUserCommand				0x0180	/* (7:0) */
#define bManualNextState			0x0186	/* (7:0) */

#define bNextState				0x0200	/* (7:0) */
#define bState					0x0202	/* (7:0) */

#define fMeteringOn				0x0280	/* (7:0) */
#define fExitOnStable				0x0282	/* (7:0) */
#define bStreamLength				0x0284	/* (7:0) */

#define fIsColdStart				0x0300	/* (7:0) */
#define bNonViewLive_ActivePipeSetupBank	0x0302	/* (7:0) */
#define bSnapShoot_ActivePipeSetupBank		0x0304	/* (7:0) */
#define fSnapShoot_NoWaiting			0x0306	/* (7:0) */
#define SensorMode				0x0308	/* (7:0) */

#define bImageSize0				0x0380	/* (7:0) */
#define uwManualHSize0				0x0383	/* (7:0) IndexLo 0x0384 */
#define uwManualHSizeMSB0			0x0383	/* (7:0) */
#define uwManualHSizeLSB0			0x0384	/* (7:0) */
#define uwManualVSize0				0x0387	/* (7:0) IndexLo 0x0388 */
#define uwManualVSizeMSB0			0x0387	/* (7:0) */
#define uwManualVSizeLSB0			0x0388	/* (7:0) */
#define uwZoomStepHSize0			0x038b	/* (7:0) IndexLo 0x038c */
#define uwZoomStepHSizeMSB0			0x038b	/* (7:0) */
#define uwZoomStepHSizeLSB0			0x038c	/* (7:0) */
#define uwZoomStepVSize0			0x038f	/* (7:0) IndexLo 0x0390 */
#define uwZoomStepVSizeMSB0			0x038f	/* (7:0) */
#define uwZoomStepVSizeLSB0			0x0390	/* (7:0) */
#define bZoomControl0				0x0392	/* (7:0) */
#define uwPanStepHSize0				0x0395	/* (7:0) IndexLo 0x0396 */
#define uwPanStepHSizeMSB0			0x0395	/* (7:0) */
#define uwPanStepHSizeLSB0			0x0396	/* (7:0) */
#define uwPanStepVSize0				0x0399	/* (7:0) IndexLo 0x039a */
#define uwPanStepVSizeMSB0			0x0399	/* (7:0) */
#define uwPanStepVSizeLSB0			0x039a	/* (7:0) */
#define bPanControl0				0x039c	/* (7:0) */
#define bCropControl0				0x039e	/* (7:0) */
#define uwManualCropHorizontalStart0		0x03a1	/* (7:0) IndexLo 0x03a2 */
#define uwManualCropHorizontalSize0		0x03a5	/* (7:0) IndexLo 0x03a6 */
#define uwManualCropVerticalStart0		0x03a9	/* (7:0) IndexLo 0x03aa */
#define uwManualCropVerticalSize0		0x03ad	/* (7:0) IndexLo 0x03ae */
#define bCropHStartMSB0				0x03a1	/* (7:0) */
#define bCropHStartLSB0				0x03a2	/* (7:0) */
#define bCropVStartMSB0				0x03a9	/* (7:0) */
#define bCropVStartLSB0				0x03aa	/* (7:0) */
#define bCropHSizeMSB0				0x03a5	/* (7:0) */
#define bCropHSizeLSB0				0x03a6	/* (7:0) */
#define bCropVSizeMSB0				0x03ad	/* (7:0) */
#define bCropVSizeLSB0				0x03ae	/* (7:0) */
#define bDataFormat0				0x03b0	/* (7:0) */
#define bBayerOutputAlignment0			0x03b2	/* (7:0) */
#define bContrast0				0x03b4	/* (7:0) */
#define bColourSaturation0			0x03b6	/* (7:0) */
#define bGamma0					0x03b8	/* (7:0) */
#define fHorizontalMirror0			0x03ba	/* (7:0) */
#define fVerticalFlip0				0x03bc	/* (7:0) */
#define bChannelID0				0x03be	/* (7:0) */

#define bImageSize1				0x0400	/* (7:0) */
#define uwManualHSize1				0x0403	/* (7:0) IndexLo 0x0404 */
#define uwManualVSize1				0x0407	/* (7:0) IndexLo 0x0408 */
#define uwZoomStepHSize1			0x040b	/* (7:0) IndexLo 0x040c */
#define uwZoomStepVSize1			0x040f	/* (7:0) IndexLo 0x0410 */
#define bZoomControl1				0x0412	/* (7:0) */
#define uwPanStepHSize1				0x0415	/* (7:0) IndexLo 0x0416 */
#define uwPanStepVSize1				0x0419	/* (7:0) IndexLo 0x041a */
#define bPanControl1				0x041c	/* (7:0) */
#define bCropControl1				0x041e	/* (7:0) */
#define uwManualCropHorizontalStart1		0x0421	/* (7:0) IndexLo 0x0422 */
#define uwManualCropHorizontalSize1		0x0425	/* (7:0) IndexLo 0x0426 */
#define uwManualCropVerticalStart1		0x0429	/* (7:0) IndexLo 0x042a */
#define uwManualCropVerticalSize1		0x042d	/* (7:0) IndexLo 0x042e */
#define bCropHStartMSB1				0x0421	/* (7:0) */
#define bCropHStartLSB1				0x0422	/* (7:0) */
#define bCropVStartMSB1				0x0429	/* (7:0) */
#define bCropVStartLSB1				0x042a	/* (7:0) */
#define bCropHSizeMSB1				0x0425	/* (7:0) */
#define bCropHSizeLSB1				0x0426	/* (7:0) */
#define bCropVSizeMSB1				0x042d	/* (7:0) */
#define bCropVSizeLSB1				0x042e	/* (7:0) */
#define bDataFormat1				0x0430	/* (7:0) */
#define bBayerOutputAlignment1			0x0432	/* (7:0) */
#define bContrast1				0x0434	/* (7:0) */
#define bColourSaturation1			0x0436	/* (7:0) */
#define bGamma1					0x0438	/* (7:0) */
#define fHorizontalMirror1			0x043a	/* (7:0) */
#define fVerticalFlip1				0x043c	/* (7:0) */
#define bChannelID1				0x043e	/* (7:0) */

#define fEnable					0x0480	/* (7:0) */
#define bInitialPipeSetupBank			0x0482	/* (7:0) */

#define CurrentPipeSetupBank			0x0500	/* (7:0) */

#define bTimeToPowerdown			0x0580	/* (7:0) */
#define fVRegSleep				0x058a	/* (7:0) */
#define fSmoothLineReading			0x058c	/* (7:0) */

#define uwExternalClockFrequencyMhzNumerator	0x0605	/* (7:0) IndexLo 0x0606 */
#define bExternalClockFrequencyMhzDenominator	0x0608	/* (7:0) */

#define fpExternalClockFrequencyMhz		0x0681	/* (7:0) IndexLo 0x0682 */

#define bSysClkMode				0x0880	/* (7:0) */
#if 0
#define bMode					0x0882	/* (7:0) */
#endif
#define bLightingFrequencyHz			0x0c80	/* (7:0) */
#define fFlickerCompatibleFrameLength		0x0c82	/* (7:0) */

#define fpFlickerFreePeriod_us			0x0d05	/* (7:0) IndexLo 0x0d06 */
#define fAntiFlickerEnabled			0x0d08	/* (7:0) */

#define uwDesiredFrameRate_Num			0x0d81	/* (7:0) IndexLo 0x0d82 */
#define uwDesiredFrameRate_Num_MSB		0x0d81	/* (7:0) */
#define uwDesiredFrameRate_Num_LSB		0x0d82	/* (7:0) */
#define bDesiredFrameRate_Den			0x0d84	/* (7:0) */

#define fpRequestedFrameRate_Hz			0x0e01	/* (7:0) IndexLo 0x0e02 */
#define fpRequestedFrameRate_Hz_MSB		0x0e01	/* (7:0) */
#define fpRequestedFrameRate_Hz_LSB		0x0e02	/* (7:0) */
#define fpMaxFrameRate_Hz			0x0e05	/* (7:0) IndexLo 0x0e06 */
#define fpMinFrameRate_Hz			0x0e09	/* (7:0) IndexLo 0x0e0a */
#define fChangePending				0x0e0c	/* (7:0) */
#define uwRequiredFrameLength_lines		0x0e0f	/* (7:0) IndexLo 0x0e10 */
#define ClipFrameRate				0x0e12	/* (7:0) */

#define fDisableFrameRateDamper			0x0e80	/* (7:0) */
#define bImpliedGainThresholdLow_num		0x0e82	/* (7:0) */
#define bImpliedGainThresholdLow_den		0x0e84	/* (7:0) */
#define bImpliedGainThresholdHigh_num		0x0e86	/* (7:0) */
#define bImpliedGainThresholdHigh_den		0x0e88	/* (7:0) */
#define bUserMinimumFrameRate_Hz		0x0e8a	/* (7:0) */
#define bUserMaximumFrameRate_Hz		0x0e8c	/* (7:0) */
#define bRelativeChange_num			0x0e8e	/* (7:0) */
#define bRelativeChange_den			0x0e90	/* (7:0) */
#define fDivorceMinFrameRateFromMaxIntegration	0x0e92	/* (7:0) */

#define fpImpliedGain				0x0f01	/* (7:0) IndexLo 0x0f02 */
#define uwMaximumFrameLength_lines		0x0f05	/* (7:0) IndexLo 0x0f06 */
#define uwMinimumFrameLength_lines		0x0f09	/* (7:0) IndexLo 0x0f0a */
#define uwFrameLengthChange_lines		0x0f0d	/* (7:0) IndexLo 0x0f0e */
#define fpDesiredAutomaticFrameRate_Hz		0x0f11	/* (7:0) IndexLo 0x0f12 */
#define uwCurrentFrameLength_lines		0x0f15	/* (7:0) IndexLo 0x0f16 */
#define uwDesiredFrameLength_lines		0x0f19	/* (7:0) IndexLo 0x0f1a */
#define fAutomaticFrameRateStable		0x0f1c	/* (7:0) */
#define fAutomaticFrameRateClip			0x0f1e	/* (7:0) */

#define uwXOffset				0x0f81	/* (7:0) IndexLo 0x0f82 */
#define uwYOffset				0x0f85	/* (7:0) IndexLo 0x0f86 */
#define uwXSize					0x0f89	/* (7:0) IndexLo 0x0f8a */
#define uwYSize					0x0f8d	/* (7:0) IndexLo 0x0f8e */

#define ExposureControls_bMode			0x1180	/* (7:0) */
#define bExposureMetering			0x1182	/* (7:0) */
#define bManualExposureTime_s_num		0x1184	/* (7:0) */
#define bManualExposureTime_s_den		0x1186	/* (7:0) */
#define fpManualDesiredExposureTime_us		0x1189	/* (7:0) IndexLo 0x118a */
#define iExposureCompensation			0x1190	/* (7:0) Signed */
#define uwDirectModeCoarseIntegration_lines	0x1195	/* (7:0) IndexLo 0x1196 */
#define uwDirectModeFineIntegration_pixels	0x1199	/* (7:0) IndexLo 0x119a */
#define uwDirectModeCodedAnalogGain		0x119d	/* (7:0) IndexLo 0x119e */
#define fpDirectModeDigitalGain			0x11a1	/* (7:0) IndexLo 0x11a2 */
#define uwFlashGunModeCoarseIntegration_lines	0x11a5	/* (7:0) IndexLo 0x11a6 */
#define uwFlashGunModeFineIntegration_pixels	0x11a9	/* (7:0) IndexLo 0x11aa */
#define uwFlashGunModeCodedAnalogGain		0x11ad	/* (7:0) IndexLo 0x11ae */
#define fpFlashGunModeDigitalGain		0x11b1	/* (7:0) IndexLo 0x11b2 */
#define fFreezeAutoExposure			0x11b4	/* (7:0) */
#define fpUserMaximumIntegrationTime_us		0x11b7	/* (7:0) IndexLo 0x11b8 */
#define fpRecommendFlashGunAnalogGainThreshold	0x11bb	/* (7:0) IndexLo 0x11bc */
#define fEnableHighClipForDesiredExposureTime	0x11be	/* (7:0) */
#define bAntiFlickerMode			0x11c0	/* (7:0) */

#define fpMaximumStep				0x1201	/* (7:0) IndexLo 0x1202 */
#define fpMinimumStep				0x1205	/* (7:0) IndexLo 0x1206 */
#define fpMinimumDesiredExposureTime_us		0x1209	/* (7:0) IndexLo 0x120a */
#define fpStepProportion			0x120d	/* (7:0) IndexLo 0x120e */
#define fpMaximumNegativeStepThreshold		0x1211	/* (7:0) IndexLo 0x1212 */
#define fpRelativeOnTargetStabilityThreshold	0x1215	/* (7:0) IndexLo 0x1216 */
#define fpDigitalGainFloor			0x1219	/* (7:0) IndexLo 0x121a */
#define fpDigitalGainCeiling			0x121d	/* (7:0) IndexLo 0x121e */
#define fpRelativeIntTimeHysThreshold		0x1221	/* (7:0) IndexLo 0x1222 */
#define fpRelativeDigitalGainHysThreshold	0x1225	/* (7:0) IndexLo 0x1226 */
#define fpRelativeCompilationProblemThreshold	0x1229	/* (7:0) IndexLo 0x122a */
#define fpRoundUpBunchFudge			0x122d	/* (7:0) IndexLo 0x122e */
#define fpFineClampThreshold			0x1231	/* (7:0) IndexLo 0x1232 */
#define fpMaximumManualExposureTime_s		0x1235	/* (7:0) IndexLo 0x1236 */
#define fpRelativeStabilityThresholdForAutoFocus 0x1239	/*  (7:0) IndexLo 0x123a */
#define bLeakShift				0x123c	/* (7:0) */

#define fpLeakyEnergy				0x1281	/* (7:0) IndexLo 0x1282 */
#define fpRelativeStep				0x1285	/* (7:0) IndexLo 0x1286 */

#define uwCoarseIntegrationPending_lines	0x1309	/* (7:0) IndexLo 0x130a */
#define uwFineIntegrationPending_pixels		0x130d	/* (7:0) IndexLo 0x130e */
#define fpAnalogGainPending			0x1311	/* (7:0) IndexLo 0x1312 */
#define fpDigitalGainPending			0x1315	/* (7:0) IndexLo 0x1316 */
#define fpDesiredExposureTime_us		0x1319	/* (7:0) IndexLo 0x131a */
#define fpCompiledExposureTime_us		0x131d	/* (7:0) IndexLo 0x131e */
#define uwCodedAnalogGainPending		0x132b	/* (7:0) IndexLo 0x132c */

#define bWhiteBalanceMode			0x1480	/* (7:0) */
#define bManualRedGain				0x1482	/* (7:0) */
#define bManualGreenGain			0x1484	/* (7:0) */
#define bManualBlueGain				0x1486	/* (7:0) */
#define fpFlashRedGain				0x148b	/* (7:0) IndexLo 0x148c */
#define fpFlashGreenGain			0x148f	/* (7:0) IndexLo 0x1490 */
#define fpFlashBlueGain				0x1493	/* (7:0) IndexLo 0x1494 */

#define bStatus					0x1500	/* (7:0) */
#define fpRedGain				0x1505	/* (7:0) IndexLo 0x1506 */
#define fpGreenGain				0x1509	/* (7:0) IndexLo 0x150a */
#define fpBlueGain				0x150d	/* (7:0) IndexLo 0x150e */

#define fpStableTotalStepThreshold		0x1581	/* (7:0) IndexLo 0x1582 */
#define fpMinimumRelativeStep			0x1585	/* (7:0) IndexLo 0x1586 */
#define fpMaximumRelativeStep			0x1589	/* (7:0) IndexLo 0x158a */
/*#define fpStepProportion                      0x158d*/ /* (7:0) IndexLo 0x158e */

#define fpRedA					0x1601	/* (7:0) IndexLo 0x1602 */
#define fpBlueA					0x1605	/* (7:0) IndexLo 0x1606 */
#define fpRedB					0x1609	/* (7:0) IndexLo 0x160a */
#define fpBlueB					0x160d	/* (7:0) IndexLo 0x160e */
#define fpMaximumDistanceAllowedFromLocus	0x1611	/* (7:0) IndexLo 0x1612 */
#define fEnableConstrainedWhiteBalance		0x1614	/* (7:0) */
#define bACCSRCCtrl				0x1616	/* (7:0) */

#define fpOutputRedGain				0x1681	/* (7:0) IndexLo 0x1682 */
#define fpOutputGreenGain			0x1685	/* (7:0) IndexLo 0x1686 */
#define fpOutputBlueGain			0x1689	/* (7:0) IndexLo 0x168a */
#define fAreGainsConstrained			0x168c	/* (7:0) */

#define fpGradientOfLocusAB			0x1701	/* (7:0) IndexLo 0x1702 */
#define fpDistanceOfInputPointFromLocusAB	0x1705	/* (7:0) IndexLo 0x1706 */
#define fpConstrainedRedPoint			0x1709	/* (7:0) IndexLo 0x170a */
#define fpConstrainedBluePoint			0x170d	/* (7:0) IndexLo 0x170e */

#define bMaxNumberOfFramesToWaitForStability	0x1880	/* (7:0) */

#define fWhiteBalanceStable			0x1900	/* (7:0) */
#define fExposureStable				0x1902	/* (7:0) */
#define fDarkCalStable				0x1904	/* (7:0) */
#define fStable					0x1906	/* (7:0) */
#define fForcedStablility			0x1908	/* (7:0) */

#define fpRedTilt				0x1985	/* (7:0) IndexLo 0x1986 */
#define fpGreenTilt				0x1989	/* (7:0) IndexLo 0x198a */
#define fpBlueTilt				0x198d	/* (7:0) IndexLo 0x198e */
#define bBlackCorrectionOffset			0x1990	/* (7:0) */

#define uwSensorAnalogGainFloor			0x1a01	/* (7:0) IndexLo 0x1a02 */
#define uwSensorAnalogGainCeiling		0x1a05	/* (7:0) IndexLo 0x1a06 */

#define bFlashMode				0x1a80	/* (7:0) */
#define uwFlashOffLine				0x1a83	/* (7:0) IndexLo 0x1a84 */

#define fFlashRecommended			0x1b00	/* (7:0) */
#define fFlashGrabComplete			0x1b02	/* (7:0) */

#define uwHorizontalOffset			0x1d01	/* (7:0) IndexLo 0x1d02 */
#define uwVerticalOffset			0x1d05	/* (7:0) IndexLo 0x1d06 */
#define iR2RCoefficient				0x1d08	/* (7:0) Signed */
#define iR2GRCoefficient			0x1d0a	/* (7:0) Signed */
#define iR2GBCoefficient			0x1d0c	/* (7:0) Signed */
#define iR2BCoefficient				0x1d0e	/* (7:0) Signed */
#define iR4RCoefficient				0x1d10	/* (7:0) Signed */
#define iR4GRCoefficient			0x1d12	/* (7:0) Signed */
#define iR4GBCoefficient			0x1d14	/* (7:0) Signed */
#define iR4BCoefficient				0x1d16	/* (7:0) Signed */

#define ScythefDisableFilter			0x1d80	/* (7:0) */
#define JackfDisableFilter			0x1e00	/* (7:0) */

#define bAntiAliasFilterSuppress		0x1e80	/* (7:0) */

#define ColourMatrixDamperfDisable		0x1f00	/* (7:0) */
#define fpLowThreshold				0x1f03	/* (7:0) IndexLo 0x1f04 */
#define fpHighThreshold				0x1f07	/* (7:0) IndexLo 0x1f08 */
#define fpMinimumOutput				0x1f0b	/* (7:0) IndexLo 0x1f0c */

#define fpGInR					0x1f81	/* (7:0) IndexLo 0x1f82 */
#define fpBInR					0x1f85	/* (7:0) IndexLo 0x1f86 */
#define fpRInG					0x1f89	/* (7:0) IndexLo 0x1f8a */
#define fpBInG					0x1f8d	/* (7:0) IndexLo 0x1f8e */
#define fpRInB					0x1f91	/* (7:0) IndexLo 0x1f92 */
#define fpGInB					0x1f95	/* (7:0) IndexLo 0x1f96 */

#define bUserPeakGain				0x2000	/* (7:0) */
#define fDisableGainDamping			0x2002	/* (7:0) */
#define fpDamperLowThreshold_Gain		0x2005	/* (7:0) IndexLo 0x2006 */
#define fpDamperHighThreshold_Gain		0x2009	/* (7:0) IndexLo 0x200a */
#define fpMinimumDamperOutput_Gain		0x200d	/* (7:0) IndexLo 0x200e */
#define bUserPeakLoThresh			0x2010	/* (7:0) */
#define fDisableCoringDamping			0x2012	/* (7:0) */
#define bUserPeakHiThresh			0x2014	/* (7:0) */
#define fpDamperLowThreshold_Coring		0x2017	/* (7:0) IndexLo 0x2018 */
#define fpDamperHighThreshold_Coring		0x201b	/* (7:0) IndexLo 0x201c */
#define fpMinimumDamperOutput_Coring		0x201f	/* (7:0) IndexLo 0x2020 */
#define bBlockControl				0x2022	/* (7:0) */

#if 0
#define w0_0					0x2081	/* (7:0) IndexLo 0x2082 */
#define w0_1					0x2085	/* (7:0) IndexLo 0x2086 */
#define w0_2					0x2089	/* (7:0) IndexLo 0x208a */
#define w1_0					0x208d	/* (7:0) IndexLo 0x208e */
#define w1_1					0x2091	/* (7:0) IndexLo 0x2092 */
#define w1_2					0x2095	/* (7:0) IndexLo 0x2096 */
#define w2_0					0x2099	/* (7:0) IndexLo 0x209a */
#define w2_1					0x209d	/* (7:0) IndexLo 0x209e */
#define w2_2					0x20a1	/* (7:0) IndexLo 0x20a2 */

#define i0					0x2101	/* (7:0) IndexLo 0x2102 */
#define i1					0x2105	/* (7:0) IndexLo 0x2106 */
#define i2					0x2109	/* (7:0) IndexLo 0x210a */

#define fRgbToYuvManuCtrl			0x2180	/* (7:0) */
#define w0_0					0x2183	/* (7:0) IndexLo 0x2184 */
#define w0_1					0x2187	/* (7:0) IndexLo 0x2188 */
#define w0_2					0x218b	/* (7:0) IndexLo 0x218c */
#define w1_0					0x218f	/* (7:0) IndexLo 0x2190 */
#define w1_1					0x2193	/* (7:0) IndexLo 0x2194 */
#define w1_2					0x2197	/* (7:0) IndexLo 0x2198 */
#define w2_0					0x219b	/* (7:0) IndexLo 0x219c */
#define w2_1					0x219f	/* (7:0) IndexLo 0x21a0 */
#define w2_2					0x21a3	/* (7:0) IndexLo 0x21a4 */
#define YinY					0x21a7	/* (7:0) IndexLo 0x21a8 */
#define YinCb					0x21ab	/* (7:0) IndexLo 0x21ac */
#define YinCr					0x21af	/* (7:0) IndexLo 0x21b0 */

#define fRgbToYuvManuCtrl			0x2200	/* (7:0) */
#define w0_0					0x2203	/* (7:0) IndexLo 0x2204 */
#define w0_1					0x2207	/* (7:0) IndexLo 0x2208 */
#define w0_2					0x220b	/* (7:0) IndexLo 0x220c */
#define w1_0					0x220f	/* (7:0) IndexLo 0x2210 */
#define w1_1					0x2213	/* (7:0) IndexLo 0x2214 */
#define w1_2					0x2217	/* (7:0) IndexLo 0x2218 */
#define w2_0					0x221b	/* (7:0) IndexLo 0x221c */
#define w2_1					0x221f	/* (7:0) IndexLo 0x2220 */
#define w2_2					0x2223	/* (7:0) IndexLo 0x2224 */
#define YinY					0x2227	/* (7:0) IndexLo 0x2228 */
#define YinCb					0x222b	/* (7:0) IndexLo 0x222c */
#define YinCr					0x222f	/* (7:0) IndexLo 0x2230 */
#endif

#define fGammaManuCtrl0				0x2280	/* (7:0) */
#define bRPeakGamma0				0x2282	/* (7:0) */
#define bGPeakGamma0				0x2284	/* (7:0) */
#define bBPeakGamma0				0x2286	/* (7:0) */
#define bRUnPeakGamma0				0x2288	/* (7:0) */
#define bGUnPeakGamma0				0x228a	/* (7:0) */
#define bBUnPeakGamma0				0x228c	/* (7:0) */

#define fGammaManuCtrl1				0x2300	/* (7:0) */
#define bRPeakGamma1				0x2302	/* (7:0) */
#define bGPeakGamma1				0x2304	/* (7:0) */
#define bBPeakGamma1				0x2306	/* (7:0) */
#define bRUnPeakGamma1				0x2308	/* (7:0) */
#define bGUnPeakGamma1				0x230a	/* (7:0) */
#define bBUnPeakGamma1				0x230c	/* (7:0) */

#define uwLumaExcursion0			0x2381	/* (7:0) IndexLo 0x2382 */
#define uwLumaMidpointTimes20			0x2385	/* (7:0) IndexLo 0x2386 */
#define uwChromaExcursion0			0x2389	/* (7:0) IndexLo 0x238a */
#define uwChromaMidpointTimes20			0x238d	/* (7:0) IndexLo 0x238e */

#define uwLumaExcursion1			0x2401	/* (7:0) IndexLo 0x2402 */
#define uwLumaMidpointTimes21			0x2405	/* (7:0) IndexLo 0x2406 */
#define uwChromaExcursion1			0x2409	/* (7:0) IndexLo 0x240a */
#define uwChromaMidpointTimes21			0x240d	/* (7:0) IndexLo 0x240e */

#define FadeToBlackfDisable			0x2480	/* (7:0) */
#define fpBlackValue				0x2483	/* (7:0) IndexLo 0x2484 */
#define fpDamperLowThreshold			0x2487	/* (7:0) IndexLo 0x2488 */
#define fpDamperHighThreshold			0x248b	/* (7:0) IndexLo 0x248c */
#define fpDamperOutput				0x248f	/* (7:0) IndexLo 0x2490 */

#define bCodeCheckEn				0x2580	/* (7:0) */
#define bBlankFormat				0x2582	/* (7:0) */
#define bSyncCodeSetup				0x2584	/* (7:0) */
#define bHSyncSetup				0x2586	/* (7:0) */
#define bVSyncSetup				0x2588	/* (7:0) */
#define bPClkSetup				0x258a	/* (7:0) */
#define fPclkEn					0x258c	/* (7:0) */
#define bOpfSpSetup				0x258e	/* (7:0) */
#define bBlankData_MSB				0x2590	/* (7:0) */
#define bBlankData_LSB				0x2592	/* (7:0) */
#define bRgbSetup				0x2594	/* (7:0) */
#define bYuvSetup				0x2596	/* (7:0) */
#define bVsyncRisingCoarseH			0x2598	/* (7:0) */
#define bVsyncRisingCoarseL			0x259a	/* (7:0) */
#define bVsyncRisingFineH			0x259c	/* (7:0) */
#define bVsyncRisingFineL			0x259e	/* (7:0) */
#define bVsyncFallingCoarseH			0x25a0	/* (7:0) */
#define bVsyncFallingCoarseL			0x25a2	/* (7:0) */
#define bVsyncFallingFineH			0x25a4	/* (7:0) */
#define bVsyncFallingFineL			0x25a6	/* (7:0) */
#define bHsyncRisingH				0x25a8	/* (7:0) */
#define bHsyncRisingL				0x25aa	/* (7:0) */
#define bHsyncFallingH				0x25ac	/* (7:0) */
#define bHsyncFallingL				0x25ae	/* (7:0) */
#define bOutputInterface			0x25b0	/* (7:0) */
#define bCCPExtraData				0x25b2	/* (7:0) */

#define NoRAfDisable				0x2600	/* (7:0) */
#define bUsage					0x2602	/* (7:0) */
#define bSplit_Kn				0x2604	/* (7:0) */
#define bSplit_Nl				0x2606	/* (7:0) */
#define bTight_Green				0x2608	/* (7:0) */
#define fDisableNoraPromoting			0x260a	/* (7:0) */
#define DamperLowThreshold			0x260d	/* (7:0) IndexLo 0x260e */
#define DamperHighThreshold			0x2611	/* (7:0) IndexLo 0x2612 */
#define MinimumDamperOutput			0x2615	/* (7:0) IndexLo 0x2616 */

static const u16 patch_p1[][2] = {
	{0x8104, 3},
	{0x8105, 1},
	{0xc900, 0x03},
	{0xc904, 0x47},
	{0xc905, 0x10},
	{0xc906, 0x80},
	{0xc907, 0x3a},
	{0x903a, 0x02},
	{0x903b, 0x47},
	{0x903c, 0x15},
	{0xc908, 0x31},
	{0xc909, 0xdc},
	{0xc90a, 0x80},
	{0xc90b, 0x44},
	{0x9044, 0x02},
	{0x9045, 0x31},
	{0x9046, 0xe2},
	{0xc90c, 0x07},
	{0xc90d, 0xe0},
	{0xc90e, 0x80},
	{0xc90f, 0x47},
	{0x9047, 0x90},
	{0x9048, 0x83},
	{0x9049, 0x81},
	{0x904a, 0xe0},
	{0x904b, 0x60},
	{0x904c, 0x08},
	{0x904d, 0x90},
	{0x904e, 0xc0},
	{0x904f, 0x43},
	{0x9050, 0x74},
	{0x9051, 0x01},
	{0x9052, 0xf0},
	{0x9053, 0x80},
	{0x9054, 0x05},
	{0x9055, 0xE4},
	{0x9056, 0x90},
	{0x9057, 0xc0},
	{0x9058, 0x43},
	{0x9059, 0xf0},
	{0x905a, 0x02},
	{0x905b, 0x07},
	{0x905c, 0xec},
	{0xc910, 0x5d},
	{0xc911, 0xca},
	{0xc912, 0x80},
	{0xc913, 0x5d},
	{0x905d, 0xa3},
	{0x905e, 0x04},
	{0x905f, 0xf0},
	{0x9060, 0xa3},
	{0x9061, 0x04},
	{0x9062, 0xf0},
	{0x9063, 0x22},
	{0xc914, 0x72},
	{0xc915, 0x92},
	{0xc916, 0x80},
	{0xc917, 0x64},
	{0x9064, 0x74},
	{0x9065, 0x01},
	{0x9066, 0x02},
	{0x9067, 0x72},
	{0x9068, 0x95},
	{0xc918, 0x47},
	{0xc919, 0xf2},
	{0xc91a, 0x81},
	{0xc91b, 0x69},
	{0x9169, 0x74},
	{0x916a, 0x02},
	{0x916b, 0xf0},
	{0x916c, 0xec},
	{0x916d, 0xb4},
	{0x916e, 0x10},
	{0x916f, 0x0a},
	{0x9170, 0x90},
	{0x9171, 0x80},
	{0x9172, 0x16},
	{0x9173, 0xe0},
	{0x9174, 0x70},
	{0x9175, 0x04},
	{0x9176, 0x90},
	{0x9177, 0xd3},
	{0x9178, 0xc4},
	{0x9179, 0xf0},
	{0x917a, 0x22},
	{0xc91c, 0x0a},
	{0xc91d, 0xbe},
	{0xc91e, 0x80},
	{0xc91f, 0x73},
	{0x9073, 0xfc},
	{0x9074, 0xa3},
	{0x9075, 0xe0},
	{0x9076, 0xf5},
	{0x9077, 0x82},
	{0x9078, 0x8c},
	{0x9079, 0x83},
	{0x907a, 0xa3},
	{0x907b, 0xa3},
	{0x907c, 0xe0},
	{0x907d, 0xfc},
	{0x907e, 0xa3},
	{0x907f, 0xe0},
	{0x9080, 0xc3},
	{0x9081, 0x9f},
	{0x9082, 0xff},
	{0x9083, 0xec},
	{0x9084, 0x9e},
	{0x9085, 0xfe},
	{0x9086, 0x02},
	{0x9087, 0x0a},
	{0x9088, 0xea},
	{0xc920, 0x47},
	{0xc921, 0x38},
	{0xc922, 0x80},
	{0xc923, 0x89},
	{0x9089, 0xec},
	{0x908a, 0xd3},
	{0x908b, 0x94},
	{0x908c, 0x20},
	{0x908d, 0x40},
	{0x908e, 0x01},
	{0x908f, 0x1c},
	{0x9090, 0x90},
	{0x9091, 0xd3},
	{0x9092, 0xd4},
	{0x9093, 0xec},
	{0x9094, 0xf0},
	{0x9095, 0x02},
	{0x9096, 0x47},
	{0x9097, 0x3d},
	{0xc924, 0x45},
	{0xc925, 0xca},
	{0xc926, 0x80},
	{0xc927, 0x98},
	{0x9098, 0x12},
	{0x9099, 0x77},
	{0x909a, 0xd6},
	{0x909b, 0x02},
	{0x909c, 0x45},
	{0x909d, 0xcd},
	{0xc928, 0x20},
	{0xc929, 0xd5},
	{0xc92a, 0x80},
	{0xc92b, 0x9e},
	{0x909e, 0x90},
	{0x909f, 0x82},
	{0x90a0, 0x18},
	{0x90a1, 0xe0},
	{0x90a2, 0xb4},
	{0x90a3, 0x03},
	{0x90a4, 0x0e},
	{0x90a5, 0x90},
	{0x90a6, 0x83},
	{0x90a7, 0xbf},
	{0x90a8, 0xe0},
	{0x90a9, 0x60},
	{0x90aa, 0x08},
	{0x90ab, 0x90},
	{0x90ac, 0x81},
	{0x90ad, 0xfc},
	{0x90ae, 0xe0},
	{0x90af, 0xff},
	{0x90b0, 0xc3},
	{0x90b1, 0x13},
	{0x90b2, 0xf0},
	{0x90b3, 0x90},
	{0x90b4, 0x81},
	{0x90b5, 0xfc},
	{0x90b6, 0xe0},
	{0x90b7, 0xff},
	{0x90b8, 0x02},
	{0x90b9, 0x20},
	{0x90ba, 0xda},
	{0xc92c, 0x70},
	{0xc92d, 0xbc},
	{0xc92e, 0x80},
	{0xc92f, 0xbb},
	{0x90bb, 0x90},
	{0x90bc, 0x82},
	{0x90bd, 0x18},
	{0x90be, 0xe0},
	{0x90bf, 0xb4},
	{0x90c0, 0x03},
	{0x90c1, 0x06},
	{0x90c2, 0x90},
	{0x90c3, 0xc1},
	{0x90c4, 0x06},
	{0x90c5, 0x74},
	{0x90c6, 0x05},
	{0x90c7, 0xf0},
	{0x90c8, 0x90},
	{0x90c9, 0xd3},
	{0x90ca, 0xa0},
	{0x90cb, 0x02},
	{0x90cc, 0x70},
	{0x90cd, 0xbf},
	{0xc930, 0x72},
	{0xc931, 0x21},
	{0xc932, 0x81},
	{0xc933, 0x3b},
	{0x913b, 0x7d},
	{0x913c, 0x02},
	{0x913d, 0x7f},
	{0x913e, 0x7b},
	{0x913f, 0x02},
	{0x9140, 0x72},
	{0x9141, 0x25},
	{0xc934, 0x28},
	{0xc935, 0xae},
	{0xc936, 0x80},
	{0xc937, 0xd2},
	{0x90d2, 0xf0},
	{0x90d3, 0x90},
	{0x90d4, 0xd2},
	{0x90d5, 0x0a},
	{0x90d6, 0x02},
	{0x90d7, 0x28},
	{0x90d8, 0xb4},
	{0xc938, 0x28},
	{0xc939, 0xb1},
	{0xc93a, 0x80},
	{0xc93b, 0xd9},
	{0x90d9, 0x90},
	{0x90da, 0x83},
	{0x90db, 0xba},
	{0x90dc, 0xe0},
	{0x90dd, 0xff},
	{0x90de, 0x90},
	{0x90df, 0xd2},
	{0x90e0, 0x08},
	{0x90e1, 0xe0},
	{0x90e2, 0xe4},
	{0x90e3, 0xef},
	{0x90e4, 0xf0},
	{0x90e5, 0xa3},
	{0x90e6, 0xe0},
	{0x90e7, 0x74},
	{0x90e8, 0xff},
	{0x90e9, 0xf0},
	{0x90ea, 0x90},
	{0x90eb, 0xd2},
	{0x90ec, 0x0a},
	{0x90ed, 0x02},
	{0x90ee, 0x28},
	{0x90ef, 0xb4},
	{0xc93c, 0x29},
	{0xc93d, 0x79},
	{0xc93e, 0x80},
	{0xc93f, 0xf0},
	{0x90f0, 0xf0},
	{0x90f1, 0x90},
	{0x90f2, 0xd2},
	{0x90f3, 0x0e},
	{0x90f4, 0x02},
	{0x90f5, 0x29},
	{0x90f6, 0x7f},
	{0xc940, 0x29},
	{0xc941, 0x7c},
	{0xc942, 0x80},
	{0xc943, 0xf7},
	{0x90f7, 0x90},
	{0x90f8, 0x83},
	{0x90f9, 0xba},
	{0x90fa, 0xe0},
	{0x90fb, 0xff},
	{0x90fc, 0x90},
	{0x90fd, 0xd2},
	{0x90fe, 0x0c},
	{0x90ff, 0xe0},
	{0x9100, 0xe4},
	{0x9101, 0xef},
	{0x9102, 0xf0},
	{0x9103, 0xa3},
	{0x9104, 0xe0},
	{0x9105, 0x74},
	{0x9106, 0xff},
	{0x9107, 0xf0},
	{0x9108, 0x90},
	{0x9109, 0xd2},
	{0x910a, 0x0e},
	{0x910b, 0x02},
	{0x910c, 0x29},
	{0x910d, 0x7f},
	{0xc944, 0x2a},
	{0xc945, 0x42},
	{0xc946, 0x81},
	{0xc947, 0x0e},
	{0x910e, 0xf0},
	{0x910f, 0x90},
	{0x9110, 0xd2},
	{0x9111, 0x12},
	{0x9112, 0x02},
	{0x9113, 0x2a},
	{0x9114, 0x48},
	{0xc948, 0x2a},
	{0xc949, 0x45},
	{0xc94a, 0x81},
	{0xc94b, 0x15},
	{0x9115, 0x90},
	{0x9116, 0x83},
	{0x9117, 0xba},
	{0x9118, 0xe0},
	{0x9119, 0xff},
	{0x911a, 0x90},
	{0x911b, 0xd2},
	{0x911c, 0x10},
	{0x911d, 0xe0},
	{0x911e, 0xe4},
	{0x911f, 0xef},
	{0x9120, 0xf0},
	{0x9121, 0xa3},
	{0x9122, 0xe0},
	{0x9123, 0x74},
	{0x9124, 0xff},
	{0x9125, 0xf0},
	{0x9126, 0x90},
	{0x9127, 0xd2},
	{0x9128, 0x12},
	{0x9129, 0x02},
	{0x912a, 0x2a},
	{0x912b, 0x48},
	{0xc900, 0x01}
};

static const u16 patch_p2[][2] = {
	{0x806f, 0x01},
	{0x058c, 0x01}
};

static const u16 patch_run_setup[][2] = {
/*	{0x2596, 0x01},*/	/* U first */
	{0x1d18, 0x00},		/* Enableconstrainedwhitebalance */
	{0x200d, 0x3c},		/* Damper PeakGain Output MSB */
	{0x200e, 0x66},		/* Damper PeakGain Output LSB */
	{0x1f03, 0x65},		/* Damper Low MSB */
	{0x1f04, 0xd1},		/* Damper Low LSB */
	{0x1f07, 0x66},		/* Damper High MSB */
	{0x1f08, 0x62},		/* Damper High LSB */
	{0x1f0b, 0x00},		/* Damper Min output MSB */
	{0x1f0c, 0x00},		/* Damper Min output LSB */
	{0x2600, 0x00},		/* Nora fDisable */
	{0x2602, 0x04},		/* Nora usage */
	{0x260d, 0x63},		/* Damper Low MSB Changed 0x63 to 0x65 */
	{0x260e, 0xd1},		/* Damper Low LSB */
	{0x2611, 0x68},		/* Damper High MSB */
	{0x2612, 0xdd},		/* Damper High LSB */
	{0x2615, 0x3a},		/* Damper Min output MSB */
	{0x2616, 0x00},		/* Damper Min output LSB */
	{0x2480, 0x00},		/* Disable */
	{0x1d8a, 0x30},		/* MAXWeightHigh */
	{0x1d91, 0x62},		/* fpDamperLowThresholdHigh MSB */
	{0x1d92, 0x4a},		/* fpDamperLowThresholdHigh LSB */
	{0x1d95, 0x65},		/* fpDamperHighThresholdHigh MSB */
	{0x1d96, 0x0e},		/* fpDamperHighThresholdHigh LSB */
	{0x1da1, 0x3a},		/* fpMinimumDamperOutputLow MSB */
	{0x1da2, 0xb8},		/* fpMinimumDamperOutputLow LSB */
	{0x1e08, 0x06},		/* MAXWeightLow */
	{0x1e0a, 0x0a},		/* MAXWeightHigh */
	{0x1601, 0x3a},		/* Red A MSB */
	{0x1602, 0x14},		/* Red A LSB */
	{0x1605, 0x3b},		/* Blue A MSB */
	{0x1606, 0x85},		/* BLue A LSB */
	{0x1609, 0x3b},		/* RED B MSB */
	{0x160a, 0x85},		/* RED B LSB */
	{0x160d, 0x3a},		/* Blue B MSB */
	{0x160e, 0x14},		/* Blue B LSB */
	{0x1611, 0x30},		/* Max Distance from Locus MSB */
	{0x1612, 0x8f},		/* Max Distance from Locus MSB */
	{0x1614, 0x01}		/* Enable constrainer */
};

static const u16 vs6624_default[][2] = {
	{0x3b4, 0x84},
	{0x3b6, 0x75},
	{0x3b8, 0x11},
	{0x434, 0x84},
	{0x436, 0x75},
	{0x438, 0x11},
	{0x3b4, 0x84},
	{0x3b6, 0x75},
	{0x3b8, 0x11},
	{0x434, 0x84},
	{0x436, 0x75},
	{0x438, 0x11},
	{0x1482, 0x80},
	{0x1484, 0x80},
	{0x1486, 0x80},
	{0x1480, 0x1},
	{0x1190, 0xfe},
	{0x1182, 0x0},
	{0xc80, 0x64},
	{0x1190, 0xfe},
	{0x2000, 0xe},
	{0x2010, 0x28},
	{0x2000, 0xe},
	{0x2010, 0x28},
	{0x3ba, 0x0},
	{0x3bc, 0x0},
	{0x38b, 0x0},
	{0x38c, 0x1},
	{0x38f, 0x0},
	{0x390, 0x1},
	{0x395, 0x0},
	{0x396, 0xf},
	{0x399, 0x0},
	{0x39a, 0xf},
	{0x308, 0x1},
	{0x2584, 0x21},
	{0xe80, 0x0},
	{0xd84, 0x1},
	{0xd82, 0xf},
	{0x2000, 0xe},
	{0x2010, 0x28},
	{0x3ba, 0x0},
	{0x3bc, 0x0},
	{0x1182, 0x0},
	{0xc80, 0x64},
	{0x1190, 0xfe},
	{0x3b4, 0x84},
	{0x3b6, 0x75},
	{0x3b8, 0x11},
	{0x434, 0x84},
	{0x436, 0x75},
	{0x438, 0x11},
	{0x1480, 0x1},
	{0x38b, 0x0},
	{0x38c, 0x1},
	{0x38f, 0x0},
	{0x390, 0x1},
	{0x395, 0x0},
	{0x396, 0xf},
	{0x399, 0x0},
	{0x39a, 0xf},
	{0x482, 0x0},
	{0x2594, 0x1},
};

#endif				/* VS6624_H */

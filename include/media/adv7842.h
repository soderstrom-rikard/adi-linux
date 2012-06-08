/*
 * adv7842 - Analog Devices ADV7842 video decoder driver
 *
 * Copyright 2011 Tandberg Telecom AS.  All rights reserved.
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef _ADV7842_
#define _ADV7842_

/* Analog input muxing modes (AFE register 0x02, [2:0]) */
enum adv7842_ain_sel {
	ADV7842_AIN1_2_3_NC_SYNC_1_2 = 0,
	ADV7842_AIN4_5_6_NC_SYNC_2_1 = 1,
	ADV7842_AIN7_8_9_NC_SYNC_3_1 = 2,
	ADV7842_AIN10_11_12_NC_SYNC_4_1 = 3,
	ADV7842_AIN9_4_5_6_SYNC_2_1 = 4,
};

/* Bus rotation and reordering (IO register 0x04, [7:5]) */
enum adv7842_op_ch_sel {
	ADV7842_OP_CH_SEL_GBR = 0,
	ADV7842_OP_CH_SEL_GRB = 1,
	ADV7842_OP_CH_SEL_BGR = 2,
	ADV7842_OP_CH_SEL_RGB = 3,
	ADV7842_OP_CH_SEL_BRG = 4,
	ADV7842_OP_CH_SEL_RBG = 5,
};

/* Primary mode (IO register 0x01, [3:0]) */
enum adv7842_prim_mode {
	ADV7842_PRIM_MODE_SDP = 0,
	ADV7842_PRIM_MODE_COMP = 1,
	ADV7842_PRIM_MODE_RGB = 2,
	ADV7842_PRIM_MODE_HDMI_COMP = 5,
	ADV7842_PRIM_MODE_HDMI_GR = 6,
};

/* Video standard select (IO register 0x00, [5:0] */
enum adv7842_vid_std_select {
	/* SDP */
	ADV7842_SDP_VID_STD_CVBS_SD_4x1 = 0x01,
	ADV7842_SDP_VID_STD_YC_SD4_x1 = 0x09,
	/* RGB */
	ADV7842_RGB_VID_STD_AUTO_GRAPH_MODE = 0x07,
	/* HDMI GR */
	ADV7842_HDMI_GR_VID_STD_AUTO_GRAPH_MODE = 0x02,
	/* HDMI COMP */
	ADV7842_HDMI_COMP_VID_STD_HD_1250P = 0x1e,
};

/* Input Color Space (IO register 0x02, [7:4]) */
enum adv7842_inp_color_space {
	ADV7842_INP_COLOR_SPACE_LIM_RGB = 0,
	ADV7842_INP_COLOR_SPACE_FULL_RGB = 1,
	ADV7842_INP_COLOR_SPACE_LIM_YCbCr_601 = 2,
	ADV7842_INP_COLOR_SPACE_LIM_YCbCr_709 = 3,
	ADV7842_INP_COLOR_SPACE_XVYCC_601 = 4,
	ADV7842_INP_COLOR_SPACE_XVYCC_709 = 5,
	ADV7842_INP_COLOR_SPACE_FULL_YCbCr_601 = 6,
	ADV7842_INP_COLOR_SPACE_FULL_YCbCr_709 = 7,
	ADV7842_INP_COLOR_SPACE_AUTO = 0xf,
};

/* Select output format (IO register 0x03, [7:0]) */
enum adv7842_op_format_sel {
	ADV7842_OP_FORMAT_SEL_SDR_ITU656_8 = 0x00,
	ADV7842_OP_FORMAT_SEL_SDR_ITU656_10 = 0x01,
	ADV7842_OP_FORMAT_SEL_SDR_ITU656_12_MODE0 = 0x02,
	ADV7842_OP_FORMAT_SEL_SDR_ITU656_12_MODE1 = 0x06,
	ADV7842_OP_FORMAT_SEL_SDR_ITU656_12_MODE2 = 0x0a,
	ADV7842_OP_FORMAT_SEL_DDR_422_8 = 0x20,
	ADV7842_OP_FORMAT_SEL_DDR_422_10 = 0x21,
	ADV7842_OP_FORMAT_SEL_DDR_422_12_MODE0 = 0x22,
	ADV7842_OP_FORMAT_SEL_DDR_422_12_MODE1 = 0x23,
	ADV7842_OP_FORMAT_SEL_DDR_422_12_MODE2 = 0x24,
	ADV7842_OP_FORMAT_SEL_SDR_444_24 = 0x40,
	ADV7842_OP_FORMAT_SEL_SDR_444_30 = 0x41,
	ADV7842_OP_FORMAT_SEL_SDR_444_36_MODE0 = 0x42,
	ADV7842_OP_FORMAT_SEL_DDR_444_24 = 0x60,
	ADV7842_OP_FORMAT_SEL_DDR_444_30 = 0x61,
	ADV7842_OP_FORMAT_SEL_DDR_444_36 = 0x62,
	ADV7842_OP_FORMAT_SEL_SDR_ITU656_16 = 0x80,
	ADV7842_OP_FORMAT_SEL_SDR_ITU656_20 = 0x81,
	ADV7842_OP_FORMAT_SEL_SDR_ITU656_24_MODE0 = 0x82,
	ADV7842_OP_FORMAT_SEL_SDR_ITU656_24_MODE1 = 0x86,
	ADV7842_OP_FORMAT_SEL_SDR_ITU656_24_MODE2 = 0x8a,
};

/* output format, may change with input */
struct adv7842_output_format {
	/* Bus rotation and reordering */
	enum adv7842_op_ch_sel op_ch_sel;

	/* Select output format */
	enum adv7842_op_format_sel op_format_sel;

	/* IO register 0x02 */
	unsigned op_656_range:1;
	unsigned rgb_out:1;
	unsigned alt_data_sat:1;

	/* IO register 0x05 */
	unsigned blank_data:1;
	unsigned insert_av_codes:1;
	unsigned replicate_av_codes:1;
	unsigned invert_cbcr:1;

	/* IO register 0x30 */
	unsigned output_bus_lsb_to_msb:1;

	/* SDP register 0x12 */
	unsigned i2p_convert:1;
};

/* Platform dependent definition */
struct adv7842_platform_data {
	/* output format for corresponding inputs */
	struct adv7842_output_format *opf;
	int num_opf;

	/* connector - HDMI or DVI? */
	unsigned connector_hdmi:1;

	/* DIS_PWRDNB: 1 if the PWRDNB pin is unused and unconnected */
	unsigned disable_pwrdnb:1;

	/* DIS_CABLE_DET_RST: 1 if the 5V pins are unused and unconnected */
	unsigned disable_cable_det_rst:1;

	/* Analog input muxing mode */
	enum adv7842_ain_sel ain_sel;

	unsigned alt_gamma:1;

	/* Primary mode */
	enum adv7842_prim_mode prim_mode;

	/* Video standard */
	enum adv7842_vid_std_select vid_std_select;

	/* Input Color Space */
	enum adv7842_inp_color_space inp_color_space;

	/* Free run */
	unsigned hdmi_free_run_mode;

	/* i2c addresses */
	u8 i2c_sdp_io;
	u8 i2c_sdp;
	u8 i2c_cp;
	u8 i2c_vdp;
	u8 i2c_afe;
	u8 i2c_hdmi;
	u8 i2c_repeater;
	u8 i2c_edid;
	u8 i2c_infoframe;
	u8 i2c_cec;
	u8 i2c_avlink;
	/* I/O expander on ADI adv7842 ez-extender board */
	u8 i2c_ex;
};

/* notify events */
#define ADV7842_HOTPLUG		1
#define ADV7842_FMT_CHANGE	2
#define ADV7842_CEC_TX		3
#define ADV7842_CEC_RX		4

struct adv7842_cec_arg {
	void *arg;
	u32 f_flags;
};

#endif

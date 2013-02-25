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

/*
 * References (c = chapter, p = page):
 * REF_01 - Analog devices, ADV7842, Register Settings Recommendations, Revision 2.5, June 2010
 * REF_02 - Analog devices, Register map documentation, Documentation of the register maps, Software manual, Rev. F, June 2010
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-chip-ident.h>
#include <media/adv7842.h>

static int debug = 3;

MODULE_DESCRIPTION("Analog Devices ADV7842 video decoder driver");
MODULE_AUTHOR("Hans Verkuil <hans.verkuil@cisco.com>");
MODULE_LICENSE("GPL");
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");


/* TODO From platform data */
/* ADV7406 system clock frequency */
#define ADV7842_fsc (28636360)

/*
 **********************************************************************
 *
 *  Arrays with configuration parameters for the ADV7842
 *
 **********************************************************************
 */
static const struct v4l2_dv_timings adv7842_timings[] = {
	V4L2_DV_BT_CEA_720X480P59_94,
	V4L2_DV_BT_CEA_720X576P50,
	V4L2_DV_BT_CEA_1280X720P30,
	V4L2_DV_BT_CEA_1280X720P50,
	V4L2_DV_BT_CEA_1280X720P60,
	V4L2_DV_BT_CEA_1920X1080P30,
	V4L2_DV_BT_CEA_1920X1080P50,
	V4L2_DV_BT_CEA_1920X1080P60,
};

struct adv7842_state {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler hdl;
	enum adv7842_prim_mode prim_mode;
	enum adv7842_vid_std_select vid_std_select;
	struct v4l2_dv_timings timings;
	struct adv7842_platform_data *pdata;
	struct adv7842_output_format *opf;
	v4l2_std_id std; /* current standard in SDP mode */
	struct workqueue_struct *work_queues;
	struct delayed_work delayed_work_enable_hotplug;
	bool connector_hdmi;

	/* i2c clients */
	struct i2c_client *i2c_sdp_io;
	struct i2c_client *i2c_sdp;
	struct i2c_client *i2c_cp;
	struct i2c_client *i2c_vdp;
	struct i2c_client *i2c_afe;
	struct i2c_client *i2c_hdmi;
	struct i2c_client *i2c_repeater;
	struct i2c_client *i2c_edid;
	struct i2c_client *i2c_infoframe;
	struct i2c_client *i2c_cec;
	struct i2c_client *i2c_avlink;

	/* controls */
	struct v4l2_ctrl *detect_tx_5v_ctrl;
	struct v4l2_ctrl *analog_sampling_phase_ctrl;
	struct v4l2_ctrl *free_run_color_ctrl_manual;
	struct v4l2_ctrl *free_run_color_ctrl;
};

struct disp_format_s {
	uint16_t frame_width;
	uint16_t frame_height;

	uint16_t image_width;
	uint16_t image_height;

	uint16_t h_front_porch;
	uint16_t v_front_porch;

	uint16_t h_sync;
	uint16_t v_sync;

	uint32_t pixel_freq;
	uint8_t flag; /* 1 = pos vsync, 2 = pos hsync */
	const char *desc;
	uint32_t preset;
	uint32_t vic; /* video id code */
};

static enum v4l2_mbus_pixelcode adv7842_codes[] = {
	V4L2_MBUS_FMT_UYVY8_2X8,
	V4L2_MBUS_FMT_UYVY8_1X16,
};

/* ----------------------------------------------------------------------- */

static inline struct adv7842_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct adv7842_state, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct adv7842_state, hdl)->sd;
}

/* ----------------------------------------------------------------------- */

static s32 adv_smbus_read_byte_data_check(struct i2c_client *client,
		u8 command, bool check)
{
	union i2c_smbus_data data;

	if (!i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			I2C_SMBUS_READ, command,
			I2C_SMBUS_BYTE_DATA, &data))
		return data.byte;
	if (check)
		v4l_err(client, "error reading %02x, %02x\n",
				client->addr, command);
	return -1;
}

static s32 adv_smbus_read_byte_data(struct i2c_client *client, u8 command)
{
	return adv_smbus_read_byte_data_check(client, command, true);
}

static s32 adv_smbus_write_byte_data(struct i2c_client *client,
					u8 command, u8 value)
{
	union i2c_smbus_data data;
	int err;
	int i;

	data.byte = value;
	for (i = 0; i < 3; i++) {
		err = i2c_smbus_xfer(client->adapter, client->addr,
				client->flags,
				I2C_SMBUS_WRITE, command,
				I2C_SMBUS_BYTE_DATA, &data);
		if (!err)
			break;
	}
	if (err < 0)
		v4l_err(client, "error writing %02x, %02x, %02x\n",
				client->addr, command, value);
	return err;
}

static s32 adv_smbus_write_i2c_block_data(struct i2c_client *client,
	       u8 command, unsigned length, const u8 *values)
{
	union i2c_smbus_data data;

	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	data.block[0] = length;
	memcpy(data.block + 1, values, length);
	return i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			      I2C_SMBUS_WRITE, command,
			      I2C_SMBUS_I2C_BLOCK_DATA, &data);
}

/* ----------------------------------------------------------------------- */

static inline int io_read(struct v4l2_subdev *sd, u8 reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return adv_smbus_read_byte_data(client, reg);
}

static inline int io_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return adv_smbus_write_byte_data(client, reg, val);
}

static inline int io_write_and_or(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return io_write(sd, reg, (io_read(sd, reg) & mask) | val);
}

static inline int avlink_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_avlink, reg);
}

static inline int avlink_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_avlink, reg, val);
}

static inline int cec_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_cec, reg);
}

static inline int cec_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_cec, reg, val);
}

static inline int cec_write_and_or(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return cec_write(sd, reg, (cec_read(sd, reg) & mask) | val);
}

static inline int infoframe_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_infoframe, reg);
}

static inline int infoframe_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_infoframe, reg, val);
}

static inline int sdp_io_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_sdp_io, reg);
}

static inline int sdp_io_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_sdp_io, reg, val);
}

static inline int sdp_io_write_and_or(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return sdp_io_write(sd, reg, (sdp_io_read(sd, reg) & mask) | val);
}

static inline int sdp_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_sdp, reg);
}

static inline int sdp_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_sdp, reg, val);
}

static inline int sdp_write_and_or(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return sdp_write(sd, reg, (sdp_read(sd, reg) & mask) | val);
}

static inline int afe_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_afe, reg);
}

static inline int afe_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_afe, reg, val);
}

static inline int afe_write_and_or(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return afe_write(sd, reg, (afe_read(sd, reg) & mask) | val);
}

static inline int rep_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_repeater, reg);
}

static inline int rep_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_repeater, reg, val);
}

static inline int rep_write_and_or(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return rep_write(sd, reg, (rep_read(sd, reg) & mask) | val);
}

static inline int edid_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_edid, reg);
}

static inline int edid_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_edid, reg, val);
}

static void adv7842_delayed_work_enable_hotplug(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct adv7842_state *state = container_of(dwork, struct adv7842_state, delayed_work_enable_hotplug);
	struct v4l2_subdev *sd = &state->sd;

	v4l2_dbg(2, debug, sd, "%s: enable hotplug\n", __func__);

	v4l2_subdev_notify(sd, ADV7842_HOTPLUG, (void *)1);

}

static inline int hdmi_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_hdmi, reg);
}

static inline int hdmi_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_hdmi, reg, val);
}

static inline int cp_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_cp, reg);
}

static inline int cp_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_cp, reg, val);
}

static inline int cp_write_and_or(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return cp_write(sd, reg, (cp_read(sd, reg) & mask) | val);
}

static inline int vdp_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_vdp, reg);
}

static inline int vdp_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7842_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_vdp, reg, val);
}

#define DIGITAL_INPUT ((state->prim_mode == ADV7842_PRIM_MODE_HDMI_COMP) || (state->prim_mode == ADV7842_PRIM_MODE_HDMI_GR))

/* ----------------------------------------------------------------------- */

#ifdef CONFIG_VIDEO_ADV_DEBUG
static void adv7842_inv_register(struct v4l2_subdev *sd)
{
	v4l2_info(sd, "0x000-0x0ff: IO Map\n");
	v4l2_info(sd, "0x100-0x1ff: AVLink Map\n");
	v4l2_info(sd, "0x200-0x2ff: CEC Map\n");
	v4l2_info(sd, "0x300-0x3ff: InfoFrame Map\n");
	v4l2_info(sd, "0x400-0x4ff: SDP_IO Map\n");
	v4l2_info(sd, "0x500-0x5ff: SDP Map\n");
	v4l2_info(sd, "0x600-0x6ff: AFE Map\n");
	v4l2_info(sd, "0x700-0x7ff: Repeater Map\n");
	v4l2_info(sd, "0x800-0x8ff: EDID Map\n");
	v4l2_info(sd, "0x900-0x9ff: HDMI Map\n");
	v4l2_info(sd, "0xa00-0xaff: CP Map\n");
	v4l2_info(sd, "0xb00-0xbff: VDP Map\n");
}

static int adv7842_g_register(struct v4l2_subdev *sd,
					struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	reg->size = 1;
	switch (reg->reg >> 8) {
	case 0:
		reg->val = io_read(sd, reg->reg & 0xff);
		break;
	case 1:
		reg->val = avlink_read(sd, reg->reg & 0xff);
		break;
	case 2:
		reg->val = cec_read(sd, reg->reg & 0xff);
		break;
	case 3:
		reg->val = infoframe_read(sd, reg->reg & 0xff);
		break;
	case 4:
		reg->val = sdp_io_read(sd, reg->reg & 0xff);
		break;
	case 5:
		reg->val = sdp_read(sd, reg->reg & 0xff);
		break;
	case 6:
		reg->val = afe_read(sd, reg->reg & 0xff);
		break;
	case 7:
		reg->val = rep_read(sd, reg->reg & 0xff);
		break;
	case 8:
		reg->val = edid_read(sd, reg->reg & 0xff);
		break;
	case 9:
		reg->val = hdmi_read(sd, reg->reg & 0xff);
		break;
	case 0xa:
		reg->val = cp_read(sd, reg->reg & 0xff);
		break;
	case 0xb:
		reg->val = vdp_read(sd, reg->reg & 0xff);
		break;
	default:
		v4l2_info(sd, "Register %03llx not supported\n", reg->reg);
		adv7842_inv_register(sd);
		break;
	}
	return 0;
}

static int adv7842_s_register(struct v4l2_subdev *sd,
					struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	switch (reg->reg >> 8) {
	case 0:
		io_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 1:
		avlink_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 2:
		cec_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 3:
		infoframe_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 4:
		sdp_io_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 5:
		sdp_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 6:
		afe_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 7:
		rep_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 8:
		edid_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 9:
		hdmi_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 0xa:
		cp_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 0xb:
		vdp_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	default:
		v4l2_info(sd, "Register %03llx not supported\n", reg->reg);
		adv7842_inv_register(sd);
		break;
	}
	return 0;
}
#endif

static int adv7842_s_detect_tx_5v_ctrl(struct v4l2_subdev *sd)
{
	struct adv7842_state *state = to_state(sd);
	/* port A only */
	return v4l2_ctrl_s_ctrl(state->detect_tx_5v_ctrl,
				((io_read(sd, 0x6f) & 0x02) >> 1));
}

static void configure_free_run(struct v4l2_subdev *sd, const struct disp_format_s *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint16_t CH1_FR_LL = ((fmt->pixel_freq / 100) > 0) ?
		((fmt->frame_width * (ADV7842_fsc / 100)) / (fmt->pixel_freq / 100)) : 0;

	v4l2_dbg(2, debug, sd, "%s\n", __func__);

	cp_write(sd, 0x8f, (CH1_FR_LL >> 8) & 0x7);	/* CH1_FR_LL */
	cp_write(sd, 0x90, CH1_FR_LL & 0xff);		/* CH1_FR_LL */
	cp_write(sd, 0xab, (fmt->frame_height >> 4) & 0xff); /* CP_LCOUNT_MAX */
	cp_write(sd, 0xac, (fmt->frame_height & 0x0f) << 4); /* CP_LCOUNT_MAX */
	/* TODO support interlaced */
/*	cp_write(sd, 0x91, 0x40);*/	/* INTERLACED */
	cp_write(sd, 0x91, 0x00);	/* PROGRESSIVE */

	/* Should only be set in auto-graphics mode [REF_02 p. 91-92] */
	if ((io_read(sd, 0x00) == 0x07) && (io_read(sd, 0x01) == 0x02)) {
		uint16_t CP_START_SAV, CP_START_EAV, CP_START_VBI, CP_END_VBI;
		uint16_t h_back_porch, v_back_porch;
		const u8 pll[2] = {
			(0xc0 | ((fmt->frame_width >> 8) & 0x1f)),
			(fmt->frame_width & 0xff)
		};

		/* setup PLL_DIV_MAN_EN and PLL_DIV_RATIO */
		/* IO-map reg. 0x16 and 0x17 should be written in sequence */
		if (adv_smbus_write_i2c_block_data(client, 0x16, 2, pll)) {
			v4l2_err(sd, "writing to reg 0x16 and 0x17 failed\n");
			return;
		}

		/* active video - horizontal timing */
		h_back_porch = fmt->frame_width - (fmt->h_front_porch + fmt->h_sync + fmt->image_width);
		CP_START_SAV = fmt->h_sync + h_back_porch - 4;
		CP_START_EAV = fmt->frame_width - fmt->h_front_porch;
		cp_write(sd, 0x26, CP_START_SAV >> 8);
		cp_write(sd, 0x27, CP_START_SAV & 0xff);
		cp_write(sd, 0x28, CP_START_EAV >> 8);
		cp_write(sd, 0x29, CP_START_EAV & 0xff);

		/* active video - vertical timing */
		v_back_porch = fmt->frame_height - (fmt->v_front_porch + fmt->v_sync + fmt->image_height);
		CP_START_VBI = fmt->frame_height - fmt->v_front_porch;
		CP_END_VBI = fmt->v_sync + v_back_porch;
		cp_write(sd, 0xa5, (CP_START_VBI >> 4) & 0xff);
		cp_write(sd, 0xa6, ((CP_START_VBI & 0xf) << 4) | ((CP_END_VBI >> 4) & 0xf));
		cp_write(sd, 0xa7, CP_END_VBI & 0xff);
	} else {
		io_write(sd, 0x16, 0x00);
		io_write(sd, 0x17, 0x00);
		cp_write(sd, 0x26, 0x00);
		cp_write(sd, 0x27, 0x00);
		cp_write(sd, 0x28, 0x00);
		cp_write(sd, 0x29, 0x00);
		cp_write(sd, 0xa5, 0x00);
		cp_write(sd, 0xa6, 0x00);
		cp_write(sd, 0xa7, 0x00);
	}
}

static int adv7842_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
/* TODO SDP ctrls
   contrast/brightness/hue/free run is acting a bit strange,
   not sure if sdp csc is correct.
 */
	switch (ctrl->id) {
	/* standard ctrls */
	case V4L2_CID_BRIGHTNESS:
		cp_write(sd, 0x3c, ctrl->val);
		sdp_write(sd, 0x14, ctrl->val);
		return 0;
	case V4L2_CID_CONTRAST:
		cp_write(sd, 0x3a, ctrl->val);
		sdp_write(sd, 0x13, ctrl->val);
		return 0;
	case V4L2_CID_SATURATION:
		cp_write(sd, 0x3b, ctrl->val);
		sdp_write(sd, 0x15, ctrl->val);
		return 0;
	case V4L2_CID_HUE:
		cp_write(sd, 0x3d, ctrl->val);
		sdp_write(sd, 0x16, ctrl->val);
		return 0;
	}
	return -EINVAL;
}

static int adv7842_g_chip_ident(struct v4l2_subdev *sd,
					struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_ADV7842, 0);
}

static int adv7842_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct adv7842_state *state = to_state(sd);

	*status = 0;

	if (io_read(sd, 0x0c) & 0x24)
		*status |= V4L2_IN_ST_NO_POWER;

	if (state->prim_mode == ADV7842_PRIM_MODE_SDP) {
		/* status from SDP block */
		if (!(sdp_read(sd, 0x5A) & 0x01))
			*status |= V4L2_IN_ST_NO_SIGNAL;

		v4l2_dbg(1, debug, sd, "%s: SDP status = 0x%x\n", __func__, *status);
	} else {
		/* status from CP block */
		if (((cp_read(sd, 0xb5) & 0xd0) != 0xd0) || !(cp_read(sd, 0xb1) & 0x80))
			/* TODO channel 2 */
			*status |= V4L2_IN_ST_NO_SIGNAL;
		if (DIGITAL_INPUT && ((io_read(sd, 0x74) & 0x03) != 0x03))
			*status |= V4L2_IN_ST_NO_SIGNAL;
#if 0
		/* TODO when application is ready for NO_H_LOCK/NO_SYNC */
		if (io_read(sd, 0x12) & 0x01)
			*status |= DIGITAL_INPUT ? V4L2_IN_ST_NO_SYNC : V4L2_IN_ST_NO_H_LOCK;
#endif

		v4l2_dbg(1, debug, sd, "%s: CP status = 0x%x\n", __func__, *status);
	}

	return 0;
}

static void adv7842_fill_optional_dv_timings_fields(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct adv7842_state *state = to_state(sd);
	int i;

	for (i = 0; i < ARRAY_SIZE(adv7842_timings); i++) {
		if (v4l_match_dv_timings(timings, &adv7842_timings[i],
					DIGITAL_INPUT ? 250000 : 1000000)) {
			*timings = adv7842_timings[i];
			break;
		}
	}
}

static int adv7842_enum_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_enum_dv_timings *timings)
{
	if (timings->index >= ARRAY_SIZE(adv7842_timings))
		return -EINVAL;
	memset(timings->reserved, 0, sizeof(timings->reserved));
	timings->timings = adv7842_timings[timings->index];
	return 0;
}

static int adv7842_query_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	u32 status;

	if (!timings)
		return -EINVAL;

	adv7842_g_input_status(sd, &status);
	if (status) {
		v4l2_dbg(1, debug, sd, "%s: chip not locked on format\n", __func__);
		memset(timings, 0, sizeof(*timings));
	} else {
		struct v4l2_bt_timings *bt = &timings->bt;

		timings->type = V4L2_DV_BT_656_1120;

		bt->width = (hdmi_read(sd, 0x07) & 0x1f) * 256 +
			hdmi_read(sd, 0x08);
		bt->height = ((hdmi_read(sd, 0x09) & 0x1f) * 256 +
				hdmi_read(sd, 0x0a)) +
			((hdmi_read(sd, 0x0b) & 0x20) ?
			 ((hdmi_read(sd, 0x0b) & 0x1f) * 256 +
			  hdmi_read(sd, 0x0c)) : 0);
		bt->interlaced = (hdmi_read(sd, 0x0b) & 0x20) ?
			V4L2_DV_INTERLACED : V4L2_DV_PROGRESSIVE;
		bt->polarities = ((hdmi_read(sd, 0x05) & 0x10) ? V4L2_DV_VSYNC_POS_POL : 0) |
			((hdmi_read(sd, 0x05) & 0x20) ? V4L2_DV_HSYNC_POS_POL : 0);
		bt->pixelclock = (((hdmi_read(sd, 0x51) << 1) + (hdmi_read(sd, 0x52) >> 7)) * 1000000) +
			((hdmi_read(sd, 0x52) & 0x7f) * 1000000) / 128;
		bt->hfrontporch = (hdmi_read(sd, 0x20) & 0x1f) * 256 +
			hdmi_read(sd, 0x21);
		bt->hsync = (hdmi_read(sd, 0x22) & 0x1f) * 256 +
			hdmi_read(sd, 0x23);
		bt->hbackporch = (hdmi_read(sd, 0x24) & 0x1f) * 256 +
			hdmi_read(sd, 0x25);
		bt->vfrontporch = ((hdmi_read(sd, 0x2a) & 0x3f) * 256 +
				hdmi_read(sd, 0x2b)) / 2;
		bt->il_vfrontporch = ((hdmi_read(sd, 0x2c) & 0x3f) * 256 +
				hdmi_read(sd, 0x2d)) / 2;
		bt->vsync = ((hdmi_read(sd, 0x2e) & 0x3f) * 256 +
				hdmi_read(sd, 0x2f)) / 2;
		bt->il_vsync = ((hdmi_read(sd, 0x30) & 0x3f) * 256 +
				hdmi_read(sd, 0x31)) / 2;
		bt->vbackporch = ((hdmi_read(sd, 0x32) & 0x3f) * 256 +
				hdmi_read(sd, 0x33)) / 2;
		bt->il_vbackporch = ((hdmi_read(sd, 0x34) & 0x3f) * 256 +
				hdmi_read(sd, 0x35)) / 2;
		adv7842_fill_optional_dv_timings_fields(sd, timings);

		v4l2_dbg(1, debug, sd, "%s: %dx%d%s\n", __func__, bt->width, bt->height, bt->interlaced ? "i" : "p");
	}
	return 0;

}

static int adv7842_s_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct adv7842_state *state = to_state(sd);
	struct v4l2_bt_timings *bt;
	struct disp_format_s fmt;

	if (!DIGITAL_INPUT) {
		v4l2_dbg(1, debug, sd, "%s: S_DV_TIMINGS not supported for analog input\n", __func__);
		return -ENOIOCTLCMD;
	}
	if (!timings)
		return -EINVAL;

	bt = &timings->bt;

	/* freerun */
	fmt.frame_width = bt->width + bt->hfrontporch + bt->hsync + bt->hbackporch;
	fmt.frame_height = bt->height + bt->vfrontporch + bt->vsync + bt->vbackporch;
	fmt.image_width = bt->width;
	fmt.image_height = bt->height;
	fmt.h_front_porch = bt->hfrontporch;
	fmt.v_front_porch = bt->vfrontporch;
	fmt.h_sync = bt->hsync;
	fmt.v_sync = bt->vsync;
	fmt.pixel_freq = bt->pixelclock;
	fmt.flag = bt->polarities;

	configure_free_run(sd, &fmt);
	adv7842_fill_optional_dv_timings_fields(sd, timings);
	state->timings = *timings;

	return 0;
}

static int adv7842_g_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct adv7842_state *state = to_state(sd);

	if (!DIGITAL_INPUT) {
		v4l2_dbg(1, debug, sd, "%s: G_DV_TIMINGS not supported for analog input\n", __func__);
		return -ENOIOCTLCMD;
	}
	if (!timings)
		return -EINVAL;

	*timings = state->timings;
	return 0;
}

static void enable_input(struct v4l2_subdev *sd, enum adv7842_prim_mode prim_mode)
{
	switch (prim_mode) {
	case ADV7842_PRIM_MODE_SDP:
	case ADV7842_PRIM_MODE_COMP:
	case ADV7842_PRIM_MODE_RGB:
		/* enable */
		io_write(sd, 0x15, 0xb0);   /* Disable Tristate of Pins (no audio) */
		break;
	case ADV7842_PRIM_MODE_HDMI_COMP:
	case ADV7842_PRIM_MODE_HDMI_GR:
		/* enable */
		hdmi_write(sd, 0x1a, 0x0a); /* Unmute audio */
		hdmi_write(sd, 0x01, 0x00); /* Enable HDMI clock terminators */
		io_write(sd, 0x15, 0xa0);   /* Disable Tristate of Pins */
		break;
	default:
		v4l2_err(sd, "%s: reserved primary mode 0x%0x\n", __func__, prim_mode);
		break;
	}
}

static void disable_input(struct v4l2_subdev *sd)
{
	/* disable */
	io_write(sd, 0x15, 0xbe);   /* Tristate all outputs from video core */
	hdmi_write(sd, 0x1a, 0x1a); /* Mute audio */
	hdmi_write(sd, 0x01, 0x78); /* Disable HDMI clock terminators */
}

static void sdp_csc_coeff(struct v4l2_subdev *sd,
			  bool manual,
			  uint16_t scaling,
			  uint16_t A1, uint16_t A2, uint16_t A3, uint16_t A4,
			  uint16_t B1, uint16_t B2, uint16_t B3, uint16_t B4,
			  uint16_t C1, uint16_t C2, uint16_t C3, uint16_t C4)
{
	/* csc auto/manual */
	sdp_io_write_and_or(sd, 0xe0, 0xbf, manual ? 0x00 : 0x40);

	/* csc scaling */
	sdp_io_write_and_or(sd, 0xe0, 0x7f, scaling == 2 ? 0x80 : 0x00);

	/* A coeff */
	sdp_io_write_and_or(sd, 0xe0, 0xe0, A1>>8);
	sdp_io_write(sd, 0xe1, A1);
	sdp_io_write_and_or(sd, 0xe2, 0xe0, A2>>8);
	sdp_io_write(sd, 0xe3, A2);
	sdp_io_write_and_or(sd, 0xe4, 0xe0, A3>>8);
	sdp_io_write(sd, 0xe5, A3);

	/* A scale */
	sdp_io_write_and_or(sd, 0xe6, 0x80, A4>>8);
	sdp_io_write(sd, 0xe7, A4);

	/* B coeff */
	sdp_io_write_and_or(sd, 0xe8, 0xe0, B1>>8);
	sdp_io_write(sd, 0xe9, B1);
	sdp_io_write_and_or(sd, 0xea, 0xe0, B2>>8);
	sdp_io_write(sd, 0xeb, B2);
	sdp_io_write_and_or(sd, 0xec, 0xe0, B3>>8);
	sdp_io_write(sd, 0xed, B3);

	/* B scale */
	sdp_io_write_and_or(sd, 0xee, 0x80, B4>>8);
	sdp_io_write(sd, 0xef, B4);

	/* C coeff */
	sdp_io_write_and_or(sd, 0xf0, 0xe0, C1>>8);
	sdp_io_write(sd, 0xf1, C1);
	sdp_io_write_and_or(sd, 0xf2, 0xe0, C2>>8);
	sdp_io_write(sd, 0xf3, C2);
	sdp_io_write_and_or(sd, 0xf4, 0xe0, C3>>8);
	sdp_io_write(sd, 0xf5, C3);

	/* C scale */
	sdp_io_write_and_or(sd, 0xf6, 0x80, C4>>8);
	sdp_io_write(sd, 0xf7, C4);
}

static void select_input(struct v4l2_subdev *sd, enum adv7842_prim_mode prim_mode, enum adv7842_vid_std_select vid_std_select)
{
	struct adv7842_state *state = to_state(sd);
	struct adv7842_output_format *opf = state->opf;

	/* output video format */
	io_write_and_or(sd, 0x02, 0xf8,
			opf->op_656_range << 2 |
			opf->rgb_out << 1 |
			opf->alt_data_sat << 0);
	io_write(sd, 0x03, opf->op_format_sel);
	io_write_and_or(sd, 0x04, 0x1f, opf->op_ch_sel << 5);
	io_write_and_or(sd, 0x05, 0xf0, opf->blank_data << 3 |
					opf->insert_av_codes << 2 |
					opf->replicate_av_codes << 1 |
					opf->invert_cbcr << 0);
	io_write_and_or(sd, 0x30, ~(1 << 4), opf->output_bus_lsb_to_msb << 4);

	switch (prim_mode) {
	case ADV7842_PRIM_MODE_SDP:

		io_write(sd, 0x00, vid_std_select); /* video std: CVBS or YC mode */
		io_write(sd, 0x01, prim_mode); /* prim mode */
		cp_write_and_or(sd, 0x81, 0xef, 0x10); /* enable embedded syncs for auto graphics mode */

		afe_write(sd, 0x00, 0x00); /* power up ADC */
		afe_write(sd, 0xc8, 0x00); /* phase control */

		io_write(sd, 0x19, 0x83); /* LLC DLL phase */
		io_write(sd, 0x33, 0x40); /* LLC DLL enable */

		io_write(sd, 0xdd, 0x90); /* Manual 2x output clock */
		/* script says register 0xde, which don't exist in manual */

		afe_write_and_or(sd, 0x02, 0x7f, 0x80); /* Manual analog input muxing mode, CVBS (6.4)*/
		if (vid_std_select == ADV7842_SDP_VID_STD_CVBS_SD_4x1) {
			afe_write(sd, 0x03, 0xb0); /* ADC0 to AIN11 (CVBS), ADC1 N/C*/
			afe_write(sd, 0x04, 0x00); /* ADC2 N/C,ADC3 N/C*/
		} else {
			afe_write(sd, 0x03, 0xa0); /* ADC0 to AIN10 (CVBS), ADC1 N/C*/
			afe_write(sd, 0x04, 0xc0); /* ADC2 to AIN12, ADC3 N/C*/
		}
		afe_write(sd, 0x0c, 0x1f); /* ADI recommend write */
		afe_write(sd, 0x12, 0x63); /* ADI recommend write */

		/* SDP setup for the AD eval board */
		sdp_io_write(sd, 0x7a, 0xa5); /* Timing Adjustment */
		sdp_io_write(sd, 0x7b, 0x8f); /* Timing Adjustment */
		sdp_io_write(sd, 0x60, 0x01); /* SDRAM reset */
		sdp_io_write(sd, 0x97, 0x00); /* Hsync width Adjustment */
		if (opf->insert_av_codes)
			sdp_io_write(sd, 0xb2, 0x6c);
		else
			sdp_io_write(sd, 0xb2, 0x60); /* Disable AV codes */

		/* SDP recommended settings */
		sdp_write(sd, 0x00, 0x7F); /* Autodetect PAL NTSC SECAM */
		sdp_write(sd, 0x01, 0x00); /* Pedestal Off */
		sdp_write(sd, 0x03, 0xE4); /* Manual VCR Gain Luma 0x40B */
		sdp_write(sd, 0x04, 0x0B); /* Manual Luma setting */
		sdp_write(sd, 0x05, 0xC3); /* Manual Chroma setting 0x3FE */
		sdp_write(sd, 0x06, 0xFE); /* Manual Chroma setting */
		sdp_write(sd, 0x12, 0x05); /* Frame TBC,3D comb enabled */
		sdp_write(sd, 0xA7, 0x00); /* ADI Recommended Write */

		if (opf->i2p_convert)
			sdp_write_and_or(sd, 0x12, 0xf7, 0x08); /* deinterlacer enabled */

		sdp_write(sd, 0xdd, 0x08); /* free run auto */

		if (opf->rgb_out) {
			/* Manual CSC mode, convert to RGB
			http://ez.analog.com/message/30063
			*/
			sdp_csc_coeff(sd, true, 2,
					0x03a7, 0x1e91, 0x1de2, 0x7d00,
					0x03a7, 0x0761, 0x0000, 0x7900,
					0x03a7, 0x0000, 0x0429, 0x7900);
		}
		v4l2_info(sd, "%s: SDP todo\n", __func__);
		break;
	case ADV7842_PRIM_MODE_COMP:
	case ADV7842_PRIM_MODE_RGB: {
		/* Automatic analog input muxing mode */
		afe_write_and_or(sd, 0x02, 0x7f, 0x00);
		/* set mode and select free run resolution */
		io_write(sd, 0x00, vid_std_select); /* video std */
		io_write(sd, 0x01, 0x02); /* prim mode */
		cp_write_and_or(sd, 0x81, 0xef, 0x10); /* enable embedded syncs for auto graphics mode */

		afe_write(sd, 0x00, 0x00); /* power up ADC */
		afe_write(sd, 0xc8, 0x00); /* phase control */

		/* set ADI recommended settings for digitizer */
		/* "ADV7842 Register Settings Recommendations (rev. 1.8, November 2010)" p. 9. */
		afe_write(sd, 0x0c, 0x1f); /* ADC Range improvement */
		afe_write(sd, 0x12, 0x63); /* ADC Range improvement */
		cp_write(sd, 0x73, 0xf2); /* Set manual gain of 0x320 (full range) */
		cp_write(sd, 0x74, 0x0c); /* Set manual gain of 0x320 (full range) */
		cp_write(sd, 0x75, 0x83); /* Set manual gain of 0x320 (full range) */
		cp_write(sd, 0x76, 0x20); /* Set manual gain of 0x320 (full range) */
		cp_write(sd, 0x3e, 0x84); /* CP core pre-gain control, enable color control */
		if (prim_mode == ADV7842_PRIM_MODE_COMP)
			cp_write(sd, 0xc3, 0x33); /* CP coast control. Component mode */
		else
			cp_write(sd, 0xc3, 0x39); /* CP coast control. Graphics mode */

		/* color space conversion */
		io_write_and_or(sd, 0x02, 0x0f, 0x10); /* force input color space to RGB full range */
	}
		break;
	case ADV7842_PRIM_MODE_HDMI_COMP:
		io_write(sd, 0x00, vid_std_select); /* video std */
		io_write(sd, 0x01, prim_mode); /* prim mode */

		io_write(sd, 0x19, 0x83);
		io_write(sd, 0x33, 0x40);
		cp_write(sd, 0xba, 0x01);
		cp_write(sd, 0x3e, 0x00);
		cp_write(sd, 0x6c, 0x00);
		afe_write(sd, 0x00, 0xff);
		afe_write(sd, 0x01, 0xfe);
		afe_write(sd, 0xb5, 0x01);
		hdmi_write(sd, 0x00, 0x32);
		hdmi_write(sd, 0xc1, 0xff);
		hdmi_write(sd, 0xc2, 0xff);
		hdmi_write(sd, 0xc3, 0xff);
		hdmi_write(sd, 0xc4, 0xff);
		hdmi_write(sd, 0xc5, 0x00);
		hdmi_write(sd, 0xc6, 0x00);
		hdmi_write(sd, 0xc0, 0xff);
		hdmi_write(sd, 0x01, 0x18);
		hdmi_write(sd, 0x0d, 0x34);
		hdmi_write(sd, 0x1a, 0x8a);
		hdmi_write(sd, 0x3d, 0x10);
		hdmi_write(sd, 0x44, 0x85);
		hdmi_write(sd, 0x46, 0x1f);
		hdmi_write(sd, 0x60, 0x88);
		hdmi_write(sd, 0x61, 0x88);
		hdmi_write(sd, 0x6c, 0x18);
		hdmi_write(sd, 0x57, 0xb6);
		hdmi_write(sd, 0x58, 0x03);
		hdmi_write(sd, 0x67, 0x20);
		hdmi_write(sd, 0x75, 0x10);
		hdmi_write(sd, 0x85, 0x1f);
		hdmi_write(sd, 0x87, 0x70);
		hdmi_write(sd, 0x89, 0x04);
		hdmi_write(sd, 0x8a, 0x1e);
		hdmi_write(sd, 0x93, 0x04);
		hdmi_write(sd, 0x94, 0x1e);
		hdmi_write(sd, 0x9d, 0x02);
		hdmi_write(sd, 0x99, 0xa1);
		hdmi_write(sd, 0x9b, 0x09);
		hdmi_write(sd, 0xc9, 0x01);
		v4l2_info(sd, "%s: HDMI todo\n", __func__);
		break;
	case ADV7842_PRIM_MODE_HDMI_GR: {
		/* Automatic analog input muxing mode */
		afe_write_and_or(sd, 0x02, 0x7f, 0x00);
		/* set mode and select free run resolution */
		hdmi_write(sd, 0x00, 0x02); /* select port A */
		io_write(sd, 0x00, vid_std_select); /* video std */
		io_write(sd, 0x01, prim_mode); /* prim mode */
		cp_write_and_or(sd, 0x81, 0xef, 0x00); /* disable embedded syncs for auto graphics mode */

		/* set ADI recommended settings for HDMI: */
		/* "ADV7842 Register Settings Recommendations (rev. 1.8, November 2010)" p. 3. */
		hdmi_write(sd, 0xc0, 0x00);
		hdmi_write(sd, 0x0d, 0x34); /* ADI recommended write */
		hdmi_write(sd, 0x3d, 0x10); /* ADI recommended write */
		hdmi_write(sd, 0x44, 0x85); /* TMDS PLL optimization */
		hdmi_write(sd, 0x46, 0x1f); /* ADI recommended write */
		hdmi_write(sd, 0x57, 0xb6); /* TMDS PLL optimization */
		hdmi_write(sd, 0x58, 0x03); /* TMDS PLL optimization */
		hdmi_write(sd, 0x60, 0x88); /* TMDS PLL optimization */
		hdmi_write(sd, 0x61, 0x88); /* TMDS PLL optimization */
		hdmi_write(sd, 0x6c, 0x18); /* Disable ISRC clearing bit, Improve robustness */
		hdmi_write(sd, 0x75, 0x10); /* DDC drive strength */
		hdmi_write(sd, 0x85, 0x1f); /* equaliser */
		hdmi_write(sd, 0x87, 0x70); /* ADI recommended write */
		hdmi_write(sd, 0x89, 0x04); /* equaliser */
		hdmi_write(sd, 0x8a, 0x1e); /* equaliser */
		hdmi_write(sd, 0x93, 0x04); /* equaliser */
		hdmi_write(sd, 0x94, 0x1e); /* equaliser */
		hdmi_write(sd, 0x99, 0xa1); /* ADI recommended write */
		hdmi_write(sd, 0x9b, 0x09); /* ADI recommended write */
		hdmi_write(sd, 0x9d, 0x02); /* equaliser */

		afe_write(sd, 0x00, 0xff); /* power down ADC */
		afe_write(sd, 0xc8, 0x40); /* phase control */

		/* set to default gain for HDMI */
		cp_write(sd, 0x73, 0x10);
		cp_write(sd, 0x74, 0x04);
		cp_write(sd, 0x75, 0x01);
		cp_write(sd, 0x76, 0x00);

		/* reset ADI recommended settings for digitizer */
		/* "ADV7842 Register Settings Recommendations (rev. 2.5, June 2010)" p. 17. */
		afe_write(sd, 0x12, 0xfb); /* ADC noise shaping filter controls */
		afe_write(sd, 0x0c, 0x0d); /* CP core gain controls */
		cp_write(sd, 0x3e, 0x84); /* CP core pre-gain control, enable color control */
		if (prim_mode == ADV7842_PRIM_MODE_HDMI_COMP)
			cp_write(sd, 0xc3, 0x33); /* CP coast control. Component mode */
		else
			cp_write(sd, 0xc3, 0x39); /* CP coast control. Graphics mode */

		/* color space conversion */
		io_write_and_or(sd, 0x02, 0x0f, 0xf0); /* autodetect color space */
	}
		break;
	default:
		v4l2_err(sd, "%s: reserved primary mode 0x%0x\n", __func__, prim_mode);
		break;
	}
}

static int adv7842_s_routing(struct v4l2_subdev *sd,
		u32 input, u32 output, u32 config)
{
	struct adv7842_state *state = to_state(sd);
	enum adv7842_prim_mode prev_prime_mode = state->prim_mode;
	enum adv7842_vid_std_select prev_vid_std_select = state->vid_std_select;

	v4l2_dbg(2, debug, sd, "%s: input %d", __func__, input);

	switch (input) {
	case 0:
		/* TODO select HDMI_COMP or HDMI_GR */
		state->prim_mode = ADV7842_PRIM_MODE_HDMI_COMP;
		state->vid_std_select = ADV7842_HDMI_COMP_VID_STD_HD_1250P;
		break;
	case 1:
		state->prim_mode = ADV7842_PRIM_MODE_RGB;
		state->vid_std_select = ADV7842_RGB_VID_STD_AUTO_GRAPH_MODE;
		break;
	case 2:
		state->prim_mode = ADV7842_PRIM_MODE_COMP;
		break;
	case 3:
		state->prim_mode = ADV7842_PRIM_MODE_SDP;
		state->vid_std_select = ADV7842_SDP_VID_STD_CVBS_SD_4x1;
		break;
	case 4:
		state->prim_mode = ADV7842_PRIM_MODE_SDP;
		state->vid_std_select = ADV7842_SDP_VID_STD_YC_SD4_x1;
		break;
	default:
		return -EINVAL;
	}

	if (output < state->pdata->num_opf) {
		state->opf = &state->pdata->opf[output];
	} else {
		state->prim_mode = prev_prime_mode;
		state->vid_std_select = prev_vid_std_select;
		return -EINVAL;
	}

	if ((state->prim_mode != prev_prime_mode) ||
	    (state->vid_std_select != prev_vid_std_select)) {
		/* only re-init if primary mode changed */
		disable_input(sd);

		select_input(sd, state->prim_mode, state->vid_std_select);

		enable_input(sd, state->prim_mode);
	}

	return 0;
}

static int adv7842_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
			     enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(adv7842_codes))
		return -EINVAL;

	*code = adv7842_codes[index];
	return 0;
}

static int adv7842_g_mbus_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *fmt)
{
	struct adv7842_state *state = to_state(sd);

	if (state->prim_mode == ADV7842_PRIM_MODE_SDP) {

		fmt->code = V4L2_MBUS_FMT_UYVY8_2X8;
		fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
		if (state->std & V4L2_STD_525_60) {
			fmt->field = V4L2_FIELD_SEQ_TB;
			fmt->width = 720;
			fmt->height = 480;
		} else {
			fmt->field = V4L2_FIELD_SEQ_BT;
			fmt->width = 720;
			fmt->height = 576;
		}
	} else {
		fmt->width = state->timings.bt.width;
		fmt->height = state->timings.bt.height;
		fmt->code = V4L2_MBUS_FMT_UYVY8_1X16;
		fmt->field = V4L2_FIELD_NONE;
		fmt->colorspace = (state->timings.bt.height <= 576) ?
			V4L2_COLORSPACE_SMPTE170M : V4L2_COLORSPACE_REC709;
	}
	return 0;
}

static int adv7842_isr(struct v4l2_subdev *sd, u32 status, bool *handled)
{
	struct adv7842_state *state = to_state(sd);
	u8 fmt_change, fmt_change_digital, cec_irq, tx_5v;

	/* format change */
	fmt_change = io_read(sd, 0x43) & 0x98;
	if (fmt_change)
		io_write(sd, 0x44, fmt_change);
	fmt_change_digital = DIGITAL_INPUT ? (io_read(sd, 0x75) & 0x03) : 0;
	if (fmt_change_digital)
		io_write(sd, 0x76, fmt_change_digital);
	if (fmt_change || fmt_change_digital) {
		v4l2_dbg(1, debug, sd, "%s: ADV7842_FMT_CHANGE\n", __func__);
		v4l2_subdev_notify(sd, ADV7842_FMT_CHANGE, NULL);
		if (handled)
			*handled = true;
	}
	/* cec controller */
	cec_irq = io_read(sd, 0x93) & 0x0f;
	/* tx 5v detect */
	tx_5v = io_read(sd, 0x70) & 0x02;
	if (tx_5v) {
		v4l2_dbg(1, debug, sd, "%s: tx_5v: 0x%x\n", __func__, tx_5v);
		io_write(sd, 0x71, tx_5v);
		adv7842_s_detect_tx_5v_ctrl(sd);
		if (handled)
			*handled = true;
	}
	return 0;
}

/*********** avi info frame CEA-861-E **************/
/* TODO move to common library */

struct aviInfoFrame {
	uint8_t f17;
	uint8_t y10;
	uint8_t a0;
	uint8_t b10;
	uint8_t s10;
	uint8_t c10;
	uint8_t m10;
	uint8_t r3210;
	uint8_t itc;
	uint8_t ec210;
	uint8_t q10;
	uint8_t sc10;
	uint8_t f47;
	uint8_t vic;
	uint8_t yq10;
	uint8_t cn10;
	uint8_t pr3210;
	uint16_t etb;
	uint16_t sbb;
	uint16_t elb;
	uint16_t srb;
};

static const char *y10Text[4] = {
	"RGB",
	"YCbCr 4:2:2",
	"YCbCr 4:4:4",
	"Future",
};

#if 0
static const char *a0Text[2] = {
	"No Active Format Information",
	"Active Format Information present",
};

static const char *b10Text[4] = {
	"Bar Data not present",
	"Vert. Bar info present",
	"Horiz. Bar info present",
	"Vert. and Horiz. Bar info present",
};

static const char *s10Text[4] = {
	"No Data",
	"Composed for overscanned display",
	"Composed for underscanned display",
	"Future",
};
#endif

static const char *c10Text[4] = {
	"No Data",
	"SMPTE 170M",
	"ITU-R 709",
	"Extended Colorimetry information valied",
};

#if 0
static const char *m10Text[4] = {
	"No Data",
	"4:3",
	"16:9",
	"Future",
};

static const char *r3210Text(uint8_t r3210)
{
	switch (r3210) {
	case 8: return "Same as frame aspect ratio";
	case 9: return "4:3 (Center)";
	case 10: return "16:9 (Center)";
	case 11: return "14:9 (Center)";
	default: return "Per DVB AFD";
	};
}
#endif

static const char *itcText[2] = {
	"No Data",
	"IT content",
};

static const char *ec210Text[8] = {
	"xvYCC601",
	"xvYCC709",
	"sYCC601",
	"AdobeYCC601",
	"AdobeRGB",
	"5 reserved",
	"6 reserved",
	"7 reserved",
};

static const char *q10Text[4] = {
	"Default",
	"Limited Range",
	"Full Range",
	"Reserved",
};

#if 0
static const char *sc10Text[4] = {
	"No known non-uniform scaling",
	"Picture scaled horizontally",
	"Picture scaled vertically",
	"Picture scaled vertically and horizontally",
};
#endif

static void parse_avi_infoframe(struct v4l2_subdev *sd, uint8_t *buf, struct aviInfoFrame *avi)
{
	avi->f17 = (buf[1] >> 7) & 0x1;
	avi->y10 = (buf[1] >> 5) & 0x3;
	avi->a0 = (buf[1] >> 4) & 0x1;
	avi->b10 = (buf[1] >> 2) & 0x3;
	avi->s10 = buf[1] & 0x3;
	avi->c10 = (buf[2] >> 6) & 0x3;
	avi->m10 = (buf[2] >> 4) & 0x3;
	avi->r3210 = buf[2] & 0xf;
	avi->itc = (buf[3] >> 7) & 0x1;
	avi->ec210 = (buf[3] >> 4) & 0x7;
	avi->q10 = (buf[3] >> 2) & 0x3;
	avi->sc10 = buf[3] & 0x3;
	avi->f47 = (buf[4] >> 7) & 0x1;
	avi->vic = buf[4] & 0x7f;
	avi->yq10 = (buf[5] >> 6) & 0x3;
	avi->cn10 = (buf[5] >> 4) & 0x3;
	avi->pr3210 = buf[5] & 0xf;
	avi->etb = buf[6] + 256*buf[7];
	avi->sbb = buf[8] + 256*buf[9];
	avi->elb = buf[10] + 256*buf[11];
	avi->srb = buf[12] + 256*buf[13];
}

static void print_avi_infoframe(struct v4l2_subdev *sd)
{
	int i;
	uint8_t buf[14];
	uint8_t avi_inf_len;
	struct aviInfoFrame avi;

	if (!(hdmi_read(sd, 0x05) & 0x80)) {
		v4l2_info(sd, "receive DVI-D signal (AVI infoframe not supported)\n");
		return;
	}
	if (!(io_read(sd, 0x60) & 0x01)) {
		v4l2_info(sd, "AVI infoframe not received\n");
		return;
	}

	if (io_read(sd, 0x88) & 0x10) {
		/* Note: the ADV7842 calculated incorrect checksums for InfoFrames
		   with a length of 14 or 15. See the ADV7842 Register Settings
		   Recommendations document for more details. */
		v4l2_info(sd, "AVI infoframe checksum error\n");
		return;
	}

	avi_inf_len = infoframe_read(sd, 0xe2);
	v4l2_info(sd, "AVI infoframe version %d (%d byte)\n",
			infoframe_read(sd, 0xe1), avi_inf_len);

	if (infoframe_read(sd, 0xe1) != 0x02)
		return;

	for (i = 0; i < 14; i++)
		buf[i] = infoframe_read(sd, i);

	v4l2_info(sd, "\t%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
			buf[8], buf[9], buf[10], buf[11], buf[12], buf[13]);

	parse_avi_infoframe(sd, buf, &avi);

	if (avi.vic)
		v4l2_info(sd, "\tVIC: %d\n", avi.vic);
	if (avi.itc)
		v4l2_info(sd, "\t%s\n", itcText[avi.itc]);

	if (avi.y10)
		v4l2_info(sd, "\t%s %s\n", y10Text[avi.y10], !avi.c10 ? "" : (avi.c10 == 0x3 ? ec210Text[avi.ec210] : c10Text[avi.c10]));
	else
		v4l2_info(sd, "\t%s %s\n", y10Text[avi.y10], q10Text[avi.q10]);
#if 0
	v4l2_info(sd, " Y1 Y0      : %2d %s\n", avi.y10, y10Text[avi.y10]);
	v4l2_info(sd, " A0         : %2d %s\n", avi.a0, a0Text[avi.a0]);
	v4l2_info(sd, " B1 B0      : %2d %s\n", avi.b10, b10Text[avi.b10]);
	v4l2_info(sd, " S1 S0      : %2d %s\n", avi.s10, s10Text[avi.s10]);
	v4l2_info(sd, " C1 C0      : %2d %s\n", avi.c10, c10Text[avi.c10]);
	v4l2_info(sd, " M1 M0      : %2d %s\n", avi.m10, m10Text[avi.m10]);
	v4l2_info(sd, " R3 R2 R1 R0: %2d %s\n", avi.r3210, r3210Text(avi.r3210));
	v4l2_info(sd, " ITC        : %2d %s\n", avi.itc, itcText[avi.itc]);
	v4l2_info(sd, " EC2 EC1 EC0: %2d %s\n", avi.ec210, ec210Text[avi.ec210]);
	v4l2_info(sd, " Q1 Q0      : %2d %s\n", avi.q10, q10Text[avi.q10]);
	v4l2_info(sd, " SC1 SC0    : %2d %s\n", avi.sc10, sc10Text[avi.sc10]);
	v4l2_info(sd, " VIC        : %d\n", avi.vic);
	v4l2_info(sd, " Pixel Repetition: %4d\n", avi.pr3210);
	v4l2_info(sd, " End Top Bar     : %4d\n", avi.etb);
	v4l2_info(sd, " Start Bottom Bar: %4d\n", avi.sbb);
	v4l2_info(sd, " End Left Bar    : %4d\n", avi.elb);
	v4l2_info(sd, " Start Right Bar : %4d\n", avi.srb);
#endif
}

static int adv7842_log_status(struct v4l2_subdev *sd)
{

	const char *prim_mode_txt[] = {
		"SDP",
		"Component",
		"Graphics",
		"Reserved",
		"CVBS & HDMI AUDIO",
		"HDMI-Comp",
		"HDMI-GR",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
	};

	struct adv7842_state *state = to_state(sd);
	uint8_t prim_mode = io_read(sd, 0x01) & 0x0f;
	v4l2_info(sd, "connector type: %s\n", state->connector_hdmi ? "HDMI" : (DIGITAL_INPUT ? "DVI-D" : "DVI-A"));
	v4l2_info(sd, "EDID %s\n", ((rep_read(sd, 0x7d) & 0x04) && (rep_read(sd, 0x77) & 0x04)) ? "enabled" : "disabled ");
	v4l2_info(sd, "CEC %s\n", !!(cec_read(sd, 0x2a) & 0x01) ? "enabled" : "disabled");

	v4l2_info(sd, "digital cable %sdetected (+5V power)\n", (io_read(sd, 0x6f) & 0x02) ? "" : "not ");
	v4l2_info(sd, "TMDS signal %sdetected%s\n", (io_read(sd, 0x6a) & 0x02) ? "" : "not ",
			(hdmi_read(sd, 0x04) & 0x02) ? " and locked" : "");
	v4l2_info(sd, "prim-mode = %s (0x%x), video std = 0x%x\n", prim_mode_txt[prim_mode], prim_mode, io_read(sd, 0x00) & 0x3f);
	if (prim_mode == ADV7842_PRIM_MODE_SDP) {
		uint8_t sdp_signal_detected = sdp_read(sd, 0x5A) & 0x01;
		/* SDP (Standard definition processor) block */
		v4l2_info(sd, "SDP: free run: %s\n", (sdp_read(sd, 0x56) & 0x01) ? "on" : "off");
		v4l2_info(sd, "SDP: %s\n", sdp_signal_detected ? "valid SD/PR signal detected" : "invalid/no signal");
		if (sdp_signal_detected) {
			const char *sdp_std_txt[] = {
				"NTSC-M/J",
				"1?",
				"NTSC-443",
				"60HzSECAM",
				"PAL-M",
				"5?",
				"PAL-60",
				"7?", "8?", "9?", "a?", "b?",
				"PAL-CombN",
				"d?",
				"PAL-BGHID",
				"SECAM"
			};
			v4l2_info(sd, "SDP: standard %s\n", sdp_std_txt[sdp_read(sd, 0x52) & 0x0f]);
			v4l2_info(sd, "SDP: %s\n", (sdp_read(sd, 0x59) & 0x08) ? "50Hz" : "60Hz");
			v4l2_info(sd, "SDP: %s\n", (sdp_read(sd, 0x57) & 0x08) ? "Interlaced" : "Progressive");
			v4l2_info(sd, "SDP: deinterlacer %s\n", (sdp_read(sd, 0x12) & 0x08) ? "enabled" : "disabled");
			v4l2_info(sd, "SDP: csc %s mode\n", (sdp_io_read(sd, 0xe0) & 0x40) ? "auto" : "manual");
		}
	} else {
		/* CP block */
		v4l2_info(sd, "CP: free run: %s\n", (!!(cp_read(sd, 0xff) & 0x10) ? "on" : "off"));
		if (cp_read(sd, 0xb1) & 0x80) {
			uint32_t bl = ((cp_read(sd, 0xb1) & 0x3f) << 8) | cp_read(sd, 0xb2);
			uint32_t lcf = ((cp_read(sd, 0xb3) & 0x7) << 8) | cp_read(sd, 0xb4);
			uint32_t lcvs = cp_read(sd, 0xb3) >> 3;
			uint32_t fcl = ((cp_read(sd, 0xb8) & 0x1f) << 8) | cp_read(sd, 0xb9);
			char hs_pol = ((cp_read(sd, 0xb5) & 0x10) ? ((cp_read(sd, 0xb5) & 0x08) ? '-' : '+') : 'x');
			char vs_pol = ((cp_read(sd, 0xb5) & 0x40) ? ((cp_read(sd, 0xb5) & 0x20) ? '-' : '+') : 'x');
			v4l2_info(sd, "CP: STDI locked: lcf (frame height - 1) = %d, bl = %d, lcvs (vsync) = %d, fcl = %d, %s, %chsync, %cvsync\n",
				  lcf, bl, lcvs, fcl, (cp_read(sd, 0xb1) & 0x40) ? "interlaced" : "progressive", hs_pol, vs_pol);
		} else {
			v4l2_info(sd, "CP: STDI not locked\n");
		}
	}

	if (DIGITAL_INPUT) {
		if ((io_read(sd, 0x74) & 0x03) == 0x03) {
			bool is_hdmi = (hdmi_read(sd, 0x05) & 0x80);
			uint32_t img_w = (hdmi_read(sd, 0x07) & 0x1f) * 256 + hdmi_read(sd, 0x08);
			uint32_t img_h = (hdmi_read(sd, 0x09) & 0x1f) * 256 + hdmi_read(sd, 0x0a);
			char *interlaced = (hdmi_read(sd, 0x0b) & 0x20) ? "i" : "p";
			uint32_t frame_w = (hdmi_read(sd, 0x1e) & 0x3f) * 256 + hdmi_read(sd, 0x1f);
			uint32_t frame_h = ((hdmi_read(sd, 0x26) & 0x3f) * 256 + hdmi_read(sd, 0x27)) / 2;
			uint32_t pix_frq = (((hdmi_read(sd, 0x51) << 1) + (hdmi_read(sd, 0x52) >> 7)) * 1000000) +
			((hdmi_read(sd, 0x52) & 0x7f) * 1000000) / 128;
			uint32_t fpKs = ((frame_w * frame_h / 1000) > 0) ?
				(pix_frq / (frame_w * frame_h / 1000)) : 0;

			v4l2_info(sd, "%s: Format:  %dx%d%s%d.%03d (%dx%d)\n",
				  is_hdmi ? "HDMI" : "DVI-D",
				  img_w, img_h, interlaced, fpKs/1000, fpKs%1000,
				  frame_w, frame_h);
			if (is_hdmi) {
				bool audio_pll_locked = hdmi_read(sd, 0x04) & 0x01;
				bool audio_sample_packet_detect = hdmi_read(sd, 0x18) & 0x01;
				if (audio_pll_locked && audio_sample_packet_detect) {
					v4l2_info(sd, "detected %s audio format\n",
						  (hdmi_read(sd, 0x07) & 0x40) ? "multi-channel" : "stereo");
				}
			}
		} else
			v4l2_info(sd, "No video detected\n");

		print_avi_infoframe(sd);
	}

	return 0;
}

static int adv7842_g_std(struct v4l2_subdev *sd, v4l2_std_id *std)
{
	struct adv7842_state *state = to_state(sd);

	if (state->prim_mode != ADV7842_PRIM_MODE_SDP)
		return -EINVAL;

	*std = state->std;
	return 0;
}

static int adv7842_s_std(struct v4l2_subdev *sd, v4l2_std_id std)
{
	struct adv7842_state *state = to_state(sd);
	u8 val;

	if (state->prim_mode != ADV7842_PRIM_MODE_SDP)
		return -EINVAL;

	if (std == V4L2_STD_NTSC_443)
		val = 0x20;
	else if (std == V4L2_STD_PAL_60)
		val = 0x10;
	else if (std == V4L2_STD_PAL_Nc)
		val = 0x08;
	else if (std == V4L2_STD_PAL_M)
		val = 0x04;
	else if (std & V4L2_STD_NTSC)
		val = 0x02;
	else if (std & V4L2_STD_PAL)
		val = 0x01;
	else if (std & V4L2_STD_SECAM)
		val = 0x40;
	else
		return -EINVAL;
	/* force the digital core into a specific video standard */
	sdp_write(sd, 0x0, val);
	/* wait 100ms, otherwise color will be lost */
	msleep(100);
	state->std = std;
	return 0;
}

static int adv7842_querystd(struct v4l2_subdev *sd, v4l2_std_id *std)
{
	struct adv7842_state *state = to_state(sd);
	u8 val;

	if (state->prim_mode != ADV7842_PRIM_MODE_SDP)
		return -EINVAL;

	/* enable autodetection block */
	sdp_write(sd, 0x0, 0x7f);
	/* wait autodetection switch */
	mdelay(10);
	/* get autodetection result */
	val = sdp_read(sd, 0x52) & 0xf;
	switch (val) {
	case 0x0:
		*std = V4L2_STD_NTSC;
		break;
	case 0x2:
		*std = V4L2_STD_NTSC_443;
		break;
	case 0x4:
		*std = V4L2_STD_PAL_M;
		break;
	case 0x6:
		*std = V4L2_STD_PAL_60;
		break;
	case 0xc:
		*std = V4L2_STD_PAL_Nc;
		break;
	case 0xe:
		*std = V4L2_STD_PAL;
		break;
	case 0xf:
		*std = V4L2_STD_SECAM;
		break;
	default:
		*std = V4L2_STD_UNKNOWN;
		break;
	}
	/* after autodetection, write back user set std */
	return adv7842_s_std(sd, state->std);
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_ctrl_ops adv7842_ctrl_ops = {
	.s_ctrl = adv7842_s_ctrl,
};

static const struct v4l2_subdev_core_ops adv7842_core_ops = {
	.log_status = adv7842_log_status,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.queryctrl = v4l2_subdev_queryctrl,
	.querymenu = v4l2_subdev_querymenu,
	.g_chip_ident = adv7842_g_chip_ident,
	.g_std = adv7842_g_std,
	.s_std = adv7842_s_std,
	.interrupt_service_routine = adv7842_isr,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = adv7842_g_register,
	.s_register = adv7842_s_register,
#endif
};

static const struct v4l2_subdev_video_ops adv7842_video_ops = {
	.s_routing = adv7842_s_routing,
	.querystd = adv7842_querystd,
	.g_input_status = adv7842_g_input_status,
	.s_dv_timings = adv7842_s_dv_timings,
	.g_dv_timings = adv7842_g_dv_timings,
	.query_dv_timings = adv7842_query_dv_timings,
	.enum_dv_timings = adv7842_enum_dv_timings,
	.enum_mbus_fmt = adv7842_enum_mbus_fmt,
	.g_mbus_fmt = adv7842_g_mbus_fmt,
	.try_mbus_fmt = adv7842_g_mbus_fmt,
	.s_mbus_fmt = adv7842_g_mbus_fmt,
};

static const struct v4l2_subdev_ops adv7842_ops = {
	.core = &adv7842_core_ops,
	.video = &adv7842_video_ops,
};

/* -------------------------- custom ctrls ---------------------------------- */

/* ----------------------------------------------------------------------- */

static int adv7842_core_init(struct v4l2_subdev *sd, const struct adv7842_platform_data *pdata)
{
	hdmi_write(sd, 0x48,
		(pdata->disable_pwrdnb ? 0x80 : 0) |
		(pdata->disable_cable_det_rst ? 0x40 : 0));

	disable_input(sd);

	/* power */
	io_write(sd, 0x0c, 0x42);   /* Power up part and power down VDP */
	io_write(sd, 0x15, 0x80);   /* Power up pads */

	/* video format */
	io_write(sd, 0x02,
			pdata->inp_color_space << 4 |
			pdata->alt_gamma << 3);

	/* TODO from platform data */
	cp_write(sd, 0x69, 0x14);   /* Enable CP CSC */
	io_write(sd, 0x06, 0xa6);   /* positive VS and HS and DE */
	io_write(sd, 0x14, 0x7f);   /* Drive strength adjusted to max */
	cp_write(sd, 0xba, (pdata->hdmi_free_run_mode << 1) | 0x01); /* HDMI free run */
	cp_write(sd, 0xf3, 0xdc); /* Low threshold to enter/exit free run mode */

	/* TODO from platform data */
	afe_write(sd, 0xb5, 0x01);  /* Setting MCLK to 256Fs */

	afe_write(sd, 0x02, pdata->ain_sel); /* Select analog input muxing mode */

	select_input(sd, pdata->prim_mode, pdata->vid_std_select);

	enable_input(sd, pdata->prim_mode);

	/* interrupts */
	io_write(sd, 0x40, 0xc2); /* Configure INT1 */
	io_write(sd, 0x41, 0xc7); /* Configure INT2 */
	io_write(sd, 0x46, 0x98); /* Enable SSPD, STDI and CP unlocked interrupts */
	io_write(sd, 0x78, 0x03); /* Enable V_LOCKED and DE_REGEN_LCK interrupts */
	io_write(sd, 0x73, 0x02); /* Enable CABLE_DET_A_ST (+5v) interrupt */

	return v4l2_ctrl_handler_setup(sd->ctrl_handler);
}

static void adv7842_unregister_clients(struct adv7842_state *state)
{
	if (state->i2c_avlink)
		i2c_unregister_device(state->i2c_avlink);
	if (state->i2c_cec)
		i2c_unregister_device(state->i2c_cec);
	if (state->i2c_infoframe)
		i2c_unregister_device(state->i2c_infoframe);
	if (state->i2c_sdp_io)
		i2c_unregister_device(state->i2c_sdp_io);
	if (state->i2c_sdp)
		i2c_unregister_device(state->i2c_sdp);
	if (state->i2c_afe)
		i2c_unregister_device(state->i2c_afe);
	if (state->i2c_repeater)
		i2c_unregister_device(state->i2c_repeater);
	if (state->i2c_edid)
		i2c_unregister_device(state->i2c_edid);
	if (state->i2c_hdmi)
		i2c_unregister_device(state->i2c_hdmi);
	if (state->i2c_cp)
		i2c_unregister_device(state->i2c_cp);
	if (state->i2c_vdp)
		i2c_unregister_device(state->i2c_vdp);
}

static struct i2c_client *adv7842_dummy_client(struct v4l2_subdev *sd,
							u8 addr, u8 io_reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	io_write(sd, io_reg, addr << 1);
	return i2c_new_dummy(client->adapter, io_read(sd, io_reg) >> 1);
}

static int adv7842_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct adv7842_state *state;
	struct adv7842_platform_data *pdata = client->dev.platform_data;
	struct v4l2_ctrl_handler *hdl;
	struct v4l2_subdev *sd;
	u16 rev;
	int err;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	v4l_dbg(1, debug, client, "detecting adv7842 client on address 0x%x\n",
			client->addr << 1);

	if (pdata == NULL) {
		v4l_err(client, "No platform data!\n");
		return -ENODEV;
	}

	state = kzalloc(sizeof(struct adv7842_state), GFP_KERNEL);
	if (state == NULL) {
		v4l_err(client, "Could not allocate adv7842_state memory!\n");
		return -ENOMEM;
	}

	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &adv7842_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	state->connector_hdmi = pdata->connector_hdmi;
	state->prim_mode = pdata->prim_mode;
	state->pdata = pdata;
	state->opf = &pdata->opf[0];

	hdl = &state->hdl;
	v4l2_ctrl_handler_init(hdl, 5);

	/* i2c access to adv7842? */
	rev = adv_smbus_read_byte_data_check(client, 0xea, false) << 8 |
		adv_smbus_read_byte_data_check(client, 0xeb, false);
	if (rev != 0x2012) {
		v4l2_info(sd, "not an adv7842 on address 0x%x (rev=0x%04x)\n",
				client->addr << 1, rev);
		err = -ENODEV;
		goto err_hdl;
	}

	/* main reset */
	io_write(sd, 0xff, 0x80);
	do {
	} while (io_read(sd, 0xff) & 0x80);

	/* add in ascending ID order */
	v4l2_ctrl_new_std(hdl, &adv7842_ctrl_ops,
			V4L2_CID_BRIGHTNESS, -128, 127, 1, 0);
	v4l2_ctrl_new_std(hdl, &adv7842_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 255, 1, 128);
	v4l2_ctrl_new_std(hdl, &adv7842_ctrl_ops,
			V4L2_CID_SATURATION, 0, 255, 1, 128);
	v4l2_ctrl_new_std(hdl, &adv7842_ctrl_ops,
			V4L2_CID_HUE, 0, 128, 1, 0);

	/* custom control - Tx 5V */

	state->i2c_avlink = adv7842_dummy_client(sd, pdata->i2c_avlink, 0xf3);
	state->i2c_cec = adv7842_dummy_client(sd, pdata->i2c_cec, 0xf4);
	state->i2c_infoframe = adv7842_dummy_client(sd, pdata->i2c_infoframe, 0xf5);
	state->i2c_sdp_io = adv7842_dummy_client(sd, pdata->i2c_sdp_io, 0xf2);
	state->i2c_sdp = adv7842_dummy_client(sd, pdata->i2c_sdp, 0xf1);
	state->i2c_afe = adv7842_dummy_client(sd, pdata->i2c_afe, 0xf8);
	state->i2c_repeater = adv7842_dummy_client(sd, pdata->i2c_repeater, 0xf9);
	state->i2c_edid = adv7842_dummy_client(sd, pdata->i2c_edid, 0xfa);
	state->i2c_hdmi = adv7842_dummy_client(sd, pdata->i2c_hdmi, 0xfb);
	state->i2c_cp = adv7842_dummy_client(sd, pdata->i2c_cp, 0xfd);
	state->i2c_vdp = adv7842_dummy_client(sd, pdata->i2c_vdp, 0xfe);
	if (!state->i2c_avlink || !state->i2c_cec || !state->i2c_infoframe ||
	    !state->i2c_sdp_io || !state->i2c_sdp || !state->i2c_afe ||
	    !state->i2c_repeater || !state->i2c_edid || !state->i2c_hdmi ||
	    !state->i2c_cp || !state->i2c_vdp) {
		err = -ENOMEM;
		v4l2_err(sd, "failed to create all i2c clients\n");
		goto err_i2c;
	}
	if (pdata->i2c_ex) {
		struct i2c_client *i2c_ex;
		i2c_ex = i2c_new_dummy(client->adapter, pdata->i2c_ex);
		/* enable 24-bit mode and sport */
		adv_smbus_write_byte_data(i2c_ex, 0x14, 0xfa);
		adv_smbus_write_byte_data(i2c_ex, 0x15, 0xff);
		adv_smbus_write_byte_data(i2c_ex, 0x0, 0x0);
		adv_smbus_write_byte_data(i2c_ex, 0x1, 0x0);
		i2c_unregister_device(i2c_ex);
	}

	/* work queues */
	state->work_queues = create_singlethread_workqueue(client->name);
	if (state->work_queues == NULL) {
		v4l2_err(sd, "Could not create work queue\n");
		err = -ENOMEM;
		goto err_i2c;
	}

	INIT_DELAYED_WORK(&state->delayed_work_enable_hotplug, adv7842_delayed_work_enable_hotplug);

	state->pad.flags = MEDIA_PAD_FL_SOURCE;
	err = media_entity_init(&sd->entity, 1, &state->pad, 0);
	if (err)
		goto err_work_queues;

	err = adv7842_core_init(sd, pdata);
	if (err)
		goto err_entity;
	if (state->prim_mode == ADV7842_PRIM_MODE_SDP) {
		state->std = V4L2_STD_UNKNOWN;
		adv7842_s_std(sd, V4L2_STD_PAL);
	}
	v4l2_info(sd, "%s found @ 0x%x (%s)\n", client->name,
			client->addr << 1, client->adapter->name);
	return 0;

err_entity:
	media_entity_cleanup(&sd->entity);
err_work_queues:
	cancel_delayed_work(&state->delayed_work_enable_hotplug);
	destroy_workqueue(state->work_queues);
err_i2c:
	adv7842_unregister_clients(state);
err_hdl:
	v4l2_ctrl_handler_free(hdl);
	kfree(state);
	return err;
}

/* ----------------------------------------------------------------------- */

static int adv7842_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv7842_state *state = to_state(sd);

	cancel_delayed_work(&state->delayed_work_enable_hotplug);
	destroy_workqueue(state->work_queues);
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	adv7842_unregister_clients(to_state(sd));
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	kfree(to_state(sd));
	return 0;
}

/* ----------------------------------------------------------------------- */

static struct i2c_device_id adv7842_id[] = {
	{ "adv7842", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adv7842_id);

static struct i2c_driver adv7842_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "adv7842",
	},
	.probe = adv7842_probe,
	.remove = adv7842_remove,
	.id_table = adv7842_id,
};

static int __init adv7842_init(void)
{
	return i2c_add_driver(&adv7842_driver);
}

static void __exit adv7842_exit(void)
{
	i2c_del_driver(&adv7842_driver);
}

module_init(adv7842_init);
module_exit(adv7842_exit);

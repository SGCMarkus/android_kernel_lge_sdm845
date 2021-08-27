#define pr_fmt(fmt)	"[Display][sw49410_rev1-ops:%s:%d] " fmt, __func__, __LINE__

#include "dsi_panel.h"
#include "lge_ddic_ops_helper.h"
#include "lge_dsi_panel_def.h"
#include "mplus/lge_mplus.h"
#include "brightness/lge_brightness_def.h"
#include "cm/lge_color_manager.h"

#define ADDR_PTLAR 0x30
#define ADDR_PLTAC 0x31
#define ADDR_RDDISPM 0x3F
#define ADDR_U2CTRL 0xCD
#define ENABLE_SCROLL	0x09
#define DISABLE_SCROLL	0x00

#define WORDS_TO_BYTE_ARRAY(w1, w2, b) do {\
		b[0] = WORD_UPPER_BYTE(w1);\
		b[1] = WORD_LOWER_BYTE(w1);\
		b[2] = WORD_UPPER_BYTE(w2);\
		b[3] = WORD_LOWER_BYTE(w2);\
} while(0)

#define MAX_HIGH_TEMP_PANEL_TUNE_LEVEL 4

extern int lge_ddic_dsi_panel_tx_cmd_set(struct dsi_panel *panel,
				enum lge_ddic_dsi_cmd_set_type type);

extern int lge_backlight_device_update_status(struct backlight_device *bd);
extern int lge_mplus_backlight_dimming(struct backlight_device *bd,
		enum lge_mplus_mode old_mp_mode, enum lge_mplus_mode cur_mp_mode);

const struct drs_res_info sw49410_rev1_res[3] = {
	{"qhd", 0, 1440, 3120},
	{"fhd", 1, 1080, 2340},
	{"hd", 3, 720, 1560},
};

#define IDX_DG_CTRL1	0
#define REG_DG_CTRL1	0xF6

#define IDX_DG_CTRL2	1
#define REG_DG_CTRL2	0xF7

#define IDX_DG_CTRL3	2
#define REG_DG_CTRL3	0xF8

#define IDX_DG_CTRL4	3
#define REG_DG_CTRL4	0xF5

#define NUM_DG_CTRL	0x10
#define OFFSET_DG_CTRL	8
#define DG_MODE_MAX	4

#define DG_OFF		0
#define DG_ON		1

#define STEP_DG_PRESET	5
#define NUM_DG_PRESET	10

#define STEP_GC_PRESET	5

enum {
	PRESET_SETP0_OFFSET = 0,
	PRESET_SETP1_OFFSET = 2,
	PRESET_SETP2_OFFSET = 5
};

static int rgb_preset[STEP_DG_PRESET][RGB_ALL] = {
	{PRESET_SETP2_OFFSET, PRESET_SETP0_OFFSET, PRESET_SETP2_OFFSET},
	{PRESET_SETP2_OFFSET, PRESET_SETP1_OFFSET, PRESET_SETP2_OFFSET},
	{PRESET_SETP0_OFFSET, PRESET_SETP0_OFFSET, PRESET_SETP0_OFFSET},
	{PRESET_SETP0_OFFSET, PRESET_SETP1_OFFSET, PRESET_SETP0_OFFSET},
	{PRESET_SETP0_OFFSET, PRESET_SETP2_OFFSET, PRESET_SETP0_OFFSET}
};

static int gc_preset[STEP_GC_PRESET][RGB_ALL] = {
	{0x00, 0x00, 0x00},
	{0x00, 0x01, 0x05},
	{0x04, 0x01, 0x00},
	{0x02, 0x00, 0x00},
	{0x02, 0x00, 0x02},
};

static char dg_ctrl_values[NUM_DG_PRESET][OFFSET_DG_CTRL] = {
	{0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40},
	{0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F},
	{0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x3E},
	{0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D},
	{0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C},

	{0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B},
	{0x3A, 0x3A, 0x3A, 0x3A, 0x3A, 0x3A, 0x3A, 0x3A},
	{0x39, 0x39, 0x39, 0x39, 0x39, 0x39, 0x39, 0x39},
	{0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38},
	{0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37}
};

#define NUM_SHA_CTRL		5
#define SHA_OFF			0
#define SHA_ON			1


#define NUM_SC_CTRL		4
#define OFFSET_SC_CTRL		1

#define NUM_SAT_CTRL		6
#define OFFSET_SAT_CTRL		8

#define NUM_HUE_CTRL		5
#define OFFSET_HUE_CTRL		2

#define IDX_SC_CTRL1		0
#define REG_SC_CTRL1		0xF3
#define NUM_SC_CTRL1		0x06
#define OFFSET_SC_CTRL1		2

#define IDX_SC_CTRL2		1
#define REG_SC_CTRL2		0xF4
#define NUM_SC_CTRL2		0x0E
#define OFFSET_SC_CTRL2		9

#define SC_MODE_MAX		4

#define LGE_SCREEN_TUNE_OFF	0
#define LGE_SCREEN_TUNE_ON	1
#define LGE_SCREEN_TUNE_GAM	2
#define LGE_SCREEN_TUNE_GAL	3
#define LGE_SAT_GAM_MODE	3
#define LGE_SAT_GAL_MODE	5

static char sha_ctrl_values[NUM_SHA_CTRL] = {0x00, 0x0D, 0x1A, 0x30, 0xD2};

static char sc_ctrl_values[NUM_SC_CTRL] = {0x00, 0x0F, 0x08, 0x0C};

static char sat_ctrl_values[NUM_SAT_CTRL][OFFSET_SAT_CTRL] = {
	{0x00, 0x38, 0x70, 0xA8, 0xE1, 0x00, 0x00, 0x00},
	{0x00, 0x3C, 0x78, 0xB4, 0xF1, 0x00, 0x00, 0x00},
	{0x00, 0x40, 0x80, 0xC0, 0x00, 0x01, 0x00, 0x00},
	{0x00, 0x43, 0x87, 0xCB, 0x00, 0x01, 0x00, 0x00},
	{0x00, 0x47, 0x8F, 0xD7, 0x00, 0x01, 0x00, 0x00},
	{0x00, 0x52, 0x8C, 0xD7, 0x00, 0x01, 0x90, 0x90},
};

static char hue_ctrl_values[NUM_HUE_CTRL][OFFSET_HUE_CTRL] = {
	{0xF7, 0x00},
	{0xF4, 0x00},
	{0xF0, 0x00},
	{0x74, 0x00},
	{0x77, 0x00},
};

static void adjust_roi(struct dsi_panel *panel, int *sr, int *er)
{
	u32 cur_res = panel->cur_mode->timing.v_active;
	int type, num = panel->num_timing_nodes;

	for (type = 0; type < num; type++) {
		if (cur_res == sw49410_rev1_res[type].height)
			break;
	}
	if (type == num) {
		pr_err("invalid height\n");
		*sr = 0;
		*er = panel->cur_mode->timing.h_active - 1;
		return;
	}

	if ((panel->lge.aod_area.w == 0) || (panel->lge.aod_area.h == 0)) {
		pr_err("area (w=%d)(h=%d), please check with application\n",
				panel->lge.aod_area.w, panel->lge.aod_area.h);
		goto full_roi;
	}

	*sr = 0;
	*er = panel->lge.aod_area.h - 1;

	return;

full_roi:
	*sr = 0;
	*er = *sr + sw49410_rev1_res[0].height - 1;
	return;
}

static void prepare_cmd(struct dsi_cmd_desc *cmds, int cmds_count, int addr, int param1, int param2)
{
	struct dsi_cmd_desc *cmd = NULL;
	char *payload = NULL;


	cmd = find_cmd(cmds, cmds_count, addr);
	if (cmd) {
		payload = (char *)cmd->msg.tx_buf;
		payload++;
		WORDS_TO_BYTE_ARRAY(param1, param2, payload);
		pr_info("30h : 0x%02x 0x%02X 0x%02X 0x%02X\n", payload[0],payload[1],payload[2],payload[3]);
	} else {
		pr_warn("cmd for addr 0x%02X not found\n", addr);
	}
}

static void prepare_scroll_cmd(struct dsi_panel *panel, struct dsi_cmd_desc *cmds,
			int cmds_count, int addr, int sr)
{
	struct dsi_cmd_desc *cmd = NULL;
	char *payload = NULL;
	u32 cur_res = panel->cur_mode->timing.v_active;
	int type, num = panel->num_timing_nodes;
	char upper, lower;

	for (type = 0; type < num; type++) {
		if (cur_res == sw49410_rev1_res[type].height)
			break;
	}
	if (type == num) {
		pr_err("invalid height\n");
		sr = 0;
		return;
	}

	if (sr < 0)	sr = 0;

	cmd = find_cmd(cmds, cmds_count, addr);
	if (cmd) {
		if (strncmp(sw49410_rev1_res[type].resolution, "qhd", 3)) {
			payload = (char *)cmd->msg.tx_buf;
			payload[1] = DISABLE_SCROLL;
			pr_info("Scroll doesn't surpported in HD and FHD.(%d)\n", type);
			return;
		} else { /* QHD */
			payload = (char *)cmd->msg.tx_buf;
			payload[1] = ENABLE_SCROLL;
			payload = (char *)cmd->msg.tx_buf + 13;
			upper = WORD_UPPER_BYTE(sr);
			lower = WORD_LOWER_BYTE(sr);
			payload[0] = (lower & 0x0f) << 4;
			payload[1] = (((upper & 0x0f) << 4) | ((lower & 0xf0) >> 4));
			pr_info("CDh : 0x%02x 0x%02x\n", payload[0], payload[1]);
		}
	} else {
		pr_warn("cmd for addr 0x%02X not found\n", addr);
	}
}

static void prepare_aod_area_sw49410_rev1(struct dsi_panel *panel, struct dsi_cmd_desc *cmds, int cmds_count)
{
	int sr = 0, er = 0;

	if (panel == NULL || cmds == NULL || cmds_count == 0)
		return;

	adjust_roi(panel, &sr, &er);
	prepare_cmd(cmds, cmds_count, ADDR_PTLAR, sr, er);
	prepare_scroll_cmd(panel, cmds, cmds_count, ADDR_U2CTRL, panel->lge.aod_area.y-1);

	return;
}

static int prepare_aod_cmds_sw49410_rev1(struct dsi_panel *panel, struct dsi_cmd_desc *cmds, int cmds_count)
{
	int rc = 0;

	if (panel == NULL || cmds == NULL || cmds_count == 0)
		return -EINVAL;

	return rc;
}

void sharpness_set_send_sw49410_rev1(struct dsi_panel *panel)
{
	u8 *payload;
	int mode;

	mutex_lock(&panel->panel_lock);

	if (panel->lge.sharpness_control == true ) {
		pr_info("skip sharpness control! \n");
		mutex_unlock(&panel->panel_lock);
		return;
	}

	payload = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_SET_SHARPNESS].cmds[0].msg.tx_buf;

	mode = panel->lge.sharpness;

	if (mode == 0) {
		payload[1] = 0x00;
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_SHARPNESS);
	} else {
		payload[1] = 0x01;
		payload[4] = mode;
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_SHARPNESS);
	}
	mutex_unlock(&panel->panel_lock);
}

static void lge_set_rgb_tune_send_sw49410_rev1(struct dsi_panel *panel, enum lge_gamma_correction_mode gc_mode)
{
	int i = 0;
	int red_index = 0, green_index = 0, blue_index = 0, dg_status = 0;
	int red_sum = 0, green_sum = 0, blue_sum = 0;
	char *payload_ctrl1 = NULL;
	char *payload_ctrl2 = NULL;
	char *payload_ctrl3 = NULL;
	char *payload_ctrl4 = NULL;

	if (panel == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	payload_ctrl1 = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY].cmds[IDX_DG_CTRL1].msg.tx_buf;
	payload_ctrl2 = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY].cmds[IDX_DG_CTRL2].msg.tx_buf;
	payload_ctrl3 = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY].cmds[IDX_DG_CTRL3].msg.tx_buf;
	payload_ctrl4 = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY].cmds[IDX_DG_CTRL4].msg.tx_buf;

	if(gc_mode == LGE_GC_MOD_NOR) {
		if (panel->lge.cm_red_step > DG_MODE_MAX)
			panel->lge.cm_red_step = DG_MODE_MAX;
		if (panel->lge.cm_green_step > DG_MODE_MAX)
			panel->lge.cm_green_step = DG_MODE_MAX;
		if (panel->lge.cm_blue_step > DG_MODE_MAX)
			panel->lge.cm_blue_step = DG_MODE_MAX;

		if ((panel->lge.cm_preset_step == RGB_DEFAULT_PRESET) &&
				(panel->lge.cm_red_step == RGB_DEFAULT_RED) &&
				(panel->lge.cm_blue_step == RGB_DEFAULT_BLUE) &&
				(panel->lge.cm_green_step == RGB_DEFAULT_GREEN)) {
			dg_status = DG_OFF;
		} else {
			dg_status = DG_ON;
		}

		if (dg_status == DG_ON) {
			red_index   = rgb_preset[panel->lge.cm_preset_step][RED] + panel->lge.cm_red_step;
			green_index = rgb_preset[panel->lge.cm_preset_step][GREEN] + panel->lge.cm_green_step;
			blue_index  = rgb_preset[panel->lge.cm_preset_step][BLUE] + panel->lge.cm_blue_step;

			pr_info("red_index=(%d) green_index=(%d) blue_index=(%d)\n", red_index, green_index, blue_index);

			for (i = 1; i < OFFSET_DG_CTRL+1; i++) {
				payload_ctrl1[i] = dg_ctrl_values[red_index][i-1];
				payload_ctrl2[i] = dg_ctrl_values[green_index][i-1];
				payload_ctrl3[i] = dg_ctrl_values[blue_index][i-1];
			}
		}
		payload_ctrl4[1] = dg_status;
	} else {
		red_index   = gc_preset[gc_mode][RED];
		green_index = gc_preset[gc_mode][GREEN];
		blue_index  = gc_preset[gc_mode][BLUE];

		for (i = 1; i < OFFSET_DG_CTRL+1; i++) {
			payload_ctrl1[i] = dg_ctrl_values[red_index][i-1];
			payload_ctrl2[i] = dg_ctrl_values[green_index][i-1];
			payload_ctrl3[i] = dg_ctrl_values[blue_index][i-1];
		}

		dg_status = DG_ON;
		payload_ctrl4[1] = dg_status;
	}

	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY);

	for (i = 1; i < NUM_DG_CTRL+1; i++) {
		red_sum += payload_ctrl1[i];
		green_sum += payload_ctrl2[i];
		blue_sum += payload_ctrl3[i];
	}

	pr_info("[0x%02x:0x%02x][0x%02x:0x%02x:%d][0x%02x:0x%02x:%d][0x%02x:0x%02x:%d]\n",
			payload_ctrl4[0], payload_ctrl4[1],
			payload_ctrl1[0], payload_ctrl1[1], red_sum,
			payload_ctrl2[0], payload_ctrl2[1], green_sum,
			payload_ctrl3[0], payload_ctrl3[1], blue_sum);

	return;
}

static void lge_set_screen_tune_send_sw49410_rev1(struct dsi_panel *panel,
		int lge_screen_tune_status)
{
	int i;
	char *payload_ctrl1 = NULL;
	char *payload_ctrl2 = NULL;

	payload_ctrl1 = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_DISP_SC_COMMAND_DUMMY].cmds[IDX_SC_CTRL1].msg.tx_buf;
	payload_ctrl2 = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_DISP_SC_COMMAND_DUMMY].cmds[IDX_SC_CTRL2].msg.tx_buf;

	if	(lge_screen_tune_status == LGE_SCREEN_TUNE_ON) {
		if (panel->lge.sc_sat_step > SC_MODE_MAX)
			panel->lge.sc_sat_step = SC_MODE_MAX;
		if (panel->lge.sc_hue_step > SC_MODE_MAX)
			panel->lge.sc_hue_step = SC_MODE_MAX;
		if (panel->lge.sc_sha_step > SC_MODE_MAX)
			panel->lge.sc_sha_step = SC_MODE_MAX;

		payload_ctrl1[1] = SHA_ON;
		payload_ctrl1[4] = sha_ctrl_values[panel->lge.sc_sha_step];
		panel->lge.sharpness_control = true;

		payload_ctrl2[1] = sc_ctrl_values[lge_screen_tune_status];
		for (i = 0; i < OFFSET_SAT_CTRL; i++)
			payload_ctrl2[i+2] = sat_ctrl_values[panel->lge.sc_sat_step][i];
		for (i = 0; i < OFFSET_HUE_CTRL; i++)
			payload_ctrl2[i+8] = hue_ctrl_values[panel->lge.sc_hue_step][i];
	} else {
		if (panel->lge.sharpness == SHA_OFF) {
			payload_ctrl1[1] = SHA_OFF;
		} else {
			payload_ctrl1[1] = SHA_ON;
			payload_ctrl1[4] = panel->lge.sharpness;
		}

		if (lge_screen_tune_status == LGE_SCREEN_TUNE_GAM) {
			panel->lge.sc_sat_step = LGE_SAT_GAM_MODE;
			payload_ctrl2[1] = sc_ctrl_values[lge_screen_tune_status];
			for (i = 0; i < OFFSET_SAT_CTRL; i++)
				payload_ctrl2[i+2] = sat_ctrl_values[panel->lge.sc_sat_step][i];
		} else if (lge_screen_tune_status == LGE_SCREEN_TUNE_GAL) {
			panel->lge.sc_sat_step = LGE_SAT_GAL_MODE;
			payload_ctrl2[1] = sc_ctrl_values[lge_screen_tune_status];
			for (i = 0; i < OFFSET_SAT_CTRL; i++)
				payload_ctrl2[i+2] = sat_ctrl_values[panel->lge.sc_sat_step][i];
		} else {
			payload_ctrl2[1] = sc_ctrl_values[lge_screen_tune_status];
		}
		panel->lge.sharpness_control = false;
	}

	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_DISP_SC_COMMAND_DUMMY);

	pr_info("[0x%02x:0x%02x:0x%02x][0x%02x:0x%02x:0x%02x:0x%02x]\n",
			payload_ctrl1[0], payload_ctrl1[1], payload_ctrl1[4],
			payload_ctrl2[0], payload_ctrl2[1], payload_ctrl2[3],
			payload_ctrl2[8]);

	return;
}

void mplus_mode_send_sw49410_rev1(struct dsi_panel *panel, enum lge_mplus_mode req_mp_mode)
{
	u8 data;
	u8 *payload;
	enum lge_dic_mplus_mode mp_mode;
	enum lge_dic_mplus_mode_set mp_mode_set;
	enum lge_mplus_mode old_mp_mode;
	enum lge_gamma_correction_mode gc_mode = LGE_GC_MOD_NOR;
	int lge_screen_tune_status = LGE_SCREEN_TUNE_OFF;

	mutex_lock(&panel->panel_lock);
	if ((panel->lge.lp_state == LGE_PANEL_LP2) || (panel->lge.lp_state == LGE_PANEL_LP1)) {
		pr_err("Panel Status LP Mode\n");
		mutex_unlock(&panel->panel_lock);
		return;
	}

	old_mp_mode = panel->lge.cur_mp_mode;

	switch (req_mp_mode) {
	case LGE_MP_PSM:
	case LGE_MP_PS2:
		mp_mode = LGE_DIC_MP_PSM;
		mp_mode_set = LGE_MODE_SET_1ST;
		break;
	case LGE_MP_GAL:
	case LGE_MP_FHB:
		mp_mode = LGE_DIC_MP_GAL;
		mp_mode_set = LGE_MODE_SET_1ST;
		break;
	case LGE_MP_HBM:
		mp_mode = LGE_DIC_MP_HBM;
		mp_mode_set = LGE_MODE_SET_1ST;
		break;
	case LGE_MP_BRI:
		mp_mode = LGE_DIC_MP_GAL;
		mp_mode_set = LGE_MODE_SET_2ND;
		break;
	case LGE_MP_HQC:
		mp_mode = LGE_DIC_MP_GAL;
		mp_mode_set = LGE_MODE_SET_3RD;
		break;
	case LGE_MP_NOR:
	default:
		mp_mode = LGE_DIC_MP_NOR;
		mp_mode_set = LGE_MODE_SET_1ST;
		break;
	}

	switch (panel->lge.screen_mode) {
	case LGE_COLOR_CIN:
		gc_mode = LGE_GC_MOD_CIN;
		break;
	case LGE_COLOR_SPO:
		gc_mode = LGE_GC_MOD_SPO;
		break;
	case LGE_COLOR_GAM:
		gc_mode = LGE_GC_MOD_GAM;
		lge_screen_tune_status = LGE_SCREEN_TUNE_GAM;
		break;
	case LGE_COLOR_MAN:
		lge_screen_tune_status = LGE_SCREEN_TUNE_ON;
		if(panel->lge.color_filter)
			gc_mode = LGE_GC_MOD_HQC;
		break;
	case LGE_COLOR_OPT:
	default:
		break;
	}

	if ((panel->lge.mp_mode == LGE_MP_GAL) &&
			(lge_screen_tune_status == LGE_SCREEN_TUNE_OFF)) {
		lge_screen_tune_status = LGE_SCREEN_TUNE_GAL;
	}

	payload = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_SET_MPLUS].cmds[0].msg.tx_buf;
	data = payload[1];
	data &= 0xCF;
	data |= (mp_mode & 0x03) << 0x4;
	payload[1] = data;
	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_MPLUS);

	mp_mode_set += LGE_DDIC_DSI_MPLUS_MODE_SET1;
	lge_ddic_dsi_panel_tx_cmd_set(panel, (enum lge_ddic_dsi_cmd_set_type)mp_mode_set);

	lge_set_rgb_tune_send_sw49410_rev1(panel, gc_mode);
	lge_set_screen_tune_send_sw49410_rev1(panel, lge_screen_tune_status);

	mutex_unlock(&panel->panel_lock);

	sharpness_set_send_sw49410_rev1(panel);

	pr_info("mp mode : %x  mp mode set : %d\n", payload[1], mp_mode_set);

	if ((old_mp_mode == LGE_MP_HBM) || (req_mp_mode == LGE_MP_HBM) ||
			(old_mp_mode == LGE_MP_FHB) || (req_mp_mode == LGE_MP_FHB)) {
		pr_info("skip brightness dimming\n");
	} else {
		if (panel->bl_config.bd) {
			mutex_lock(&panel->bl_config.bd->update_lock);
			lge_mplus_backlight_dimming(panel->bl_config.bd, old_mp_mode,
					req_mp_mode);
			mutex_unlock(&panel->bl_config.bd->update_lock);
		} else {
			pr_err("brightness device null!\n");
		}
	}
	panel->lge.cur_mp_mode = req_mp_mode;

	pr_info("[mp_hd:%d][hdr:%d][hl:%d][max:%d][mp_adv:%d][mp:%d][gc:%d][sc_tun:%d][shap:%d]\n",
			panel->lge.mplus_hd, panel->lge.hdr_mode, panel->lge.hl_mode,
			panel->lge.mp_max, panel->lge.adv_mp_mode, panel->lge.mp_mode,
			gc_mode, lge_screen_tune_status, panel->lge.sharpness);
}

void mplus_mode_set_sub_sw49410_rev1(struct dsi_panel *panel)
{
	if (panel->lge.mplus_hd != LGE_MP_OFF)
		mplus_mode_send_sw49410_rev1(panel, panel->lge.mplus_hd);
	else if (panel->lge.hdr_mode)
		mplus_mode_send_sw49410_rev1(panel, LGE_MP_NOR);
	else if (panel->lge.hl_mode)
		mplus_mode_send_sw49410_rev1(panel, LGE_MP_HBM);
	else if (panel->lge.mp_max != LGE_MP_OFF)
		mplus_mode_send_sw49410_rev1(panel, LGE_MP_FHB);
	else if (panel->lge.adv_mp_mode != LGE_MP_OFF)
		mplus_mode_send_sw49410_rev1(panel, panel->lge.adv_mp_mode);
	else
		mplus_mode_send_sw49410_rev1(panel, panel->lge.mp_mode);
}

void lge_set_screen_mode_sw49410_rev1(struct dsi_panel *panel, bool send_cmd)
{
	switch (panel->lge.screen_mode) {
	case LGE_COLOR_ECO:
	case LGE_COLOR_GAM:
		panel->lge.adv_mp_mode = LGE_MP_PSM;
		break;
	case LGE_COLOR_CIN:
	case LGE_COLOR_SPO:
		panel->lge.adv_mp_mode = LGE_MP_NOR;
		break;
	case LGE_COLOR_MAN:
		if(panel->lge.color_filter)
			panel->lge.adv_mp_mode = LGE_MP_HQC;
		else
			panel->lge.adv_mp_mode = LGE_MP_NOR;
		break;
	case LGE_COLOR_OPT:
	case LGE_COLOR_MAX:
	default:
		panel->lge.adv_mp_mode = LGE_MP_OFF;
		break;
	}

	pr_info("screen_mode : %d, adv_mp_mode : %d\n", panel->lge.screen_mode, panel->lge.adv_mp_mode);

	mplus_mode_set_sub_sw49410_rev1(panel);
}

static void lge_set_rgb_tune_sw49410_rev1(struct dsi_panel *panel, bool send_cmd)
{
        mplus_mode_set_sub_sw49410_rev1(panel);
}

static void lge_set_screen_tune_sw49410_rev1(struct dsi_panel *panel)
{
        lge_set_screen_mode_sw49410_rev1(panel, true);
}

/* Hidden Menu Mplus Set */
enum lge_mplus_mode mplus_hd_get_sw49410_rev1(struct dsi_panel *panel)
{
	return panel->lge.mplus_hd;
}

void mplus_hd_set_sw49410_rev1(struct dsi_panel *panel, int mode)
{
	panel->lge.mplus_hd = mode;
	pr_debug("mplus_hd : %d\n", panel->lge.mplus_hd);

	mplus_mode_set_sub_sw49410_rev1(panel);

	if (panel->bl_config.bd)
		lge_backlight_device_update_status(panel->bl_config.bd);
}

/* High Luminance Mplus Set */
int hl_mode_get_sw49410_rev1(struct dsi_panel *panel)
{
	return panel->lge.hl_mode;
}

void hl_mode_set_sw49410_rev1(struct dsi_panel *panel, int mode)
{
	panel->lge.hl_mode = mode;
	pr_debug("hl_mode : %d\n", panel->lge.hl_mode);

	mplus_mode_set_sub_sw49410_rev1(panel);
}

/* Max Brightness Mplus Set */
enum lge_mplus_mode mplus_max_get_sw49410_rev1(struct dsi_panel *panel)
{
	return panel->lge.mp_max;
}

void mplus_max_set_sw49410_rev1(struct dsi_panel *panel, int mode)
{
	panel->lge.mp_max = mode;
	pr_debug("mp_max : %d\n", panel->lge.mp_max);

	mplus_mode_set_sub_sw49410_rev1(panel);
}

/* Advanced Mplus Set */
enum lge_mplus_mode mplus_mode_get_sw49410_rev1(struct dsi_panel *panel)
{
	return panel->lge.adv_mp_mode;
}

void mplus_mode_set_sw49410_rev1(struct dsi_panel *panel, int mode)
{
	panel->lge.adv_mp_mode = mode;
	pr_debug("adv_mp_mode : %d\n", panel->lge.adv_mp_mode);

	mplus_mode_set_sub_sw49410_rev1(panel);
}

/* Normal Mplus Set */
enum lge_mplus_mode image_enhance_get_sw49410_rev1(struct dsi_panel *panel)
{
	return panel->lge.mp_mode;
}

void image_enhance_set_sw49410_rev1(struct dsi_panel *panel, int mode)
{
	panel->lge.mp_mode = mode;
	pr_debug("mplus : %d\n", panel->lge.mp_mode);

	mplus_mode_set_sub_sw49410_rev1(panel);
}

enum lge_blmap_type mp_blmap_sw49410_rev1_sub(struct dsi_panel *panel, enum lge_mplus_mode req_mode)
{
	enum lge_blmap_type bl_type;

	if ((panel->lge.mplus_hd == LGE_MP_OFF) && (panel->lge.hl_mode)) {
		switch (req_mode) {
		case LGE_MP_PS2:
		case LGE_MP_PSM:
			bl_type = LGE_BLMAP_PSM_HL;
			break;
		case LGE_MP_HBM:
			bl_type = LGE_BLMAP_HBM_HD;
			break;
		case LGE_MP_FHB:
			bl_type = LGE_BLMAP_MAX;
			break;
		case LGE_MP_GAL:
			bl_type = LGE_BLMAP_GAL_HL;
			break;
		case LGE_MP_BRI:
			bl_type = LGE_BLMAP_BRI_HL;
			break;
		case LGE_MP_HQC:
			bl_type = LGE_BLMAP_HQC_HL;
			break;
		case LGE_MP_NOR:
		default:
			bl_type = LGE_BLMAP_DEFAULT_HL;
			break;
		}
	} else {
		switch (req_mode) {
		case LGE_MP_PS2:
		case LGE_MP_PSM:
			bl_type = LGE_BLMAP_PSM;
			break;
		case LGE_MP_HBM:
			bl_type = LGE_BLMAP_HBM_HD;
			break;
		case LGE_MP_FHB:
			bl_type = LGE_BLMAP_MAX;
			break;
		case LGE_MP_GAL:
			bl_type = LGE_BLMAP_GAL;
			break;
		case LGE_MP_BRI:
			bl_type = LGE_BLMAP_BRI;
			break;
		case LGE_MP_HQC:
			bl_type = LGE_BLMAP_HQC;
			break;
		case LGE_MP_NOR:
		default:
			bl_type = LGE_BLMAP_DEFAULT;
			break;
		}
	}
	return bl_type;
}

enum lge_blmap_type mp_blmap_sw49410_rev1(struct dsi_panel *panel)
{
	enum lge_blmap_type bl_type = LGE_BLMAP_DEFAULT;

	if (panel->lge.mplus_hd != LGE_MP_OFF)
		bl_type = mp_blmap_sw49410_rev1_sub(panel, panel->lge.mplus_hd);
	else if (panel->lge.mp_max != LGE_MP_OFF)
		bl_type = mp_blmap_sw49410_rev1_sub(panel, LGE_MP_FHB);
	else if (panel->lge.hdr_mode)
		bl_type = LGE_BLMAP_DEFAULT;
	else if(panel->lge.adv_mp_mode != LGE_MP_OFF)
		bl_type = mp_blmap_sw49410_rev1_sub(panel, panel->lge.adv_mp_mode);
	else
		bl_type = mp_blmap_sw49410_rev1_sub(panel, panel->lge.mp_mode);
	return bl_type;
}

int hdr_mode_set_sw49410_rev1(struct dsi_panel *panel, int input) {
	u8 *payload1, *payload2;

	mutex_lock(&panel->panel_lock);
	payload1 = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_SET_IMAGE_ENHANCEMENT].cmds[CABC_REG0].msg.tx_buf;
	payload2 = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_SET_IMAGE_ENHANCEMENT].cmds[CABC_REG1].msg.tx_buf;
	panel->lge.hdr_mode = input;

	if (input == HDR_OFF) {
		payload1[1] = IE_ON;
		payload2[4] = CABC_ON;
		pr_info("hdr mode off, 0x%02x:0x%02x 0x%02x:0x%02x\n",
				payload1[0], payload1[1], payload2[0], payload2[4]);
	} else if (input > HDR_OFF) {
		payload1[1] = IE_OFF;
		payload2[4] = CABC_OFF;
		pr_info("hdr mode off, 0x%02x:0x%02x 0x%02x:0x%02x\n",
				payload1[0], payload1[1], payload2[0], payload2[4]);
	} else {
		pr_err("invalid value.\n");
	}

	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_IMAGE_ENHANCEMENT);
	mutex_unlock(&panel->panel_lock);

	/* set mplus mode */
	mplus_mode_set_sub_sw49410_rev1(panel);
	if(panel->bl_config.bd)
		lge_backlight_device_update_status(panel->bl_config.bd);

	return input;
}

void sharpness_set_sw49410_rev1(struct dsi_panel *panel, int mode)
{
	panel->lge.sharpness = mode;
	pr_info("sharpness : %d\n", panel->lge.sharpness);

	sharpness_set_send_sw49410_rev1(panel);
	return;
}

void high_temp_mode_set_sw49410_rev1(struct dsi_panel *panel, int mode)
{
	mutex_lock(&panel->panel_lock);

	if (panel->lge.high_temp_tune_mode == mode) {
		pr_info("skip high_temp_panel_tune_mode control! mode:%d\n", mode);
		mutex_unlock(&panel->panel_lock);
		return;
	}

	if (mode < 0 || mode > MAX_HIGH_TEMP_PANEL_TUNE_LEVEL) {
		pr_info("invalid high_temp_panel_tune_mode:%d\n", mode);
		mutex_unlock(&panel->panel_lock);
		return;
	}

	panel->lge.high_temp_tune_mode = mode;
	pr_info("high_temp_tune_mode : %d\n", panel->lge.high_temp_tune_mode);

	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_HIGH_TEMP_TUNE0 + mode);

	mutex_unlock(&panel->panel_lock);
	return;
}

struct lge_ddic_ops sw49410_rev1_ops = {
	.store_aod_area = store_aod_area,
	.prepare_aod_cmds = prepare_aod_cmds_sw49410_rev1,
	.prepare_aod_area = prepare_aod_area_sw49410_rev1,
	.bist_ctrl = NULL,
	.release_bist = NULL,
	.lge_set_screen_mode = lge_set_screen_mode_sw49410_rev1,
	.lge_set_rgb_tune = lge_set_rgb_tune_sw49410_rev1,
	.lge_set_screen_tune  = lge_set_screen_tune_sw49410_rev1,
	.hdr_mode_set = hdr_mode_set_sw49410_rev1,
	.sharpness_set = sharpness_set_sw49410_rev1,
	.mplus_hd_get = mplus_hd_get_sw49410_rev1,
	.mplus_hd_set = mplus_hd_set_sw49410_rev1,
	.hl_mode_get = hl_mode_get_sw49410_rev1,
	.hl_mode_set = hl_mode_set_sw49410_rev1,
	.mplus_max_get = mplus_max_get_sw49410_rev1,
	.mplus_max_set = mplus_max_set_sw49410_rev1,
	.mplus_mode_get = mplus_mode_get_sw49410_rev1,
	.mplus_mode_set = mplus_mode_set_sw49410_rev1,
	.image_enhance_get = image_enhance_get_sw49410_rev1,
	.image_enhance_set = image_enhance_set_sw49410_rev1,
	.mp_blmap = mp_blmap_sw49410_rev1,
	.mp_blmap_sub = mp_blmap_sw49410_rev1_sub,
	.high_temp_mode_set = high_temp_mode_set_sw49410_rev1,
};

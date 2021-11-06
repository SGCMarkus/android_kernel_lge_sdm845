#define pr_fmt(fmt)	"[Display][sw43402-ops:%s:%d] " fmt, __func__, __LINE__

#include <linux/delay.h>
#include "dsi_panel.h"
#include "lge_ddic_ops_helper.h"
#include "lge_dsi_panel_def.h"
#include "lge_dsi_panel.h"
#include "cm/lge_color_manager.h"
#include "err_detect/lge_err_detect.h"

#define ADDR_PTLAR 0x30
#define ADDR_PLTAC 0x31
#define ADDR_RDDISPM 0x3F

#define ADDR_ERR_DETECT 0x9F

#define WORDS_TO_BYTE_ARRAY(w1, w2, b) do {\
		b[0] = WORD_UPPER_BYTE(w1);\
		b[1] = WORD_LOWER_BYTE(w1);\
		b[2] = WORD_UPPER_BYTE(w2);\
		b[3] = WORD_LOWER_BYTE(w2);\
} while(0)

extern int lge_mdss_dsi_panel_cmd_read(struct dsi_panel *panel,
					u8 cmd, int cnt, char* ret_buf);
extern int lge_ddic_dsi_panel_tx_cmd_set(struct dsi_panel *panel,
				enum lge_ddic_dsi_cmd_set_type type);
extern int lge_backlight_device_update_status(struct backlight_device *bd);
extern char* get_payload_addr(struct dsi_panel *panel, enum lge_ddic_dsi_cmd_set_type type, int position);
extern int get_payload_cnt(struct dsi_panel *panel, enum lge_ddic_dsi_cmd_set_type type, int position);

const struct drs_res_info sw43402_res[3] = {
	{"qhd", 0, 1440, 2880},
	{"fhd", 1, 1080, 2160},
	{"hd", 3, 720, 1440},
};

struct backup_info sw43402_backup_list[] = {
	{"lge", LGE_DDIC_DSI_SET_NOLP, "duty_init_set", 0xCA, 2},
	{"lge", LGE_DDIC_DSI_SET_NOLP, "duty_band1", 0xCB, 2},
	{"lge", LGE_DDIC_DSI_SET_NOLP, "duty_band2", 0xCC, 2},
	{"lge", LGE_DDIC_DSI_SET_WB_DEFAULT, "dg_ctrl_1", 0xCF, 1},
	{"lge", LGE_DDIC_DSI_SET_WB_DEFAULT, "dg_ctrl_2", 0xD0, 1},
	{"lge", LGE_DDIC_DSI_SET_NOLP, "duty_init_set", 0xCA, 2},
	{"lge", LGE_DDIC_DSI_BC_DEFAULT_DIMMING, "bc_dimming_set", 0xB8, 1},
	{"lge", LGE_DDIC_DSI_SET_VR_MODE_POST_OFF, "duty_init_set", 0xCA, 1},
	{"lge", LGE_DDIC_DSI_SET_VR_MODE_POST_OFF, "duty_band1", 0xCB, 1},
	{"lge", LGE_DDIC_DSI_SET_VR_MODE_POST_OFF, "duty_band2", 0xCC, 1},
};

#define IDX_DG_CTRL1 1
#define REG_DG_CTRL1 0xCF
#define NUM_DG_CTRL1 0x09
#define OFFSET_DG_CTRL1 3

#define IDX_DG_CTRL2 2
#define REG_DG_CTRL2 0xD0
#define NUM_DG_CTRL2 0x1B
#define OFFSET_DG_CTRL2 9

#define STEP_DG_PRESET 5
#define NUM_DG_PRESET  24

static int rgb_preset[STEP_DG_PRESET][RGB_ALL] = {
	{PRESET_SETP2_INDEX, PRESET_SETP0_INDEX, PRESET_SETP2_INDEX},
	{PRESET_SETP1_INDEX, PRESET_SETP0_INDEX, PRESET_SETP1_INDEX},
	{PRESET_SETP0_INDEX, PRESET_SETP0_INDEX, PRESET_SETP0_INDEX},
	{PRESET_SETP0_INDEX, PRESET_SETP1_INDEX, PRESET_SETP0_INDEX},
	{PRESET_SETP0_INDEX, PRESET_SETP2_INDEX, PRESET_SETP0_INDEX}
};

static char dg_ctrl1_values[NUM_DG_PRESET][OFFSET_DG_CTRL1] = {
	{0x03, 0x05, 0xAF},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},

	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},

	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},

	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},

	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B}
};

static char dg_ctrl2_values[NUM_DG_PRESET][OFFSET_DG_CTRL2] = {
	{0x00, 0xFF, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80},
	{0x00, 0xFB, 0x7F, 0xFE, 0x7E, 0xFD, 0x7D, 0xFC, 0x7C},
	{0x00, 0xF7, 0x7E, 0xFD, 0x7C, 0xFB, 0x7A, 0xF9, 0x78},
	{0x00, 0xF3, 0x7E, 0xFC, 0x7B, 0xF9, 0x78, 0xF6, 0x75},
	{0x00, 0xEF, 0x7D, 0xFB, 0x79, 0xF7, 0x75, 0xF3, 0x71},

	{0x00, 0xEB, 0x7D, 0xFA, 0x78, 0xF5, 0x73, 0xF0, 0x6E},
	{0x00, 0xE7, 0x7C, 0xF9, 0x76, 0xF3, 0x70, 0xED, 0x6A},
	{0x00, 0xE3, 0x7C, 0xF8, 0x75, 0xF1, 0x6E, 0xEA, 0x67},
	{0x00, 0xDF, 0x7B, 0xF7, 0x73, 0xEF, 0x6B, 0xE7, 0x63},
	{0x00, 0xDB, 0x7B, 0xF6, 0x72, 0xED, 0x69, 0xE4, 0x60},

	{0x00, 0xD7, 0x7A, 0xF5, 0x70, 0xEB, 0x66, 0xE1, 0x5C},
	{0x00, 0xD3, 0x7A, 0xF4, 0x6F, 0xE9, 0x64, 0xDE, 0x59},
	{0x00, 0xCF, 0x79, 0xF3, 0x6D, 0xE7, 0x61, 0xDB, 0x55},
	{0x00, 0xCB, 0x79, 0xF2, 0x6C, 0xE5, 0x5F, 0xD8, 0x52},
	{0x00, 0xC7, 0x78, 0xF1, 0x6A, 0xE3, 0x5C, 0xD5, 0x4E},

	{0x00, 0xC3, 0x78, 0xF0, 0x69, 0xE1, 0x5A, 0xD2, 0x4B},
	{0x00, 0xBF, 0x77, 0xEF, 0x67, 0xDF, 0x57, 0xCF, 0x47},
	{0x00, 0xBB, 0x77, 0xEE, 0x66, 0xDD, 0x55, 0xCC, 0x44},
	{0x00, 0xB7, 0x76, 0xED, 0x64, 0xDB, 0x52, 0xC9, 0x40},
	{0x00, 0xB3, 0x76, 0xEC, 0x63, 0xD9, 0x50, 0xC6, 0x3D},

	{0x00, 0xAF, 0x75, 0xEB, 0x61, 0xD7, 0x4D, 0xC3, 0x39},
	{0x00, 0xAB, 0x75, 0xEA, 0x60, 0xD5, 0x4B, 0xC0, 0x36},
	{0x00, 0xA7, 0x74, 0xE9, 0x5E, 0xD3, 0x48, 0xBD, 0x32},
	{0x00, 0xA3, 0x74, 0xE8, 0x5D, 0xD1, 0x46, 0xBA, 0x2F}
};

static void adjust_roi(struct dsi_panel *panel, int *sr, int *er)
{
	u32 cur_res = panel->cur_mode->timing.v_active;
	int type, num = panel->num_timing_nodes;

	for (type = 0; type < num; type++) {
		if (cur_res == sw43402_res[type].height)
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

	if (!strncmp(sw43402_res[type].resolution, "hd", 2)) {
		goto full_roi;
	}
	else if (!strncmp(sw43402_res[type].resolution, "fhd", 3)) {
		*sr = (((panel->lge.aod_area.y) >> 2) << 2);
		*er = *sr + panel->lge.aod_area.h - 1;
	} else {
		*sr = panel->lge.aod_area.y;
		*er = *sr + panel->lge.aod_area.h - 1;
	}

	return;

full_roi:
	*sr = 0;
	*er = *sr + sw43402_res[0].height - 1;
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
	} else {
		pr_warn("cmd for addr 0x%02X not found\n", addr);
	}
}

static void prepare_aod_area_sw43402(struct dsi_panel *panel, struct dsi_cmd_desc *cmds, int cmds_count)
{
	int sr = 0, er = 0;

	if (panel == NULL || cmds == NULL || cmds_count == 0)
		return;

	adjust_roi(panel, &sr, &er);
	prepare_cmd(cmds, cmds_count, ADDR_PTLAR, sr, er);

	return;
}

static int prepare_aod_cmds_sw43402(struct dsi_panel *panel, struct dsi_cmd_desc *cmds, int cmds_count)
{
	int rc = 0;

	if (panel == NULL || cmds == NULL || cmds_count == 0)
		return -EINVAL;

	return rc;
}

static void lge_bc_dim_set_sw43402(struct dsi_panel *panel, u8 bc_dim_en, u8 bc_dim_f_cnt)
{
	mutex_lock(&panel->panel_lock);

	if (bc_dim_en == BC_DIM_ON) {
		((u8 *)(panel->lge.lge_cmd_sets[LGE_DDIC_DSI_BC_DEFAULT_DIMMING].cmds[1].msg.tx_buf))[22] = bc_dim_f_cnt;
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_BC_DEFAULT_DIMMING);
	}

	((u8 *)(panel->lge.lge_cmd_sets[LGE_DDIC_DSI_BC_DIMMING].cmds->msg.tx_buf))[1] = (bc_dim_en << 4);
	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_BC_DIMMING);

	mdelay(15);

	pr_info("BC_DIM_EN: 0x%x, FRAMES: 0x%x\n",
			((u8 *)(panel->lge.lge_cmd_sets[LGE_DDIC_DSI_BC_DIMMING].cmds->msg.tx_buf))[1],
			((u8 *)(panel->lge.lge_cmd_sets[LGE_DDIC_DSI_BC_DEFAULT_DIMMING].cmds[1].msg.tx_buf))[22]);

	mutex_unlock(&panel->panel_lock);
}

static int lge_set_therm_dim_sw43402(struct dsi_panel *panel, int input)
{
	u8 bc_dim_en;
	u8 bc_dim_f_cnt;

	if (panel->bl_config.bd->props.brightness < BC_DIM_BRIGHTNESS_THERM) {
		pr_info("Normal Mode. Skip therm dim. Current Brightness: %d\n", panel->bl_config.bd->props.brightness);
		return -EINVAL;
	}

	bc_dim_en = BC_DIM_ON;
	if (input)
		bc_dim_f_cnt = BC_DIM_FRAMES_THERM;
	else
		bc_dim_f_cnt = BC_DIM_FRAMES_NORMAL;

	if (panel->lge.use_bc_dimming_work)
		cancel_delayed_work_sync(&panel->lge.bc_dim_work);

	lge_bc_dim_set_sw43402(panel, bc_dim_en, bc_dim_f_cnt);

	panel->bl_config.bd->props.brightness = BC_DIM_BRIGHTNESS_THERM;
	lge_backlight_device_update_status(panel->bl_config.bd);

	if (panel->lge.use_bc_dimming_work)
		schedule_delayed_work(&panel->lge.bc_dim_work, BC_DIM_TIME);

	return 0;
}

static int lge_get_brightness_dim_sw43402(struct dsi_panel *panel)
{
	u8 bc_dim_en;
	u8 bc_dim_f_cnt;

	bc_dim_en = ((u8 *)(panel->lge.lge_cmd_sets[LGE_DDIC_DSI_BC_DIMMING].cmds->msg.tx_buf))[1];
	bc_dim_f_cnt = ((u8 *)(panel->lge.lge_cmd_sets[LGE_DDIC_DSI_BC_DEFAULT_DIMMING].cmds[1].msg.tx_buf))[22];

	pr_info("BC_DIM_EN: 0x%x, FRAMES: 0x%x\n", bc_dim_en, bc_dim_f_cnt);

	return bc_dim_f_cnt;
}

static void lge_set_brightness_dim_sw43402(struct dsi_panel *panel, int input)
{
	u8 bc_dim_en;
	u8 bc_dim_f_cnt;

	if (input == BC_DIM_OFF) {
		bc_dim_en = BC_DIM_OFF;
		bc_dim_f_cnt = BC_DIM_FRAMES_NORMAL;
	} else {
		bc_dim_en = BC_DIM_ON;
		if (input < BC_DIM_MIN_FRAMES)
			bc_dim_f_cnt = BC_DIM_MIN_FRAMES;
		else if (input > BC_DIM_MAX_FRAMES)
			bc_dim_f_cnt = BC_DIM_MAX_FRAMES;
		else
			bc_dim_f_cnt = input;
	}

	lge_bc_dim_set_sw43402(panel, bc_dim_en, bc_dim_f_cnt);
}

static void lge_set_custom_rgb_sw43402(struct dsi_panel *panel, bool send_cmd)
{
	int i = 0;
	int red_index, green_index, blue_index = 0;
	char *payload_ctrl1 = NULL;
	char *payload_ctrl2 = NULL;

	if (panel == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	mutex_lock(&panel->panel_lock);
	//cm_rgb_step : 0~11
	//rgb_index 0~11 + 0~12
	red_index   = rgb_preset[panel->lge.cm_preset_step][RED] + panel->lge.cm_red_step;
	green_index = rgb_preset[panel->lge.cm_preset_step][GREEN] + panel->lge.cm_green_step;
	blue_index  = rgb_preset[panel->lge.cm_preset_step][BLUE] + panel->lge.cm_blue_step;

	pr_info("red_index=(%d) green_index=(%d) blue_index=(%d)\n", red_index, green_index, blue_index);

	payload_ctrl1 = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY].cmds[IDX_DG_CTRL1].msg.tx_buf;
	payload_ctrl2 = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY].cmds[IDX_DG_CTRL2].msg.tx_buf;

	for (i = 1; i < OFFSET_DG_CTRL1+1 ; i++) {
		payload_ctrl1[i] = dg_ctrl1_values[red_index][i-1];  //payload_ctrl1[1][2][3]
		payload_ctrl1[i+OFFSET_DG_CTRL1] = dg_ctrl1_values[green_index][i-1]; //payload_ctrl1[4][5][6]
		payload_ctrl1[i+OFFSET_DG_CTRL1*2] = dg_ctrl1_values[blue_index][i-1]; //payload_ctrl1[7][8][9]
	}

	for (i = 1; i < OFFSET_DG_CTRL2+1 ; i++) {
		payload_ctrl2[i] = dg_ctrl2_values[red_index][i-1]; //payload_ctrl2[1]~[9]
		payload_ctrl2[i+OFFSET_DG_CTRL2] = dg_ctrl2_values[green_index][i-1]; //payload_ctrl2[10]~[18]
		payload_ctrl2[i+OFFSET_DG_CTRL2*2] = dg_ctrl2_values[blue_index][i-1]; //payload_ctrl2[19]~[27]
	}

	for (i = 0; i < NUM_DG_CTRL1; i++) {
		pr_info("Reg:0x%02x [%d:0x%02x]\n",
				REG_DG_CTRL1, i, (*(u8 *)(panel->lge.lge_cmd_sets[LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY].cmds[IDX_DG_CTRL1].msg.tx_buf+(i+1))));
	}

	for (i = 0; i < NUM_DG_CTRL2; i++) {
		pr_info("Reg:0x%02x [%d:0x%02x]\n",
				REG_DG_CTRL2, i, (*(u8 *)(panel->lge.lge_cmd_sets[LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY].cmds[IDX_DG_CTRL2].msg.tx_buf+(i+1))));
	}

	if (send_cmd) {
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY);
	}

	mutex_unlock(&panel->panel_lock);
	return;
}

static void lge_display_control_store_sw43402(struct dsi_panel *panel, bool send_cmd)
{
	char *payload1 = NULL;
	char *payload2 = NULL;

	mutex_lock(&panel->panel_lock);

	payload1 = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_DISP_CTRL_COMMAND_1].cmds[0].msg.tx_buf;
	payload2 = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_DISP_CTRL_COMMAND_2].cmds[0].msg.tx_buf;

	payload1[1] = ((panel->lge.dgc_status << 7) & 0x80) | \
					((panel->lge.vr_status << 3) & 0x08) | \
					((panel->lge.color_manager_status << 2) & 0x04) | \
					(panel->lge.color_manager_mode & 0x03);

	payload2[1] = ((panel->lge.hdr_hbm_lut << 6) & 0xC0) | \
					((panel->lge.ddic_hdr << 4) & 0x30) | \
					((panel->lge.hbm_mode << 2) & 0x0C) | \
					(panel->lge.acl_mode & 0x03);

	pr_info("ctrl-command-1: 0x%02x, ctrl-command-2: 0x%02x\n",
			(*(u8 *)(panel->lge.lge_cmd_sets[LGE_DDIC_DSI_DISP_CTRL_COMMAND_1].cmds[0].msg.tx_buf+1)),
			(*(u8 *)(panel->lge.lge_cmd_sets[LGE_DDIC_DSI_DISP_CTRL_COMMAND_2].cmds[0].msg.tx_buf+1)));

	if (send_cmd) {
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_DISP_CTRL_COMMAND_1);
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_DISP_CTRL_COMMAND_2);
	}

	mutex_unlock(&panel->panel_lock);
	return;
}

static void lge_send_screen_mode_cmd_sw43402(struct dsi_panel *panel, int index)
{
	mutex_lock(&panel->panel_lock);

	if (index == 0) {
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_CM_DCI_P3);
		pr_info("send DSI_CMD_SET_CM_DCI_P3\n");
	} else if (index == 1) {
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_CM_SRGB);
		pr_info("send DSI_CMD_SET_CM_SRGB\n");
	} else if (index == 2) {
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_CM_ADOBE);
		pr_info("send DSI_CMD_SET_CM_ADOBE\n");
	} else if (index == 3) {
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_CM_NATIVE);
		pr_info("send DSI_CMD_SET_CM_NATIVE\n");
	} else {
		pr_info("No send cmd %d\n", panel->lge.screen_mode);
	}

	mutex_unlock(&panel->panel_lock);
}

static void lge_set_screen_mode_sw43402(struct dsi_panel *panel, bool send_cmd)
{
	mutex_lock(&panel->panel_lock);

	if (panel->lge.screen_mode == 0) {
		panel->lge.color_manager_mode = panel->lge.color_manager_table[panel->lge.screen_mode].color_manager_mode;
		panel->lge.color_manager_status = panel->lge.color_manager_table[panel->lge.screen_mode].color_manager_status;
		panel->lge.dgc_status = 0x00;
	} else if (panel->lge.screen_mode == 10) {
		pr_info("preset : %d, red = %d, green = %d, blue = %d\n",
				panel->lge.cm_preset_step, panel->lge.cm_red_step,
				panel->lge.cm_green_step, panel->lge.cm_blue_step);
		panel->lge.color_manager_mode = panel->lge.color_manager_table[0].color_manager_mode;
		panel->lge.color_manager_status = panel->lge.color_manager_table[0].color_manager_status;
		if (panel->lge.cm_preset_step == 2 &&
				!(panel->lge.cm_red_step | panel->lge.cm_green_step | panel->lge.cm_blue_step)) {
			panel->lge.dgc_status = 0x00;
		} else {
			mutex_unlock(&panel->panel_lock);
			lge_set_custom_rgb_sw43402(panel, send_cmd);
			mutex_lock(&panel->panel_lock);
			panel->lge.dgc_status = 0x01;
		}
	} else {
		panel->lge.color_manager_mode = panel->lge.color_manager_table[panel->lge.screen_mode].color_manager_mode;
		panel->lge.color_manager_status = panel->lge.color_manager_table[panel->lge.screen_mode].color_manager_status;
		panel->lge.dgc_status = 0x01;

		if (send_cmd && panel->lge.lge_cmd_sets[LGE_DDIC_DSI_SET_WB_DEFAULT].count)
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_WB_DEFAULT);
	}
	mutex_unlock(&panel->panel_lock);

	lge_display_control_store_sw43402(panel, send_cmd);

	if (send_cmd)
		lge_send_screen_mode_cmd_sw43402(panel, panel->lge.color_manager_mode);

	pr_info("color_manager_status %d color_manager_mode %d\n",
			panel->lge.color_manager_status, panel->lge.color_manager_mode);

	return;
}

static void lge_set_hdr_hbm_lut_sw43402(struct dsi_panel *panel, int input)
{
	panel->lge.hdr_hbm_lut = input & 0x03;
	lge_display_control_store_sw43402(panel, true);

	pr_info("hdr_hbm_lut status %d\n", panel->lge.hdr_hbm_lut);
}

static void lge_set_acl_mode_sw43402(struct dsi_panel *panel, int input)
{
	panel->lge.acl_mode = input & 0x03;
	lge_display_control_store_sw43402(panel, true);

	pr_info("acl_mode status %d\n", panel->lge.acl_mode);
}

static void lge_set_video_enhancement_sw43402(struct dsi_panel *panel, int input)
{
	panel->lge.ddic_hdr = input & 0x03;
	lge_display_control_store_sw43402(panel, true);

	pr_info("send cmds to %s the video enhancer \n",
		(input == true) ? "enable" : "disable");

	if (panel->lge.use_bc_dimming_work)
		cancel_delayed_work_sync(&panel->lge.bc_dim_work);

	lge_bc_dim_set_sw43402(panel, BC_DIM_ON, BC_DIM_FRAMES_VE);

	lge_backlight_device_update_status(panel->bl_config.bd);

	if (panel->lge.use_bc_dimming_work)
		schedule_delayed_work(&panel->lge.bc_dim_work, BC_DIM_TIME);
}

int lge_set_low_persist_mode_disable_sw43402(struct dsi_panel *panel)
{
	int rc = 0;

	if (!panel){
		pr_err("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	/* UNLOCK REGISTER */
	if (panel->lge.use_ddic_reg_lock)
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_REGISTER_UNLOCK);

	panel->lge.vr_status = 0x00;
	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_VR_MODE_PRE_OFF);
	mutex_unlock(&panel->panel_lock);

	lge_display_control_store_sw43402(panel, true);

	mutex_lock(&panel->panel_lock);
	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_VR_MODE_POST_OFF);

	/* LOCK REGISTER */
	if (panel->lge.use_ddic_reg_lock)
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_REGISTER_LOCK);

	mutex_unlock(&panel->panel_lock);

	return rc;
}

int lge_set_low_persist_mode_enable_sw43402(struct dsi_panel *panel)
{
	int rc = 0;

	if (!panel){
		pr_err("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	/* UNLOCK REGISTER */
	if (panel->lge.use_ddic_reg_lock)
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_REGISTER_UNLOCK);

	panel->lge.vr_status = 0x01;
	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_VR_MODE_PRE_ON);
	mutex_unlock(&panel->panel_lock);

	lge_display_control_store_sw43402(panel, true);

	mutex_lock(&panel->panel_lock);
	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_VR_MODE_POST_ON);

	/* LOCK REGISTER */
	if (panel->lge.use_ddic_reg_lock)
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_REGISTER_LOCK);
	mutex_unlock(&panel->panel_lock);

	return rc;
}

int hdr_mode_set_sw43402(struct dsi_panel *panel, int input) {

	if (!panel)
		return -EINVAL;

	if (!panel->lge.color_manager_default_status) {
		pr_info("Color manager is disabled as default. Ignore color manager status control.\n");
		return -EINVAL;
	}

	panel->lge.color_manager_status = ((input == 0) ? 0x01 : 0x00);

	lge_display_control_store_sw43402(panel, true);
	lge_send_screen_mode_cmd_sw43402(panel, panel->lge.color_manager_mode);

	pr_info("color_manager_status : %d \n", panel->lge.color_manager_status);

	return 0;
}

static int control_bist_cmds_sw43402(struct dsi_panel *panel, bool enable)
{
	int rc = 0;
	bool state_changed = false;

	if (panel == NULL)
		return -EINVAL;

	if (enable) {
		if (panel->lge.bist_on == 0) {
			state_changed = true;
		}
		panel->lge.bist_on++;
	} else {
		if (panel->lge.bist_on == 1) {
			state_changed = true;
		}
		panel->lge.bist_on--;
		if (panel->lge.bist_on < 0) {
			pr_err("FIX: bist_on has negative count = %d\n", panel->lge.bist_on);
			panel->lge.bist_on = 0;
		}
	}

	pr_info("state_changed=%d, count=%d, enable=%d\n", state_changed, panel->lge.bist_on, enable);
	if (state_changed) {
		mutex_lock(&panel->panel_lock);
		if (enable) {
			rc = lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_BIST_ON);
			if (rc)
				pr_err("failed to send BIST_ON cmd, rc=%d\n", rc);
		} else {
			rc = lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_BIST_OFF);
			if (rc)
				pr_err("failed to send BIST_OFF cmd, rc=%d\n", rc);
		}
		mutex_unlock(&panel->panel_lock);
	}

	return rc;
}

static int release_bist_cmds_sw43402(struct dsi_panel *panel)
{
	int rc = 0;

	if (panel == NULL)
		return -EINVAL;

	if (panel->lge.bist_on > 0) {
		panel->lge.bist_on = 1;
		if (control_bist_cmds_sw43402(panel, false) < 0) {
			pr_warn("fail to bist control\n");
		}
	}

	return rc;
}

static int get_current_resolution_sw43402(struct dsi_panel *panel)
{
	u8 reg;

	lge_mdss_dsi_panel_cmd_read(panel, (u8)ADDR_RDDISPM, 1, &reg);

	return (int)reg;
}

static void get_support_resolution_sw43402(int idx, void *input)
{
	struct drs_res_info *res = (struct drs_res_info *)input;

	res->data = sw43402_res[idx].data;
	res->height = sw43402_res[idx].height;

	return;
}

static struct backup_info* get_ddic_backup_list_sw43402(int *cnt)
{
	int count = sizeof(sw43402_backup_list)/sizeof(sw43402_backup_list[0]);
	*cnt = count;

	return &sw43402_backup_list[0];
}

static int set_pps_cmds_sw43402(struct dsi_panel *panel, enum lge_ddic_dsi_cmd_set_type type)
{
	struct msm_display_dsc_info *dsc;

	if (!panel) {
		pr_err("panel is null\n");
		return -EINVAL;
	}

	dsc = &panel->cur_mode->priv_info->dsc;
	if (!dsc) {
		pr_err("dsc is null\n");
		return -EINVAL;
	}

	switch (type) {
	case LGE_DDIC_DSI_SET_LP2:
	case LGE_DDIC_DSI_AOD_AREA:
		pr_info("LP2: pic_height : %d -> %d\n",
				dsc->pic_height,
				panel->lge.aod_area.h);
		panel->lge.pps_orig = dsc->pic_height;
		dsc->pic_height = panel->lge.aod_area.h;
		break;
	case LGE_DDIC_DSI_SET_NOLP:
		pr_info("NOLP: pic_height : %d -> %d\n",
				dsc->pic_height,
				panel->cur_mode->timing.v_active);
		panel->lge.pps_orig = dsc->pic_height;
		dsc->pic_height = panel->cur_mode->timing.v_active;
		break;
	default:
		pr_warn("WARNING: not supported type\n");
		break;
	};

	return 0;
}

static int unset_pps_cmds_sw43402(struct dsi_panel *panel, enum lge_ddic_dsi_cmd_set_type type)
{
	struct msm_display_dsc_info *dsc;

	if (!panel) {
		pr_err("panel is null\n");
		return -EINVAL;
	}

	dsc = &panel->cur_mode->priv_info->dsc;
	if (!dsc) {
		pr_err("dsc is null\n");
		return -EINVAL;
	}

	switch (type) {
	case LGE_DDIC_DSI_SET_LP2:
	case LGE_DDIC_DSI_SET_NOLP:
		pr_info("pic_height : %d -> %d\n",
				dsc->pic_height,
				panel->lge.pps_orig);
		dsc->pic_height = panel->lge.pps_orig;
		break;
	default:
		pr_warn("WARNING: not supported type\n");
		break;
	};

	return 0;
}

void lge_check_vert_black_line_sw43402(struct dsi_panel *panel)
{
	int rc = 0;

	mutex_lock(&panel->panel_lock);
	pr_info("send LGE_DDIC_DSI_DETECT_BLACK_VERT_LINE\n");
	rc = lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_DETECT_BLACK_VERT_LINE);
	mutex_unlock(&panel->panel_lock);

	if (rc)
		pr_err("failed to send DETECT_BLACK_VERT_LINE cmd, rc=%d\n", rc);
}

void lge_check_vert_white_line_sw43402(struct dsi_panel *panel)
{
	int rc = 0;

	mutex_lock(&panel->panel_lock);
	pr_info("send LGE_DDIC_DSI_DETECT_WHITE_VERT_LINE\n");
	rc = lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_DETECT_WHITE_VERT_LINE);
	mutex_unlock(&panel->panel_lock);

	if (rc)
		pr_err("failed to send DETECT_WHITE_VERT_LINE cmd, rc=%d\n", rc);
}

void lge_check_vert_line_restore_sw43402(struct dsi_panel *panel)
{
	int rc = 0;

	pr_info("%s\n", __func__);

	mutex_lock(&panel->panel_lock);
	pr_info("send LGE_DDIC_DSI_DETECT_VERT_LINE_RESTORE");
	rc = lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_DETECT_VERT_LINE_RESTORE);
	mutex_unlock(&panel->panel_lock);

	if (rc)
		pr_err("failed to send DETECT_VERT_LINE_RESTORE cmd, rc=%d\n", rc);
}

static void err_detect_work_sw43402(struct work_struct *work)
{
	struct delayed_work *dw = to_delayed_work(work);
	struct lge_dsi_panel *lge_panel = container_of(dw, struct lge_dsi_panel, err_detect_int_work);
	struct dsi_panel *panel = container_of(lge_panel, struct dsi_panel, lge);
	u8 reg;

	if (!panel) {
		pr_err("invalid ctrl data\n");
		return;
	}

	mutex_lock(&panel->panel_lock);
	if (!dsi_panel_initialized(panel)) {
		pr_err("Panel off state. Ignore screen_mode set cmd\n");
		mutex_unlock(&panel->panel_lock);
		return;
	}

	if (panel->lge.err_detect_mask >= 0) {
		if (panel->lge.use_ddic_reg_lock)
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_REGISTER_UNLOCK);
		lge_mdss_dsi_panel_cmd_read(panel, (u8)ADDR_ERR_DETECT, 1, &reg);
		if (panel->lge.use_ddic_reg_lock)
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_REGISTER_LOCK);

		pr_info("Reg[0x%x]=0x%x,\n", ADDR_ERR_DETECT, reg);
		panel->lge.err_detect_result = reg;
	}
	mutex_unlock(&panel->panel_lock);

	if (panel->lge.err_detect_crash_enabled && panel->lge.err_detect_result) {
		pr_err("error is detected. BUG() \n");
		BUG();
	}
}

static irqreturn_t err_detect_irq_handler_sw43402(int irq, void *data)
{
	struct dsi_panel *panel = (struct dsi_panel *)data;

	pr_info("\n");
	queue_delayed_work(panel->lge.err_detect_int_workq, &panel->lge.err_detect_int_work,
	msecs_to_jiffies(50));
	return IRQ_HANDLED;
}

static int set_err_detect_mask_sw43402(struct dsi_panel *panel)
{
	int rc = 0;
	enum lge_ddic_dsi_cmd_set_type cmd_type = LGE_DDIC_DSI_ESD_DETECT;

	if (!panel) {
		pr_err("invalid ctrl data\n");
		return -EINVAL;
	}

	if (!dsi_panel_initialized(panel)) {
		pr_err("Panel off state. Ignore screen_mode set cmd\n");
		return -EINVAL;
	}

	switch (panel->lge.err_detect_mask) {
		case 0:
			cmd_type = LGE_DDIC_DSI_ESD_DETECT;
			break;
		case 1:
			cmd_type = LGE_DDIC_DSI_MEM_ERR_DETECT;
			break;
		case 2:
			cmd_type = LGE_DDIC_DSI_LINE_DEFECT_DETECT;
			break;
		default:
			pr_warn("out of range, set esd detect.\n");
			break;
	}

	/* UNLOCK REGISTER */
	if (panel->lge.use_ddic_reg_lock)
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_REGISTER_UNLOCK);

	rc = lge_ddic_dsi_panel_tx_cmd_set(panel, cmd_type);
	if (rc)
		pr_err("failed to send %d cmd, rc=%d\n", cmd_type, rc);

	/* LOCK REGISTER */
	if (panel->lge.use_ddic_reg_lock)
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_REGISTER_LOCK);

	return rc;
}

struct lge_ddic_ops sw43402_ops = {
	.store_aod_area = store_aod_area,
	.prepare_aod_cmds = prepare_aod_cmds_sw43402,
	.prepare_aod_area = prepare_aod_area_sw43402,
	.lge_bc_dim_set = lge_bc_dim_set_sw43402,
	.lge_set_therm_dim = lge_set_therm_dim_sw43402,
	.lge_get_brightness_dim = lge_get_brightness_dim_sw43402,
	.lge_set_brightness_dim = lge_set_brightness_dim_sw43402,
	.lge_set_custom_rgb = lge_set_custom_rgb_sw43402,
	.lge_display_control_store = lge_display_control_store_sw43402,
	.lge_set_screen_mode = lge_set_screen_mode_sw43402,
	.lge_send_screen_mode_cmd = lge_send_screen_mode_cmd_sw43402,
	.lge_set_hdr_hbm_lut = lge_set_hdr_hbm_lut_sw43402,
	.lge_set_acl_mode = lge_set_acl_mode_sw43402,
	.lge_set_video_enhancement = lge_set_video_enhancement_sw43402,
	.vr_enable = lge_set_low_persist_mode_enable_sw43402,
	.vr_disable = lge_set_low_persist_mode_disable_sw43402,
	.hdr_mode_set = hdr_mode_set_sw43402,
	.bist_ctrl = control_bist_cmds_sw43402,
	.release_bist = release_bist_cmds_sw43402,
	.get_current_res = get_current_resolution_sw43402,
	.get_support_res = get_support_resolution_sw43402,
	.get_reg_backup_list = get_ddic_backup_list_sw43402,
	.set_pps_cmds = set_pps_cmds_sw43402,
	.unset_pps_cmds = unset_pps_cmds_sw43402,
	.lge_check_vert_black_line = lge_check_vert_black_line_sw43402,
	.lge_check_vert_white_line = lge_check_vert_white_line_sw43402,
	.lge_check_vert_line_restore = lge_check_vert_line_restore_sw43402,
	.err_detect_work = err_detect_work_sw43402,
	.err_detect_irq_handler = err_detect_irq_handler_sw43402,
	.set_err_detect_mask = set_err_detect_mask_sw43402,
};

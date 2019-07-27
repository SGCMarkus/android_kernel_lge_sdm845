#define pr_fmt(fmt)	"[Display][sw49409-ops:%s:%d] " fmt, __func__, __LINE__

#include "dsi_panel.h"
#include "lge_ddic_ops_helper.h"
#include "lge_dsi_panel_def.h"
#include "mplus/lge_mplus.h"
#include "brightness/lge_brightness_def.h"

#define ADDR_PTLAR 0x30
#define ADDR_PLTAC 0x31
#define ADDR_RDDISPM 0x3F
#define ADDR_U2CTRL 0xCD

#define WORDS_TO_BYTE_ARRAY(w1, w2, b) do {\
		b[0] = WORD_UPPER_BYTE(w1);\
		b[1] = WORD_LOWER_BYTE(w1);\
		b[2] = WORD_UPPER_BYTE(w2);\
		b[3] = WORD_LOWER_BYTE(w2);\
} while(0)

extern int lge_ddic_dsi_panel_tx_cmd_set(struct dsi_panel *panel,
				enum lge_ddic_dsi_cmd_set_type type);
extern int lge_backlight_device_update_status(struct backlight_device *bd);
extern int lge_mplus_backlight_dimming(struct backlight_device *bd,
		enum lge_mplus_mode old_mp_mode, enum lge_mplus_mode cur_mp_mode);

const struct drs_res_info sw49409_res[3] = {
	{"qhd", 0, 1440, 2880},
	{"fhd", 1, 1080, 2160},
	{"hd", 3, 720, 1440},
};

static void adjust_roi(struct dsi_panel *panel, int *sr, int *er)
{
	u32 cur_res = panel->cur_mode->timing.v_active;
	int type, num = panel->num_timing_nodes;

	for (type = 0; type < num; type++) {
		if (cur_res == sw49409_res[type].height)
			break;
	}
	if (type == num) {
		pr_err("invalid height\n");
		*sr = 0;
		*er = panel->cur_mode->timing.h_active - 1;
		return;
	}

	pr_info("type=%d, v_active=%d, h_active= %d\n", type, panel->cur_mode->timing.v_active, panel->cur_mode->timing.h_active);

	if ((panel->lge.aod_area.w == 0) || (panel->lge.aod_area.h == 0)) {
		pr_err("area (w=%d)(h=%d), please check with application\n",
				panel->lge.aod_area.w, panel->lge.aod_area.h);
		goto full_roi;
	}

	if (!strncmp(sw49409_res[type].resolution, "hd", 2)) {
		goto full_roi;
	}
	else if (!strncmp(sw49409_res[type].resolution, "fhd", 3)) {
		*sr = 0;
		*er = panel->lge.aod_area.h - 1;
		pr_info("FHD start=%d, end=%d\n", *sr, *er);

	} else {
		*sr = 0;
		*er = panel->lge.aod_area.h - 1;
		pr_info("QHD start=%d, end=%d\n", *sr, *er);
	}

	return;

full_roi:
	*sr = 0;
	*er = *sr + sw49409_res[0].height - 1;
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

	// TODO:remove, Temporary code
	if(true)	return;

	for (type = 0; type < num; type++) {
		if (cur_res == sw49409_res[type].height)
			break;
	}
	if (type == num) {
		pr_err("invalid height\n");
		sr = 0;
		return;
	}

	if (strncmp(sw49409_res[type].resolution, "qhd", 3)) {
		pr_info("Scroll doesn't surpported in HD and FHD.(%d)\n", type);
		return;
	}

	if (sr < 0)	sr = 0;

	cmd = find_cmd(cmds, cmds_count, addr);
	if (cmd) {
		payload = (char *)cmd->msg.tx_buf + 13;
		upper = WORD_UPPER_BYTE(sr);
		lower = WORD_LOWER_BYTE(sr);
		payload[0] = (lower & 0x0f) << 4;
		payload[1] = (((upper & 0x0f) << 4) | ((lower & 0xf0) >> 4));
		pr_info("CDh : 0x%02x 0x%02x\n", payload[0], payload[1]);
	} else {
		pr_warn("cmd for addr 0x%02X not found\n", addr);
	}
}

static int prepare_aod_cmds_sw49409(struct dsi_panel *panel, struct dsi_cmd_desc *cmds, int cmds_count)
{
	int rc = 0, sr = 0, er = 0;

	if (panel == NULL || cmds == NULL || cmds_count == 0)
		return -EINVAL;

	adjust_roi(panel, &sr, &er);
	prepare_cmd(cmds, cmds_count, ADDR_PTLAR, sr, er);
	prepare_scroll_cmd(panel, cmds, cmds_count, ADDR_U2CTRL, panel->lge.aod_area.y-1);

	return rc;
}

/* MPLUS MODE set to 0 : Normal Mode
 * MPLUS MODE set to 1 : PSM Mode
 * MPLUS MODE set to 2 : HBM Mode
 * MPLUS MODE set to 3 : Gallery Mode
*/
enum lge_mplus_mode mplus_mode_get_sw49409(struct dsi_panel *panel)
{
	u8 data;
	u8 *payload;
	enum lge_mplus_mode mp_mode;

	mutex_lock(&panel->panel_lock);
	payload = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_SET_MPLUS].cmds[0].msg.tx_buf;
	data = payload[1];
	data = (data >> 0x4) & 0x03;
	mutex_unlock(&panel->panel_lock);

	switch (data) {
	case LGE_DIC_MP_NOR:
		mp_mode = LGE_MP_NOR;
		break;
	case LGE_DIC_MP_PSM:
		mp_mode = LGE_MP_PSM;
		break;
	case LGE_DIC_MP_GAL:
		mp_mode = LGE_MP_GAL;
		break;
	case LGE_DIC_MP_HBM:
		mp_mode = LGE_MP_HBM;
		break;
	default:
		mp_mode = LGE_MP_NOR;
		break;
	}
	pr_info("mp : %d hd_mp : %d hl %d rd_mode : %d\n", panel->lge.mp_mode,
			panel->lge.mplus_hd, panel->lge.hl_mode, mp_mode);

	return mp_mode;
}

void mplus_mode_send_sw49409(struct dsi_panel *panel, enum lge_mplus_mode req_mp_mode)
{
	u8 data;
	u8 *payload;
	int mp_mode;
	enum lge_mplus_mode old_mp_mode;

	if ((panel->lge.lp_state == LGE_PANEL_LP2) || (panel->lge.lp_state == LGE_PANEL_LP1)) {
		pr_err("Panel Status LP Mode\n");
		return;
	}

	old_mp_mode = mplus_mode_get_sw49409(panel);

	switch (req_mp_mode) {
	case LGE_MP_PSM:
		mp_mode = LGE_DIC_MP_PSM;
		break;
	case LGE_MP_GAL:
		mp_mode = LGE_DIC_MP_GAL;
		break;
	case LGE_MP_HBM:
		mp_mode = LGE_DIC_MP_HBM;
		break;
	default: /* LGE_MP_NOR */
		mp_mode = LGE_DIC_MP_NOR;
		break;
	}

	mutex_lock(&panel->panel_lock);
	payload = (char *)panel->lge.lge_cmd_sets[LGE_DDIC_DSI_SET_MPLUS].cmds[0].msg.tx_buf;
	data = (mp_mode & 0x03) << 0x6;
	payload[1] = data;
	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_MPLUS);
	mutex_unlock(&panel->panel_lock);

	pr_info("CMD : %x %x mp mode : %x \n", payload[0], payload[1], mp_mode);

	if((old_mp_mode == LGE_MP_HBM) || (req_mp_mode == LGE_MP_HBM)) {
		pr_info("skip brightness dimming\n");
	} else {
		if(panel->bl_config.bd) {
			mutex_lock(&panel->bl_config.bd->update_lock);
			lge_mplus_backlight_dimming(panel->bl_config.bd, old_mp_mode,
					req_mp_mode);
			mutex_unlock(&panel->bl_config.bd->update_lock);
		} else {
			pr_err("brightness device null!\n");
		}
	}
}

void mplus_mode_set_sub_sw49409(struct dsi_panel *panel)
{
	if(panel->lge.mplus_hd != LGE_MP_OFF) { 		/* Hidden Menu Mode */
		mplus_mode_send_sw49409(panel, panel->lge.mplus_hd);
	} else if(panel->lge.hl_mode) { 				/* HBM Mode */
		mplus_mode_send_sw49409(panel, LGE_MP_HBM);
	} else { 										/* Normal Mode */
		mplus_mode_send_sw49409(panel, panel->lge.mp_mode);
	}
}

void mplus_mode_set_sw49409(struct dsi_panel *panel, int mode)
{
	panel->lge.mplus_hd = mode;
	pr_info("mplus_hd : %d\n", panel->lge.mplus_hd);

	mplus_mode_set_sub_sw49409(panel);
}

enum lge_mplus_mode image_enhance_get_sw49409(struct dsi_panel *panel)
{
	return panel->lge.mp_mode;
}

void image_enhance_set_sw49409(struct dsi_panel *panel, int mode)
{
	panel->lge.mp_mode = mode;
	pr_info("mplus : %d\n", panel->lge.mp_mode);

	mplus_mode_set_sub_sw49409(panel);
}

int hl_mode_get_sw49409(struct dsi_panel *panel)
{
	return panel->lge.hl_mode;
}

void hl_mode_set_sw49409(struct dsi_panel *panel, int mode)
{
	panel->lge.hl_mode = mode;
	pr_info("hl_mode : %d\n", panel->lge.hl_mode);

	mplus_mode_set_sub_sw49409(panel);
}

enum lge_blmap_type mp_blmap_sw49409_sub(struct dsi_panel *panel, enum lge_mplus_mode mp_mode)
{
	enum lge_blmap_type bl_type;

	if(panel->lge.hl_mode) {
		switch (mp_mode) {
		case LGE_MP_PSM:
			bl_type = LGE_BLMAP_PSM_HL;
			break;
		case LGE_MP_GAL:
			bl_type = LGE_BLMAP_GAL_HL;
			break;
		case LGE_MP_HBM:
			bl_type = LGE_BLMAP_HBM_HD;
			break;
		default: /* LGE_MP_NOR_HL */
			bl_type = LGE_BLMAP_DEFAULT_HL;
			break;
		}
	} else {
		switch (mp_mode) {
		case LGE_MP_PSM:
			bl_type = LGE_BLMAP_PSM;
			break;
		case LGE_MP_GAL:
			bl_type = LGE_BLMAP_GAL;
			break;
		case LGE_MP_HBM:
			bl_type = LGE_BLMAP_HBM_HD;
			break;
		default: /* LGE_MP_NOR */
			bl_type = LGE_BLMAP_DEFAULT;
			break;
		}
	}
	return bl_type;
}

enum lge_blmap_type mp_blmap_sw49409(struct dsi_panel *panel)
{
	enum lge_blmap_type bl_type = LGE_BLMAP_DEFAULT;

	if(panel->lge.mplus_hd != LGE_MP_OFF)
		bl_type = mp_blmap_sw49409_sub(panel, panel->lge.mplus_hd);
	else
		bl_type = mp_blmap_sw49409_sub(panel, panel->lge.mp_mode);

	return bl_type;
}

struct lge_ddic_ops sw49409_ops = {
	.store_aod_area = store_aod_area,
	.prepare_aod_cmds = prepare_aod_cmds_sw49409,
	.bist_ctrl = NULL,
	.release_bist = NULL,
	.hdr_mode_set = NULL,
	.image_enhance_set = image_enhance_set_sw49409,
	.image_enhance_get = image_enhance_get_sw49409,
	.mplus_mode_set = mplus_mode_set_sw49409,
	.mplus_mode_get = mplus_mode_get_sw49409,
	.hl_mode_set = hl_mode_set_sw49409,
	.hl_mode_get = hl_mode_get_sw49409,
	.mp_blmap = mp_blmap_sw49409,
	.mp_blmap_sub = mp_blmap_sw49409_sub,
};

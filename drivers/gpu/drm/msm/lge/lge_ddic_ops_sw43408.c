#define pr_fmt(fmt)	"[Display][sw43408-ops:%s:%d] " fmt, __func__, __LINE__

#include "dsi_panel.h"
#include "lge_ddic_ops_helper.h"

#define ADDR_PTLAR 0x30
#define ADDR_PLTAC 0x31

#define WORDS_TO_BYTE_ARRAY(w1, w2, b) do {\
		b[0] = WORD_UPPER_BYTE(w1);\
		b[1] = WORD_LOWER_BYTE(w1);\
		b[2] = WORD_UPPER_BYTE(w2);\
		b[3] = WORD_LOWER_BYTE(w2);\
} while(0)

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

static int prepare_aod_cmds_sw43408(struct dsi_panel *panel, struct dsi_cmd_desc *cmds, int cmds_count)
{
	int rc = 0;

	if (panel == NULL || cmds == NULL || cmds_count == 0)
		return -EINVAL;

	prepare_cmd(cmds, cmds_count, ADDR_PTLAR, panel->lge.aod_area.y, panel->lge.aod_area.y + panel->lge.aod_area.h);

	return rc;
}

struct lge_ddic_ops sw43408_ops = {
	.store_aod_area = store_aod_area,
	.prepare_aod_cmds = prepare_aod_cmds_sw43408,
	.hdr_mode_set = NULL,
	.bist_ctrl = NULL,
	.release_bist = NULL,
};

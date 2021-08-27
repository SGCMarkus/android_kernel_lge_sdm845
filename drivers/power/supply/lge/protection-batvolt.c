#define pr_fmt(fmt) "BVP: %s: " fmt, __func__
#define pr_batvolt(reason, fmt, ...)			\
do {							\
	if (pr_debugmask & (reason))			\
		pr_err(fmt, ##__VA_ARGS__);		\
	else						\
		pr_debug(fmt, ##__VA_ARGS__);		\
} while (0)

static int pr_debugmask;

#include <linux/of.h>
#include <linux/workqueue.h>

#include "veneer-primitives.h"

#define BVP_NOTREADY	INT_MAX
#define BVP_VOTER	"BVP"

enum multi_fv_type {
	MFV_CC = 0,
	MFV_CV,
	MFV_MAX,
};

static void protection_batvolt_work(struct work_struct *work);

static struct protection_batvolt_struct {
	/* for protection behavoirs */
	bool		      (*bvp_get)(int* vnow_mv, int* icap_ma);
	struct voter_entry	bvp_voter;
	struct delayed_work	bvp_dwork;
	/* thresholds */
	int			threshold_vbat_limit;
	int			threshold_vbat_clear;
	int			threshold_ibat_rated;
	int			cv_max_ibat_rawsoc;
	int			cv_max_ibat;
	int			threshold_cv_ibat_rated;
	/* dt contents */
	int			step_ibat_ma;
	int			step_poll_ms;
	/* IR compensation Vbatt */
	bool irc_enabled;
	bool is_irc_full;  /* whether irc is supported with full range */
	int irc_resistance;
	/* multi - fv(float voltage) */
	bool multi_fv_enabled;
	int multi_fv_mvolt[MFV_MAX];
} bvp_me = {
	.bvp_get	= NULL,
	.bvp_voter	= { .type = VOTER_TYPE_INVALID },
	.bvp_dwork	= __DELAYED_WORK_INITIALIZER(bvp_me.bvp_dwork,
		protection_batvolt_work, 0),

	.threshold_vbat_limit    = BVP_NOTREADY,
	.threshold_vbat_clear    = BVP_NOTREADY,
	.threshold_ibat_rated    = BVP_NOTREADY,
	.cv_max_ibat_rawsoc      = BVP_NOTREADY,
	.cv_max_ibat             = BVP_NOTREADY,
	.threshold_cv_ibat_rated = BVP_NOTREADY,

	.step_ibat_ma		= BVP_NOTREADY,
	.step_poll_ms		= BVP_NOTREADY,

	.irc_enabled = false,
	.is_irc_full = false,
	.irc_resistance = 0,

	.multi_fv_enabled = false,
	.multi_fv_mvolt = {4450, 4430},
};

static bool set_multi_fv(int mode)
{
	int batt_profile_fv = 0;
	int charger = 0, mode_local = mode;
	int rc = 0;

	if (!bvp_me.multi_fv_enabled)
		return false;

	rc = get_veneer_param(
			VENEER_FEED_CHARGER_TYPE, &charger);
	if ((charger == CHARGING_SUPPLY_TYPE_UNKNOWN) ||
		(charger == CHARGING_SUPPLY_TYPE_FLOAT)   ||
		(charger == CHARGING_SUPPLY_TYPE_NONE)    ||
		(charger == CHARGING_SUPPLY_USB_2P0)      ||
		(charger == CHARGING_SUPPLY_USB_3PX)      ||
		(charger == CHARGING_SUPPLY_FACTORY_56K)  ||
		(charger == CHARGING_SUPPLY_FACTORY_130K) ||
		(charger == CHARGING_SUPPLY_FACTORY_910K) ||
		(charger == CHARGING_SUPPLY_WIRELESS_5W)  ||
		(charger == CHARGING_SUPPLY_WIRELESS_9W)  ){
        mode_local = MFV_CV;
    }

	rc = get_veneer_param(
			VENEER_FEED_BATT_PROFILE_FV_VOTER, &batt_profile_fv);
	if (!rc && batt_profile_fv > 0) {
		if (batt_profile_fv != bvp_me.multi_fv_mvolt[mode_local]) {
			rc = set_veneer_param(
				VENEER_FEED_BATT_PROFILE_FV_VOTER,
				bvp_me.multi_fv_mvolt[mode_local]);
			pr_batvolt(UPDATE, "%s multi-fv=%dmV...\n",
				(mode_local == MFV_CC) ? "initialize," : "enter CV mode,",
				bvp_me.multi_fv_mvolt[mode_local]);
		}
	}

	return !rc;
}

static void protection_batvolt_work(struct work_struct *work) {
	// Filled from client
	int vbat_now = 0, icap_now = 0, ui_soc = 0, raw_soc = 0;
	// Set in this work
	int icap_new = 0, chg_now = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	int virc_new = 0, ibat_now = 0;
	int irc_enabled = bvp_me.irc_enabled, irc_resistance = bvp_me.irc_resistance;

	if (!get_veneer_param(VENEER_FEED_IRC_ENABLED, &irc_enabled))
		bvp_me.irc_enabled = !!irc_enabled;
	if (!get_veneer_param(VENEER_FEED_IRC_RESISTANCE, &irc_resistance)) {
		bvp_me.is_irc_full = !!(irc_resistance / 1000);
		bvp_me.irc_resistance = irc_resistance % 1000;
	}
	if (get_veneer_param(VENEER_FEED_BATT_PSY_IBAT_NOW, &ibat_now))
		ibat_now = 0;
	if (get_veneer_param(VENEER_FEED_CAPACITY, &ui_soc))
		ui_soc = 0;
	if (get_veneer_param(VENEER_FEED_CAPACITY_RAW, &raw_soc))
		raw_soc = 0;
	if (get_veneer_param(VENEER_FEED_CHARGE_TYPE, &chg_now))
		chg_now = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	if (bvp_me.bvp_get(&vbat_now, &icap_now)) {
		if (irc_enabled
			&& chg_now == POWER_SUPPLY_CHARGE_TYPE_FAST
			&& bvp_me.irc_resistance > 0
			&& vbat_now <= MAX_IRC_VOLTAGE ) {
			virc_new = vbat_now - ((ibat_now * bvp_me.irc_resistance) / 1000);
			if (!bvp_me.is_irc_full && (ibat_now < bvp_me.threshold_ibat_rated))
				virc_new = vbat_now;

			pr_batvolt(UPDATE,
				"IR Compensated(res=%d, %d): "
				"virc=%d(-%d), vbatt=%d, ibat=%d, icap=%d, isix=%d\n",
				bvp_me.irc_resistance, irc_resistance, virc_new, vbat_now - virc_new, vbat_now,
				ibat_now, icap_now, bvp_me.threshold_ibat_rated);

		} else
			virc_new = vbat_now;

		if (virc_new <= bvp_me.threshold_vbat_limit) {
			pr_batvolt(VERBOSE, "Under voltage (%d)\n", virc_new);

			if (veneer_voter_enabled(&bvp_me.bvp_voter) && virc_new <= bvp_me.threshold_vbat_clear) {
				pr_batvolt(UPDATE, "Clear batvolt protection\n");
				veneer_voter_release(&bvp_me.bvp_voter);
			}

			goto done;
		}

		if (bvp_me.threshold_cv_ibat_rated > 0) {
			/* for NEW SDM845 models of Style3, Cayman-SDM845 */
			if (chg_now != POWER_SUPPLY_CHARGE_TYPE_TAPER
				&& icap_now <= bvp_me.threshold_ibat_rated) {
				pr_batvolt(VERBOSE, "Under C-rate (%d)\n", icap_now);
				goto done;
			}

			if (chg_now == POWER_SUPPLY_CHARGE_TYPE_TAPER
				&& icap_now <= bvp_me.threshold_cv_ibat_rated) {
				if (bvp_me.multi_fv_enabled) {
					set_multi_fv(MFV_CV);
				}
				else {
					pr_batvolt(VERBOSE, "Under C-rate (%d) on CV\n", icap_now);
				}
				goto done;
			}
		}
		else {
			/* for OLD SDM845 models of G7, V35, V40 */
			if (icap_now <= bvp_me.threshold_ibat_rated) {
				pr_batvolt(VERBOSE, "Under C-rate (%d)\n", icap_now);
				goto done;
			}
		}
	}
	else {
		pr_batvolt(UPDATE, "Host is not ready\n");
		goto done;
	}

	icap_new = (icap_now - bvp_me.step_ibat_ma) / bvp_me.step_ibat_ma * bvp_me.step_ibat_ma;
	if (chg_now == POWER_SUPPLY_CHARGE_TYPE_TAPER &&
		raw_soc >= bvp_me.cv_max_ibat_rawsoc)
		icap_new = min(icap_new, bvp_me.cv_max_ibat);

	veneer_voter_set(&bvp_me.bvp_voter, icap_new);

	if (ui_soc >= 100)
		set_multi_fv(MFV_CV);
	else
		set_multi_fv(MFV_CC);

	pr_batvolt(UPDATE, "Condition : %dmv, %dma => Reduce IBAT to %d\n",
		virc_new, icap_now, icap_new);
done:
	if (ui_soc >= 100)
		set_multi_fv(MFV_CV);
	schedule_delayed_work(to_delayed_work(work), msecs_to_jiffies(bvp_me.step_poll_ms));
	return;
}

void protection_batvolt_refresh(bool is_charging) {
	static bool is_started = false;

	bool is_ready = bvp_me.threshold_vbat_limit != BVP_NOTREADY
		&& bvp_me.threshold_ibat_rated != BVP_NOTREADY
		&& is_charging;

	if (is_ready) {
		if (!is_started) {
			schedule_delayed_work(&bvp_me.bvp_dwork, 0);
			is_started = true;
		}
		else
			; // Skip to handle BVP
	}
	else {
		veneer_voter_release(&bvp_me.bvp_voter);
		cancel_delayed_work_sync(&bvp_me.bvp_dwork);
		is_started = false;
	}
}

bool protection_batvolt_create(struct device_node* dnode, int mincap,
	bool (*feed_protection_batvolt)(int* vnow_mv, int* icap_ma)) {
	int ret = 0, threshold_ibat_pct = 0, threshold_cv_ibat_pct = 0;
	pr_debugmask = ERROR | UPDATE;

	/* Parse device tree */
	ret = of_property_read_s32(dnode, "lge,threshold-ibat-pct", &threshold_ibat_pct);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,threshold-ibat-pct' ret=%d\n", ret);
		goto destroy;
	}
	else
		bvp_me.threshold_ibat_rated = mincap * threshold_ibat_pct / 100;

	ret = of_property_read_s32(dnode, "lge,threshold-cv-ibat-pct", &threshold_cv_ibat_pct);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,threshold-cv-ibat-pct' ret=%d\n", ret);
		bvp_me.threshold_cv_ibat_rated = -9999;
	}
	else
		bvp_me.threshold_cv_ibat_rated = mincap * threshold_cv_ibat_pct / 100;

	ret = of_property_read_s32(dnode, "lge,threshold-vbat-limit", &bvp_me.threshold_vbat_limit);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,threshold-vbat-limit' ret=%d\n", ret);
		goto destroy;
	}
	ret = of_property_read_s32(dnode, "lge,threshold-vbat-clear", &bvp_me.threshold_vbat_clear);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,threshold-vbat-clear' ret=%d\n", ret);
		goto destroy;
	}
	ret = of_property_read_s32(dnode, "lge,step-ibat-ma", &bvp_me.step_ibat_ma);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,step-ibat-ma' ret=%d\n", ret);
		goto destroy;
	}
	ret = of_property_read_s32(dnode, "lge,step-poll-ms", &bvp_me.step_poll_ms);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,step-poll-ms' ret=%d\n", ret);
		goto destroy;
	}

	bvp_me.multi_fv_enabled = of_property_read_bool(dnode, "lge,mulit-fv-enable");
	ret = of_property_read_u32_array(dnode,
		"lge,multi-fv-mvolt", bvp_me.multi_fv_mvolt, MFV_MAX);
	if (ret < 0) {
		bvp_me.multi_fv_mvolt[0] = 4450;
		bvp_me.multi_fv_mvolt[1] = 4430;
	}

	/* Fill callback */
	if (!feed_protection_batvolt) {
		pr_batvolt(ERROR, "feed func should not be null\n");
		goto destroy;
	}
	else
		bvp_me.bvp_get = feed_protection_batvolt;

	/* Register voter */
	if (!veneer_voter_register(&bvp_me.bvp_voter, BVP_VOTER, VOTER_TYPE_IBAT, false)) {
		pr_batvolt(ERROR, "Failed to register the BVP voter\n");
		goto destroy;
	}

	pr_batvolt(UPDATE, "Complete to create, "
		"threshold_vbat_limit(%d), threshold_vbat_clear(%d), threshold_ibat_rated(%d), "
		"step_ibat_ma(%d), step_poll_ms(%d)\n",
		bvp_me.threshold_vbat_limit, bvp_me.threshold_vbat_clear, bvp_me.threshold_ibat_rated,
		bvp_me.step_ibat_ma, bvp_me.step_poll_ms);

	return true;

destroy:
	protection_batvolt_destroy();
	return false;
}

void protection_batvolt_destroy(void) {
	cancel_delayed_work_sync(&bvp_me.bvp_dwork);
	veneer_voter_unregister(&bvp_me.bvp_voter);
	bvp_me.bvp_get = NULL;

	bvp_me.threshold_vbat_limit	= BVP_NOTREADY;
	bvp_me.threshold_vbat_clear	= BVP_NOTREADY;
	bvp_me.threshold_ibat_rated	= BVP_NOTREADY;
	bvp_me.threshold_cv_ibat_rated	= BVP_NOTREADY;

	bvp_me.step_ibat_ma		= BVP_NOTREADY;
	bvp_me.step_poll_ms		= BVP_NOTREADY;
}

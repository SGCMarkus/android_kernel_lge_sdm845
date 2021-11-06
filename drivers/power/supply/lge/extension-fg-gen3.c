/*
 * CAUTION! :
 * 	This file will be included at the end of "qpnp-fg-gen3.c".
 * 	So "qpnp-fg-gen3.c" should be touched before you start to build.
 * 	If not, your work will not be applied to the built image
 * 	because the build system doesn't care the update time of this file.
 */

#include <linux/thermal.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include "veneer-primitives.h"

#define LGE_FG_INITVAL -1

struct _fake {
	int temperature;
	int capacity;
	int uvoltage;
};

struct _fginfo {
/* Capacity */
	int capacity_rescaled;
	int capacity_monotonic;
	int capacity_chargecnt;
	int capacity_learned;
/* v/i ADCs */
	int battery_inow;
	int battery_vnow;
	int battery_vpredict;
	int battery_ocv;
	int input_vusb;
	int input_iusb;
	int input_vwlc;
	int input_iwlc;
	int input_aicl;
/* Temperature */
	int temp_compensated;
	int temp_thermister;
	int temp_vts;
/* Impedance */
	int impedance_esr;
	int impedance_rslow;
/* Misc */
	int misc_cycle;
/* SoCs */
	int misc_battsoc;
	int misc_ccsocsw;
	int misc_ccsoc;
};

#define TCOMP_TABLE_MAX 3
#define TCOMP_COUNT 25
struct tcomp_param {
	bool load_done;
	int load_max;
	bool icoeff_load_done;
	struct tcomp_entry {
		int temp_cell;
		int temp_bias;
	} table[TCOMP_TABLE_MAX][TCOMP_COUNT];
	int icoeff;

	bool rise_cut;
	int rise_cut_trig;
	bool fall_cut;
	int fall_cut_trig;

	bool qnovo_charging;
	bool logging;
};

struct _rescale {
	bool lge_monotonic;
	int	criteria;	// 0 ~ 255
	int	rawsoc;		// 0 ~ 255
	int	result;		// 0 ~ 100
};

struct smooth_param {
/*
 * To update batt_therm immediately when wakeup from suspend,
 * using below flag in suspend/resume callback
 */
	bool		smooth_filter_enable;
	bool		batt_therm_ready;
	bool		enable_flag;
	int		therm_backup;
	ktime_t		last_time;
};

enum tcomp_chg_type {
	TCOMP_CHG_NONE = 0,
	TCOMP_CHG_USB,
	TCOMP_CHG_WLC_LCDOFF,
	TCOMP_CHG_WLC_LCDON
};

enum vote_direction {
	VOTE_INIT = -4,
	VOTE_NONE,
	VOTE_SUSPEND,
	VOTE_DECREASE,
	VOTE_STAY,
	VOTE_INCREASE,
	VOTE_ELECTION,
};

enum current_thres {
	HIGH_CURR = 0,
	MED_CURR,
	LOW_CURR,
	UNIT_KI,
};

enum limit_ki {
	MIN_KI = 0,
	MAX_KI,
	KI_COEFF_LIMIT,
};

#define BACKUP_MAX_CNT		6
#define LOOP_TIME_NORMAL	10
#define LOOP_TIME_ALERT		5

#define CHANGE_POINT		50
#define CHANGE_POINT_INC	100
#define SCALING_FACTOR_INC	2

struct ki_coeff_param {
	enum vote_direction	backup_vote;
	bool reach_vfloat;
	bool chg_enable;
	bool dischg_enable;
	int ki_dischg[UNIT_KI][UNIT_KI];
	int ki_chg[UNIT_KI];
	int loop_cnt;
	int stay_cnt;
	int soc_alert;
	int ki_limit[KI_COEFF_LIMIT];
	int ki_comp;
	int ki_comp_bck;
	int margin_vfloat_mv;
	struct mutex direction_lock;
};

/* Gloval variable for extension-fg-gen4 */
static struct _fake fake = {
	.temperature = LGE_FG_INITVAL,
	.capacity = LGE_FG_INITVAL,
	.uvoltage = LGE_FG_INITVAL,
};

static struct _fginfo fggen3 = {
/* Capacity  */ LGE_FG_INITVAL, LGE_FG_INITVAL, LGE_FG_INITVAL, LGE_FG_INITVAL,
/* v/i ADCs  */ LGE_FG_INITVAL, LGE_FG_INITVAL, LGE_FG_INITVAL, LGE_FG_INITVAL,
                LGE_FG_INITVAL, LGE_FG_INITVAL, LGE_FG_INITVAL, LGE_FG_INITVAL,
                LGE_FG_INITVAL,
/* Temp      */ LGE_FG_INITVAL, LGE_FG_INITVAL, -1000,
/* impedance */ LGE_FG_INITVAL, LGE_FG_INITVAL,
/* Misc      */ LGE_FG_INITVAL,
/* SoCs      */ LGE_FG_INITVAL, LGE_FG_INITVAL, LGE_FG_INITVAL,
};

static struct tcomp_param tcomp = {
	.load_done = false,
	.icoeff_load_done = false,
	.icoeff = 0,

	.rise_cut = false,
	.rise_cut_trig = -999,
	.fall_cut = false,
	.fall_cut_trig = -999,

	.qnovo_charging = false,
	.logging = false,
};

/* For SoC rescaling, .rawsoc(0~255) is updated ONLY ON
 * 'fg_delta_msoc_irq_handler' and it is rescaled to 0~100
 */
static struct _rescale rescale = {
	.lge_monotonic = false,
	.criteria = 247,
	.rawsoc = LGE_FG_INITVAL,
	.result = LGE_FG_INITVAL,
};

static struct smooth_param smooth_therm = {
	.smooth_filter_enable = false,
	.batt_therm_ready = false,
	.enable_flag = false,
	.therm_backup = 0,
	.last_time.tv64 = 0,
};

static struct ki_coeff_param ki_coeff = {
	.backup_vote = VOTE_INIT,
	.reach_vfloat = false,
	.chg_enable = false,
	.dischg_enable = false,
	.loop_cnt = 0,
	.stay_cnt = 0,
	.soc_alert = 0,
	.ki_comp = 0,
	.ki_comp_bck = 0,
	.margin_vfloat_mv = 0,
};

static int get_charging_type(struct fg_chip *chip)
{
	union power_supply_propval val = { 0, };
	char slimit[20] = "";

	if (chip->batt_psy) {
		if (!power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_STATUS_RAW, &val))
			if (val.intval != POWER_SUPPLY_STATUS_CHARGING)
				return TCOMP_CHG_NONE;
	}

	if (chip->usb_psy) {
		if (!power_supply_get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_PRESENT, &val))
			if (val.intval)
				return TCOMP_CHG_USB;
	}

	if (chip->dc_psy) {
		if (!power_supply_get_property(chip->dc_psy,
			POWER_SUPPLY_PROP_PRESENT, &val)) {
			if (val.intval) {
				if (unified_nodes_show("charging_restriction", slimit)) {
					if (slimit[0] == '-')
						return TCOMP_CHG_WLC_LCDOFF;
					return TCOMP_CHG_WLC_LCDON;
				} else
					return TCOMP_CHG_WLC_LCDOFF;
			}
		}
	}

	return TCOMP_CHG_NONE;
}

static int get_batt_temp_current(struct fg_chip *chip)
{
	static int ichg = 0;
	bool is_cc = false;
	bool is_fast_charging = false;
	union power_supply_propval val = { 0, };

	if (!chip || !chip->batt_psy || !chip->usb_psy)
		return 0;

	if (!power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_STATUS_RAW, &val)
				&& val.intval == POWER_SUPPLY_STATUS_CHARGING) {
		if (!power_supply_get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_REAL_TYPE, &val)) {

			if (val.intval == POWER_SUPPLY_TYPE_USB_HVDCP ||
				val.intval == POWER_SUPPLY_TYPE_USB_HVDCP_3 ||
				val.intval == POWER_SUPPLY_TYPE_USB_PD)
				is_fast_charging = true;

			if (!power_supply_get_property(chip->batt_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &val) &&
				(val.intval == POWER_SUPPLY_CHARGE_TYPE_TRICKLE ||
				 val.intval == POWER_SUPPLY_CHARGE_TYPE_FAST))
				is_cc = true;

			/*  in case of fast charging, fcc is refered instead of
				real current for avoiding qni negative pulse effect */
			if (is_fast_charging && is_cc ) {
				ichg = !power_supply_get_property(chip->batt_psy,
							POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val) ?
								val.intval / -1000 : ichg;
				goto out;
			} else {
				/* if charging current is over 25mA,
					batt_therm compensation current keeps the previous value */
				if (!power_supply_get_property(chip->batt_psy,
						POWER_SUPPLY_PROP_CURRENT_NOW, &val) &&
							val.intval > 25000)
					ichg = val.intval / 1000;

				goto out;
			}
		}
	}

	ichg = 0;

out:
	return ichg;
}

static int filtered_batt_therm(bool changed, int comp, int batt)
{
	bool tbl_changed = changed;
	int battemp_cell = batt;
	int battemp_comp = comp;
	static int pre_battemp_cell = -9999;
	static int pre_battemp_comp = -9999;
	static bool is_filtering_rise = false;
	static bool is_filtering_fall = false;
	int battemp_cell_diff = 0;
	int battemp_comp_diff = 0;

	/* if it is not in the trigger condition, No apply */
	if (!((tbl_changed && tcomp.rise_cut
			&& (battemp_comp > tcomp.rise_cut_trig)) ||
		(tbl_changed && tcomp.fall_cut
			&& (battemp_comp < tcomp.fall_cut_trig))))
		tbl_changed = false;

	if ((tbl_changed || is_filtering_rise || is_filtering_fall)
		&& (pre_battemp_cell > -9999 && pre_battemp_comp > -9999)) {
		battemp_cell_diff = battemp_cell - pre_battemp_cell;
		battemp_comp_diff = battemp_comp - pre_battemp_comp;
		// rise
		if (tcomp.rise_cut && (battemp_comp >= pre_battemp_comp)) {
			if (is_filtering_fall)
				is_filtering_fall = false;

			if (battemp_comp_diff > battemp_cell_diff) {
				is_filtering_rise = true;
				if ( battemp_cell_diff > 0 )
					battemp_comp = pre_battemp_comp + battemp_cell_diff;
				else
					battemp_comp = pre_battemp_comp;
			}
			else {
				is_filtering_rise = false;
			}
		}
		// fall
		else if (tcomp.fall_cut) {
			if (is_filtering_rise)
				is_filtering_rise = false;

			if (battemp_cell_diff > battemp_comp_diff ) {
				is_filtering_fall = true;
				if (battemp_cell_diff < 0)
					battemp_comp = pre_battemp_comp + battemp_cell_diff;
				else
					battemp_comp = pre_battemp_comp;
			}
			else {
				is_filtering_fall = false;
			}
		}
		else if (tcomp.rise_cut) {
			if (is_filtering_rise)
				is_filtering_rise = false;
		}
	}

	pre_battemp_cell = battemp_cell;
	pre_battemp_comp = battemp_comp;
	return battemp_comp;
}

#define TEMP_CATCHUP_SEC_MAX		150
#define TEMP_CATCHUP_SEC_PER_DEGREE	30
#define MAX_CATCHUP_TEMP (TEMP_CATCHUP_SEC_MAX/TEMP_CATCHUP_SEC_PER_DEGREE)
#define TEMP_UPDATE_TIME_SEC		5
#define DIVIDE_TEMP			10
#define DIVIDE_MS_TO_S			1000

static void catchup_batt_therm(int temp, int temp_backup, int ts_diff)
{
	int catchup_time_s = 0, catchup_temp = 0, change_therm = 0;

	change_therm = abs(temp - temp_backup);
	if ((change_therm / DIVIDE_TEMP) < MAX_CATCHUP_TEMP)
		catchup_time_s = change_therm * TEMP_CATCHUP_SEC_PER_DEGREE / DIVIDE_TEMP;
	else
		catchup_time_s = TEMP_CATCHUP_SEC_MAX;

	if (catchup_time_s <= 0)
		catchup_time_s = 1;

	ts_diff++;
	/* Scaling temp in case of time_diff < catchup_time */
	if (ts_diff <= catchup_time_s) {
		catchup_temp = ((catchup_time_s - ts_diff)*temp_backup + (ts_diff * temp))/catchup_time_s;

		pr_info("catchup_temp ts_diff:%d, catchup_time_s:%d, "
			"temp:%d, therm_backup:%d, result_temp:%d\n",
			ts_diff, catchup_time_s,
			temp, temp_backup, catchup_temp);
		smooth_therm.therm_backup = catchup_temp;
	}
	/* Out of target time, doesn't scaling */
	else {
		pr_debug("catchup_skip ts_diff:%d, catchup_time_s:%d, "
			"temp:%d, therm_backup:%d, result_temp:%d\n",
			ts_diff, catchup_time_s,
			temp, temp_backup, temp);
		smooth_therm.therm_backup = temp;
	}
}

/* calculate_battery_temperature
 *     bias  : 1st compensation by predefined diffs
 *     icomp : 2nd compensation by (i^2 * k)
 */
static int calculate_battery_temperature(/* @Nonnull */ struct fg_chip *chip)
{
	int battemp_bias, battemp_icomp, battemp_cell = 0;
	int i, temp, ichg = 0, tbl_pt = 0;
	union power_supply_propval val = { 0, };
	bool tbl_changed = false;
	static int pre_tbl_pt = -1;
	int ts_diff = 0;
	ktime_t now;

	if (smooth_therm.smooth_filter_enable) {
		/* Check now ktime */
		now = ktime_get();

		if ((smooth_therm.last_time.tv64 == 0 && smooth_therm.therm_backup == 0)
					|| !smooth_therm.batt_therm_ready) {
			smooth_therm.last_time = now;
			pr_debug("Initialize time and temp, ts=%lld, backup=%d, ready=%d\n",
					smooth_therm.last_time.tv64, smooth_therm.therm_backup,
					smooth_therm.batt_therm_ready);
			smooth_therm.enable_flag = false;
			goto skip_time;
		}

		ts_diff = ((unsigned long)ktime_ms_delta(now, smooth_therm.last_time)/DIVIDE_MS_TO_S);

		/* Update batt_therm every 5sec */
		if (ts_diff < TEMP_UPDATE_TIME_SEC) {
			pr_debug("ts_diff(%d) is lower than UPDATE_SEC. now(%lld), ts_bef(%lld)\n",
					ts_diff, now.tv64, smooth_therm.last_time.tv64);
			goto out_temp;
		}
		if (!smooth_therm.enable_flag)
			smooth_therm.enable_flag = true;
	}
skip_time:
	if (fg_get_battery_temp(chip, &battemp_cell)){
		pr_info("get real batt therm error\n");
		return LGE_FG_INITVAL;
	}

	if (!tcomp.load_done) {
		pr_info("not ready tcomp table. raw temp=%d\n", battemp_cell);
		return battemp_cell;
	}

	if (!tcomp.icoeff_load_done) {
		pr_info("not ready icoeff. raw temp=%d\n", battemp_cell);
		return battemp_cell;
	}

	if (!smooth_therm.batt_therm_ready)
		smooth_therm.batt_therm_ready = true;

	if (tcomp.load_max > 1) {
		switch (get_charging_type(chip)) {
			case TCOMP_CHG_WLC_LCDOFF: tbl_pt = 1; break;
			case TCOMP_CHG_WLC_LCDON:  tbl_pt = 2; break;
			default: tbl_pt = 0; break;
		}
	}
	else
		tbl_pt = 0;

	if (pre_tbl_pt >= 0 )
		if (pre_tbl_pt != tbl_pt)
			tbl_changed = true;
	pre_tbl_pt = tbl_pt;

	/* Compensating battemp_bias */
	for (i = 0; i < TCOMP_COUNT; i++) {
		if (battemp_cell < tcomp.table[tbl_pt][i].temp_cell)
			break;
	}

	if (i == 0)
		battemp_bias = tcomp.table[tbl_pt][0].temp_bias;
	else if (i == TCOMP_COUNT)
		battemp_bias = tcomp.table[tbl_pt][TCOMP_COUNT-1].temp_bias;
	else
		battemp_bias =
		(	(tcomp.table[tbl_pt][i].temp_bias -
				tcomp.table[tbl_pt][i-1].temp_bias)
			* (battemp_cell - tcomp.table[tbl_pt][i-1].temp_cell)
			/ (tcomp.table[tbl_pt][i].temp_cell -
				tcomp.table[tbl_pt][i-1].temp_cell)
		) + tcomp.table[tbl_pt][i-1].temp_bias;

	/* Compensating battemp_icomp */
	if (chip->batt_psy) {
		if (tcomp.qnovo_charging) {
			ichg = get_batt_temp_current(chip);
		}
		else {
			if (!power_supply_get_property(
				chip->batt_psy, POWER_SUPPLY_PROP_STATUS_RAW, &val)
				&& val.intval == POWER_SUPPLY_STATUS_CHARGING
				&& !power_supply_get_property(
					chip->batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val)
				&& val.intval > 0)
				ichg = ((val.intval) / 1000);
		}
	} else {
		pr_info("Battery is not available, %d(=%d+%d) as batt temp\n",
			battemp_cell + battemp_bias, battemp_cell, battemp_bias);
	}

	battemp_icomp = ichg * ichg * tcomp.icoeff / 10000000;
	temp = battemp_cell + battemp_bias - battemp_icomp;

	if (tcomp.logging)
		pr_info("Battery temperature : "
				"%d = %d (cell) + %d (bias) - %d (icomp), "
				"icoeff = %d, ichg = %d\n",
			temp, battemp_cell, battemp_bias, battemp_icomp,
			tcomp.icoeff, ichg);

	if (tcomp.rise_cut || tcomp.fall_cut)
		return filtered_batt_therm(tbl_changed, temp, battemp_cell);

	if (smooth_therm.smooth_filter_enable) {
		if (smooth_therm.enable_flag) {
			catchup_batt_therm(temp, smooth_therm.therm_backup, ts_diff);
			smooth_therm.last_time = now;
		}
		/* In some cases, Using default batt_therm */
		else {
			smooth_therm.therm_backup = temp;
		}
	}
	else {
		return temp;
	}

out_temp:
	return smooth_therm.therm_backup;
}

#define LGE_FG_CHARGING         1
#define LGE_FG_DISCHARGING      0
static int  lge_is_fg_charging(struct fg_chip *chip)
{
	union power_supply_propval val = { 0, };
	bool power_status = false;

	if (!chip || !chip->batt_psy || !chip->usb_psy)
		return -1;

	if (!power_supply_get_property(chip->batt_psy,
					POWER_SUPPLY_PROP_STATUS_RAW, &val))
		power_status = (val.intval == POWER_SUPPLY_STATUS_CHARGING) ? true : false;
	else
		return -1;

	if (!power_status)
		return LGE_FG_DISCHARGING;

	if (!power_supply_get_property(chip->batt_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW, &val)) {
		if (val.intval > 25000 )
			return LGE_FG_CHARGING;
		else
			return LGE_FG_DISCHARGING;
	}

	return -1;
}

static int calculate_rescaled_soc(struct fg_chip *chip)
{
	int msoc_raw = 0, rc = 0;
	int new_result = 0;

	rc = fg_get_msoc_raw(chip, &msoc_raw);
	if (rc < 0)
		pr_info("Error in getting msoc_raw, rc=%d\n", rc);

	new_result = min(FULL_CAPACITY,
		DIV_ROUND_CLOSEST(msoc_raw * FULL_CAPACITY, rescale.criteria));

	rescale.rawsoc = msoc_raw;

	if (!rescale.lge_monotonic) {
		rescale.result = new_result;
		return 0;
	}

	if (rescale.result <= 0 ||
		max(rescale.result, new_result) - min(rescale.result, new_result) > 5 )
		rescale.result = new_result;

	switch (lge_is_fg_charging(chip)) {
		case LGE_FG_CHARGING:
			pr_info("fg_rescale: charging: %d = max(old=%d, new=%d)\n",
				max(rescale.result, new_result), rescale.result, new_result);
			rescale.result = max(rescale.result, new_result);
			break;
		case LGE_FG_DISCHARGING:
			pr_info("fg_rescale: discharging: %d = min(old=%d, new=%d)\n",
				min(rescale.result, new_result), rescale.result, new_result);
			rescale.result = min(rescale.result, new_result);
			break;
		default:
			pr_info("fg_rescale: error: old=%d, new=%d\n", rescale.result, new_result);
			rescale.result = new_result;
			break;
	}

	return 0;
}

static int extension_battery_cycle(/* @Nonnull */ struct fg_chip *chip) {
	static int bucket_sum_backup = 0;

	int cycle = 0;
	if (chip->cyc_ctr.en) {
		int i, bucket_sum_now = 0;

		mutex_lock(&chip->cyc_ctr.lock);
		for (i=0; i<BUCKET_COUNT; i++)
			bucket_sum_now += chip->cyc_ctr.count[i];
		cycle = bucket_sum_now / BUCKET_COUNT;
		mutex_unlock(&chip->cyc_ctr.lock);

		if (bucket_sum_backup != bucket_sum_now) {
			pr_info("Battery cycle : %d (= %d / %d)\n",
				cycle, bucket_sum_now, BUCKET_COUNT);
			bucket_sum_backup = bucket_sum_now;
		}
	}

	return cycle;
}

static void fggen3_snapshot_print(void) {
	/* To remove TAG in the logs, 'printk' is used */

	printk("PMINFO: [CSV] "
	/* Capacity    */ "cSYS:%d, cMNT:%d, cCHG:%d, cLRN:%d, "
	/* v/i ADCs    */ "iBAT:%d, vBAT:%d, vPRD:%d, vOCV:%d, vUSB:%d, iUSB:%d, vWLC:%d, iWCL:%d, AiCL:%d, "
	/* Temperature */ "tSYS:%d, tORI:%d, tVTS:%d, "
	/* Impedance   */ "rTOT:%d, rESR:%d, rSLW:%d, "
	/* Misc        */ "CYCLE:%d, "
	/* SoCs        */ "sBATT:%d, sCCSW:%d, sCC:%d\n",

	/* Capacity    */ fggen3.capacity_rescaled*10, fggen3.capacity_monotonic, fggen3.capacity_chargecnt, fggen3.capacity_learned,
	/* Battery     */ fggen3.battery_inow, fggen3.battery_vnow, fggen3.battery_vpredict, fggen3.battery_ocv,
	/* Input       */ fggen3.input_vusb, fggen3.input_iusb, fggen3.input_vwlc, fggen3.input_iwlc, fggen3.input_aicl,
	/* Temperature */ fggen3.temp_compensated, fggen3.temp_thermister, fggen3.temp_vts,
	/* Impedance   */ fggen3.impedance_esr + fggen3.impedance_rslow,
			  fggen3.impedance_esr, fggen3.impedance_rslow,
	/* Misc        */ fggen3.misc_cycle,
	/* SoCs        */ fggen3.misc_battsoc, fggen3.misc_ccsocsw, fggen3.misc_ccsoc);
}

static void fggen3_snapshot_inputnow(int* vusb, int* iusb, int* vwlc, int* iwlc, int* aicl) {
	struct power_supply* psy_main  = power_supply_get_by_name("main");
	struct power_supply* psy_usb   = power_supply_get_by_name("usb");
	struct power_supply* psy_dc    = power_supply_get_by_name("dc");
	union power_supply_propval val = { 0, };

	*aicl = (psy_main && !power_supply_get_property(psy_main, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED, &val))
		? val.intval/1000 : LGE_FG_INITVAL;
	*vusb = (psy_usb && !power_supply_get_property(psy_usb, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val))
		? val.intval/1000 : LGE_FG_INITVAL;
	*iusb = (psy_usb && !power_supply_get_property(psy_usb, POWER_SUPPLY_PROP_INPUT_CURRENT_NOW, &val))
		? val.intval/1000 : LGE_FG_INITVAL;
	*vwlc = (psy_dc && !power_supply_get_property(psy_dc, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val))
		? val.intval/1000 : LGE_FG_INITVAL;
	*iwlc = (psy_dc && !power_supply_get_property(psy_dc, POWER_SUPPLY_PROP_INPUT_CURRENT_NOW, &val))
		? val.intval/1000 : LGE_FG_INITVAL;

	if (psy_main)
		power_supply_put(psy_main);
	if (psy_usb)
		power_supply_put(psy_usb);
	if (psy_dc)
		power_supply_put(psy_dc);
}

static void fggen3_snapshot_update(struct power_supply *psy) {
	struct thermal_zone_device*	tzd = thermal_zone_get_zone_by_name("vts-virt-therm");
	struct fg_chip*			fg3 = power_supply_get_drvdata(psy);
	int				buf = 0;
	union power_supply_propval	val = { 0, };

	if (!tzd || !fg3) {
		// Not ready to log
		return;
	}

/* Capacity */
	fggen3.capacity_rescaled = rescale.result < 0
		? LGE_FG_INITVAL : rescale.result;
	fggen3.capacity_monotonic = !fg_get_msoc_raw(fg3, &buf)
		? buf * 1000 / 255 : LGE_FG_INITVAL;
	fggen3.capacity_chargecnt = !fg_get_charge_counter(fg3, &buf)
		? buf / 1000 : LGE_FG_INITVAL;
	fggen3.capacity_learned
		= (fg3->cl.learned_cc_uah) / 1000;
/* Battery */
	fggen3.battery_inow = !fg_get_battery_current(fg3, &buf)
		? buf / 1000 : LGE_FG_INITVAL;
	fggen3.battery_vnow = !fg_get_battery_voltage(fg3, &buf)
		? buf / 1000 : LGE_FG_INITVAL;
	fggen3.battery_vpredict = !fg_get_sram_prop(fg3, FG_SRAM_VOLTAGE_PRED, &buf)
		? buf / 1000 : LGE_FG_INITVAL;
	fggen3.battery_ocv = !fg_get_sram_prop(fg3, FG_SRAM_OCV, &buf)
		? buf / 1000 : LGE_FG_INITVAL;
/* Input */
	fggen3_snapshot_inputnow(&fggen3.input_vusb, &fggen3.input_iusb,
		&fggen3.input_vwlc, &fggen3.input_iwlc,
		&fggen3.input_aicl);
/* Temperature */
	fggen3.temp_compensated
		= calculate_battery_temperature(fg3);
	fggen3.temp_thermister = !fg_get_battery_temp(fg3, &buf)
		? buf : LGE_FG_INITVAL;
	fggen3.temp_vts = !thermal_zone_get_temp(tzd, &buf)
		? buf / 100 : -1000;
/* Impedance */
	fggen3.impedance_esr = !fg_get_sram_prop(fg3, FG_SRAM_ESR, &buf)
		? buf : LGE_FG_INITVAL;
	fggen3.impedance_rslow = !fg_get_sram_prop(fg3, FG_SRAM_RSLOW, &buf)
		? buf : LGE_FG_INITVAL;
/* Misc */
	fggen3.misc_cycle = !power_supply_get_property(fg3->fg_psy,
			POWER_SUPPLY_PROP_CYCLE_COUNT, &val)
		? val.intval : LGE_FG_INITVAL;
/* SoCs */
	fggen3.misc_battsoc = !fg_get_sram_prop(fg3, FG_SRAM_BATT_SOC, &buf)
		? ((u32)buf >> 24)*1000/255 : LGE_FG_INITVAL;
	fggen3.misc_ccsocsw = !fg_get_sram_prop(fg3, FG_SRAM_CC_SOC_SW, &buf)
		? div_s64((s64)buf*1000, CC_SOC_30BIT) : LGE_FG_INITVAL;
	fggen3.misc_ccsoc = !fg_get_sram_prop(fg3, FG_SRAM_CC_SOC, &buf)
		? div_s64((s64)buf*1000, CC_SOC_30BIT) : LGE_FG_INITVAL;

	/* logging finally */
	fggen3_snapshot_print();
}

#define KI_CURR_LOWTH_DISCHG_DEFAULT	500
#define KI_CURR_HIGHTH_DISCHG_DEFAULT	1000
static int old_ki_coeff_low = -1;
static int old_ki_coeff_med = -1;
static int old_ki_coeff_hi = -1;
int override_fg_adjust_ki_coeff_dischg(struct fg_chip *chip, int ki_low, int ki_med, int ki_high)
{
	int rc, i, msoc;
	int ki_curr_lowth = KI_CURR_LOWTH_DISCHG_DEFAULT;
	int ki_curr_highth = KI_CURR_HIGHTH_DISCHG_DEFAULT;
	int ki_coeff_low = -1, ki_coeff_med = -1, ki_coeff_hi = -1;
	int cutoff_volt = chip->dt.cutoff_volt_mv;
	int cutoff_curr = chip->dt.cutoff_curr_ma;
	u8 buf[4], val;

	if (!chip->ki_coeff_dischg_en)
		return 0;

	rc = fg_get_msoc(chip, &msoc);
	if (rc < 0) {
		pr_err("Error in getting capacity, rc=%d\n", rc);
		return rc;
	}

	if (!(chip->dt.dynamic_ki_en) &&
		chip->esr_flt_sts == LOW_TEMP &&
		chip->charge_status == POWER_SUPPLY_STATUS_DISCHARGING) {
		cutoff_volt = chip->dt.cutoff_lt_volt_mv;
		cutoff_curr = chip->dt.cutoff_lt_curr_ma;
		for (i = KI_COEFF_SOC_LEVELS - 1; i >= 0; i--) {
			if (msoc < chip->dt.ki_coeff_soc[i]) {
				ki_curr_lowth = chip->dt.ki_curr_lowth_dischg;
				ki_curr_highth = chip->dt.ki_curr_highth_dischg;
				ki_coeff_low = chip->dt.ki_coeff_low_dischg[i];
				ki_coeff_med = chip->dt.ki_coeff_med_dischg[i];
				ki_coeff_hi = chip->dt.ki_coeff_hi_dischg[i];
				break;
			}
		}
	}
	else {
		ki_curr_lowth = chip->dt.ki_curr_lowth_dischg;
		ki_curr_highth = chip->dt.ki_curr_highth_dischg;
		ki_coeff_low = ki_low;
		ki_coeff_med = ki_med;
		ki_coeff_hi = ki_high;
	}

	if ((old_ki_coeff_low == ki_coeff_low)
		&& (old_ki_coeff_med == ki_coeff_med)
		&& (old_ki_coeff_hi == ki_coeff_hi))
		return 0;

	old_ki_coeff_low = ki_coeff_low;
	old_ki_coeff_med = ki_coeff_med;
	old_ki_coeff_hi = ki_coeff_hi;

	fg_encode(chip->sp, FG_SRAM_KI_CURR_LOWTH_DISCHG, ki_curr_lowth, &val);
	rc = fg_sram_write(chip,
			chip->sp[FG_SRAM_KI_CURR_LOWTH_DISCHG].addr_word,
			chip->sp[FG_SRAM_KI_CURR_LOWTH_DISCHG].addr_byte, &val,
			chip->sp[FG_SRAM_KI_CURR_LOWTH_DISCHG].len,
			FG_IMA_DEFAULT);
	if (rc < 0) {
		pr_err("Error in writing ki_coeff_low_dischg, rc=%d\n", rc);
		return rc;
	}

	fg_encode(chip->sp, FG_SRAM_KI_CURR_HIGHTH_DISCHG, ki_curr_highth, &val);
	rc = fg_sram_write(chip,
			chip->sp[FG_SRAM_KI_CURR_HIGHTH_DISCHG].addr_word,
			chip->sp[FG_SRAM_KI_CURR_HIGHTH_DISCHG].addr_byte, &val,
			chip->sp[FG_SRAM_KI_CURR_HIGHTH_DISCHG].len,
			FG_IMA_DEFAULT);
	if (rc < 0) {
		pr_err("Error in writing ki_coeff_low_dischg, rc=%d\n", rc);
		return rc;
	}

	fg_encode(chip->sp, FG_SRAM_KI_COEFF_LOW_DISCHG, ki_coeff_low, &val);
	rc = fg_sram_write(chip,
			chip->sp[FG_SRAM_KI_COEFF_LOW_DISCHG].addr_word,
			chip->sp[FG_SRAM_KI_COEFF_LOW_DISCHG].addr_byte, &val,
			chip->sp[FG_SRAM_KI_COEFF_LOW_DISCHG].len,
			FG_IMA_DEFAULT);
	if (rc < 0) {
		pr_err("Error in writing ki_coeff_low, rc=%d\n", rc);
		return rc;
	}

	fg_encode(chip->sp, FG_SRAM_KI_COEFF_MED_DISCHG, ki_coeff_med, &val);
	rc = fg_sram_write(chip,
			chip->sp[FG_SRAM_KI_COEFF_MED_DISCHG].addr_word,
			chip->sp[FG_SRAM_KI_COEFF_MED_DISCHG].addr_byte, &val,
			chip->sp[FG_SRAM_KI_COEFF_MED_DISCHG].len,
			FG_IMA_DEFAULT);
	if (rc < 0) {
		pr_err("Error in writing ki_coeff_med, rc=%d\n", rc);
		return rc;
	}

	fg_encode(chip->sp, FG_SRAM_KI_COEFF_HI_DISCHG, ki_coeff_hi, &val);
	rc = fg_sram_write(chip,
			chip->sp[FG_SRAM_KI_COEFF_HI_DISCHG].addr_word,
			chip->sp[FG_SRAM_KI_COEFF_HI_DISCHG].addr_byte, &val,
			chip->sp[FG_SRAM_KI_COEFF_HI_DISCHG].len,
			FG_IMA_DEFAULT);
	if (rc < 0) {
		pr_err("Error in writing ki_coeff_hi, rc=%d\n", rc);
		return rc;
	}

	fg_encode(chip->sp, FG_SRAM_CUTOFF_VOLT, cutoff_volt, buf);
	rc = fg_sram_write(chip, chip->sp[FG_SRAM_CUTOFF_VOLT].addr_word,
			chip->sp[FG_SRAM_CUTOFF_VOLT].addr_byte, buf,
			chip->sp[FG_SRAM_CUTOFF_VOLT].len, FG_IMA_DEFAULT);
	if (rc < 0) {
		pr_err("Error in writing cutoff_volt, rc=%d\n", rc);
		return rc;
	}

	fg_encode(chip->sp, FG_SRAM_CUTOFF_CURR, cutoff_curr, buf);
	rc = fg_sram_write(chip, chip->sp[FG_SRAM_CUTOFF_CURR].addr_word,
			chip->sp[FG_SRAM_CUTOFF_CURR].addr_byte, buf,
			chip->sp[FG_SRAM_CUTOFF_CURR].len, FG_IMA_DEFAULT);
	if (rc < 0) {
		pr_err("Error in writing cutoff_curr, rc=%d\n", rc);
		return rc;
	}

	fg_dbg(chip, FG_LGE,
		"Wrote [ki_coeff: low=%d, med=%d, hi=%d]-"
		"[ki curr: lowth=%d, highth=%d]-[cutoff: %dmV, %dmA]-"
		"[status: %s, soc=%d, temp=%s]\n",
		ki_coeff_low, ki_coeff_med, ki_coeff_hi, ki_curr_lowth, ki_curr_highth,
		cutoff_volt, cutoff_curr,
		(chip->charge_status == POWER_SUPPLY_STATUS_DISCHARGING) ? "DISC": "CHG",
		msoc, (chip->esr_flt_sts == LOW_TEMP) ? "low": "room or relex");

	return 0;
}

int override_fg_parse_for_battery_charactistics(struct fg_chip *chip, struct device_node *node)
{
	int rc, temp;

	rc = of_property_read_u32(node, "lge,fg-cutoff-voltage", &temp);
	if (!rc)
		chip->dt.cutoff_volt_mv = temp;

	rc = of_property_read_u32(node, "lge,fg-cutoff-current", &temp);
	if (!rc)
		chip->dt.cutoff_curr_ma = temp;

	chip->dt.cutoff_lt_volt_mv = chip->dt.cutoff_volt_mv;
	rc = of_property_read_u32(node, "lge,fg-cutoff-lt-voltage", &temp);
	if (!rc)
		chip->dt.cutoff_lt_volt_mv = temp;

	chip->dt.cutoff_lt_curr_ma = chip->dt.cutoff_curr_ma;
	rc = of_property_read_u32(node, "lge,fg-cutoff-lt-current", &temp);
	if (!rc)
		chip->dt.cutoff_lt_curr_ma = temp;

	rc = of_property_read_u32(node, "lge,fg-esr-tight-lt-filter-micro-pct", &temp);
	if (!rc)
		chip->dt.esr_tight_lt_flt_upct = temp;

	rc = of_property_read_u32(node, "lge,fg-esr-broad-lt-filter-micro-pct", &temp);
	if (!rc)
		chip->dt.esr_broad_lt_flt_upct = temp;

	fg_dbg(chip, FG_LGE,
		"Wrote [battery charactistics: cutoff=(%dmV, %dmA), cutoff_lt=(%dmV, %dmA), "
		"esr_tight_lt_filter=%d, esr_broad_lt_filter=%d]\n",
		chip->dt.cutoff_volt_mv, chip->dt.cutoff_curr_ma,
		chip->dt.cutoff_lt_volt_mv, chip->dt.cutoff_lt_curr_ma,
		chip->dt.esr_tight_lt_flt_upct, chip->dt.esr_broad_lt_flt_upct);

	return 0;
}

static void polling_voltage_gap(struct work_struct *work);
static void ki_coeff_set_trigger(struct work_struct *work);

int override_fg_parse_ki_coefficient(struct fg_chip *chip, struct device_node *node)
{
	int rc, i, temp;

	chip->dt.dynamic_ki_en = of_property_read_bool(node, "lge,dynamic-ki-en");
	if (chip->dt.dynamic_ki_en) {
		ki_coeff.chg_enable = of_property_read_bool(node,
			"lge,ki-coeff-chg-enable");
		ki_coeff.dischg_enable = of_property_read_bool(node,
			"lge,ki-coeff-dischg-enable");
		rc = of_property_read_u32(node,
			"lge,ki-coeff-soc-alert", &ki_coeff.soc_alert);
		if (rc < 0)
			ki_coeff.soc_alert = 0;
		rc = of_property_read_u32(node,
			"lge,ki-coeff-comp", &ki_coeff.ki_comp);
		if (rc < 0)
			ki_coeff.ki_comp = 200;
		ki_coeff.ki_comp_bck = ki_coeff.ki_comp;
		rc = of_property_read_u32(node,
			"lge,ki-coeff-margin-vfloat-mv", &ki_coeff.margin_vfloat_mv);
		if (rc < 0)
			ki_coeff.margin_vfloat_mv = 0;
		rc = fg_parse_dt_property_u32_array(node, "lge,ki-coeff-limit",
			ki_coeff.ki_limit, KI_COEFF_LIMIT);
		if (rc < 0) {
			ki_coeff.ki_limit[MIN_KI] = 0;
			ki_coeff.ki_limit[MAX_KI] = 5000;
		}

		mutex_init(&ki_coeff.direction_lock);
		INIT_DELAYED_WORK(&chip->polling_voltage_gap_work, polling_voltage_gap);
		INIT_DELAYED_WORK(&chip->ki_coeff_work, ki_coeff_set_trigger);
		schedule_delayed_work(&chip->polling_voltage_gap_work, msecs_to_jiffies(5000));

		pr_info("dynamic:enabled chg:%d dischg:%d soc_alert:%d "
					"comp:%d margin:%d limit:<%d %d>\n",
				ki_coeff.chg_enable, ki_coeff.dischg_enable, ki_coeff.soc_alert,
				ki_coeff.ki_comp, ki_coeff.margin_vfloat_mv,
				ki_coeff.ki_limit[MIN_KI], ki_coeff.ki_limit[MAX_KI]);

	}

	rc = of_property_read_u32(node, "qcom,ki-coeff-full-dischg", &temp);
	if (!rc)
		chip->dt.ki_coeff_full_soc_dischg = temp;

	rc = of_property_read_u32(node, "lge,ki-coeff-curr-lowth-dischg", &temp);
	if (!rc)
		chip->dt.ki_curr_lowth_dischg = temp;

	rc = of_property_read_u32(node, "lge,ki-coeff-curr-highth-dischg", &temp);
	if (!rc)
		chip->dt.ki_curr_highth_dischg = temp;

	rc = fg_parse_dt_property_u32_array(node, "lge,ki-coeff-soc-dischg",
		chip->dt.ki_coeff_soc, KI_COEFF_SOC_LEVELS);
	if (rc < 0)
		return rc;

	rc = fg_parse_dt_property_u32_array(node, "lge,ki-coeff-low-dischg",
		chip->dt.ki_coeff_low_dischg, KI_COEFF_SOC_LEVELS);
	if (rc < 0)
		return rc;

	rc = fg_parse_dt_property_u32_array(node, "lge,ki-coeff-med-dischg",
		chip->dt.ki_coeff_med_dischg, KI_COEFF_SOC_LEVELS);
	if (rc < 0)
		return rc;

	rc = fg_parse_dt_property_u32_array(node, "lge,ki-coeff-hi-dischg",
		chip->dt.ki_coeff_hi_dischg, KI_COEFF_SOC_LEVELS);
	if (rc < 0)
		return rc;

	rc = of_property_read_u32(node, "lge,ki-coeff-hi-chg", &temp);
	if (!rc)
		chip->dt.ki_coeff_hi_chg = temp;

	for (i = 0; i < KI_COEFF_SOC_LEVELS; i++) {
		if (chip->dt.ki_coeff_soc[i] < 0 ||
			chip->dt.ki_coeff_soc[i] > FULL_CAPACITY) {
			pr_err("Error in ki_coeff_soc_dischg values\n");
			return -EINVAL;
		}

		if (chip->dt.ki_coeff_low_dischg[i] < 0 ||
			chip->dt.ki_coeff_low_dischg[i] > KI_COEFF_MAX) {
			pr_err("Error in ki_coeff_low_dischg values\n");
			return -EINVAL;
		}

		if (chip->dt.ki_coeff_med_dischg[i] < 0 ||
			chip->dt.ki_coeff_med_dischg[i] > KI_COEFF_MAX) {
			pr_err("Error in ki_coeff_med_dischg values\n");
			return -EINVAL;
		}

		if (chip->dt.ki_coeff_hi_dischg[i] < 0 ||
			chip->dt.ki_coeff_hi_dischg[i] > KI_COEFF_MAX) {
			pr_err("Error in ki_coeff_hi_dischg values\n");
			return -EINVAL;
		}
	}
	chip->ki_coeff_dischg_en = true;

	return 0;
}

///////////////////////////////////////////////////////////////////////////////

#define PROPERTY_CONSUMED_WITH_SUCCESS	0
#define PROPERTY_CONSUMED_WITH_FAIL	EINVAL
#define PROPERTY_BYPASS_REASON_NOENTRY	ENOENT
#define PROPERTY_BYPASS_REASON_ONEMORE	EAGAIN

static int workaround_backup_bms_property[POWER_SUPPLY_PROP_CYCLE_COUNTS + 1];

static enum power_supply_property extension_bms_appended [] = {
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_UPDATE_NOW,
};

static int extension_bms_get_property_pre(struct power_supply *psy,
	enum power_supply_property prop, union power_supply_propval *val) {
	int rc = PROPERTY_CONSUMED_WITH_SUCCESS;

	struct fg_chip* chip = power_supply_get_drvdata(psy);
	struct irq_desc* irq = irq_to_desc(chip->irqs[SOC_UPDATE_IRQ].irq);

	switch (prop) {
	case POWER_SUPPLY_PROP_RESISTANCE:
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
	case POWER_SUPPLY_PROP_CHARGE_NOW_RAW:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		if (irq && !irq->depth) {
			val->intval = workaround_backup_bms_property[prop];
			pr_info("[W/A] BBP) SOC_UPDATE_IRQ enabled! (==Reading blocked!) "
				"Skip to read property(%d) and reuse %d for faster access time\n",
				prop, val->intval);
		}
		else {
			pr_debug("[W/A] BBP) Not blocked! Bypass to get prp (%d) from bms psy\n",
				prop);
			rc = -PROPERTY_BYPASS_REASON_ONEMORE;
		}
		break;

	case POWER_SUPPLY_PROP_CAPACITY :
		// Battery fake setting has top priority
		if (fake.capacity != LGE_FG_INITVAL)
			val->intval = fake.capacity;
		else if (rescale.result != LGE_FG_INITVAL)
			val->intval = rescale.result;
		else
			rc = -PROPERTY_BYPASS_REASON_ONEMORE;
		break;

	case POWER_SUPPLY_PROP_TEMP :
		if (fake.temperature == LGE_FG_INITVAL)
			val->intval = calculate_battery_temperature(chip); // Use compensated temperature
		else
			val->intval = fake.temperature;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW :
		if (fake.uvoltage == LGE_FG_INITVAL)
			rc = -PROPERTY_BYPASS_REASON_ONEMORE;
		else
			val->intval = fake.uvoltage;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_BOOT : {
		int buf = 0;
		val->intval = !fg_get_sram_prop(chip, FG_SRAM_VOLTAGE_PRED, &buf)
			? buf : LGE_FG_INITVAL;
	}	break;

	case POWER_SUPPLY_PROP_ESR_COUNT : {
		int buf = 0;
		val->intval = !fg_get_sram_prop(chip, FG_SRAM_ESR, &buf)
			? buf : LGE_FG_INITVAL;
	}	break;

	case POWER_SUPPLY_PROP_UPDATE_NOW :
		/* Do nothing and just consume getting */
		val->intval = -1;
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = extension_battery_cycle(chip);
		break;

	default:
		rc = -PROPERTY_BYPASS_REASON_NOENTRY;
		break;
	}

	return rc;
}

static int extension_bms_get_property_post(struct power_supply *psy,
	enum power_supply_property prop, union power_supply_propval *val, int rc) {

	switch (prop) {
	case POWER_SUPPLY_PROP_RESISTANCE:
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
	case POWER_SUPPLY_PROP_CHARGE_NOW_RAW:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		workaround_backup_bms_property[prop] = val->intval;
		break;

	default:
		break;
	}

	return rc;
}

static int extension_bms_set_property_pre(struct power_supply *psy,
	enum power_supply_property prop, const union power_supply_propval *val) {

	int  rc = PROPERTY_CONSUMED_WITH_SUCCESS;
	int* fakeset = NULL;

	switch (prop) {
	case POWER_SUPPLY_PROP_ESR_COUNT : {
		struct fg_chip *chip = power_supply_get_drvdata(psy);
		if (!val->intval) {
			pm_stay_awake(chip->dev);
			schedule_work(&chip->esr_filter_work);
		}
		else
			__fg_esr_filter_config(chip, RELAX_TEMP);
	}	break;

	case POWER_SUPPLY_PROP_UPDATE_NOW :
		if (val->intval)
			fggen3_snapshot_update(psy);
		break;

	case POWER_SUPPLY_PROP_TEMP :
		fakeset = &fake.temperature;
		break;
	case POWER_SUPPLY_PROP_CAPACITY :
		fakeset = &fake.capacity;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW :
		fakeset = &fake.uvoltage;
		break;

	default:
		rc = -PROPERTY_BYPASS_REASON_NOENTRY;
	}

	if (fakeset && *fakeset != val->intval) {
		struct fg_chip *chip = power_supply_get_drvdata(psy);
		*fakeset = val->intval;

		power_supply_changed(chip->batt_psy);
	}

	return rc;
}

static int extension_bms_set_property_post(struct power_supply *psy,
	enum power_supply_property prop, const union power_supply_propval *val, int rc) {
	return rc;
}

///////////////////////////////////////////////////////////////////////////////
enum power_supply_property* extension_bms_properties(void) {
	static enum power_supply_property extended_properties[ARRAY_SIZE(fg_psy_props) + ARRAY_SIZE(extension_bms_appended)];
	int size_original = ARRAY_SIZE(fg_psy_props);
	int size_appended = ARRAY_SIZE(extension_bms_appended);

	memcpy(extended_properties, fg_psy_props,
		size_original * sizeof(enum power_supply_property));
	memcpy(&extended_properties[size_original], extension_bms_appended,
		size_appended * sizeof(enum power_supply_property));

	veneer_extension_pspoverlap(fg_psy_props, size_original,
		extension_bms_appended, size_appended);

	return extended_properties;
}

size_t extension_bms_num_properties(void) {
	return ARRAY_SIZE(fg_psy_props) + ARRAY_SIZE(extension_bms_appended);
}

int extension_bms_get_property(struct power_supply *psy,
	enum power_supply_property prop, union power_supply_propval *val) {

	int rc = extension_bms_get_property_pre(psy, prop, val);
	if (rc == -PROPERTY_BYPASS_REASON_NOENTRY || rc == -PROPERTY_BYPASS_REASON_ONEMORE)
		rc = fg_psy_get_property(psy, prop, val);
	rc = extension_bms_get_property_post(psy, prop, val, rc);

	return rc;
}

int extension_bms_set_property(struct power_supply *psy,
	enum power_supply_property prop, const union power_supply_propval *val) {

	int rc = extension_bms_set_property_pre(psy, prop, val);
	if (rc == -PROPERTY_BYPASS_REASON_NOENTRY || rc == -PROPERTY_BYPASS_REASON_ONEMORE)
		rc = fg_psy_set_property(psy, prop, val);
	rc = extension_bms_set_property_post(psy, prop, val, rc);

	return rc;
}

int extension_bms_property_is_writeable(struct power_supply *psy,
	enum power_supply_property prop) {
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_UPDATE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CAPACITY_RAW:
		rc = 1;
		break;
	default:
		rc = fg_property_is_writeable(psy, prop);
		break;
	}
	return rc;
}

static int extension_fg_gen3_set_ki_coeff_chg(struct fg_chip *chip)
{
	int rc;
	u8 val;
	static int old_ki_chg_hi = -1;

	if (old_ki_chg_hi == ki_coeff.ki_chg[HIGH_CURR])
		return 0;

	fg_encode(chip->sp, FG_SRAM_KI_COEFF_HI_CHG,
		ki_coeff.ki_chg[HIGH_CURR], &val);
	rc = fg_sram_write(chip,
		chip->sp[FG_SRAM_KI_COEFF_HI_CHG].addr_word,
		chip->sp[FG_SRAM_KI_COEFF_HI_CHG].addr_byte, &val,
		chip->sp[FG_SRAM_KI_COEFF_HI_CHG].len,
		FG_IMA_DEFAULT);
	if (rc < 0) {
		pr_err("Error in writing ki_coeff_hi_chg, rc=%d\n", rc);
		return rc;
	}
	old_ki_chg_hi= ki_coeff.ki_chg[HIGH_CURR];
	fg_dbg(chip, FG_LGE,
		"Wrote [ki_coeff_chg: hi=%d]-[status:%s]\n",
		old_ki_chg_hi,
		(chip->charge_status == POWER_SUPPLY_STATUS_DISCHARGING) ? "DISC": "CHG");

	return rc;
}

static void polling_voltage_gap(struct work_struct *work)
{
	struct fg_chip *chip = container_of(work, struct fg_chip,
			polling_voltage_gap_work.work);
	static int backup_gap[BACKUP_MAX_CNT] = {0, };
	int buf = 0, volt_predict = 0, volt_batt = 0, i = 0, msoc = 0;
	int total_gap = 0, slope_volt_gap = 0, real_slope = 0;
	int schedule_time_s = 0;
	bool quick_update = false;
	enum vote_direction direction = VOTE_STAY;

	if (ki_coeff.backup_vote == VOTE_INIT)
		goto out_init_polling;

	if (rescale.result < 0)
		goto out_polling;
	msoc = rescale.result;

	volt_batt = !fg_get_battery_voltage(chip, &buf)
		? buf / 1000 : LGE_FG_INITVAL;
	volt_predict = !fg_get_sram_prop(chip, FG_SRAM_VOLTAGE_PRED, &buf)
		? buf / 1000 : LGE_FG_INITVAL;

	if (volt_predict >= (chip->bp.vbatt_full_mv - ki_coeff.margin_vfloat_mv)) {
		volt_predict = min(chip->bp.vbatt_full_mv, volt_predict);
		ki_coeff.reach_vfloat = true;
	}
	else {
		ki_coeff.reach_vfloat = false;
	}

	if (msoc < ki_coeff.soc_alert)
		quick_update = true;

	if (ki_coeff.loop_cnt == BACKUP_MAX_CNT)
		ki_coeff.loop_cnt--;

	for (i = ki_coeff.loop_cnt ; i > 0 ; i--) {
		backup_gap[i] = backup_gap[i-1];
		total_gap += backup_gap[i];
	}

	ki_coeff.loop_cnt = min(ki_coeff.loop_cnt+1, BACKUP_MAX_CNT);
	backup_gap[0] = volt_batt - volt_predict;
	total_gap += backup_gap[0];
	if (quick_update) {
		slope_volt_gap = (total_gap*CHANGE_POINT_INC) / (BACKUP_MAX_CNT*LOOP_TIME_ALERT);
		real_slope = (total_gap*CHANGE_POINT_INC) / (ki_coeff.loop_cnt*LOOP_TIME_ALERT);
		schedule_time_s = LOOP_TIME_ALERT * 1000;
	}
	else {
		slope_volt_gap = (total_gap*CHANGE_POINT_INC) / (BACKUP_MAX_CNT*LOOP_TIME_NORMAL);
		real_slope = (total_gap*CHANGE_POINT_INC) / (ki_coeff.loop_cnt*LOOP_TIME_NORMAL);
		schedule_time_s = LOOP_TIME_NORMAL * 1000;
	}

	pr_debug("vbat:%d vPredict:%d, backup_gap=[%d %d %d %d %d %d]\n",
		volt_batt, volt_predict, backup_gap[0], backup_gap[1],
		backup_gap[2], backup_gap[3], backup_gap[4], backup_gap[5]);
	pr_debug("slope(%d) = total_gap(%d)*100 (%d) / (6)*5 (30)\n",
			slope_volt_gap, total_gap, total_gap*100);

	switch(chip->charge_status) {
	case POWER_SUPPLY_STATUS_CHARGING:
		if (!(ki_coeff.chg_enable))
			goto out_polling;

		if (slope_volt_gap >= (CHANGE_POINT))
		//have to increase vpredict -> down KI for dischg
			direction = VOTE_DECREASE;
		else if (slope_volt_gap < (CHANGE_POINT_INC*-1)/(quick_update ? SCALING_FACTOR_INC : 1))
		//have to decrease vpredict -> raise KI for dischg
			direction = VOTE_INCREASE;
		else
			direction = VOTE_STAY;

		if (direction == VOTE_STAY) {
			ki_coeff.stay_cnt++;
			if (ki_coeff.stay_cnt >= BACKUP_MAX_CNT) {
				direction = VOTE_DECREASE;
				ki_coeff.stay_cnt = 0;
			}
		}
		else {
			ki_coeff.stay_cnt = 0;
		}

		if (ki_coeff.reach_vfloat)
			direction = max(direction-1, VOTE_DECREASE);

		break;
	case POWER_SUPPLY_STATUS_DISCHARGING:
		if (!(ki_coeff.dischg_enable))
			goto out_polling;

		if (slope_volt_gap >= (CHANGE_POINT))
		//have to increase vpredict -> raise KI for dischg
			direction = VOTE_INCREASE;
		else if (slope_volt_gap < (CHANGE_POINT_INC*-1)/(quick_update ? SCALING_FACTOR_INC : 1))
		//have to decrease vpredict -> down KI for dischg
			direction = VOTE_DECREASE;
		else
			direction = VOTE_STAY;

		if (quick_update && (direction != VOTE_STAY)) {
			//At low soc, we directly call below function
			ki_coeff.ki_comp = ki_coeff.ki_comp_bck*abs(real_slope)/10;
			ki_coeff.backup_vote = direction;
			extension_fg_gen3_set_ki_coeff(chip);
		}
		else if (ki_coeff.ki_comp != ki_coeff.ki_comp_bck){
			ki_coeff.ki_comp = ki_coeff.ki_comp_bck;
		}

		break;
	default:
		direction = VOTE_STAY;
		break;
	}

	if (direction != ki_coeff.backup_vote) {
		pr_info("direction=%d, backup=%d, tot_gap=%d, cnt=%d, slope=%d, real_slope=%d, "
			"vfloat=%d, stay=%d, alert=%d, charge=%s\n",
			direction, ki_coeff.backup_vote, total_gap, ki_coeff.loop_cnt, slope_volt_gap,
			real_slope, ki_coeff.reach_vfloat, ki_coeff.stay_cnt, quick_update,
			(chip->charge_status == POWER_SUPPLY_STATUS_CHARGING) ? "CHG" : "DISCHG");

		if ((direction == VOTE_STAY) ||
			(ki_coeff.backup_vote == VOTE_ELECTION)) {
			for (i = ki_coeff.loop_cnt - 1 ; i >= 0 ; i--) {
				backup_gap[i] = 0;
			}
			ki_coeff.loop_cnt = 0;
			direction = VOTE_STAY;
		}
	}

out_polling:
	ki_coeff.backup_vote = direction;
out_init_polling:
	schedule_delayed_work(&chip->polling_voltage_gap_work,
			msecs_to_jiffies(schedule_time_s));
}

static void extension_fg_gen3_clear_ki_coeff_chg(struct fg_chip *chip)
{
	int rc = 0;

	if (chip->dt.ki_coeff_hi_chg != -EINVAL) {
		ki_coeff.ki_chg[HIGH_CURR] = chip->dt.ki_coeff_hi_chg;
		rc = extension_fg_gen3_set_ki_coeff_chg(chip);

		if (rc < 0) {
			pr_err("Error in writing ki_coeff_chg, rc=%d\n", rc);
		}
	}
}

static void extension_fg_gen3_clear_ki_coeff_dischg(struct fg_chip *chip)
{
	int rc = 0, i = 0, msoc = 0;
	int ki_coeff_low = KI_COEFF_LOW_DISCHG_DEFAULT;
	int ki_coeff_med = KI_COEFF_MED_DISCHG_DEFAULT;
	int ki_coeff_hi = KI_COEFF_HI_DISCHG_DEFAULT;

	for (i = 0 ; i <= LOW_CURR ; i++) {
		ki_coeff.ki_dischg[i][HIGH_CURR] = chip->dt.ki_coeff_hi_dischg[i];
		ki_coeff.ki_dischg[i][MED_CURR] = chip->dt.ki_coeff_med_dischg[i];
		ki_coeff.ki_dischg[i][LOW_CURR] = chip->dt.ki_coeff_low_dischg[i];
		pr_debug("setting ki_dischg[%d]high:%d med:%d low%d\n",
				i,
				ki_coeff.ki_dischg[i][HIGH_CURR],
				ki_coeff.ki_dischg[i][MED_CURR],
				ki_coeff.ki_dischg[i][LOW_CURR]);
	}

	msoc = rescale.result;
	for (i = KI_COEFF_SOC_LEVELS - 1; i >= 0; i--) {
		if (msoc < chip->dt.ki_coeff_soc[i]) {
			ki_coeff_hi = ki_coeff.ki_dischg[i][HIGH_CURR];
			ki_coeff_med = ki_coeff.ki_dischg[i][MED_CURR];
			ki_coeff_low = ki_coeff.ki_dischg[i][LOW_CURR];
			break;
		}
	}
	rc = override_fg_adjust_ki_coeff_dischg(chip,
			ki_coeff_low, ki_coeff_med, ki_coeff_hi);

	if (rc < 0) {
		pr_err("Error in writing ki_coeff_dischg, rc=%d\n", rc);
	}
}
static int extension_fg_gen3_set_ki_coeff(struct fg_chip *chip)
{
	int rc = 0, i = 0, msoc = 0;
	int ki_coeff_low = KI_COEFF_LOW_DISCHG_DEFAULT;
	int ki_coeff_med = KI_COEFF_MED_DISCHG_DEFAULT;
	int ki_coeff_hi = KI_COEFF_HI_DISCHG_DEFAULT;
	enum vote_direction current_set = VOTE_NONE;

	if (rescale.result < 0)
		return 0;

	mutex_lock(&ki_coeff.direction_lock);
	msoc = rescale.result;
	current_set = ki_coeff.backup_vote;

	switch (current_set) {
	case VOTE_SUSPEND:
		if (ki_coeff.dischg_enable &&
				(chip->charge_status ==	POWER_SUPPLY_STATUS_DISCHARGING))
			extension_fg_gen3_clear_ki_coeff_dischg(chip);
		else if (ki_coeff.chg_enable &&
				(chip->charge_status ==	POWER_SUPPLY_STATUS_CHARGING))
			extension_fg_gen3_clear_ki_coeff_chg(chip);
		break;
	case VOTE_INIT:
		extension_fg_gen3_clear_ki_coeff_chg(chip);
		extension_fg_gen3_clear_ki_coeff_dischg(chip);
		ki_coeff.backup_vote = VOTE_STAY;
		break;
	case VOTE_NONE:
		if (ki_coeff.chg_enable)
			extension_fg_gen3_clear_ki_coeff_chg(chip);
		if (ki_coeff.dischg_enable)
			extension_fg_gen3_clear_ki_coeff_dischg(chip);
		ki_coeff.backup_vote = VOTE_STAY;
		ki_coeff.stay_cnt = 0;
		ki_coeff.ki_comp = ki_coeff.ki_comp_bck;
		if (delayed_work_pending(&chip->polling_voltage_gap_work)) {
			pr_info("Cancel the pending voltage check work\n");
			cancel_delayed_work_sync(&chip->polling_voltage_gap_work);
		}
		schedule_delayed_work(&chip->polling_voltage_gap_work, msecs_to_jiffies(5000));
		break;
	case VOTE_INCREASE:
	case VOTE_DECREASE:
		if (ki_coeff.dischg_enable &&
				(chip->charge_status == POWER_SUPPLY_STATUS_DISCHARGING)) {
			for (i = KI_COEFF_SOC_LEVELS - 1; i >= 0; i--) {
				if (msoc < chip->dt.ki_coeff_soc[i]) {
					ki_coeff.ki_dischg[i][HIGH_CURR] =
						max(ki_coeff.ki_dischg[i][HIGH_CURR] + current_set*ki_coeff.ki_comp, ki_coeff.ki_limit[MIN_KI]);
					ki_coeff.ki_dischg[i][HIGH_CURR] =
						min(ki_coeff.ki_dischg[i][HIGH_CURR], ki_coeff.ki_limit[MAX_KI]);
					ki_coeff.ki_dischg[i][MED_CURR] =
						max(ki_coeff.ki_dischg[i][MED_CURR] + current_set*ki_coeff.ki_comp, ki_coeff.ki_limit[MIN_KI]);
					ki_coeff.ki_dischg[i][MED_CURR] =
						min(ki_coeff.ki_dischg[i][MED_CURR], ki_coeff.ki_limit[MAX_KI]);

					ki_coeff_hi = ki_coeff.ki_dischg[i][HIGH_CURR];
					ki_coeff_med = ki_coeff.ki_dischg[i][MED_CURR];
					break;
				}
			}

			rc = override_fg_adjust_ki_coeff_dischg(chip,
					ki_coeff_low, ki_coeff_med, ki_coeff_hi);
			if (rc < 0) {
				pr_err("Error in writing ki_coeff_dischg, rc=%d\n", rc);
				mutex_unlock(&ki_coeff.direction_lock);
				return rc;
			}

			pr_debug("dischg High: %d, Med: %d, Low:%d\n",
				ki_coeff_hi, ki_coeff_med, ki_coeff_low);
		}
		else if (ki_coeff.chg_enable &&
				(chip->charge_status == POWER_SUPPLY_STATUS_CHARGING)) {
			ki_coeff.ki_chg[HIGH_CURR] = max(ki_coeff.ki_chg[HIGH_CURR] + current_set*ki_coeff.ki_comp, ki_coeff.ki_limit[MIN_KI]);
			ki_coeff.ki_chg[HIGH_CURR] = min(ki_coeff.ki_chg[HIGH_CURR], ki_coeff.ki_limit[MAX_KI]);

			rc = extension_fg_gen3_set_ki_coeff_chg(chip);
			if (rc < 0) {
				pr_err("Error in writing ki_coeff_chg, rc=%d\n", rc);
				mutex_unlock(&ki_coeff.direction_lock);
				return rc;
			}
			pr_debug("chg High: %d\n", ki_coeff.ki_chg[HIGH_CURR]);
		}
		ki_coeff.backup_vote = VOTE_ELECTION;

		break;
	default:

		break;
	}
	/* When change charging status */
	if (chip->prev_charge_status != chip->charge_status) {
		ki_coeff.backup_vote = VOTE_ELECTION;
		ki_coeff.stay_cnt = 0;
		ki_coeff.ki_comp = ki_coeff.ki_comp_bck;
		if (ki_coeff.dischg_enable &&
				(chip->charge_status ==	POWER_SUPPLY_STATUS_CHARGING))
			extension_fg_gen3_clear_ki_coeff_dischg(chip);
		else if (ki_coeff.chg_enable &&
				(chip->charge_status ==	POWER_SUPPLY_STATUS_DISCHARGING))
			extension_fg_gen3_clear_ki_coeff_chg(chip);
	}

	mutex_unlock(&ki_coeff.direction_lock);
	return rc;
}

static void override_fg_hw_init(struct fg_chip *chip)
{
	if (!(chip->dt.dynamic_ki_en)) {
		extension_fg_gen3_clear_ki_coeff_chg(chip);
	}
}

static void ki_coeff_set_trigger(struct work_struct *work)
{
	struct fg_chip *chip = container_of(work, struct fg_chip,
			ki_coeff_work.work);

	mutex_lock(&ki_coeff.direction_lock);
	ki_coeff.backup_vote = VOTE_NONE;
	mutex_unlock(&ki_coeff.direction_lock);

	extension_fg_gen3_set_ki_coeff(chip);
}

struct device_node* extension_get_batt_profile(struct device_node* container,
	int resistance_id) {
       /* Search with resistance_id and
	* Hold the result to an unified node(sysfs) for the fab process
	*/
	char buffer [8] = { '\0', };
	struct device_node* profile = of_batterydata_get_best_profile(container,
		resistance_id, NULL);

	/* If no matching, set it as default */
	if (!profile) {
		#define DTNODE_DEFAULT_BATTERY_ROOT "lge-battery-supplement"
		#define DTNODE_DEFAULT_BATTERY_NAME "default-battery-name"
		#define DTNODE_DEFAULT_BATTERY_KOHM "default-battery-kohm"

		struct device_node* node = of_find_node_by_name(NULL,
			DTNODE_DEFAULT_BATTERY_ROOT);
		const char* name = of_property_read_string(node, DTNODE_DEFAULT_BATTERY_NAME,
			&name) ? NULL : name;
		int kohm = of_property_read_u32(node, DTNODE_DEFAULT_BATTERY_KOHM, &kohm)
			? 0 : kohm;
		profile = of_batterydata_get_best_profile(container,
			kohm, name);
		pr_err("Getting default battery profile(%s): %s\n", name,
			profile ? "success" : "fail");
	}

	// At this time, 'battery_valid' may be true always for the embedded battery model
	#define UNINODE_BATTERY_VALID	"battery_valid"
	snprintf(buffer, sizeof(buffer), "%d", !!profile);
	unified_nodes_store(UNINODE_BATTERY_VALID, buffer, sizeof(buffer));

	return profile;
}

static int extension_fg_load_icoeff_dt(struct fg_chip *chip)
{
	struct device_node* tcomp_dtroot;
	struct device_node* tcomp_override;
	int dt_icomp = 0;

	if (tcomp.icoeff_load_done) {
		pr_info("icoeff had already been loaded.\n");
		return 0;
	}

	if (!chip->soc_reporting_ready) {
		pr_info("fG profile is not ready.\n");
		return LGE_FG_INITVAL;
	}

	tcomp_dtroot = of_find_node_by_name(NULL, "lge-battery-supplement");
	if (!tcomp_dtroot) {
		pr_info("failed to find lge-battery-supplement\n");
		return LGE_FG_INITVAL;
	}

	if (chip->bp.batt_type_str) {
		tcomp_override = of_find_node_by_name(
				tcomp_dtroot, chip->bp.batt_type_str);
		if (tcomp_override &&
				of_property_read_u32(
					tcomp_override, "tempcomp-icoeff", &dt_icomp) >= 0)
			pr_info("ICOEFF is overridden to %d for %s\n", dt_icomp, chip->bp.batt_type_str);
	}

	if (!dt_icomp) {
		if (of_property_read_u32(tcomp_dtroot, "tempcomp-icoeff", &dt_icomp) >= 0) {
			pr_info("ICOEFF is set to %d by default\n", dt_icomp);
		} else {
			pr_info("ICOEFF isn't set. error.\n");
			return -1;
		}
	}

	tcomp.icoeff = dt_icomp;
	tcomp.icoeff_load_done = true;
	return 0;
}

static int extension_fg_load_dt(void)
{
	const char str_tempcomp[TCOMP_TABLE_MAX][30] = {
		"tempcomp-offset",
		"tempcomp-offset-wlc-lcdoff",
		"tempcomp-offset-wlc-lcdon"
	};

	struct device_node* tcomp_dtroot;
	int dtarray_count = TCOMP_COUNT * 2;
	u32 dtarray_data [TCOMP_COUNT * 2];
	int i = 0, j = 0;

	if (tcomp.load_done) {
		pr_info("tcomp table had already been loaded.\n");
		return 0;
	}

	tcomp_dtroot = of_find_node_by_name(NULL, "lge-battery-supplement");
	if (!tcomp_dtroot) {
		pr_info("failed to find lge-battery-supplement\n");
		return -1;
	}

	if (of_property_read_bool(tcomp_dtroot, "tempcomp-offset-wlc-enable"))
		tcomp.load_max = 3;
	else
		tcomp.load_max = 1;

	for (j = 0; j < tcomp.load_max; j++ ) {
		/* Finding tcomp_table and tcomp_icoeff */
		if (of_property_read_u32_array(tcomp_dtroot, str_tempcomp[j],
				dtarray_data, dtarray_count) >= 0) {
			for (i = 0; i < dtarray_count; i += 2) {
				tcomp.table[j][i/2].temp_cell = dtarray_data[i];
				tcomp.table[j][i/2].temp_bias = dtarray_data[i+1];
				pr_debug("Index = %02d : %4d - %4d\n",
					i/2,
					tcomp.table[j][i/2].temp_cell,
					tcomp.table[j][i/2].temp_bias);
			}
		} else {
			pr_info("%s is not found, error\n", str_tempcomp[j]);
			tcomp.table[j][0].temp_cell = INT_MAX;
			tcomp.table[j][0].temp_bias = 0;
			return -1;
		}
	}

	tcomp.rise_cut = of_property_read_bool(tcomp_dtroot,
		"tempcomp-offset-wlc-rise-filter-enable");
	if (tcomp.rise_cut)
		of_property_read_u32(tcomp_dtroot,
			"tempcomp-offset-wlc-rise-filter-trigger", &tcomp.rise_cut_trig);
	tcomp.fall_cut = of_property_read_bool(tcomp_dtroot,
		"tempcomp-offset-wlc-fall-filter-enable");
	if (tcomp.fall_cut)
		of_property_read_u32(tcomp_dtroot,
			"tempcomp-offset-wlc-fall-filter-trigger", &tcomp.fall_cut_trig);

	of_property_read_u32(tcomp_dtroot,
		"capacity-raw-full", &rescale.criteria);
	rescale.lge_monotonic = of_property_read_bool(tcomp_dtroot,
		"lg-monotonic-soc-enable");
	tcomp.logging = of_property_read_bool(tcomp_dtroot,
		"tempcomp-logging");
	tcomp.qnovo_charging = of_property_read_bool(tcomp_dtroot,
		"tempcomp-qnovo-charging");

	smooth_therm.smooth_filter_enable = of_property_read_bool(tcomp_dtroot,
		"tempcomp-smooth-filter-enable");

	if (j == tcomp.load_max) {
		tcomp.load_done = true;

		pr_info("[tempcomp config] table count: %s (%d/%d), "
			"rise cut filter: %s (trigger = %d degree), "
			"fall cut filter: %s (trigger = %d degree)\n",
			(j == tcomp.load_max) ? "done" : "error", j, tcomp.load_max,
			tcomp.rise_cut ? "enabled" : "disabled", tcomp.rise_cut_trig,
			tcomp.fall_cut ? "enabled" : "disabled", tcomp.fall_cut_trig);
	}

	return 0;
}

static int extension_fg_gen3_suspend(struct fg_chip *chip)
{
	if (smooth_therm.smooth_filter_enable)
		smooth_therm.batt_therm_ready = false;

	if (chip->dt.dynamic_ki_en) {
		cancel_delayed_work_sync(&chip->polling_voltage_gap_work);
		cancel_delayed_work_sync(&chip->ki_coeff_work);
	}

	return 0;
}

static int extension_fg_gen3_resume(struct fg_chip *chip)
{
	if (chip->dt.dynamic_ki_en) {
		schedule_delayed_work(&chip->ki_coeff_work, msecs_to_jiffies(5000));
	}

	return 0;
}

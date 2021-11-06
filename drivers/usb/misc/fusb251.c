/*
 * LGE FUSB251 Moisture Detection Driver
 *
 * Copyright (C) 2020 LG Electronics, Inc.
 * Author: Jayci Choi <jayci.choi@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/alarmtimer.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/fusb251.h>

#ifdef CONFIG_LGE_USB
#include <soc/qcom/lge/board_lge.h>
#endif

#ifdef CONFIG_LGE_DUAL_SCREEN
#include <linux/lge_ds3.h>
#endif

#ifdef CONFIG_LGE_USB_SBU_SWITCH
#include <linux/usb/lge_sbu_switch.h>
#endif

struct i2c_client *__client = NULL;

static unsigned int cc_floating_time_ms = 3000;
module_param(cc_floating_time_ms, uint, 0644);

static unsigned int fix_cc_sink_time_ms = 30000;
module_param(fix_cc_sink_time_ms, uint, 0644);

static unsigned int recheck_dry_time_ms  = 60000;
module_param(recheck_dry_time_ms, uint, 0644);

static int disable_moisture = 0;
module_param(disable_moisture, uint, 0600);

static const char * const fusb251_state_strings[] = {
	"Dry",
	"CC_Detecting",
	"CC_Floating",
	"SBU_Floating",
	"CC_SBU_Auto_Detecting",
	"CC_Moisture_Detected",
	"SBU_Moisture_Detected",
	"Type-C_Connected",
	"Dual Screen Connected",
	"Error_Recovery",
};

struct fusb251 {
	struct i2c_client		*client;

	struct gpio_desc		*intb_desc;
	struct regulator		*intb_pu18;

	struct workqueue_struct		*wq;
	struct work_struct		i2c_work;
	struct work_struct		sm_work;
	struct delayed_work		enable_mos_work;
	struct delayed_work		check_reg_work;
	struct alarm			timer;

	struct mutex			irq_lock;
	struct mutex			i2c_lock;

	struct power_supply		*usb_psy;
	struct notifier_block		psy_nb;
#ifdef CONFIG_LGE_USB_SBU_SWITCH
	struct lge_sbu_switch_desc      lge_sbu_switch_desc;
	struct lge_sbu_switch_instance *lge_sbu_switch_inst;
#endif

	enum power_supply_typec_mode	typec_mode;

	atomic_t			pm_suspended;

	int				intb_irq;

	int				rev;
	int				state;
	int				ovp_state;
	int				sbu_check_count;

	bool                            cc_mos_detected;
	bool                            sbu_mos_detected;
	bool				sbu_fault;

	bool				cc_ovp;
	bool				sbu_ovp;

	bool				typec_connected;
	bool				ds_connected;

	bool				moisture_enable;
	bool				auto_moisture_enable;
	bool				vbus_present;
	bool				initialized;
};

static void fusb251_init(struct fusb251 *);
static void fusb251_dump_register(struct fusb251 *);
static void fusb251_enable_moisture_detect(struct fusb251 *, bool);
static void fusb251_set_state(struct fusb251 *, int);
static void fusb251_reset(struct fusb251 *);

static int fusb251_read_reg(struct i2c_client *client, int reg)
{
	struct fusb251 *fusb;
	int ret;
	int retry = 0;

	if (!client)
		return -ENODEV;

	fusb = i2c_get_clientdata(client);
	mutex_lock(&fusb->i2c_lock);
	do {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0) {
			pr_err("%s: err %d, and try again\n", __func__, ret);
			mdelay(100);
		}
	} while ((retry++ < 5) && (ret < 0));

	if (ret < 0) {
		pr_err("%s: err %d\n", __func__, ret);
	}

	mutex_unlock(&fusb->i2c_lock);
	return ret;
}

static int fusb251_write_reg(struct i2c_client *client, int reg, u8 value)
{
	struct fusb251 *fusb;
	int ret, i;

	if (!client)
		return -ENODEV;

	fusb = i2c_get_clientdata(client);

	mutex_lock(&fusb->i2c_lock);
	for (i = 0; i < 5; i++) {
		ret = i2c_smbus_write_byte_data(client, reg, value);

		/* When writing to RESET_DEVICE register,
		 * IC is reset and returned as an error
		 * as soon as i2c byte write is done. */
		if (reg == FUSB251_RESET && value == RESET_BIT)
			ret = 0;

		if (ret < 0) {
			pr_err("%s: err %d, and try again\n", __func__, ret);
			mdelay(100);
		} else {
			break;
		}
	}

	if (ret < 0) {
		pr_err("%s: err %d\n", __func__, ret);
	}

	mutex_unlock(&fusb->i2c_lock);
	return ret;
}

static void fusb251_print_reg(int reg, u8 value)
{
	switch (reg) {
	case FUSB251_CONTROL:
		pr_info("FUSB251 CONTROL - MAN_SW:%d, CC_MOS:%d, SBU_MOS:%d, "
				"SBUFT:%d, AUTO_SBU:%d, EN_DRY:%d, "
				"DEBUG_ACC:%d\n",
				(value & EN_MAN_SW_CTRL_BIT) ? 1 : 0,
				(value & EN_CC_MOS_BIT) ? 1 : 0,
				(value & EN_SBU_MOS_BIT) ? 1 : 0,
				(value & EN_SBUFT_BIT) ? 1 : 0,
				(value & EN_AUTO_SBU_BIT) ? 1 : 0,
				(value & EN_DRY_BIT) ? 1 : 0,
				(value & EN_DEBUG_ACC_DET_BIT) ? 1 : 0);
		break;
	case FUSB251_INTERRUPT:
		pr_info("FUSB251 IRQ - OVP:%d, OVP_REC:%d, MOS_CHG:%d, DRY_CHG:%d,"
				"CC1_TIMER:%d, CC2_TIMER:%d\n",
				(value & OVP_INT_BIT) ? 1: 0,
				(value & OVP_REC_INT_BIT) ? 1: 0,
				(value & MOS_CHG_INT_BIT) ? 1: 0,
				(value & DRY_CHG_INT_BIT) ? 1: 0,
				(value & CC1_TIMER_INT_BIT) ? 1: 0,
				(value & CC2_TIMER_INT_BIT) ? 1: 0);
		break;
	case FUSB251_INTERRUPT_MASK:
		pr_info("FUSB251 IRQ_MASK - OVP:%d, OVP_REC:%d, MOS_DET:%d, DRY_DET:%d,"
				"CC1_TIMER:%d, CC2_TIMER:%d\n",
				(value & MASK_OVP_BIT) ? 1 : 0,
				(value & MASK_OVP_REC_BIT) ? 1 : 0,
				(value & MASK_MOS_DETECT_BIT) ? 1 : 0,
				(value & MASK_DRY_DETECT_BIT) ? 1 : 0,
				(value & MASK_CC1_TIMER_BIT) ? 1 : 0,
				(value & MASK_CC2_TIMER_BIT) ? 1 : 0);
		break;
	case FUSB251_STATUS:
		pr_info("FUSB251 STATUS - OVP_CC:%d, OVP_SBU:%d, LOOK4DRY:%d, LOOK4SBU:%d,"
				"LOOK4CC:%d\n",
				(value & OVP_CC_BIT) ? 1 : 0,
				(value & OVP_SBU_BIT) ? 1 : 0,
				(value & LOOK4DRY_BIT) ? 1 : 0,
				(value & LOOK4SBU_BIT) ? 1 : 0,
				(value & LOOK4CC_BIT) ? 1 : 0);
		break;
	case FUSB251_SWITCH_CONTROL:
		pr_info("FUSB251 SWITCH_CTRL - CC:%d, SBU:%d\n",
				(value & CC_SW_CTRL_BIT) ? 1 : 0,
				value & SBU_SW_CTRL_MASK);
		break;
	case FUSB251_MOISTURE_STATUS:
		pr_info("FUSB251 MOS_STATUS - CC1_MOS:%d, CC2_MOS:%d, SBU1_MOS:%d,"
				" SBU2_MOS:%d, SBU1_FT:%d, SBU2_FT:%d"
				" FAULT:0x%x\n",
				(value & CC1_MOS_BIT) ? 1 : 0,
				(value & CC2_MOS_BIT) ? 1 : 0,
				(value & SBU1_MOS_BIT) ? 1 : 0,
				(value & SBU2_MOS_BIT) ? 1 : 0,
				(value & SBU1_FT_BIT) ? 1 : 0,
				(value & SBU2_FT_BIT) ? 1 : 0,
				value & FAULT_MASK);
		break;
	case FUSB251_THRESHOLD1:
		pr_info("FUSB251 SBU_MOS_THR:0x%x\n",
				value & (unsigned int)SBU_MOS_DET_THR_MASK);
		break;
	default:
		pr_info("Reg:0x%x, value:0x%x\n", reg, value);
		break;
	}
}

// Enable if Manual Switch Control needed
#if 0
static void fusb251_switch_control(struct fusb251 *fusb, int cc_sw, int sbu_sw)
{
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;

	dev_info("%s: set cc_sw=%d, sbu_sw=%d, factory_mode=%d\n",
			__func__, cc_sw, sbu_sw, sbu_sw >= 2 ? 1 : 0);
	fusb251_write_reg(client, FUSB251_SWITCH_CONTROL,
			CC_SW_CTRL_BIT & cc_sw | SBU_SW_CTRL_MASK & sbu_sw << 1);
}
#endif

bool fusb251_complete_probe(void)
{
	return __client ? true : false;
}
EXPORT_SYMBOL(fusb251_complete_probe);

void fusb251_update_state(int n_oe, int sel)
{
	struct fusb251 *fusb;
	int value = 0;
	int control = 0;

	if (!__client) {
		return;
	}

	fusb = i2c_get_clientdata(__client);
	control = fusb251_read_reg(__client, FUSB251_CONTROL);
	pr_info("%s: CONTROL: current [0x%x]\n",__func__, control);

	if (sel)
		value = CC_SW_CTRL_BIT |
			(SBU_SW_CTRL_MASK & SBU1_FM_SBU2_OPEN);
	else
		value = CC_SW_CTRL_BIT |
			(SBU_SW_CTRL_MASK & CLOSE_SBU1_SBU2);

	if (n_oe)
		value = value & (SBU_SW_CTRL_MASK & 0);

	if (fusb && fusb->typec_connected)
		control = 0;

	control |= EN_MAN_SW_CTRL_BIT;

	pr_info("%s: SWITCH_CONTROL: [0x%x]\n", __func__, value);
	pr_info("%s: CONTROL: new [0x%x]\n", __func__, control);

	fusb251_write_reg(__client, FUSB251_SWITCH_CONTROL, value);
	mdelay(10);
	fusb251_write_reg(__client, FUSB251_CONTROL, control);
}
EXPORT_SYMBOL(fusb251_update_state);

void fusb251_enable_moisture(bool enable)
{
	struct fusb251 *fusb = i2c_get_clientdata(__client);

	fusb251_enable_moisture_detect(fusb, enable);
	if (enable) {
		fusb251_set_state(fusb, FUSB251_STATE_DRY);
	} else {
#ifdef CONFIG_LGE_DUAL_SCREEN
		if (check_ds_connect_state() >= DS_STATE_ACC_ID_CONNECTED)
			fusb251_set_state(fusb, FUSB251_STATE_DS_CONNECTED);
		else
#endif
		fusb251_set_state(fusb, FUSB251_STATE_TYPEC_CONNECTED);
	}

}
EXPORT_SYMBOL(fusb251_enable_moisture);

static void kick_sm(struct fusb251 *fusb, int ms)
{
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;

//	fusb->sm_queued = true;

	alarm_cancel(&fusb->timer);
	if (ms) {
		dev_dbg(dev, "delay %dms", ms);
		alarm_start_relative(&fusb->timer, ms_to_ktime(ms));
	} else {
		pm_stay_awake(dev);
		queue_work(fusb->wq, &fusb->sm_work);
	}
}

static enum alarmtimer_restart fusb251_timeout(struct alarm *alarm, ktime_t now)
{
	struct fusb251 *fusb = container_of(alarm, struct fusb251, timer);
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;

	pm_stay_awake(dev);
	if (atomic_read(&fusb->pm_suspended)) {
		dev_info(dev, "%s: device is suspended\n", __func__);
		alarm_start_relative(&fusb->timer, ms_to_ktime(100));
	} else {
		queue_work(fusb->wq, &fusb->sm_work);
	}

	return ALARMTIMER_NORESTART;
}

static void fusb251_enable_moisture_detect(struct fusb251 *fusb, bool enable)
{
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;
	int status, control = 0;
	bool look4cc;

	pm_stay_awake(dev);
//	control = fusb251_read_reg(client, FUSB251_CONTROL);

//	dev_info(dev, "%s: current control:0x%x, enable:%d\n", __func__,
//			control, enable);

	if (disable_moisture || fusb->typec_connected) {
		enable = false;
	}

	dev_info(dev, "%s: enable=%d\n", __func__, enable);
	if (enable) {
		fusb251_write_reg(client, FUSB251_RESET, MOS_RESET_BIT);
		mdelay(10);

		fusb->moisture_enable = true;
		control = EN_MAN_SW_CTRL_BIT | EN_CC_MOS_BIT | /*EN_AUTO_SBU_BIT |*/
			EN_DRY_BIT | EN_DEBUG_ACC_DET_BIT;
	} else {
		fusb->moisture_enable = false;
		control = EN_MAN_SW_CTRL_BIT | EN_DEBUG_ACC_DET_BIT;
//			control & (!EN_CC_MOS_BIT & !EN_AUTO_SBU_BIT &
//			!EN_DRY_BIT);
	}
	fusb251_write_reg(client, FUSB251_CONTROL, control);
	fusb->auto_moisture_enable = false;
	mdelay(10);
	status = fusb251_read_reg(client, FUSB251_STATUS);
	look4cc = (status & LOOK4CC_BIT) ? true : false;

	if (enable != look4cc) {
		fusb251_set_state(fusb, FUSB251_STATE_ERROR_RECOVERY);
	}

	/* check written register values */
	//ontrol = fusb251_read_reg(client, FUSB251_CONTROL);
	//ev_info(dev, "%s: new control:0x%x, enable:%d\n", __func__,
	//	control, enable);
	pm_relax(dev);
}

static void fusb251_run_sbu_detect(struct fusb251 *fusb, bool enable)
{
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;
	int control = 0;

	pm_stay_awake(dev);

	dev_dbg(dev, "%s: enable=%d\n", __func__, enable);

	if (!fusb->cc_mos_detected) {
		dev_info(dev, "CC moisture not detected\n");
		pm_relax(dev);
		return;
	}

	fusb251_write_reg(client, FUSB251_RESET, MOS_RESET_BIT);
	mdelay(10);

	if (enable) {
		control = EN_MAN_SW_CTRL_BIT | EN_CC_MOS_BIT |
			EN_AUTO_SBU_BIT | EN_DRY_BIT | EN_DEBUG_ACC_DET_BIT;
		fusb->auto_moisture_enable = true;
	} else {
		control = EN_MAN_SW_CTRL_BIT | EN_CC_MOS_BIT |
			EN_DRY_BIT | EN_DEBUG_ACC_DET_BIT;
		fusb->auto_moisture_enable = false;
	}
	fusb251_write_reg(client, FUSB251_CONTROL, control);
	pm_relax(dev);
}

static void fusb251_ovp(struct fusb251 *fusb)
{
	struct i2c_client *client = fusb->client;

	fusb->ovp_state = (fusb->cc_ovp ? FUSB251_CC_OVP_DETECTED : 0) +
		(fusb->sbu_ovp ? FUSB251_SBU_OVP_DETECTED : 0);

#ifdef CONFIG_LGE_PM_VENNER_PSY
	if (fusb->ovp_state >= FUSB251_CC_OVP_DETECTED) {
		struct power_supply* veneer =
			power_supply_get_by_name("veneer");

		if (veneer) {
			union power_supply_propval vccover =
					{ .intval = EXCEPTION_WIRED_VCCOVER, };
			power_supply_set_property(veneer,
					POWER_SUPPLY_PROP_CHARGE_NOW_ERROR,
					&vccover);
			power_supply_put(veneer);
		}
	}
#endif
}

static void fusb251_ovp_recovery(struct fusb251 *fusb)
{
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;
	int ret = 0;

	fusb->ovp_state = fusb->cc_ovp = fusb->sbu_ovp = 0;
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	struct fusb251 *fusb = container_of(nb, struct fusb251, psy_nb);
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;
	union power_supply_propval val;
	enum power_supply_typec_mode typec_mode;
	int ret;
	int vbus_present;

	if (ptr != fusb->usb_psy || evt != PSY_EVENT_PROP_CHANGED)
		return 0;

	ret = power_supply_get_property(fusb->usb_psy,
			POWER_SUPPLY_PROP_PRESENT, &val);
	if (ret) {
		dev_err(dev, "Unable to read USB PRESENT: %d\n", ret);
		return ret;
	}
	vbus_present = val.intval;

	ret = power_supply_get_property(fusb->usb_psy,
			POWER_SUPPLY_PROP_TYPEC_MODE, &val);

	if (ret < 0) {
		dev_err(dev, "Unable to read USB TYPEC_MODE: %d\n", ret);
		return ret;
	}
	typec_mode = val.intval;

	dev_info(dev, "typec:%d, vbus:%d\n",
			typec_mode, vbus_present);

	if (fusb->vbus_present != vbus_present) {
		fusb->vbus_present = vbus_present;
		if (fusb->vbus_present) {
			fusb->typec_connected = true;
			schedule_delayed_work(&fusb->enable_mos_work, 0);
			fusb251_set_state(fusb, FUSB251_STATE_TYPEC_CONNECTED);
		} else {
			fusb->typec_connected = false;
			if (!fusb->ds_connected) {
				schedule_delayed_work(&fusb->enable_mos_work, 0);
				fusb251_set_state(fusb, FUSB251_STATE_DRY);
			} else {
				fusb251_set_state(fusb, FUSB251_STATE_DS_CONNECTED);
			}
		}
	} else 	if (typec_mode != fusb->typec_mode) {
		switch (typec_mode) {
		case POWER_SUPPLY_TYPEC_SINK:
		case POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE:
		case POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY:
		case POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER:
		case POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY:
			fusb->typec_connected = true;
			schedule_delayed_work(&fusb->enable_mos_work, 0);
			fusb251_set_state(fusb, FUSB251_STATE_TYPEC_CONNECTED);
			break;
		case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
		case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
		case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
			if (fusb->vbus_present) {
				fusb->typec_connected = true;
				schedule_delayed_work(&fusb->enable_mos_work, 0);
				fusb251_set_state(fusb, FUSB251_STATE_TYPEC_CONNECTED);
			}
			break;
		case POWER_SUPPLY_TYPEC_NONE:
			fusb->typec_connected = false;
			if (!fusb->ds_connected) {
				schedule_delayed_work(&fusb->enable_mos_work, 0);
				fusb251_set_state(fusb, FUSB251_STATE_DRY);
			} else {
				fusb251_set_state(fusb, FUSB251_STATE_DS_CONNECTED);
			}
			break;
		default:
			break;
		}
		fusb->typec_mode = typec_mode;
	}

	return 0;
}

static void enable_moisture_work(struct work_struct *w)
{
	struct fusb251 *fusb = container_of(w, struct fusb251,
			enable_mos_work.work);
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;

	dev_info(dev, "%s: typec_connected=%d\n", __func__,
			fusb->typec_connected);

	if (!fusb->typec_connected) {
		if (fusb->state > FUSB251_STATE_CC_DETECTING)
			fusb251_set_state(fusb, FUSB251_STATE_DRY);

		fusb251_enable_moisture_detect(fusb, true);
	} else {
		fusb251_enable_moisture_detect(fusb, false);
	}
}

static void fusb251_dump_register(struct fusb251 *fusb)
{
	struct i2c_client *client = fusb->client;
	int control = 0;
	int irq_mask = 0;
	int status = 0;
	int switch_ctrl = 0;
	int sbu_thr = 0;
	int cc_thr = 0;
	int timer2 = 0;

	control = fusb251_read_reg(client, FUSB251_CONTROL);
	fusb251_print_reg(FUSB251_CONTROL, control);

	irq_mask = fusb251_read_reg(client, FUSB251_INTERRUPT_MASK);
	fusb251_print_reg(FUSB251_INTERRUPT_MASK, irq_mask);

	status = fusb251_read_reg(client, FUSB251_STATUS);
	fusb251_print_reg(FUSB251_STATUS, status);

	switch_ctrl = fusb251_read_reg(client, FUSB251_SWITCH_CONTROL);
	fusb251_print_reg(FUSB251_SWITCH_CONTROL, switch_ctrl);

	sbu_thr = fusb251_read_reg(client, FUSB251_THRESHOLD1);
	pr_info("%s: SBU_MOS_THR:0x%x\n", __func__,
			sbu_thr & (unsigned int)SBU_MOS_DET_THR_MASK);

	cc_thr = fusb251_read_reg(client, FUSB251_THRESHOLD1);
	pr_info("%s: CC_MOS_THR:0x%x\n",__func__,
			cc_thr & (unsigned int)CC_MOS_DET_THR_MASK);

	timer2 = fusb251_read_reg(client, FUSB251_TIMER2);
	pr_info("%s: TIMER2:0x%x\n",__func__, timer2);
}

static void check_reg_work(struct work_struct *w)
{
	struct fusb251 *fusb = container_of(w, struct fusb251,
			check_reg_work.work);

	fusb251_dump_register(fusb);
	schedule_delayed_work(&fusb->check_reg_work, 1000);
}

static void fusb251_set_state(struct fusb251 *fusb, int state)
{
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;

	dev_info(dev, "%s: %s -> %s\n",
			__func__, fusb251_state_strings[fusb->state],
			fusb251_state_strings[state]);

	if (disable_moisture)
		return;

	fusb->state = state;
	switch (state) {
	case FUSB251_STATE_DRY:
		kick_sm(fusb, 0);
		break;

	case FUSB251_STATE_CC_DETECTING:
		kick_sm(fusb, 0);
		break;

	case FUSB251_STATE_CC_FLOATING:
		kick_sm(fusb, cc_floating_time_ms);
		break;

	case FUSB251_STATE_SBU_FLOATING:
		kick_sm(fusb, cc_floating_time_ms);
		break;

	case FUSB251_STATE_CC_MOS_DETECTED:
		fusb->sbu_check_count = 0;
		kick_sm(fusb, 0);
		break;

	case FUSB251_STATE_CC_SBU_AUTO_DETECTING:
		kick_sm(fusb, 0);
		break;

	case FUSB251_STATE_SBU_MOS_DETECTED:
		kick_sm(fusb, 0);
		break;

	case FUSB251_STATE_TYPEC_CONNECTED:
		kick_sm(fusb, 0);
		break;
	case FUSB251_STATE_DS_CONNECTED:
		fusb->ds_connected = true;
		kick_sm(fusb, 0);
		break;

	case FUSB251_STATE_ERROR_RECOVERY:
		kick_sm(fusb, 0);
		break;

	default:
		dev_err(dev, "%s: No action for state %s\n", __func__,
				fusb251_state_strings[fusb->state]);
		break;
	}
}

static void fusb251_sm(struct work_struct *w)
{
	struct fusb251 *fusb = container_of(w, struct fusb251, sm_work);
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;

	union power_supply_propval val = { 0, };
	int status, ret = 0;
	int rerun_sm_ms = 0;

	alarm_cancel(&fusb->timer);

	if (disable_moisture)
		goto done;

	dev_info(dev, "%s: %s\n", __func__,
			fusb251_state_strings[fusb->state]);

	switch (fusb->state) {
	case FUSB251_STATE_DRY:
		val.intval = 0;
		ret = power_supply_set_property(fusb->usb_psy,
				POWER_SUPPLY_PROP_MOISTURE_DETECTED, &val);
		if (ret < 0) {
			dev_err(dev, "Couldn't set PROP_MOISTURE_DETECTED,"
					"ret=%d\n", ret);
		}
#ifdef CONFIG_LGE_USB_SBU_SWITCH
		lge_sbu_switch_put(fusb->lge_sbu_switch_inst,
				LGE_SBU_SWITCH_FLAG_SBU_MD);
#endif

		val.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		power_supply_set_property(fusb->usb_psy,
				POWER_SUPPLY_PROP_TYPEC_POWER_ROLE,
				&val);
		if (ret < 0) {
			dev_err(dev, "Couldn't set PROP_TYPEC_POWER_ROLE,"
					"ret=%d\n", ret);
		}

		fusb->cc_mos_detected = false;
		fusb->sbu_mos_detected = false;
		fusb->sbu_fault = false;
		fusb->ds_connected = false;

		power_supply_changed(fusb->usb_psy);
		if (!fusb->typec_connected) {
			fusb251_set_state(fusb, FUSB251_STATE_CC_DETECTING);
		}


		break;

	case FUSB251_STATE_CC_DETECTING:
		fusb251_enable_moisture_detect(fusb, true);
		status = fusb251_read_reg(client, FUSB251_STATUS);
		if (!(status & LOOK4CC_BIT))
			fusb251_set_state(fusb, FUSB251_STATE_ERROR_RECOVERY);
		break;

	case FUSB251_STATE_CC_FLOATING:
	case FUSB251_STATE_SBU_FLOATING:
		if (!fusb->typec_connected) {
			fusb->cc_mos_detected = true;
			val.intval = POWER_SUPPLY_TYPEC_PR_SINK;
			power_supply_set_property(fusb->usb_psy,
					POWER_SUPPLY_PROP_TYPEC_POWER_ROLE, &val);
			if (ret < 0) {
				dev_err(dev, "Couldn't set PROP_TYPEC_POWER_ROLE,"
						"ret=%d\n", ret);
			}
			fusb251_set_state(fusb, FUSB251_STATE_CC_SBU_AUTO_DETECTING);
		}
		break;

	case FUSB251_STATE_CC_SBU_AUTO_DETECTING:
		if (fusb->auto_moisture_enable)
			fusb251_set_state(fusb, FUSB251_STATE_DRY);

		val.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		power_supply_set_property(fusb->usb_psy,
				POWER_SUPPLY_PROP_TYPEC_POWER_ROLE, &val);
		if (ret < 0) {
			dev_err(dev, "Couldn't set PROP_TYPEC_POWER_ROLE,"
					"ret=%d\n", ret);
		}
		fusb251_run_sbu_detect(fusb, true);
		fusb->sbu_fault = false;
		kick_sm(fusb, recheck_dry_time_ms);
		break;

	case FUSB251_STATE_CC_MOS_DETECTED:
		rerun_sm_ms = fix_cc_sink_time_ms;

		if (fusb->sbu_fault || fusb->sbu_check_count > 0) {
			fusb251_enable_moisture_detect(fusb, false);
			fusb->state = FUSB251_STATE_DRY;
			if (fusb->sbu_check_count > 0)
				rerun_sm_ms = 0;
		}

		status = fusb251_read_reg(client, FUSB251_STATUS);
		if (status & LOOK4SBU_BIT)
			fusb->sbu_check_count++;

		fusb->cc_mos_detected = true;
		val.intval = POWER_SUPPLY_TYPEC_PR_SINK;
		power_supply_set_property(fusb->usb_psy,
				POWER_SUPPLY_PROP_TYPEC_POWER_ROLE, &val);
		if (ret < 0) {
			dev_err(dev, "Couldn't set PROP_TYPEC_POWER_ROLE,"
					"ret=%d\n", ret);
		}
		kick_sm(fusb, rerun_sm_ms);
		break;

	case FUSB251_STATE_SBU_MOS_DETECTED:
		status = fusb251_read_reg(client, FUSB251_STATUS);
		if (!(status & LOOK4DRY_BIT)) {
			fusb251_set_state(fusb, FUSB251_STATE_ERROR_RECOVERY);
			break;
		}

#ifdef CONFIG_LGE_USB_SBU_SWITCH
		lge_sbu_switch_get(fusb->lge_sbu_switch_inst,
				LGE_SBU_SWITCH_FLAG_SBU_MD);
#endif

		val.intval = 1;
		ret = power_supply_set_property(fusb->usb_psy,
				POWER_SUPPLY_PROP_MOISTURE_DETECTED, &val);
		if (ret < 0) {
			dev_err(dev, "Couldn't set PROP_MOISTURE_DETECTED,"
					"ret =%d", ret);
		}

		val.intval = POWER_SUPPLY_TYPEC_PR_SINK;
		power_supply_set_property(fusb->usb_psy,
				POWER_SUPPLY_PROP_TYPEC_POWER_ROLE, &val);
		if (ret < 0) {
			dev_err(dev, "Couldn't set PROP_TYPEC_POWER_ROLE,"
					"ret=%d\n", ret);
		}

		fusb->sbu_mos_detected = true;
		fusb->sbu_fault = false;
		power_supply_changed(fusb->usb_psy);
		kick_sm(fusb, recheck_dry_time_ms);
		break;

	case FUSB251_STATE_TYPEC_CONNECTED:
		dev_info(dev, "Type-C Connected\n");
		fusb251_enable_moisture_detect(fusb, false);
		break;

	case FUSB251_STATE_DS_CONNECTED:
		dev_info(dev, "Dual Screen Connected\n");
		fusb->cc_mos_detected = false;
		fusb->sbu_mos_detected = false;
		break;

	case FUSB251_STATE_ERROR_RECOVERY:
		fusb251_reset(fusb);
		break;

	default:
		dev_err(dev, "%s:Unhandled state %s\n", __func__,
				fusb251_state_strings[fusb->state]);
		break;
	}

done:
	pm_relax(dev);
}

static void i2c_work(struct work_struct *w)
{
	struct fusb251 *fusb = container_of(w, struct fusb251, i2c_work);
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;
	int irq_state = 0;
	int status = 0;
	int mos_status = 0;

	// Read Register and Process

	mutex_lock(&fusb->irq_lock);

	if (!fusb->initialized) {
		dev_err(dev, "FUSB251 not initialized\n");
		fusb251_init(fusb);
		goto done;
	}

	irq_state = fusb251_read_reg(client, FUSB251_INTERRUPT);
	if(irq_state < 0) {
		dev_err(dev, "Register Read Failed\n");
		goto done;
	}

	status = fusb251_read_reg(client, FUSB251_STATUS);
	if(status < 0) {
		dev_err(dev, "Register Read Failed\n");
		goto done;
	}

	mos_status = fusb251_read_reg(client, FUSB251_MOISTURE_STATUS);
	if(mos_status < 0) {
		dev_err(dev, "Register Read Failed\n");
		goto done;
	}

	fusb251_print_reg(FUSB251_INTERRUPT, irq_state);
	fusb251_print_reg(FUSB251_MOISTURE_STATUS, mos_status);
	fusb251_print_reg(FUSB251_STATUS, status);

	cancel_delayed_work(&fusb->enable_mos_work);

	if ((irq_state & CC1_TIMER_INT_BIT) ||
			(irq_state & CC2_TIMER_INT_BIT)) {
		dev_dbg(dev, "Timer Expire irq occured\n");
		if ((mos_status & SBU1_FT_BIT) ||
				(mos_status & SBU2_FT_BIT)) {
			if (!fusb->typec_connected &&
					!fusb->sbu_mos_detected) {
				if (fusb->state <= FUSB251_STATE_CC_DETECTING)
					fusb251_set_state(fusb, FUSB251_STATE_SBU_FLOATING);
			}
		} else {
			if (fusb->typec_connected || fusb->vbus_present) {
				fusb251_set_state(fusb, FUSB251_STATE_TYPEC_CONNECTED);
			} else {
				if (fusb->state <= FUSB251_STATE_CC_DETECTING)
					fusb251_set_state(fusb, FUSB251_STATE_CC_FLOATING);
				else
					fusb251_set_state(fusb, FUSB251_STATE_CC_MOS_DETECTED);
			}
		}
	}

	if (irq_state & MOS_CHG_INT_BIT) {
		dev_dbg(dev, "Moisture Changed irq occured\n");
		if ((mos_status & CC1_MOS_BIT) ||
				(mos_status & CC2_MOS_BIT)) {
			if ((mos_status & SBU1_MOS_BIT) ||
					(mos_status & SBU2_MOS_BIT)) {
				fusb251_set_state(fusb,
						FUSB251_STATE_SBU_MOS_DETECTED);
			} else {
				if (mos_status & FAULT_MASK) {
					dev_dbg(dev, "Moisture was detected "
						"on CC, but not found on SBU\n");
					fusb->sbu_fault = true;
				}
				if (fusb->state <= FUSB251_STATE_CC_DETECTING)
					fusb251_set_state(fusb, FUSB251_STATE_CC_FLOATING);
				else
					fusb251_set_state(fusb, FUSB251_STATE_CC_MOS_DETECTED);
			}
		} else if ((mos_status & SBU1_MOS_BIT) ||
				(mos_status & SBU2_MOS_BIT)) {
			if (fusb->state >= FUSB251_STATE_CC_SBU_AUTO_DETECTING) {
				fusb251_set_state(fusb,
						FUSB251_STATE_SBU_MOS_DETECTED);
			} else {
				dev_dbg(dev, "%s: SBU moisture only?\n",
						__func__);
			}
		} else if (mos_status & FAULT_MASK) {
			if ((mos_status & FAULT_MASK) == CC_MOS_BUT_NOT_SBU)
				fusb251_set_state(fusb, FUSB251_STATE_CC_MOS_DETECTED);
			else if ((mos_status & FAULT_MASK) == SBU_FLOAT)
				fusb251_set_state(fusb, FUSB251_STATE_SBU_MOS_DETECTED);
		} else if ((mos_status & SBU1_FT_BIT) ||
				(mos_status & SBU2_FT_BIT)) {
			if (fusb->state <= FUSB251_STATE_CC_DETECTING)
				fusb251_set_state(fusb, FUSB251_STATE_SBU_FLOATING);
		} else {
			dev_dbg(dev, "%s: MOS IRQ occured but no mos bit set\n", __func__);
			if (fusb->typec_connected)
				fusb251_set_state(fusb, FUSB251_STATE_TYPEC_CONNECTED);
		}
	}

	if (irq_state & DRY_CHG_INT_BIT) {
		if (fusb->vbus_present) {
			dev_dbg(dev, "Dry irq caused by VBUS\n");

		} else {
			dev_dbg(dev, "Dry Changed irq occured\n");
			fusb->cc_mos_detected = false;
			fusb->sbu_mos_detected = false;
			fusb251_set_state(fusb, FUSB251_STATE_DRY);
		}
	}

	if (irq_state & OVP_INT_BIT) {
		dev_dbg(dev, "OVP irq occured\n");
		if (status & OVP_CC_BIT) {
			dev_info(dev, "CC OVP occured\n");
			fusb->cc_ovp = true;
		} else if ((status & OVP_CC_BIT) == 0) {
			fusb->cc_ovp = false;
		}

		if (status & OVP_SBU_BIT) {
			dev_dbg(dev, "SBU OVP occured\n");
			fusb->sbu_ovp = true;
		} else if ((status & OVP_SBU_BIT) == 0) {
			fusb->sbu_ovp = false;
		}
		fusb251_ovp(fusb);
	}
	if (irq_state & OVP_REC_INT_BIT) {
		dev_dbg(dev, "OVP recovery irq occured\n");
		fusb251_ovp_recovery(fusb);
	}

done:
	mutex_unlock(&fusb->irq_lock);
	pm_relax(dev);
}

static void fusb251_init(struct fusb251 *fusb)
{
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;
	int ret = 0;

	fusb->initialized = false;

	ret = fusb251_read_reg(client, FUSB251_PRODUCT_ID);

	fusb->rev = ret & REVISION_ID_MASK;
	dev_info(dev, "%s: DEV_ID: 0x%x, PID: FUSB251UCX, REV_ID: %d\n",
			__func__, ret & DEVICE_ID_MASK, fusb->rev);


	ret = fusb251_write_reg(client, FUSB251_RESET, RESET_BIT);
	if (ret < 0) {
		dev_err(dev, "%s: I2C Failed(%d)\n", __func__, ret);
		return;
	}

	mdelay(10);

	/* Should reset the Moisture Detection State Machine after booting? */
	ret = fusb251_write_reg(client, FUSB251_RESET, MOS_RESET_BIT);
	if (ret < 0) {
		dev_err(dev, "%s: I2C Failed(%d)\n", __func__, ret);
		return;
	}

	/* Temporary Disable every interrupts for init FUSB251 */
	ret = fusb251_write_reg(client, FUSB251_INTERRUPT_MASK,
			MASK_OVP_BIT | MASK_OVP_REC_BIT | MASK_MOS_DETECT_BIT |
			MASK_DRY_DETECT_BIT | MASK_CC1_TIMER_BIT |
			MASK_CC2_TIMER_BIT);
	if (ret < 0) {
		dev_err(dev, "%s: I2C Failed(%d)\n", __func__, ret);
		return;
	}

	/* Setting CC Settle Time and Number of ADC Read
	 * Initial Setting is 400 usec and 3 times */
	ret = fusb251_write_reg(client, FUSB251_TIMER2,
			(CC_SETTLE_TIME_MASK & CC_SETTLE_TIME_600US) |
			(NUM_ADC_READ_MASK & ADC_READ_3_TIMES));
	if (ret < 0) {
		dev_err(dev, "%s: I2C Failed(%d)\n", __func__, ret);
		return;
	}

	/* Setting CC & SBU Moisture Detection Threshold
	 * Initial Setting is 747 kohm and 320kohm */
	ret = fusb251_write_reg(client, FUSB251_THRESHOLD1,
			(CC_MOS_DET_THR_MASK & CC_MOS_THR_747K) |
			(SBU_MOS_DET_THR_MASK & SBU_MOS_THR_480K));
	if (ret < 0) {
		dev_err(dev, "%s: I2C Failed(%d)\n", __func__, ret);
		return;
	}

	/* Setting Dry Check Threshold & SBU Floating Detection Threshold
	 * Initial Setting is 747 kohm and 200mv */
	ret = fusb251_write_reg(client, FUSB251_THRESHOLD2,
			(VDRY_THR_MASK & VDRY_THR_960K) |
			(SBU_FLOAT_DET_THR_MASK & SBU_FT_DET_THR_300MV));
	if (ret < 0) {
		dev_err(dev, "%s: I2C Failed(%d)\n", __func__, ret);
		return;
	}

	/* Setting Dry Check Timer
	 * Initial Setting is 10sec */
	ret = fusb251_write_reg(client, FUSB251_TIMER, TDRY_MASK & TDRY_TIME_10S);
	if (ret < 0) {
		dev_err(dev, "%s: I2C Failed(%d)\n", __func__, ret);
		return;
	}

	/* Setting CC and SBU Switch Control
	 * Initial Setting is close all CC and SBU pins */
	ret = fusb251_write_reg(client, FUSB251_SWITCH_CONTROL,
			CC_SW_CTRL_BIT | (SBU_SW_CTRL_MASK & CLOSE_SBU1_SBU2));
	if (ret < 0) {
		dev_err(dev, "%s: I2C Failed(%d)\n", __func__, ret);
		return;
	}

	/* Setting Interrupt Mask to allow every interrupts */
	ret = fusb251_write_reg(client, FUSB251_INTERRUPT_MASK,
			MASK_CC1_TIMER_BIT | MASK_CC2_TIMER_BIT);
	if (ret < 0) {
		dev_err(dev, "%s: I2C Failed(%d)\n", __func__, ret);
		return;
	}

	/* Setting CC and SBU  Moisture Detection. */
	ret = fusb251_write_reg(client, FUSB251_CONTROL,
			EN_MAN_SW_CTRL_BIT | EN_CC_MOS_BIT |
			/*EN_AUTO_SBU_BIT | */EN_DRY_BIT | EN_DEBUG_ACC_DET_BIT);
	if (ret < 0) {
		dev_err(dev, "%s: I2C Failed(%d)\n", __func__, ret);
		return;
	}

	fusb->initialized = true;
}

static void fusb251_reset(struct fusb251 *fusb) {
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;


	fusb251_init(fusb);
	fusb251_enable_moisture_detect(fusb, true);

	fusb251_set_state(fusb, FUSB251_STATE_DRY);

	fusb->cc_mos_detected = false;
	fusb->sbu_mos_detected = false;

	dev_dbg(dev, "Reset and re-init FUSB251\n");
}

static irqreturn_t fusb251_intb_irq(int irq, void *data)
{
	struct fusb251 *fusb = (struct fusb251 *)data;
	struct device *dev = &fusb->client->dev;
	int intb_value = 0;

	if (!fusb) {
		pr_err("%s: called before init\n", __func__);
		return IRQ_HANDLED;
	}

	if (atomic_read(&fusb->pm_suspended)) {
		dev_info(dev, "%s: device is suspended\n", __func__);
		return IRQ_HANDLED;
	}

	intb_value = gpiod_get_value_cansleep(fusb->intb_desc);

	if (!intb_value) {
		pm_stay_awake(dev);
		dev_info(dev, "INTB irq occured\n");
		if (!queue_work(fusb->wq, &fusb->i2c_work))
			dev_err(dev, "%s: can't alloc work\n", __func__);
	}

	return IRQ_HANDLED;
}

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct fusb251 *fusb;
	int reset = 0;

	if (!__client) {
		return size;
	}

	fusb = i2c_get_clientdata(__client);
	sscanf(buf, "%d", &reset);

	if (reset) {
		fusb251_reset(fusb);
	}

	return size;
}
static DEVICE_ATTR(reset, S_IWUSR | S_IWGRP, NULL, reset_store);

static ssize_t mos_reset_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct fusb251 *fusb;
	int reset = 0;

	if (!__client) {
		return size;
	}

	fusb = i2c_get_clientdata(__client);

	sscanf(buf, "%d", &reset);

	if (reset) {
		fusb251_reset(fusb);
	}

	return size;
}
static DEVICE_ATTR(mos_reset, S_IWUSR | S_IWGRP, NULL, mos_reset_store);

static ssize_t control_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int control = 0;

	control = fusb251_read_reg(__client, FUSB251_CONTROL);

	return sprintf(buf, "MAN_SW:%d, CC_MOS:%d, SBU_MOS:%d, SBUFT:%d,"
			"AUTO_SBU:%d, EN_DRY:%d, DEBUG_ACC:%d\n",
			(control & EN_MAN_SW_CTRL_BIT) ? 1 : 0,
			(control & EN_CC_MOS_BIT) ? 1 : 0,
			(control & EN_SBU_MOS_BIT) ? 1 : 0,
			(control & EN_SBUFT_BIT) ? 1 : 0,
			(control & EN_AUTO_SBU_BIT) ? 1 : 0,
			(control & EN_DRY_BIT) ? 1 : 0,
			(control & EN_DEBUG_ACC_DET_BIT) ? 1 : 0);
}

static ssize_t control_reg_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int control = 0;

	sscanf(buf, "%d", &control);

	dev_dbg(dev, "Write Control Reg:0x%x\n", control);

	fusb251_write_reg(__client, FUSB251_CONTROL, control);

	return size;
}
static DEVICE_ATTR(control_reg, S_IRUGO | S_IWUSR | S_IWGRP,
		control_reg_show, control_reg_store);

static ssize_t intb_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fusb251 *fusb;
	int intb = 0;

	if (!__client) {
		return 0;
	}

	fusb = i2c_get_clientdata(__client);

	intb = gpiod_get_value(fusb->intb_desc);

	return sprintf(buf, "intb:%d\n", intb);
}
static DEVICE_ATTR(intb_gpio, S_IRUGO , intb_gpio_show, NULL);

static ssize_t switch_control_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int switch_control = 0;

	switch_control = fusb251_read_reg(__client, FUSB251_SWITCH_CONTROL);

	return sprintf(buf, "SWITCH_CONTROL:0x%x\n",
			switch_control);
}

static ssize_t switch_control_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int switch_control = 0;

	sscanf(buf, "%d", &switch_control);

	dev_dbg(dev, "Write SWITCH_CONTROL Reg:0x%x\n",
			switch_control);

	fusb251_write_reg(__client, FUSB251_SWITCH_CONTROL,
			switch_control);
	return size;
}
static DEVICE_ATTR(switch_control, S_IRUGO | S_IWUSR | S_IWGRP,
		switch_control_show, switch_control_store);

static ssize_t irq_mask_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int irq_mask = 0;

	irq_mask = fusb251_read_reg(__client, FUSB251_INTERRUPT_MASK);

	return sprintf(buf, "OVP:%d, OVP_REC:%d, MOS_DET:%d, DRY_DET:%d,"
			"CC1_TIMER:%d, CC2_TIMER:%d\n",
			(irq_mask & MASK_OVP_BIT) ? 1 : 0,
			(irq_mask & MASK_OVP_REC_BIT) ? 1 : 0,
			(irq_mask & MASK_MOS_DETECT_BIT) ? 1 : 0,
			(irq_mask & MASK_DRY_DETECT_BIT) ? 1 : 0,
			(irq_mask & MASK_CC1_TIMER_BIT) ? 1 : 0,
			(irq_mask & MASK_CC2_TIMER_BIT) ? 1 : 0);
}

static ssize_t irq_mask_reg_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int irq_mask = 0;

	sscanf(buf, "%d", &irq_mask);

	dev_dbg(dev, "Write IRQ_MASK Reg:0x%x\n", irq_mask);

	fusb251_write_reg(__client, FUSB251_INTERRUPT_MASK, irq_mask);

	return size;
}
static DEVICE_ATTR(irq_mask_reg, S_IRUGO | S_IWUSR | S_IWGRP,
		irq_mask_reg_show, irq_mask_reg_store);


static ssize_t status_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int status = 0;

	status = fusb251_read_reg(__client, FUSB251_STATUS);

	return sprintf(buf, "OVP_CC:%d, OVP_SBU:%d, LOOK4DRY:%d, LOOK4SBU:%d,"
			"LOOK4CC:%d\n",
			(status & OVP_CC_BIT) ? 1 : 0,
			(status & OVP_SBU_BIT) ? 1 : 0,
			(status & LOOK4DRY_BIT) ? 1 : 0,
			(status & LOOK4SBU_BIT) ? 1 : 0,
			(status & LOOK4CC_BIT) ? 1 : 0);
}
static DEVICE_ATTR(status_reg, S_IRUGO, status_reg_show, NULL);

static ssize_t moi_status_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int status = 0;

	status = fusb251_read_reg(__client, FUSB251_MOISTURE_STATUS);

	return sprintf(buf, "CC1_MOS:%d, CC2_MOS:%d, SBU1_MOS:%d, SBU2_MOS:%d,"
			"SBU1_FT:%d, SBU2_FT:%d\n",
			(status & CC1_MOS_BIT) ? 1 : 0,
			(status & CC2_MOS_BIT) ? 1 : 0,
			(status & SBU1_MOS_BIT) ? 1 : 0,
			(status & SBU2_MOS_BIT) ? 1 : 0,
			(status & SBU1_FT_BIT) ? 1 : 0,
			(status & SBU2_FT_BIT) ? 1 : 0);
}
static DEVICE_ATTR(moi_status_reg, S_IRUGO, moi_status_reg_show, NULL);

static ssize_t sbu_mos_thr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int sbu_thr = 0;

	sbu_thr = fusb251_read_reg(__client, FUSB251_THRESHOLD1);

	return sprintf(buf, "SBU_MOS_THR:0x%x\n",
			sbu_thr & (unsigned int)SBU_MOS_DET_THR_MASK);
}

static ssize_t sbu_mos_thr_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int sbu_thr = 0;

	sscanf(buf, "%d", &sbu_thr);

	dev_dbg(dev, "Write SBU_MOS_THR Reg:0x%x\n",
			sbu_thr & (unsigned int)SBU_MOS_DET_THR_MASK);

	fusb251_write_reg(__client, FUSB251_THRESHOLD1,
			sbu_thr & SBU_MOS_DET_THR_MASK);
	return size;
}
static DEVICE_ATTR(sbu_mos_thr, S_IRUGO | S_IWUSR | S_IWGRP,
		sbu_mos_thr_show, sbu_mos_thr_store);

static ssize_t cc_mos_thr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int cc_thr = 0;

	cc_thr = fusb251_read_reg(__client, FUSB251_THRESHOLD1);

	return sprintf(buf, "CC_MOS_THR:0x%x\n",
			cc_thr & (unsigned int)CC_MOS_DET_THR_MASK);
}

static ssize_t cc_mos_thr_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int cc_thr = 0;

	sscanf(buf, "%d", &cc_thr);

	dev_dbg(dev, "Write CC_MOS_THR Reg:0x%x\n",
			cc_thr & (unsigned int)CC_MOS_DET_THR_MASK);

	fusb251_write_reg(__client, FUSB251_THRESHOLD1,
			cc_thr & CC_MOS_DET_THR_MASK);
	return size;
}
static DEVICE_ATTR(cc_mos_thr, S_IRUGO | S_IWUSR | S_IWGRP,
		cc_mos_thr_show, cc_mos_thr_store);

static ssize_t thr2_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int thr = 0;

	thr = fusb251_read_reg(__client, FUSB251_THRESHOLD2);

	return sprintf(buf, "THR2:0x%x\n",
			thr);
}

static ssize_t thr2_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int thr = 0;

	sscanf(buf, "%d", &thr);

	dev_dbg(dev, "Write THR2 Reg:0x%x\n",
			thr);

	fusb251_write_reg(__client, FUSB251_THRESHOLD2,
			thr);
	return size;
}
static DEVICE_ATTR(thr2, S_IRUGO | S_IWUSR | S_IWGRP,
		thr2_show, thr2_store);

static void fusb251_sysfs_init(struct device *dev, struct fusb251 *fusb)
{
	int ret = 0;

	ret = device_create_file(dev, &dev_attr_reset);
	if (ret) {
		device_remove_file(dev, &dev_attr_reset);
		return;
	}

	ret = device_create_file(dev, &dev_attr_mos_reset);
	if (ret) {
		device_remove_file(dev, &dev_attr_mos_reset);
		return;
	}

	ret = device_create_file(dev, &dev_attr_control_reg);
	if (ret) {
		device_remove_file(dev, &dev_attr_control_reg);
		return;
	}

	ret = device_create_file(dev, &dev_attr_irq_mask_reg);
	if (ret) {
		device_remove_file(dev, &dev_attr_irq_mask_reg);
		return;
	}

	ret = device_create_file(dev, &dev_attr_status_reg);
	if (ret) {
		device_remove_file(dev, &dev_attr_status_reg);
		return;
	}

	ret = device_create_file(dev, &dev_attr_moi_status_reg);
	if (ret) {
		device_remove_file(dev, &dev_attr_moi_status_reg);
		return;
	}

	ret = device_create_file(dev, &dev_attr_sbu_mos_thr);
	if (ret) {
		device_remove_file(dev, &dev_attr_sbu_mos_thr);
		return;
	}

	ret = device_create_file(dev, &dev_attr_cc_mos_thr);
	if (ret) {
		device_remove_file(dev, &dev_attr_cc_mos_thr);
		return;
	}

	ret = device_create_file(dev, &dev_attr_thr2);
	if (ret) {
		device_remove_file(dev, &dev_attr_thr2);
		return;
	}

	ret = device_create_file(dev, &dev_attr_switch_control);
	if (ret) {
		device_remove_file(dev, &dev_attr_switch_control);
		return;
	}

	ret = device_create_file(dev, &dev_attr_intb_gpio);
	if (ret) {
		device_remove_file(dev, &dev_attr_intb_gpio);
		return;
	}
}

static void fusb251_sysfs_deinit(struct device *dev)
{
	device_remove_file(dev, &dev_attr_reset);
	device_remove_file(dev, &dev_attr_control_reg);
	device_remove_file(dev, &dev_attr_irq_mask_reg);
	device_remove_file(dev, &dev_attr_status_reg);
	device_remove_file(dev, &dev_attr_moi_status_reg);
	device_remove_file(dev, &dev_attr_sbu_mos_thr);
	device_remove_file(dev, &dev_attr_cc_mos_thr);
	device_remove_file(dev, &dev_attr_thr2);
	device_remove_file(dev, &dev_attr_switch_control);
	device_remove_file(dev, &dev_attr_intb_gpio);
}


static int fusb251_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct fusb251 *fusb;
	struct pinctrl* gpio_pinctrl;
	struct pinctrl_state* gpio_state;
	bool is_factory_boot = false;

	int ret = 0;

	pr_debug("%s\n", __func__);

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "smbus data not supported\n");
		return -EIO;
	}

	fusb = devm_kzalloc(&client->dev,
			sizeof(struct fusb251), GFP_KERNEL);
	if (!fusb) {
		dev_err(dev, "can't alloc fusb251\n");
		ret = -ENOMEM;
		goto free_client;
	}

	fusb->intb_pu18 = devm_regulator_get(&client->dev, "intb_pu18");
	if (IS_ERR(fusb->intb_pu18)) {
		dev_err(dev, "regulator intb_pu18 get failed\n");
		return -EPROBE_DEFER;
	}

	if (regulator_count_voltages(fusb->intb_pu18) > 0) {
		ret = regulator_set_voltage(fusb->intb_pu18, 1800000, 1800000);
		if (ret) {
			dev_err(dev, "regulator set failed intb_pu18\n");
			return ret;
		}
	}

	fusb->intb_desc = devm_gpiod_get(dev, "fusb251,intb", GPIOD_IN);
	if (IS_ERR(fusb->intb_desc)) {
		dev_err(dev, "couldn't get intb gpio_desc\n");
		fusb->intb_desc = NULL;
		goto skip_int;
	}
	gpiod_direction_input(fusb->intb_desc);
	fusb->intb_irq = gpiod_to_irq(fusb->intb_desc);

	ret = devm_request_threaded_irq(&client->dev, fusb->intb_irq,
			NULL, fusb251_intb_irq,
			IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
			"fusb251-int", fusb);
	if(ret) {
		dev_err(dev, "Cannot request intb irq\n");
		goto skip_int;
	}
	enable_irq_wake(fusb->intb_irq);

	gpio_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(gpio_pinctrl)) {
		dev_err(dev, "Failed to get pinctrl (%ld)\n",
				PTR_ERR(gpio_pinctrl));
		ret = PTR_ERR(gpio_pinctrl);
		goto free_dev;
	}

	gpio_state = pinctrl_lookup_state(gpio_pinctrl, "default");
	if (IS_ERR_OR_NULL(gpio_state)) {
		dev_err(dev, "pinstate not found, %ld\n", PTR_ERR(gpio_state));
		ret = PTR_ERR(gpio_state);
		goto free_dev;
	}

	ret = pinctrl_select_state(gpio_pinctrl, gpio_state);
	if (ret < 0) {
		dev_err(dev, "cannot set pins %d\n", ret);
		goto free_dev;
	}

skip_int:
	fusb->client = client;
	i2c_set_clientdata(client, fusb);

	fusb->wq = alloc_ordered_workqueue("fusb_wq", WQ_HIGHPRI);
	if (!fusb->wq) {
		ret = -ENOMEM;
		goto free_dev;
	};

	alarm_init(&fusb->timer, ALARM_BOOTTIME, fusb251_timeout);
	INIT_WORK(&fusb->i2c_work, i2c_work);
	INIT_WORK(&fusb->sm_work, fusb251_sm);
	INIT_DELAYED_WORK(&fusb->enable_mos_work, enable_moisture_work);
	INIT_DELAYED_WORK(&fusb->check_reg_work, check_reg_work);

	ret = device_init_wakeup(dev, true);
	if (ret)
		goto destroy_wq;

	fusb->usb_psy = power_supply_get_by_name("usb");
	if (!fusb->usb_psy) {
		dev_err(dev, "Could not get USB power_supply,"
				" deferring probe\n");
		ret = -EPROBE_DEFER;
		goto destroy_wq;
	}

	fusb->psy_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&fusb->psy_nb);
	if (ret < 0)
		return ret;

	mutex_init(&fusb->irq_lock);
	/* is mutex in i2c comm needed? */
	mutex_init(&fusb->i2c_lock);

	fusb251_init(fusb);
	fusb251_sysfs_init(dev, fusb);

	if (lge_get_factory_boot() || lge_get_laf_mode()) {
		dev_err(dev, "factory cable connected,"
				"disable moisture detection\n");
		is_factory_boot = true;
	} else {
		is_factory_boot = false;
	}

#ifdef CONFIG_LGE_USB_SBU_SWITCH
        fusb->lge_sbu_switch_desc.flags = LGE_SBU_SWITCH_FLAG_SBU_MD;
        fusb->lge_sbu_switch_inst = devm_lge_sbu_switch_instance_register(dev,
				&fusb->lge_sbu_switch_desc);
        if (!fusb->lge_sbu_switch_inst) {
                dev_dbg(dev, "Could not get lge_sbu_switch\n");
        }
#endif

	fusb251_update_state(0,0);
	fusb->moisture_enable = !is_factory_boot;
	if (is_factory_boot) {
		disable_moisture = true;
		fusb251_enable_moisture_detect(fusb, false);
	}

	if (!!gpiod_get_value_cansleep(fusb->intb_desc)) {
		pm_stay_awake(dev);
		queue_work(fusb->wq, &fusb->i2c_work);
	}

	__client = client;

	return 0;

destroy_wq:
	destroy_workqueue(fusb->wq);
free_dev:
	devm_kfree(dev, fusb);
free_client:
	i2c_set_clientdata(client, NULL);
	return ret;
}

static int fusb251_remove(struct i2c_client *client)
{
	struct fusb251 *fusb = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	if (!fusb) {
		return -ENODEV;
	}

	alarm_cancel(&fusb->timer);
	power_supply_unreg_notifier(&fusb->psy_nb);
	fusb251_sysfs_deinit(dev);

	if (fusb->intb_irq > 0)
		devm_free_irq(dev, fusb->intb_irq, fusb);

	cancel_delayed_work(&fusb->enable_mos_work);
	cancel_delayed_work(&fusb->check_reg_work);

	if (fusb->wq)
		destroy_workqueue(fusb->wq);
	mutex_destroy(&fusb->irq_lock);
	mutex_destroy(&fusb->i2c_lock);

	i2c_set_clientdata(client, NULL);
	i2c_unregister_device(client);
	devm_kfree(dev, fusb);

	return 0;
}

#ifdef CONFIG_PM
static int fusb251_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb = i2c_get_clientdata(client);
	dev_dbg(dev, "%s\n", __func__);

	atomic_set(&fusb->pm_suspended, 1);
	return 0;
}

static int fusb251_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb = i2c_get_clientdata(client);
	int status;
	int intb_value;

	if (disable_moisture)
		return 0;

	atomic_set(&fusb->pm_suspended, 0);
	dev_dbg(dev, "%s\n", __func__);
	intb_value = gpiod_get_value_cansleep(fusb->intb_desc);

	if (!intb_value) {
		pm_stay_awake(dev);
		dev_info(dev, "INTB irq occured\n");
		if (!queue_work(fusb->wq, &fusb->i2c_work))
			dev_err(dev, "%s: can't alloc work\n", __func__);
	}

	return 0;
}

static const struct dev_pm_ops fusb251_dev_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(fusb251_suspend, fusb251_resume)
};
#endif

static const struct i2c_device_id fusb251_id_table[] = {
	{"fusb251", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, fusb251_id_table);

#ifdef CONFIG_OF
static struct of_device_id fusb251_match_table[] = {
	{ .compatible = "lge,fusb251",},
	{},
};
MODULE_DEVICE_TABLE(of, fusb251_match_table);
#else
#define fusb251_match_table NULL
#endif

static struct i2c_driver fusb251_i2c_driver = {
	.driver = {
		.name = "lge,fusb251",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(fusb251_match_table),
#ifdef CONFIG_PM
		.pm = &fusb251_dev_pm_ops,
#endif
	},
	.probe = fusb251_probe,
	.remove = fusb251_remove,
	.id_table = fusb251_id_table,
};

static __init int fusb251_i2c_init(void)
{
	return i2c_add_driver(&fusb251_i2c_driver);
}

static __exit void fusb251_i2c_exit(void)
{
	i2c_del_driver(&fusb251_i2c_driver);
}

module_init(fusb251_i2c_init);
module_exit(fusb251_i2c_exit);

MODULE_DESCRIPTION("LGE FUSB251 Moisture Detection Driver");
MODULE_AUTHOR("Jayci Choi <jayci.choi@lge.com");
MODULE_LICENSE("GPL V2");

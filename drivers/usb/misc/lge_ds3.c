/*
 * LGE USB DS3 driver
 *
 * Copyright (C) 2019 LG Electronics, Inc.
 * Author: Hansun Lee <hansun.lee@lge.com>
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
#define DEBUG
//#define VERBOSE_DEBUG

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/extcon.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/usb.h>
#include <linux/usb/usbpd.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>

#include <linux/lge_ds3.h>
#include <linux/hall_ic.h>
//#include <linux/extcon-provider.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_LGE_USB_SBU_SWITCH
#include <linux/usb/lge_sbu_switch.h>
#endif

#include "usbpd.h"
#include "../../gpu/drm/msm/lge/dp/lge_dp_def.h"
extern bool lge_get_mfts_mode(void);

static bool use_primary_usb;
module_param(use_primary_usb, bool, 0644);

static bool usb_sudden_disconnect_check;
module_param(usb_sudden_disconnect_check, bool, 0644);

static bool usb_2nd_host_test;
module_param(usb_2nd_host_test, bool, 0644);

static bool force_set_hallic;
module_param(force_set_hallic, bool, 0644);

static unsigned int ds_accid_reg_en_delay_ms = 50;
module_param(ds_accid_reg_en_delay_ms, uint, 0644);

static unsigned int ds_recheck_accid_ms = 2000;
module_param(ds_recheck_accid_ms, uint, 0644);

static unsigned int ds_usb_check_time_ms = 3000;
module_param(ds_usb_check_time_ms, uint, 0644);

static unsigned int ds_vconn_recovery_time_ms = 100;
module_param(ds_vconn_recovery_time_ms, uint, 0644);

static unsigned int ds_power_recovery_count = 5;
module_param(ds_power_recovery_count, uint, 0644);

static unsigned int usb_recovery_time_ms = 2000;
module_param(usb_recovery_time_ms, uint, 0644);

static unsigned int acc_high_threshold_uv;
module_param(acc_high_threshold_uv, uint, 0644);

static unsigned int acc_low_threshold_uv;
module_param(acc_low_threshold_uv, uint, 0644);

#define DP_USBPD_VDM_STATUS		0x10
#define DP_USBPD_VDM_CONFIGURE		0x11
#define ACCID_CHANNEL			VADC_AMUX2_GPIO_PU2 //VADC_AMUX2_GPIO

enum ds_state {
	STATE_UNKNOWN,
	STATE_DS_USB_WAIT,
	STATE_DS_STARTUP,
	STATE_DS_READY,
	STATE_DS_RECOVERY,
	STATE_DS_RECOVERY_POWER_OFF,
	STATE_DS_RECOVERY_POWER_ON,
	STATE_DS_RECOVERY_USB_WAIT,
	STATE_DS_DLOAD,
};

static const char * const ds_state_strings[] = {
	"Unknown",
	"DS_USB_Wait",
	"DS_Startup",
	"DS_Ready",
	"DS_Recovery",
	"DS_Recovery_Power_Off",
	"DS_Recovery_Power_On",
	"DS_Recovery_USB_Wait",
	"DS_Dload",
};

enum ds3_usb {
	DS_USB_DISCONNECTED = 0,
	DS_USB_CONNECTED,
	DS_USB_DLOAD_CONNECTED,
};

struct ds3 {
	struct device			*dev;

	struct workqueue_struct		*wq;
	struct work_struct		sm_work;
	struct delayed_work		ds_acc_detect_work;
	struct hrtimer			acc_timer;
	struct hrtimer			timer;
	bool				sm_queued;
	enum ds_state			current_state;

	struct power_supply		*usb_psy;
	struct notifier_block		psy_nb;
	enum power_supply_typec_mode	typec_mode;

	struct regulator		*vconn;

	struct extcon_dev		*extcon[2];
	struct gpio_desc		*dd_sw_sel;
	struct gpio_desc		*load_sw_on;
	struct gpio_desc		*ds_en;
	struct gpio_desc		*acc_id_detect_en;
	struct notifier_block		nb;
	struct usb_device		*udev;

	bool				is_ds_connected;
	enum ds3_usb			is_ds_usb_connected;
	bool				is_ds_hal_ready;
	int				is_ds_recovery;
	bool				is_dp_configured;
	bool				is_dp_hpd_high;
	bool				is_accid_connected;

	bool				is_usb_connected;
	bool				vbus_present;
	bool				vbus_recovery;
	bool				acc_det_vcomp;
	bool				acc_det_vadc;
	int				acc_det_count;
	int				pd_active;

	struct usbpd			*usbpd;

#ifdef CONFIG_LGE_USB_SBU_SWITCH
	struct lge_sbu_switch_desc      sbu_desc;
	struct lge_sbu_switch_instance  *sbu_inst;
#endif
	int				acc_detect;
	struct qpnp_vadc_chip		*accid_vadc;
	int				acc_high_thr;
	int				acc_low_thr;
};

enum pd_control_msg_type {
	MSG_RESERVED = 0,
	MSG_GOODCRC,
	MSG_GOTOMIN,
	MSG_ACCEPT,
	MSG_REJECT,
	MSG_PING,
	MSG_PS_RDY,
	MSG_GET_SOURCE_CAP,
	MSG_GET_SINK_CAP,
	MSG_DR_SWAP,
	MSG_PR_SWAP,
	MSG_VCONN_SWAP,
	MSG_WAIT,
	MSG_SOFT_RESET,
	MSG_NOT_SUPPORTED = 0x10,
};

enum usbpd_data_msg_type {
	MSG_SOURCE_CAPABILITIES = 1,
	MSG_REQUEST,
	MSG_BIST,
	MSG_SINK_CAPABILITIES,
	MSG_VDM = 0xF,
};

static struct ds3 *__ds3 = NULL;

static bool hallic_status = false;
static bool *ds3_connected = NULL;

static const unsigned int ds_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static void kick_sm(struct ds3 *ds3, int ms);
static void ds_set_state(struct ds3 *ds3, enum ds_state next_state);

extern struct hallic_dev luke_sdev;
extern void request_dualscreen_recovery(void);
extern struct lge_dp_display *get_lge_dp(void);
extern void call_disconnect_uevent(void);
static bool check_ds3_accid(struct ds3 *ds3);
#ifdef CONFIG_LGE_USB_MOISTURE_FUSB251
extern void fusb251_enable_moisture(bool);
#endif

void set_hallic_status(bool enable)
{
	struct ds3 *ds3 = __ds3;

	hallic_status = enable;

	if (!ds3) {
		pr_err("%s: %d (not ready yet)\n", __func__, enable);
		return;
	}

	dev_dbg(ds3->dev, "%s: %d\n", __func__, enable);

	if (ds3->acc_det_vcomp || ds3->acc_det_vadc) {
		if (enable) {
			ds3->is_accid_connected = check_ds3_accid(ds3);
			if (ds3->is_accid_connected)
				ds3->acc_det_count = 0;
		} else {
			hrtimer_cancel(&ds3->acc_timer);
			ds3->is_accid_connected = 0;
			ds3->acc_det_count = 0;
		}
	} else {
		dev_info(ds3->dev, "%s: there is no acc_det_vcomp, use hallic only\n", __func__);
		ds3->is_accid_connected = enable;
	}

	dev_dbg(ds3->dev, "typec:%d vbus:%d pd:%d ds:%d hallic:%d accid:%d"
		" usb:%d ds_usb:%d ds_recovery:%d\n",
		ds3->typec_mode,
		ds3->vbus_present,
		ds3->pd_active,
		ds3->is_ds_connected,
		hallic_status,
		ds3->is_accid_connected,
		ds3->is_usb_connected,
		ds3->is_ds_usb_connected,
		ds3->is_ds_recovery);

	if (enable && ds3->is_accid_connected) {
		if (!ds3->is_ds_connected)
			ds_set_state(ds3, STATE_DS_STARTUP);
	} else {
		if (ds3->is_ds_connected)
			kick_sm(ds3, 0);
	}
}
EXPORT_SYMBOL(set_hallic_status);
static int set_ds_extcon_state(unsigned int id, int state)
{
	int ret = 0;
	struct lge_dp_display *lge_dp = get_lge_dp();

	ret = extcon_set_state_sync(lge_dp->dd_extcon_sdev[0], id, state);

	return ret;
}

static void hallic_state_notify(struct ds3 *ds3, struct hallic_dev *hdev,
				int state)
{
	char name_buf[40];
	char state_buf[40];
	char *uevent[3] = { name_buf, state_buf, NULL };

	if (!hdev || !hdev->dev) {
		dev_err(ds3->dev, "hallic_dev is NULL\n");
		return;
	}

	if (!lge_get_mfts_mode())
		hdev->state = state;

	snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=%s", hdev->name);
	snprintf(state_buf, sizeof(state_buf), "SWITCH_STATE=%d", state);

	kobject_uevent_env(&hdev->dev->kobj, KOBJ_CHANGE, uevent);
	dev_dbg(ds3->dev, "%s: %s\n", __func__, name_buf);
	dev_dbg(ds3->dev, "%s: %s\n", __func__, state_buf);
}

bool is_ds_connected(void)
{
	struct ds3 *ds3 = __ds3;
	bool ret = ds3_connected ? *ds3_connected : false;

	if (!ds3)
		pr_debug("%s: %d (not ready yet)\n", __func__, ret);

	return ret;
}
EXPORT_SYMBOL(is_ds_connected);

int check_ds_connect_state(void)
{
	struct ds3 *ds3 = __ds3;

	if (!ds3)
		return 0;

	if (ds3->is_dp_hpd_high)
		return DS_STATE_HPD_ENABLED;
	else if (ds3->is_dp_configured)
		return DS_STATE_DP_CONNECTED;
	else if (ds3->is_accid_connected)
		return DS_STATE_ACC_ID_CONNECTED;
	else if (hallic_status)
		return DS_STATE_HALLIC_CONNECTED;

	return DS_STATE_DISCONNECTED;
}
EXPORT_SYMBOL(check_ds_connect_state);

void set_vbus_recovery(bool set)
{
	struct ds3 *ds3 = __ds3;

	if (!ds3)
		return;

	ds3->vbus_recovery = set;
}
EXPORT_SYMBOL(set_vbus_recovery);

static void dp_configure(struct usbpd_svid_handler *handler)
{
	u8 cmd[] = {
		USBPD_SVDM_DISCOVER_MODES,
		USBPD_SVDM_ENTER_MODE,
		DP_USBPD_VDM_STATUS,
		DP_USBPD_VDM_CONFIGURE,
	};
	const u32 vdos_nul = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(cmd); i++) {
		switch (cmd[i]) {
		case USBPD_SVDM_DISCOVER_MODES: {
			struct {
				u8 port_cap:2;
				u8 sig_supp:4;
				u8 recep_ind:1;
				u8 r2_0:1;
				u8 dfp_d_pins;
				u8 ufp_d_pins;
				u8 reserved;
			} vdos = {
				.port_cap = 1, // UFP_D
				.sig_supp = 1, // DP v1.3 signaling rate
				.dfp_d_pins = 0x0C, // Pin Assignments: C, D
			};

			handler->svdm_received(handler,
					USBPD_SVDM_DISCOVER_MODES,
					SVDM_CMD_TYPE_RESP_ACK,
					(u32 *)&vdos, 1);
			break;
		}

		case USBPD_SVDM_ENTER_MODE:
			handler->svdm_received(handler,
					USBPD_SVDM_ENTER_MODE,
					SVDM_CMD_TYPE_RESP_ACK,
					&vdos_nul, 0);
			break;

		case DP_USBPD_VDM_STATUS: {
			struct {
				u32 conn:2;
				u32 power_low:1;
				u32 adaptor_func:1;
				u32 multi_func:1;
				u32 usb_config:1;
				u32 exit_dp:1;
				u32 hpd_state:1;
				u32 irq_hpd:1;
				u32 reserved:23;
			} vdos = {
				.conn = 2, // UFP_D
				.adaptor_func = 1,
				.multi_func = 1,
			};

			handler->svdm_received(handler,
					DP_USBPD_VDM_STATUS,
					SVDM_CMD_TYPE_RESP_ACK,
					(u32 *)&vdos, 1);
			break;
		}

		case DP_USBPD_VDM_CONFIGURE:
			handler->svdm_received(handler,
					DP_USBPD_VDM_CONFIGURE,
					SVDM_CMD_TYPE_RESP_ACK,
					&vdos_nul, 0);

			break;
		}
	}
}

static int ds_dp_config(struct ds3 *ds3, bool config)
{
	struct device *dev = ds3->dev;
	struct usbpd_svid_handler *handler;

	dev_info(dev, "%s: config:%d\n", __func__, config);

	handler = usbpd_find_dp_handler(ds3->usbpd);
	if (!handler) {
		dev_err(dev, "%s: No DP handler found\n", __func__);
		return -ENODEV;
	}

	if (config) {
		ds3->is_dp_configured = true;
		handler->connect(handler);
		dp_configure(handler);

		if (ds3->is_ds_usb_connected == DS_USB_CONNECTED) {
			set_ds_extcon_state(EXTCON_DISP_DS2, 1);
			dev_err(dev, "%s: current luke state = %d\n", __func__,
				luke_sdev.state);
			hallic_state_notify(ds3, &luke_sdev, 1);
		}
	} else {
		ds3->is_dp_configured = false;
		handler->disconnect(handler);
	}

#ifdef CONFIG_LGE_USB_SBU_SWITCH
	if (config)
		lge_sbu_switch_get(ds3->sbu_inst, LGE_SBU_SWITCH_FLAG_SBU_AUX);
	else
		lge_sbu_switch_put(ds3->sbu_inst, LGE_SBU_SWITCH_FLAG_SBU_AUX);
#endif

	return 0;
}

static int ds3_dp_hpd(struct ds3 *ds3, bool hpd)
{
	struct device *dev = ds3->dev;
	struct usbpd_svid_handler *handler;
	struct {
		u32 conn:2;
		u32 power_low:1;
		u32 adaptor_func:1;
		u32 multi_func:1;
		u32 usb_config:1;
		u32 exit_dp:1;
		u32 hpd_state:1;
		u32 irq_hpd:1;
		u32 reserved:23;
	} vdos = {
		.conn = 2, // UFP_D
		.adaptor_func = 1,
		.multi_func = 1,
		.hpd_state = hpd,
	};

	handler = usbpd_find_dp_handler(ds3->usbpd);
	if (!handler) {
		dev_err(dev, "%s: No DP handler found\n", __func__);
		return -ENODEV;
	}

	dev_info(dev, "%s: is_dp_hpd_high:%d hpd: %d\n", __func__,
			ds3->is_dp_hpd_high, hpd);

	if (ds3->is_dp_hpd_high && hpd)
		ds3_dp_hpd(ds3, false);

	if (ds3->is_dp_hpd_high == hpd) {
		dev_dbg(dev, "%s: duplicated value is set\n", __func__);
		return 0;
	}
	ds3->is_dp_hpd_high = hpd;

	handler->svdm_received(handler,
			USBPD_SVDM_ATTENTION,
			SVDM_CMD_TYPE_INITIATOR,
			(u32 *)&vdos, 1);

	return 0;
}

static void stop_usb_host(struct ds3 *ds3)
{
	struct device *dev = ds3->dev;
	struct extcon_dev *extcon = ds3->extcon[use_primary_usb ? 1 : 0];

	if (!extcon)
		return;

	if (extcon_get_state(extcon, EXTCON_USB_HOST) == 0)
		return;

	dev_dbg(dev, "%s\n", __func__);

	extcon_set_state_sync(extcon, EXTCON_USB_HOST, 0);
}

static void start_usb_host(struct ds3 *ds3)
{
	struct device *dev = ds3->dev;
	struct extcon_dev *extcon = ds3->extcon[use_primary_usb ? 1 : 0];

	if (!extcon)
		return;

	if (extcon_get_state(extcon, EXTCON_USB_HOST) == 1)
		return;

	dev_dbg(dev, "%s\n", __func__);

	if (use_primary_usb) {
		union extcon_property_value val;
		val.intval = 1;
		extcon_set_property(extcon, EXTCON_USB_HOST,
				    EXTCON_PROP_USB_TYPEC_POLARITY, val);
		val.intval = 0;
		extcon_set_property(extcon, EXTCON_USB_HOST,
				    EXTCON_PROP_USB_SS, val);
	}
	extcon_set_state_sync(extcon, EXTCON_USB_HOST, 1);
}

static bool is_start_usb_host(struct ds3 *ds3)
{
	struct extcon_dev *extcon = ds3->extcon[use_primary_usb ? 1 : 0];

	if (!extcon)
		return false;

	return extcon_get_state(extcon, EXTCON_USB_HOST) == 1;
}

static int ds3_usb_notify(struct notifier_block *nb, unsigned long action,
			  void *data)
{
	struct ds3 *ds3 = container_of(nb, struct ds3, nb);
	struct device *dev = ds3->dev;
	struct usb_device *udev = data;

	dev_vdbg(dev, "%s: dev num:%d path:%s\n", __func__,
		udev->devnum, udev->devpath);
	dev_vdbg(dev, "%s: bus num:%d name:%s\n", __func__,
		udev->bus->busnum, udev->bus->bus_name);

	if (usb_2nd_host_test)
		return NOTIFY_DONE;

	switch (action) {
	case USB_DEVICE_ADD:
		if (!udev->parent)
			return NOTIFY_DONE;

		dev_info(dev, "%s: USB_DEVICE_ADD: idVendor:%04x idProduct:%04x bcdDevice:%04x\n",
			__func__,
			udev->descriptor.idVendor,
			udev->descriptor.idProduct,
			udev->descriptor.bcdDevice);

		ds3->is_usb_connected = true;

		if (!ds3->is_ds_connected)
			return NOTIFY_DONE;

		if (!IS_DS3_ANY_USB(udev) )
			return NOTIFY_DONE;

		set_ds_extcon_state(EXTCON_DISP_DS2, 1);

		// DS3 or DS3 USB Connected
		if (IS_DS3_USB(udev)) {
			dev_dbg(dev, "%s: FW_VER: %s-V%02u%c_XX\n", __func__,
#if defined (CONFIG_MACH_SDM845_CAYMANSLM)
				udev->product ? udev->product : DS3_CAYMAN_PRODUCT_STR,
#else
				udev->product ? udev->product : DS3_TIME_PRODUCT_STR,
#endif
				(udev->descriptor.bcdDevice >> 8) & 0xff,
				'a' + ((udev->descriptor.bcdDevice & 0xff) % 26/*a-z*/));

			ds3->is_ds_usb_connected = DS_USB_CONNECTED;

		// DS3 Dload USB Connected
		} else if (IS_DS3_DLOAD_USB(udev)) {
			ds3->is_ds_usb_connected = DS_USB_DLOAD_CONNECTED;
			ds_set_state(ds3, STATE_DS_DLOAD);

			dev_err(dev, "%s: currunt luke state = %d\n",
				__func__, luke_sdev.state);
			call_disconnect_uevent();
			hallic_state_notify(ds3, &luke_sdev, 0);
		}

		return NOTIFY_OK;

	case USB_DEVICE_REMOVE:
		if (!udev->parent)
			return NOTIFY_DONE;

		dev_info(dev, "%s: USB_DEVICE_REMOVE: idVendor:%04x idProduct:%04x\n",
			__func__,
			udev->descriptor.idVendor,
			udev->descriptor.idProduct);

		ds3->is_usb_connected = false;

		if (!IS_DS3_ANY_USB(udev))
			return NOTIFY_DONE;

		ds3->is_ds_usb_connected = DS_USB_DISCONNECTED;
		ds3->is_ds_hal_ready = false;

		// DS3 USB Disconnected
		if (IS_DS3_USB(udev)) {
			BUG_ON(usb_sudden_disconnect_check);

			if (ds3->vbus_recovery) {
				ds_set_state(ds3, STATE_DS_USB_WAIT);
				ds3->vbus_recovery = false;
			} else if (ds3->is_ds_connected && ds3->is_ds_recovery <= 0) {
				stop_usb_host(ds3);
				start_usb_host(ds3);
				ds_set_state(ds3, STATE_DS_RECOVERY_USB_WAIT);
			} else if (hallic_status && ds3->is_accid_connected) {
				set_hallic_status(true);
			}

		// DS3 Dload USB Disconnected
		} else if (IS_DS3_DLOAD_USB(udev)) {

		}

		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static void ds3_sm(struct work_struct *w)
{
	struct ds3 *ds3 = container_of(w, struct ds3, sm_work);
	struct device *dev = ds3->dev;
	int ret = 0;

	hrtimer_cancel(&ds3->timer);
	ds3->sm_queued = false;

	if (usb_2nd_host_test) {
		if (ds3->typec_mode == POWER_SUPPLY_TYPEC_NONE) {
			stop_usb_host(ds3);
			if (ds3->dd_sw_sel)
				gpiod_direction_output(ds3->dd_sw_sel, 0);
		} else {
			if (ds3->dd_sw_sel)
				gpiod_direction_output(ds3->dd_sw_sel, 1);
			start_usb_host(ds3);
		}
		goto sm_done;
	}

	dev_info(dev, "%s: %s\n", __func__,
			ds_state_strings[ds3->current_state]);

	// disconnect
	if (!hallic_status || !ds3->is_accid_connected) {
		if (!ds3->is_ds_connected) {
			dev_dbg(dev, "%s: DS3 is already disconnected\n",
				__func__);
			goto sm_done;
		}

		dev_info(dev, "%s: DS disconnect\n", __func__);
		ds3->is_ds_connected = false;

		// Secondary USB
		if (is_start_usb_host(ds3)) {
			stop_usb_host(ds3);
			if (!use_primary_usb && ds3->dd_sw_sel)
				gpiod_direction_output(ds3->dd_sw_sel, 0);
		}

		ds_dp_config(ds3, false);

		// Disable DisplayPort
		dev_err(dev, "%s: currunt luke state = %d\n", __func__,
				luke_sdev.state);
		set_ds_extcon_state(EXTCON_DISP_DS2, 0);
		call_disconnect_uevent();
		hallic_state_notify(ds3, &luke_sdev, 0);

		// DS Power Off
		if (ds3->ds_en) {
			gpiod_direction_output(ds3->ds_en, 0);
		}
		gpiod_direction_output(ds3->load_sw_on, 0);

		ds3->is_dp_configured = false;
		ds3->is_dp_hpd_high = false;

		ds3->is_ds_recovery = 0;

		ds3->current_state = STATE_UNKNOWN;

		set_ds3_start(false);

#ifdef CONFIG_LGE_USB_MOISTURE_FUSB251
		fusb251_enable_moisture(true);
#endif

		goto sm_done;
	}

	switch (ds3->current_state) {
	case STATE_UNKNOWN:
	case STATE_DS_STARTUP:
		if (ds3->is_ds_connected) {
			dev_dbg(dev, "%s: DS3 is already connected\n",
				__func__);
			goto sm_done;
		}

#ifdef CONFIG_LGE_USB_MOISTURE_FUSB251
		fusb251_enable_moisture(false);
#endif
		ds3->is_ds_connected = true;

		// DS Power On
		if (ds3->ds_en) {
			gpiod_direction_output(ds3->ds_en, 1);
		} else if (ds3->vconn) {
			ret = regulator_enable(ds3->vconn);
			if (ret)
				dev_err(dev, "Unable to enable vconn\n");
		}
		gpiod_direction_output(ds3->load_sw_on, 1);

		// Secondary USB
		if (!is_start_usb_host(ds3)) {
			if (!use_primary_usb && ds3->dd_sw_sel)
				gpiod_direction_output(ds3->dd_sw_sel, 1);
			start_usb_host(ds3);
		}

		ret = ds_dp_config(ds3, true);
		if (ret) {
			ds3->is_ds_connected = false;
			stop_usb_host(ds3);
			if (ds3->dd_sw_sel)
				gpiod_direction_output(ds3->dd_sw_sel, 0);
			ds_set_state(ds3, STATE_UNKNOWN);
			goto sm_done;
		}

		ds3->is_dp_hpd_high = false;
		set_ds3_start(true);

		if (!ds3->is_ds_usb_connected) {
			ds_set_state(ds3, STATE_DS_USB_WAIT);
			goto sm_done;
		}

		ds3->current_state = STATE_DS_STARTUP;
		goto sm_done;
		break;

	case STATE_DS_USB_WAIT:
		if (!ds3->is_ds_usb_connected)
			ds_set_state(ds3, STATE_DS_RECOVERY_POWER_OFF);
		break;

	case STATE_DS_READY:
		if (ds3->is_dp_configured) {
			dev_err(dev, "%s: current luke state = %d\n", __func__,
					luke_sdev.state);
			hallic_state_notify(ds3, &luke_sdev, 1);
		}
		break;

	case STATE_DS_RECOVERY:
		if (ds3->is_ds_usb_connected)
			break;

		dev_info(dev, "%s: %s %d\n", __func__,
				ds_state_strings[ds3->current_state],
				ds3->is_ds_recovery);

		if (ds3->is_ds_recovery > ds_power_recovery_count)
			ds3_dp_hpd(ds3, false);

		ds3->is_ds_recovery--;
		ds_set_state(ds3, STATE_DS_RECOVERY_POWER_OFF);
		break;

	case STATE_DS_RECOVERY_POWER_OFF:
		// 2nd USB off
		stop_usb_host(ds3);

#if 0
		/* blocks until USB host is completely stopped */
		ret = extcon_blocking_sync(ds3->extcon, EXTCON_USB_HOST, 0);
		if (ret) {
			dev_err(ds3->dev, "%s: err(%d) stopping host", ret);
			break;
		}
#endif

		if (ds3->is_ds_recovery > ds_power_recovery_count) {
			ds3->current_state = STATE_DS_RECOVERY_POWER_ON;
			kick_sm(ds3, 0);
			break;
		}

		// Power Off
		if (ds3->ds_en) {
			gpiod_direction_output(ds3->ds_en, 0);
		}
		gpiod_direction_output(ds3->load_sw_on, 0);

		ds_set_state(ds3, STATE_DS_RECOVERY_POWER_ON);
		break;

	case STATE_DS_RECOVERY_POWER_ON:
		// 2nd USB on
		 start_usb_host(ds3);

#if 0
		 /* blocks until USB host is completely started */
		 ret = extcon_blocking_sync(ds3->extcon, EXTCON_USB_HOST, 0);
		 if (ret) {
			 dev_err(ds3->dev, "%s: err(%d) starting host", ret);
			 break;
		 }
#endif

		if (ds3->is_ds_recovery > ds_power_recovery_count) {
			ds_set_state(ds3, STATE_DS_RECOVERY_USB_WAIT);
			break;
		}

		if (ds3->ds_en) {
			gpiod_direction_output(ds3->ds_en, 1);
		}
		gpiod_direction_output(ds3->load_sw_on, 1);

		ds_set_state(ds3, STATE_DS_RECOVERY_USB_WAIT);
		break;

	case STATE_DS_RECOVERY_USB_WAIT:
		if (ds3->is_ds_recovery <= 0)
			ds3->is_ds_recovery = ds_power_recovery_count;
		ds_set_state(ds3, STATE_DS_RECOVERY);
		break;

	case STATE_DS_DLOAD:
		break;

	default:
		dev_err(dev, "%s: Unhandled state %s\n", __func__,
			ds_state_strings[ds3->current_state]);
		break;
	}

sm_done:
	if (!ds3->sm_queued)
		pm_relax(ds3->dev);
}

static void ds_set_state(struct ds3 *ds3, enum ds_state next_state)
{
	struct device *dev = ds3->dev;
	dev_dbg(dev, "%s: %s -> %s\n", __func__,
			ds_state_strings[ds3->current_state],
			ds_state_strings[next_state]);

	ds3->current_state = next_state;

	switch (next_state) {
	case STATE_DS_USB_WAIT:
		kick_sm(ds3, ds_usb_check_time_ms);
		break;

	case STATE_DS_STARTUP:
		kick_sm(ds3, 0);
		break;

	case STATE_DS_READY:
		kick_sm(ds3, 0);
		break;

	case STATE_DS_RECOVERY:
		if (ds3->is_ds_recovery <= 0)
			break;

		kick_sm(ds3, ds_usb_check_time_ms);
		break;

	case STATE_DS_RECOVERY_POWER_OFF:
		if (ds_power_recovery_count <= 0)
			break;

		kick_sm(ds3, 0);
		break;

	case STATE_DS_RECOVERY_POWER_ON:
		kick_sm(ds3, ds_vconn_recovery_time_ms);
		break;

	case STATE_DS_RECOVERY_USB_WAIT:
		kick_sm(ds3, ds_usb_check_time_ms);
		break;

	case STATE_DS_DLOAD:
		ds3->is_ds_recovery = 0;
		ds3_dp_hpd(ds3, false);
		break;

	default:
		dev_err(dev, "%s: No action for state %s\n", __func__,
				ds_state_strings[ds3->current_state]);
		break;
	}
}

static void kick_sm(struct ds3 *ds3, int ms)
{
	pm_stay_awake(ds3->dev);
	ds3->sm_queued = true;

	if (ms) {
		dev_dbg(ds3->dev, "delay %d ms", ms);
		hrtimer_start(&ds3->timer, ms_to_ktime(ms), HRTIMER_MODE_REL);
	} else {
		queue_work(ds3->wq, &ds3->sm_work);
	}
}

static enum hrtimer_restart ds_timeout(struct hrtimer *timer)
{
	struct ds3 *ds3 = container_of(timer, struct ds3, timer);

	queue_work(ds3->wq, &ds3->sm_work);

	return HRTIMER_NORESTART;
}

static void ds_acc_detect(struct work_struct *w)
{
	struct ds3 *ds3 = container_of(w, struct ds3, ds_acc_detect_work.work);

	if (hallic_status && !ds3->is_accid_connected)
		set_hallic_status(true);
}

static enum hrtimer_restart ds_acc_timeout(struct hrtimer *timer)
{
	struct ds3 *ds3 = container_of(timer, struct ds3, acc_timer);

	schedule_delayed_work(&ds3->ds_acc_detect_work, 0);

	return HRTIMER_NORESTART;
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	struct ds3 *ds3 = container_of(nb, struct ds3, psy_nb);
	struct device *dev = ds3->dev;
	union power_supply_propval val;
	enum power_supply_typec_mode typec_mode;
	int ret;

	if (ptr != ds3->usb_psy || evt != PSY_EVENT_PROP_CHANGED)
		return 0;

	ret = power_supply_get_property(ds3->usb_psy,
			POWER_SUPPLY_PROP_PD_ACTIVE, &val);
	if (ret) {
		dev_err(dev, "Unable to read PD_ACTIVE: %d\n", ret);
		return ret;
	}
	ds3->pd_active = val.intval;

	ret = power_supply_get_property(ds3->usb_psy,
			POWER_SUPPLY_PROP_PRESENT, &val);
	if (ret) {
		dev_err(dev, "Unable to read USB PRESENT: %d\n", ret);
		return ret;
	}
	ds3->vbus_present = val.intval;

	ret = power_supply_get_property(ds3->usb_psy,
			POWER_SUPPLY_PROP_TYPEC_MODE, &val);
	if (ret < 0) {
		dev_err(dev, "Unable to read USB TYPEC_MODE: %d\n", ret);
		return ret;
	}
	typec_mode = val.intval;

	if (force_set_hallic)
		hallic_status = true;

	dev_dbg(dev, "typec:%d vbus:%d pd:%d ds:%d hallic:%d accid:%d"
		" usb:%d ds_usb:%d ds_recovery:%d\n",
		typec_mode,
		ds3->vbus_present,
		ds3->pd_active,
		ds3->is_ds_connected,
		hallic_status,
		ds3->is_accid_connected,
		ds3->is_usb_connected,
		ds3->is_ds_usb_connected,
		ds3->is_ds_recovery);

	ds3->typec_mode = typec_mode;

	return 0;
}

static ssize_t ds2_hal_ready_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct ds3 *ds3 = dev_get_drvdata(dev);
	bool ready;
	int ret;

	if (!ds3->is_ds_connected)
		return -ENODEV;

	ret = strtobool(buf, &ready);
	if (ret < 0)
		return ret;

	dev_info(ds3->dev, "%s: ready:%d, recovery:%d\n", __func__,
			ready, ds3->is_ds_recovery);

	if (!ready)
		return size;

	ds3->is_ds_hal_ready = true;

	if (ds3->is_ds_recovery || ds3->current_state == STATE_DS_RECOVERY) {
		ds3->is_ds_recovery = 0;
		request_dualscreen_recovery();
	}
	ds_set_state(ds3, STATE_DS_READY);

	return size;
}
static DEVICE_ATTR_WO(ds2_hal_ready);

static ssize_t ds2_pd_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ds3 *ds3 = dev_get_drvdata(dev);
	dev_dbg(ds3->dev, "%s: hpd_high:%d\n", __func__, ds3->is_dp_hpd_high);
	return scnprintf(buf, PAGE_SIZE, "%d", ds3->is_dp_hpd_high);
}

static ssize_t ds2_pd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct ds3 *ds3 = dev_get_drvdata(dev);
	int hpd_high, refresh_layer;

	if (sscanf(buf, "%d%d", &hpd_high, &refresh_layer) <= 0) {
		dev_err(ds3->dev, "%s: invalid agument: %s", __func__, buf);
		return -EINVAL;
	}

	dev_info(ds3->dev, "%s: hpd_high:%d refresh_layer:%d\n", __func__,
			hpd_high, refresh_layer);

	if (!ds3->is_dp_configured) {
		dev_info(ds3->dev, "%s: dp is not configured\n", __func__);
		return size;
	}

	ds3_dp_hpd(ds3, hpd_high);

	return size;
}

static DEVICE_ATTR_RW(ds2_pd);

static ssize_t ds2_recovery_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct ds3 *ds3 = dev_get_drvdata(dev);
	bool recovery;
	int ret;

	if (!ds3->is_ds_connected)
		return -ENODEV;

	ret = strtobool(buf, &recovery);
	if (ret < 0)
		return ret;

	dev_info(ds3->dev, "%s: recovery:%d\n", __func__, recovery);

	if (!recovery)
		return size;

	ds3->is_ds_recovery = ds_power_recovery_count;
	if (ds3->is_ds_recovery > 0)
		ds3->is_ds_recovery++;
	ds_set_state(ds3, STATE_DS_RECOVERY_POWER_OFF);

	return size;
}
// #define RECOVERY_PATH "/sys/class/dualscreen/ds2/ds2_recovery"
static DEVICE_ATTR_WO(ds2_recovery);

static bool check_ds3_accid(struct ds3 *ds3)
{
	struct qpnp_vadc_result results;
	bool connected = false;
	int val;
	int i;

	if (ds3->acc_id_detect_en) {
	    gpiod_direction_output(ds3->acc_id_detect_en, 1);
	    pr_info("%s: acc_id_detect_on !\n",__func__);
	    msleep(ds_accid_reg_en_delay_ms);
	}
	else
		pr_info("%s: Use PM845 GPIO9 internal pull-up for ACC_ID\n",__func__);

	for (i = 0; i < 3; i++) {
		qpnp_vadc_read(ds3->accid_vadc, ACCID_CHANNEL, &results);
		val = (int)results.physical;
		dev_dbg(ds3->dev, "ACC_DETECT VADC=%d\n", val);
		if (val <= acc_high_threshold_uv &&
				val >= acc_low_threshold_uv) {
			connected = true;
			break;
		}
		msleep(20);
	}

	if (ds3->acc_id_detect_en)
		gpiod_direction_output(ds3->acc_id_detect_en, 0);

	if (!connected) {
		if (ds3->acc_det_count < 5) {
			ds3->acc_det_count++;

			dev_dbg(ds3->dev, "hallic detected but acc_id isn't. start timer\n");
			hrtimer_start(&ds3->acc_timer,
				      ms_to_ktime(ds_recheck_accid_ms),
				      HRTIMER_MODE_REL);
		} else {
			dev_info(ds3->dev, "acc_detect retry count exceeded\n");
		}
	}

	return connected;
}

static int ds3_probe_vadc(struct ds3 *ds3)
{
	struct qpnp_vadc_chip *accid_vadc = NULL;
	struct device *dev = ds3->dev;
	int ret = 0;

	accid_vadc = qpnp_get_vadc(dev, "channel");
	if (IS_ERR(accid_vadc)) {
		dev_err(dev, "channel probe defer\n");
		ret = PTR_ERR(accid_vadc);
		dev_err(dev, "Unable to get VADC dev\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "lge,acc-high-thr",
						&ds3->acc_high_thr);
	ret = of_property_read_u32(dev->of_node, "lge,acc-low-thr",
						&ds3->acc_low_thr);
	if (ret) {
		dev_err(dev, "error reading acc threshold level property\n");
		return ret;
	}

	acc_high_threshold_uv = ds3->acc_high_thr;
	acc_low_threshold_uv = ds3->acc_low_thr;
	ds3->accid_vadc = accid_vadc;
	ds3->acc_det_count = 0;

	return 0;
}

static int ds3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ds3 *ds3;
	static struct class *ds3_class;
	static struct device *ds3_dev;
	struct usbpd *pd = NULL;
	int ret = 0;

	dev_info(dev, "%s\n", __func__);

	pd = devm_usbpd_get_by_phandle(dev, "usbpd");
	if (IS_ERR(pd)) {
		dev_err(dev, "usbpd phandle failed (%ld)\n", PTR_ERR(pd));
		return (PTR_ERR(pd) == -EAGAIN) ? -EPROBE_DEFER : PTR_ERR(pd);
	}

	ds3 = devm_kzalloc(dev, sizeof(*ds3), GFP_KERNEL);
	if (!ds3) {
		dev_err(dev, "out of memory\n");
		return -ENOMEM;
	}

	dev_set_drvdata(dev, ds3);
	ds3->dev = dev;
	ds3->usbpd = pd;

	ds3->usb_psy = power_supply_get_by_name("usb");
	if (!ds3->usb_psy) {
		dev_err(dev, "couldn't get USB power_supply, deferring probe\n");
		ret = -EPROBE_DEFER;
		goto err;
	}

	ds3->load_sw_on = devm_gpiod_get(dev, "lge,load-sw-on", GPIOD_OUT_LOW);
	if (IS_ERR(ds3->load_sw_on)) {
		ret = PTR_ERR(ds3->load_sw_on);
		dev_err(dev, "couldn't get load-sw-on: %d\n", ret);
		goto err;
	}

	ds3->ds_en = devm_gpiod_get(dev, "lge,ds-en", GPIOD_OUT_LOW);
	if (IS_ERR(ds3->ds_en)) {
		ret = PTR_ERR(ds3->ds_en);
		dev_err(dev, "Unable to get ds_en: %d\n", ret);
		ds3->ds_en = NULL;
	}

	ds3->acc_id_detect_en = devm_gpiod_get(dev, "lge,acc-id-detect-en",
					       GPIOD_OUT_LOW);
	if (IS_ERR(ds3->acc_id_detect_en)) {
		ret = PTR_ERR(ds3->acc_id_detect_en);
		dev_err(dev, "Unable to get acc_id_detect_en: %d\n", ret);
		ds3->acc_id_detect_en = NULL;
	}

	ds3->extcon[0] = devm_extcon_dev_allocate(dev, ds_extcon_cable);
	if (IS_ERR(ds3->extcon[0])) {
		ret = PTR_ERR(ds3->extcon[0]);
		dev_err(dev, "failed to allocate extcon device: %d\n", ret);
		goto err;
	}

	ret = devm_extcon_dev_register(dev, ds3->extcon[0]);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device: %d\n", ret);
		goto err;
	}

	ds3->extcon[1] = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR(ds3->extcon[1])) {
		if (PTR_ERR(ds3->extcon[1]) != -ENODEV)
			return PTR_ERR(ds3->extcon[1]);
		ds3->extcon[0] = NULL;
	}

	use_primary_usb = device_property_read_bool(dev, "lge,use-primary-usb");

	ds3->dd_sw_sel = devm_gpiod_get(dev, "lge,dd-sw-sel", GPIOD_OUT_LOW);
	if (IS_ERR(ds3->dd_sw_sel)) {
		ret = PTR_ERR(ds3->dd_sw_sel);
		dev_err(dev, "couldn't get dd-sw-sel gpio: %d\n", ret);
		ds3->dd_sw_sel = NULL;
	}

	ret = device_init_wakeup(ds3->dev, true);
	if (ret < 0)
		goto err;

	ds3->wq = alloc_ordered_workqueue("ds_wq", WQ_HIGHPRI);
	if (!ds3->wq)
		return -ENOMEM;

	ds3->acc_det_vadc = device_property_read_bool(dev,
			"lge,acc_det_vadc");
	if (ds3->acc_det_vadc) {
		ret = ds3_probe_vadc(ds3);
		if (ret < 0) {
			dev_err(dev, "failed to register vadc: %d\n", ret);
			goto err;
		}
	}

	hrtimer_init(&ds3->acc_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ds3->acc_timer.function = ds_acc_timeout;

	hrtimer_init(&ds3->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ds3->timer.function = ds_timeout;

#ifdef CONFIG_LGE_USB_SBU_SWITCH
	ds3->sbu_desc.flags = LGE_SBU_SWITCH_FLAG_SBU_AUX;
	ds3->sbu_inst = devm_lge_sbu_switch_instance_register(ds3->dev,
							&ds3->sbu_desc);
	if (IS_ERR_OR_NULL(ds3->sbu_inst)) {
		dev_err(dev, "Couldn't register lge_sbu_switch rc=%ld\n",
				PTR_ERR(ds3->sbu_inst));
		return PTR_ERR(ds3->sbu_inst);
	}
#endif

	INIT_WORK(&ds3->sm_work, ds3_sm);
	INIT_DELAYED_WORK(&ds3->ds_acc_detect_work, ds_acc_detect);
#ifdef CONFIG_LGE_DUAL_SCREEN_USB_WA
	INIT_DELAYED_WORK(&ds3->ds_usb_wa_work, ds_usb_wa);
#endif

	ds3->psy_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&ds3->psy_nb);
	if (ret < 0)
		goto err;

	ds3_class = class_create(THIS_MODULE, "dualscreen");
	if (IS_ERR(ds3_class)) {
		ret = PTR_ERR(ds3_class);
		dev_err(dev, "failed to create dualscreen class: %d\n", ret);
		goto err_create_ds3_class;
	}

	ds3_dev = device_create(ds3_class, NULL, 0, ds3, "ds2");
	if (IS_ERR(ds3_dev)) {
		ret = PTR_ERR(ds3_dev);
		dev_err(dev, "failed to create device: %d\n", ret);
		goto err_create_ds3_dev;
	}

	ret = device_create_file(ds3_dev, &dev_attr_ds2_hal_ready);
	if (ret < 0) {
		dev_err(dev, "failed to create ds2_hal_ready node: %d\n", ret);
		goto err_create_ds3_hal_ready;
	}

	ret = device_create_file(ds3_dev, &dev_attr_ds2_pd);
	if (ret < 0) {
		dev_err(dev, "failed to create ds3_pd node: %d\n", ret);
		goto err_create_ds3_pd;
	}

	ret = device_create_file(ds3_dev, &dev_attr_ds2_recovery);
	if (ret < 0) {
		dev_err(dev, "failed to create ds2_recovery node: %d\n", ret);
		goto err_create_ds3_recovery;
	}
	ds3->nb.notifier_call = ds3_usb_notify;
	usb_register_notify(&ds3->nb);

	__ds3 = ds3;

	ds3_connected = &ds3->is_dp_configured;

	/* force read initial power_supply values */
	psy_changed(&ds3->psy_nb, PSY_EVENT_PROP_CHANGED, ds3->usb_psy);

	return 0;

err_create_ds3_recovery:
	device_remove_file(ds3_dev, &dev_attr_ds2_pd);
err_create_ds3_pd:
	device_remove_file(ds3_dev, &dev_attr_ds2_hal_ready);
err_create_ds3_hal_ready:
	device_unregister(ds3_dev);
err_create_ds3_dev:
	class_destroy(ds3_class);
err_create_ds3_class:
	power_supply_unreg_notifier(&ds3->psy_nb);
err:
	return ret;
}

static void ds3_shutdown(struct platform_device *pdev)
{
	struct ds3 *ds3 = platform_get_drvdata(pdev);

	power_supply_unreg_notifier(&ds3->psy_nb);

	hrtimer_cancel(&ds3->acc_timer);
	ds3->sm_queued = false;

	stop_usb_host(ds3);
	if (ds3->dd_sw_sel)
		gpiod_direction_output(ds3->dd_sw_sel, 0);

	if (ds3->acc_detect)
		gpio_free(ds3->acc_detect);
}

static const struct of_device_id ds3_match_table[] = {
	{ .compatible = "lge,usb_ds3" },
	{ }
};
MODULE_DEVICE_TABLE(of, ds3_match_table);

static struct platform_driver ds3_driver = {
	.driver = {
		.name = "lge_usb_ds3",
		.of_match_table = ds3_match_table,
	},
	.probe = ds3_probe,
	.shutdown = ds3_shutdown,
};
module_platform_driver(ds3_driver);

MODULE_AUTHOR("Hansun Lee <hansun.lee@lge.com>");
MODULE_DESCRIPTION("LGE USB DS3 driver");
MODULE_LICENSE("GPL v2");

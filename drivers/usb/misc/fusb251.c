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
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/fusb251.h>

#ifdef CONFIG_LGE_USB
#include <soc/qcom/lge/board_lge.h>
#endif

struct i2c_client *__client = NULL;

struct fusb251 {
	struct i2c_client	*client;

	struct gpio_desc	*intb_desc;

	struct workqueue_struct	*wq;
	struct work_struct	i2c_work;
	struct delayed_work	enable_mos_work;
	struct mutex		irq_lock;
	struct mutex		i2c_lock;

	struct power_supply	*usb_psy;

	int			intb_irq;

	int			rev;
	int			state;
	int			ovp_state;

	bool			cc_ovp;
	bool			sbu_ovp;

	bool			typec_connected;

	bool			moisture_enable;
};

static int fusb251_read_reg(struct i2c_client *client, int reg)
{
	int ret;
	int retry = 0;

	if (!client)
		return -ENODEV;

	do {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0) {
			pr_err("%s: err %d\n", __func__, ret);
		}
	} while ((retry++ < 5) && (ret < 0));

	return ret;
}

static int fusb251_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret, i;

	for (i = 0; i <3; i++) {
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (ret < 0) {
			pr_err("%s: err %d, and try again\n", __func__, ret);
			mdelay(50);
		} else {
			break;
		}
	}

	if (ret < 0) {
		pr_err("%s: err %d\n", __func__, ret);
	}

	return ret;
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

void fusb251_update_state(int n_oe, int sel)
{
	int value = 0;

	value = fusb251_read_reg(__client, FUSB251_SWITCH_CONTROL);
	dev_info(&__client->dev, "%s: SWITCH_CONTROL:0x%x\n", __func__, value);

	if (sel)
		value = value | CC_SW_CTRL_BIT |
				(SBU_SW_CTRL_MASK & 0x6);
	else
		value = value | CC_SW_CTRL_BIT |
				(SBU_SW_CTRL_MASK & 0x2);

	if (n_oe)
		value = value | (SBU_SW_CTRL_MASK & 0);

	fusb251_write_reg(__client, FUSB251_SWITCH_CONTROL, value);
}
EXPORT_SYMBOL(fusb251_update_state);


void fusb251_set_state(struct fusb251 *fusb, int state)
{
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;
	union power_supply_propval val = {0};
	int ret;

	dev_info(dev, "%s: current_state=%d, state=%d\n",
			__func__, fusb->state, state);

	if (fusb->state != state)
	{
		val.intval = state;
		ret = power_supply_set_property(fusb->usb_psy,
				POWER_SUPPLY_PROP_MOISTURE_DETECTED,
				&val);
		if (ret < 0) {
			dev_err(dev, "Couldn't set PROP_MOISTURE_DETECTED,"
					"ret=%d\n", ret);
		}
		power_supply_changed(fusb->usb_psy);

		fusb->state = state;
	}
}

static void fusb251_enable_moisture_detect(struct fusb251 *fusb, bool enable)
{
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;
	int control = 0;

	control = fusb251_read_reg(client, FUSB251_CONTROL);

	dev_info(dev, "%s: current control:0x%x, enable:%d\n", __func__,
			control, enable);

	if (enable) {
		fusb->moisture_enable = true;
		control = control | EN_CC_MOS_BIT | EN_SBU_MOS_BIT |
			EN_DRY_BIT | EN_DEBUG_ACC_DET_BIT;
	} else {
		fusb->moisture_enable = false;
		control = control & !EN_CC_MOS_BIT & !EN_SBU_MOS_BIT &
			!EN_DRY_BIT;
	}
	fusb251_write_reg(client, FUSB251_CONTROL, control);
}

static void fusb251_ovp(struct fusb251 *fusb)
{
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;
	int ret = 0;

	ret = fusb251_read_reg(client, FUSB251_STATUS);
	dev_info(dev, "%s: STATUS:0x%x\n", __func__, ret);

	if (ret & OVP_CC_BIT) {
		dev_info(dev, "CC OVP occured\n");
		fusb->cc_ovp = true;
	} else if ((ret & OVP_CC_BIT) == 0) {
		fusb->cc_ovp = false;
	}

	if (ret & OVP_SBU_BIT) {
		dev_info(dev, "SBU OVP occured\n");
		fusb->sbu_ovp = true;
	} else if ((ret & OVP_SBU_BIT) == 0) {
		fusb->sbu_ovp = false;
	}

	fusb->ovp_state = (fusb->cc_ovp ? FUSB251_CC_OVP_DETECTED : 0) +
		(fusb->sbu_ovp ? FUSB251_SBU_OVP_DETECTED : 0);
}

static void fusb251_ovp_recovery(struct fusb251 *fusb)
{
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;
	int ret = 0;

	ret = fusb251_read_reg(client, FUSB251_STATUS);
	dev_info(dev, "%s: STATUS:0x%x\n", __func__, ret);

	fusb->ovp_state = fusb->cc_ovp = fusb->sbu_ovp = 0;
}


static void enable_moisture_work(struct work_struct *w)
{
	struct fusb251 *fusb = container_of(w, struct fusb251, i2c_work);

	if (!fusb->typec_connected)
		fusb251_enable_moisture_detect(fusb, true);
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

	irq_state = fusb251_read_reg(client, FUSB251_INTERRUPT);
	status = fusb251_read_reg(client, FUSB251_STATUS);
	mos_status = fusb251_read_reg(client, FSUB251_MOISTURE_STATUS);

	dev_info(dev, "%s: INTR:0x%x, STATUS:0x%x, MOS_STATUS:0x%x\n",
			__func__, irq_state, status, mos_status);

	/* When an Over-voltage evet is detected on either CCx or SBUx,
	 * the FUSB251 will active OVP circuit to open the switch within tOVP.
	 * The OVP event is independent between CC and SBU.
	 * Any CC channel OVP opens the CC1 and CC2 switch path only,
	 * and SBU switch is keep closed.
	 * OVP on SBU opens the SBU switch only.
	 * Upon the detection of an over-voltage event, the INTB pin is
	 * asserted low to notify processor with register update.
	 *
	 * When OVP condition is removed from the port, the switch path
	 * is closed automatically and OVP recovery interrupt is generated
	 * to notify processor.
	 * Getting interrupt service is not required to close the switch back.
	 * */

	if ((irq_state & CC1_TIMER_INT_BIT) ||
			(irq_state & CC2_TIMER_INT_BIT)) {
		if ((mos_status & SBU1_FT_BIT) ||
				(mos_status & SBU2_MOS_BIT)) {
			fusb251_set_state(fusb, FUSB251_STATE_MOS_DETECTECT);
		} else {
			dev_info(dev, "Cable connected\n");
		}
	}

	if (irq_state & MOS_CHG_INT_BIT) {
		if ((mos_status & CC1_MOS_BIT) ||
				(mos_status & CC2_MOS_BIT)) {
			/* CC MOS only
			 * set PR_SINK needed? */
			if ((mos_status & SBU1_MOS_BIT) ||
					(mos_status & SBU2_MOS_BIT)) {
				fusb251_set_state(fusb,
						FUSB251_STATE_MOS_DETECTECT);
			} else {
				if (mos_status & FAULT_MASK) {
					schedule_delayed_work(&fusb->enable_mos_work,
							5 * HZ);
					dev_info(dev, "Moisture was detected on CC"
					"but not found on SBU\n");
				}
			}
		}
	}

	if (irq_state & DRY_CHG_INT_BIT) {
		dev_info(dev, "Dry Changed irq occured\n");
		fusb251_set_state(fusb, FUSB251_STATE_DRY);
		fusb251_enable_moisture_detect(fusb, true);
	}

	if (irq_state & OVP_INT_BIT) {
		dev_info(dev, "OVP irq occured\n");
		fusb251_ovp(fusb);
	}
	if (irq_state & OVP_REC_INT_BIT) {
		dev_info(dev, "OVP recovery irq occured\n");
		fusb251_ovp_recovery(fusb);
	}

	mutex_unlock(&fusb->irq_lock);

}

static void fusb251_init(struct fusb251 *fusb)
{
	struct i2c_client *client = fusb->client;
	struct device *dev = &client->dev;
	int ret = 0;

	dev_info(dev, "%s\n", __func__);

	ret = fusb251_read_reg(client, FUSB251_PRODUCT_ID);
	if ((ret & PRODUCT_ID_MASK) == 0x00) {
		fusb->rev = ret & REVISION_ID_MASK;
		dev_info(dev, "%s: PID: FUSB251UCX, REV_ID: %d\n",
				__func__, fusb->rev);
	}

	/* Should reset the Moisture Detection State Machine after booting? */
	fusb251_write_reg(client, FUSB251_RESET, MOS_RESET_BIT);

	/* Temporary Disable every interrupts for init FUSB251 */
	fusb251_write_reg(client, FUSB251_INTERRUPT_MASK,
			MASK_OVP_BIT | MASK_OVP_REC_BIT | MASK_MOS_DETECT_BIT |
			MASK_DRY_DETECT_BIT | MASK_CC1_TIMER_BIT |
			MASK_CC2_TIMER_BIT);

	/* Setting Dry Check Timer
	 * Initial Setting is 10sec */
	fusb251_write_reg(client, FUSB251_TIMER, TDRY_MASK & 0x7);

	/* Setting CC Settle Time and Number of ADC Read
	 * Initial Setting is 400 usec and 2 times */
	fusb251_write_reg(client, FUSB251_TIMER2, (CC_SETTLE_TIME_MASK & 0x0) |
			(NUM_ADC_READ_MASK & 0x4));

	/* Setting CC & SBU Moisture Detection Threshold
	 * Initial Setting is 747 kohm and 320kohm */
	fusb251_write_reg(client, FUSB251_THRESHOLD1,
			(CC_MOS_DET_THR_MASK & 0xd) |
			(SBU_MOS_DET_THR_MASK & 0x90));

	/* Setting Dry Check Threshold & SBU Floating Detection Threshold
	 * Initial Setting is 960 kohm and 700mv */
	fusb251_write_reg(client, FUSB251_THRESHOLD2,
			(VDRY_THR_MASK & 0xe) |
			(SBU_FLOAT_DET_THR_MASK & 0x60));

	/* Setting CC and SBU Switch Control
	 * Initial Setting is close all CC and SBU pins */
	fusb251_write_reg(client, FUSB251_SWITCH_CONTROL,
			CC_SW_CTRL_BIT | (SBU_SW_CTRL_MASK & 0x2));

	/* Setting Interrupt Mask to allow every interrupts
	 * FUSB251 has INTB pin to notify processor the status change
	 * in the device.
	 * Once INTB pin is triggered to low, the pin stays low
	 * until the interrupt register is read by processor.
	 * The interrupt maks registers are default disabled.
	 * If any interrupt mask bit is set, the corresponding interrupt
	 * register bit won't set, so INTB pin won't be triggered. */
	fusb251_write_reg(client, FUSB251_INTERRUPT_MASK, 0);

	/* Setting CC and SBU  Moisture Detection.
	 * If CC port is on DRP or Source mode, moisture inside connector
	 * can make leakage path from CC to SBU,
	 * which makes SBU can have float voltage similar to the shape of CC.
	 * So the SBU float voltage can be detected if moisture is present
	 * while CC is on toggle or SRC mode.
	 *
	 * SBU float voltage detection can be started
	 * with the EN_SBUFT register set.
	 * If the bit is set, FUSB251 starts monitoring voltage on both
	 * SBU1 and SBU2, and if the voltage on eigher port is the smae or
	 * above the threshold, he moisture_status register is set with
	 * an interrupt.
	 * With the interrupt, processor can turn off the CC and SBU
	 * switch path to protect from corrosion, or processor could further
	 * moisture check using force SBU detection.
	 *
	 * SBU float voltage detection also can be enabled when CC moisture
	 * detection result is timer expire, which can happen where there is
	 * no DRP toggle on CC.
	 *
	 * Once SBU float voltage detection is enabled, it keeps monitoring
	 * until moisture found.
	 *
	 * Force SBU detection is initiated by EN_SBU or Auto_EN_SBU bit.
	 * Auto_EN_SBU bit is for pre-set before moisture is detected,
	 * EN_SBU can be set at anytime when SBU moisture detection is needed.
	 * After either CC or SBU float voltage detection detected moisture,
	 * if Auto_EN_SBU bit is set, FUSB251 goes to moisture detection
	 * on SBU with the same way of CC oisture detection,
	 * using 1volt source with 320kohm pull up,
	 * which is secondary moisture detection using SBU.
	 *
	 * Force SBU detection makes open the both CC and SBU switches
	 * and detects moisture on SBU.
	 * The detection time of SBU force detection is 1msec.
	 *
	 * If moisture is present in connector and if source device provides
	 * VBUS, there could be OVP interrupt on SBU by leakage path
	 * between VBUS and SBU.
	 *
	 * */
	fusb251_write_reg(client, FUSB251_CONTROL, EN_DEBUG_ACC_DET_BIT);

}

static irqreturn_t fusb251_intb_irq(int irq, void *data)
{
	struct fusb251 *fusb = (struct fusb251 *)data;
	struct device *dev = &fusb->client->dev;

	if (!fusb) {
		pr_err("%s: called before init\n", __func__);
		return IRQ_HANDLED;
	}

	if (gpiod_get_value_cansleep(fusb->intb_desc)) {
		dev_info(dev, "INTB irq occured\n");
		if (!queue_work(fusb->wq, &fusb->i2c_work))
			dev_err(dev, "%s: can't alloc work\n", __func__);
	}

	return IRQ_HANDLED;
}

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct fusb251 *fusb = i2c_get_clientdata(__client);
	int reset = 0;

	sscanf(buf, "%d", &reset);

	if (reset) {
		fusb251_write_reg(__client, FUSB251_RESET, MOS_RESET_BIT);
		mdelay(10);

		fusb251_init(fusb);
		fusb251_enable_moisture_detect(fusb, true);

		fusb251_set_state(fusb, FUSB251_STATE_DRY);

		dev_info(dev, "Reset and re-init FUSB251\n");
	}

	return size;
}
static DEVICE_ATTR(reset, S_IWUSR | S_IWGRP, NULL, reset_store);

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

	dev_info(dev, "Write Control Reg:0x%x\n", control);

	fusb251_write_reg(__client, FUSB251_CONTROL, control);

	return size;
}
static DEVICE_ATTR(control_reg, S_IRUGO | S_IWUSR | S_IWGRP,
		control_reg_show, control_reg_store);

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

	dev_info(dev, "Write IRQ_MASK Reg:0x%x\n", irq_mask);

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

	 status = fusb251_read_reg(__client, FSUB251_MOISTURE_STATUS);

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

	dev_info(dev, "Write SBU_MOS_THR Reg:0x%x\n",
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

	dev_info(dev, "Write CC_MOS_THR Reg:0x%x\n",
			cc_thr & (unsigned int)CC_MOS_DET_THR_MASK);

	fusb251_write_reg(__client, FUSB251_THRESHOLD1,
			cc_thr & CC_MOS_DET_THR_MASK);
	return size;
}
static DEVICE_ATTR(cc_mos_thr, S_IRUGO | S_IWUSR | S_IWGRP,
		cc_mos_thr_show, cc_mos_thr_store);

static ssize_t dry_thr_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int dry_thr = 0;

	dry_thr = fusb251_read_reg(__client, FUSB251_THRESHOLD2);

	return sprintf(buf, "DRY_THR:0x%x\n",
			dry_thr & (unsigned int)VDRY_THR_MASK);
}

static ssize_t dry_thr_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t size)
{
	int dry_thr = 0;

	sscanf(buf, "%d", &dry_thr);

	dev_info(dev, "Write DRY_THR Reg:0x%x\n",
			dry_thr & (unsigned int)VDRY_THR_MASK);

	fusb251_write_reg(__client, FUSB251_THRESHOLD2,
			dry_thr & VDRY_THR_MASK);
	return size;
}
static DEVICE_ATTR(dry_thr, S_IRUGO | S_IWUSR | S_IWGRP,
		dry_thr_show, dry_thr_store);

static void fusb251_sysfs_init(struct device *dev, struct fusb251 *fusb)
{
	int ret = 0;

	ret = device_create_file(dev, &dev_attr_reset);
	if (ret) {
		device_remove_file(dev, &dev_attr_reset);
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

	ret = device_create_file(dev, &dev_attr_dry_thr);
	if (ret) {
		device_remove_file(dev, &dev_attr_dry_thr);
		return;
	}
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

	pr_info("%s\n", __func__);

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

	fusb->intb_desc = devm_gpiod_get(dev, "fusb251,intb", GPIOD_IN);
	if (IS_ERR(fusb->intb_desc)) {
		dev_err(dev, "couldn't get intb gpio_desc\n");
		fusb->intb_desc = NULL;
		goto skip_int;
	}

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
		ret = ret;
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

	INIT_WORK(&fusb->i2c_work, i2c_work);
	INIT_DELAYED_WORK(&fusb->enable_mos_work, enable_moisture_work);

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

	mutex_init(&fusb->irq_lock);
	/* is mutex in i2c comm needed? */
//	mutex_init(&fusb->i2c_lock);

	fusb251_init(fusb);
	fusb251_sysfs_init(dev, fusb);

	if (lge_get_factory_boot() || lge_get_laf_mode()) {
		dev_err(dev, "factory cable connected,"
				"disable moisture detection\n");
		is_factory_boot = true;
	} else {
		is_factory_boot = false;
	}

	fusb251_enable_moisture_detect(fusb, !is_factory_boot);

	if (!!gpiod_get_value_cansleep(fusb->intb_desc)) {
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


	if (fusb->intb_irq > 0)
		devm_free_irq(dev, fusb->intb_irq, fusb);

	if (fusb->wq)
		destroy_workqueue(fusb->wq);
	mutex_destroy(&fusb->irq_lock);

	i2c_set_clientdata(client, NULL);
	i2c_unregister_device(client);
	devm_kfree(dev, fusb);

	return 0;
}

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

/*
 * lge_sbu_switch Port Protection Switch driver
 *
 * Copyright (C) 2018 LG Electronics, Inc.
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
//#define DEBUG

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/usb/lge_sbu_switch.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>

#ifdef CONFIG_LGE_PM_VENEER_PSY
#include "../../power/supply/lge/veneer-primitives.h"
#endif

static unsigned long turn_on_delay_us = 100000;
module_param(turn_on_delay_us, ulong, 0644);

extern void fusb251_update_state(int n_oe, int sel);

struct lge_sbu_switch {
	struct device			*dev;

	/* GPIOs */
	struct gpio_desc		*oe_desc;
	int				oe;

	struct gpio_desc		*sel_desc;
	int				sel;

	struct gpio_desc		*ovp_desc;
	int				ovp_irq;
	int				ovp;

	/* Flags */
	atomic_t			flags[LGE_SBU_SWITCH_MODE_MAX];
	unsigned long			cur_flag;

	struct mutex			lock;
	struct list_head		inst_list;
};

static struct lge_sbu_switch *__lge_sbu_switch = NULL;

static char *flag_to_string(unsigned long flag)
{
	switch (flag) {
	case LGE_SBU_SWITCH_FLAG_SBU_DISABLE:	return "SBU Disable";
	case LGE_SBU_SWITCH_FLAG_SBU_MD:	return "SBU Moisture Detected";
	case LGE_SBU_SWITCH_FLAG_SBU_FACTORY_ID: return "Factory Cable ID";
	case LGE_SBU_SWITCH_FLAG_SBU_MD_ING:	return "SBU Moisture Detecting";
	case LGE_SBU_SWITCH_FLAG_EDGE_MD:	return "Edge Moisture Detected";
	case LGE_SBU_SWITCH_FLAG_SBU_AUX:	return "AUX";
	case LGE_SBU_SWITCH_FLAG_SBU_UART:	return "UART";
	case LGE_SBU_SWITCH_FLAG_SBU_USBID:	return "USB ID";
	case LGE_SBU_SWITCH_FLAG_EDGE_MD_ING:	return "Edge Moisture Detecting";
	default:			return "Unknown";
	}
}

static unsigned long find_next_flag_bit(unsigned long flags,
					unsigned long offset)
{
	unsigned long i;

	if (offset >= LGE_SBU_SWITCH_MODE_MAX ||
	    (!(flags & GENMASK(LGE_SBU_SWITCH_MODE_MAX - 1, offset))))
		return LGE_SBU_SWITCH_MODE_MAX;

	for (i = offset; i < LGE_SBU_SWITCH_MODE_MAX; i++) {
		if (flags & BIT(i))
			break;
	}

	return i;
}

static unsigned long find_first_flag_bit(unsigned long flags)
{
	return find_next_flag_bit(flags, 0UL);
}

#define for_each_flag_bit(bit, flags)				\
	for ((bit) = find_first_flag_bit(flags);		\
	     (bit) < LGE_SBU_SWITCH_MODE_MAX;			\
	     (bit) = find_next_flag_bit(flags, (bit) + 1))

static unsigned long update_state(struct lge_sbu_switch *lge_sbu_switch)
{
	int oe, sel;
	unsigned i;

	for (i = 0; i < LGE_SBU_SWITCH_MODE_MAX; i++) {
		if (atomic_read(&lge_sbu_switch->flags[i]))
			break;
	}

	if (lge_sbu_switch->cur_flag == BIT(i))
		return lge_sbu_switch->cur_flag;

	switch (i) {
	case LGE_SBU_SWITCH_MODE_SBU_DISABLE:
		oe = 1;
		sel = lge_sbu_switch->sel;
		break;
	case LGE_SBU_SWITCH_MODE_SBU_FACTORY_ID:
		oe = 0;
		sel = 1;
		break;
	case LGE_SBU_SWITCH_MODE_SBU_MD:
	case LGE_SBU_SWITCH_MODE_SBU_MD_ING:
		oe = 0;
		sel = 1;
		break;
	case LGE_SBU_SWITCH_MODE_EDGE_MD:
	case LGE_SBU_SWITCH_MODE_EDGE_MD_ING:
		oe = 0;
		sel = 0;
		break;
	case LGE_SBU_SWITCH_MODE_SBU_AUX:
	case LGE_SBU_SWITCH_MODE_SBU_UART:
		oe = 0;
		sel = 0;
		break;
	case LGE_SBU_SWITCH_MODE_SBU_USBID:
		oe = 0;
		sel = 1;
		break;
	default:
		oe = 1;
		sel = lge_sbu_switch->sel;
		break;
	}

#ifdef CONFIG_LGE_USB_MOISTURE_FUSB251
	if (lge_sbu_switch->oe != oe || lge_sbu_switch->sel != sel) {
		if (lge_sbu_switch->sel != sel)
			gpiod_set_value(lge_sbu_switch->sel_desc, sel);
		fusb251_update_state(oe, sel);
	}

	if (i == LGE_SBU_SWITCH_MODE_SBU_USBID)
		usleep_range(turn_on_delay_us, turn_on_delay_us);
#else
	if (oe) {
		if (lge_sbu_switch->oe != oe)
			gpiod_set_value(lge_sbu_switch->oe_desc, oe);
		if (lge_sbu_switch->sel != sel)
			gpiod_set_value(lge_sbu_switch->sel_desc, sel);
	} else {
		if (lge_sbu_switch->sel != sel)
			gpiod_set_value(lge_sbu_switch->sel_desc, sel);
		if (lge_sbu_switch->oe != oe)
			gpiod_set_value(lge_sbu_switch->oe_desc, oe);

		/*
		 * lge_sbu_switch Turn-On Time, S, /OE to output. Max 400us
		 */
		if (sel && ((lge_sbu_switch->oe != oe) || (lge_sbu_switch->sel != sel)))
			usleep_range(turn_on_delay_us, turn_on_delay_us);
	}
#endif
	lge_sbu_switch->oe = oe;
	lge_sbu_switch->sel = sel;
	lge_sbu_switch->cur_flag = BIT(i);

	dev_info(lge_sbu_switch->dev, "/OE(%d), SEL(%d), current flag is \"%s\"\n",
		 oe, sel, flag_to_string(lge_sbu_switch->cur_flag));

	return BIT(i);
}

int lge_sbu_switch_get(struct lge_sbu_switch_instance *inst, unsigned long flag)
{
	struct lge_sbu_switch *lge_sbu_switch = __lge_sbu_switch;
	unsigned long flag_bit;

	if (!inst || flag >= LGE_SBU_SWITCH_FLAG_MAX || !(inst->desc->flags & flag))
		return -EINVAL;

	dev_dbg(lge_sbu_switch->dev, "%s get \"%s\"\n",
		dev_driver_string(inst->dev), flag_to_string(flag));

	mutex_lock(&lge_sbu_switch->lock);

	flag_bit = find_first_flag_bit(flag);
	if (test_and_set_bit(flag_bit, &inst->flags))
		goto out;

	atomic_inc(&lge_sbu_switch->flags[flag_bit]);
	update_state(lge_sbu_switch);
out:
	mutex_unlock(&lge_sbu_switch->lock);

	return lge_sbu_switch->cur_flag == flag ? 0 : -EBUSY;
}
EXPORT_SYMBOL(lge_sbu_switch_get);

int lge_sbu_switch_put(struct lge_sbu_switch_instance *inst, unsigned long flag)
{
	struct lge_sbu_switch *lge_sbu_switch = __lge_sbu_switch;
	unsigned long flag_bit;

	if (!inst || flag >= LGE_SBU_SWITCH_FLAG_MAX || !(inst->desc->flags & flag))
		return -EINVAL;

	dev_dbg(lge_sbu_switch->dev, "%s put \"%s\"\n",
		dev_driver_string(inst->dev), flag_to_string(flag));

	mutex_lock(&lge_sbu_switch->lock);

	flag_bit = find_first_flag_bit(flag);
	if (!test_and_clear_bit(flag_bit, &inst->flags))
		goto out;

	atomic_dec(&lge_sbu_switch->flags[flag_bit]);
	update_state(lge_sbu_switch);
out:
	mutex_unlock(&lge_sbu_switch->lock);

	return lge_sbu_switch->cur_flag == flag ? 0 : -EBUSY;
}
EXPORT_SYMBOL(lge_sbu_switch_put);

unsigned long lge_sbu_switch_get_current_flag(struct lge_sbu_switch_instance *inst)
{
	struct lge_sbu_switch *lge_sbu_switch = __lge_sbu_switch;
	unsigned long cur_flag;

	if (!lge_sbu_switch || !inst)
		return LGE_SBU_SWITCH_FLAG_MAX;

	mutex_lock(&lge_sbu_switch->lock);
	cur_flag = lge_sbu_switch->cur_flag;
	mutex_unlock(&lge_sbu_switch->lock);

	dev_dbg(lge_sbu_switch->dev, "current flag is \"%s\"\n",
		flag_to_string(cur_flag));

	return cur_flag;
}
EXPORT_SYMBOL(lge_sbu_switch_get_current_flag);

static irqreturn_t lge_sbu_switch_ovp_irq_thread(int irq, void *data)
{
	struct lge_sbu_switch *lge_sbu_switch = (struct lge_sbu_switch *)data;
	int ovp = 0;
	struct list_head *pos;
	struct list_head *tmp;

	if (lge_sbu_switch->ovp_desc)
		ovp = !gpiod_get_value(lge_sbu_switch->ovp_desc);
	dev_dbg(lge_sbu_switch->dev, "ovp old(%d), new(%d)\n", lge_sbu_switch->ovp, ovp);

	if (ovp == lge_sbu_switch->ovp)
		return IRQ_HANDLED;

	dev_info(lge_sbu_switch->dev, "ovp (%d)\n", ovp);
	lge_sbu_switch->ovp = ovp;

#ifdef CONFIG_LGE_PM_VENEER_PSY
	if (ovp) {
		// CC-ov interrupt is bound to the falling edge of SDM-GPIO-79
		struct power_supply* veneer = power_supply_get_by_name("veneer");

		pr_info("Overvolatge on CC detected (true)\n");

		if (veneer) {
			union power_supply_propval
				vccover = { .intval = EXCEPTION_WIRED_VCCOVER, };
			power_supply_set_property(veneer,
				POWER_SUPPLY_PROP_CHARGE_NOW_ERROR, &vccover);
			power_supply_put(veneer);
		}
	}
#endif

	list_for_each_safe(pos, tmp, &lge_sbu_switch->inst_list) {
		struct lge_sbu_switch_instance *inst = container_of(pos,
			struct lge_sbu_switch_instance, list);

		if (inst->desc->ovp_callback)
			inst->desc->ovp_callback(inst, ovp);
	}

	return IRQ_HANDLED;
}

int lge_sbu_switch_get_ovp_state(struct lge_sbu_switch_instance *inst)
{
	struct lge_sbu_switch *lge_sbu_switch = __lge_sbu_switch;

	if (!lge_sbu_switch || !inst || !lge_sbu_switch->ovp_desc)
		return -EINVAL;

	return lge_sbu_switch->ovp;
}
EXPORT_SYMBOL(lge_sbu_switch_get_ovp_state);

void devm_lge_sbu_switch_instance_unregister(struct device *dev,
				      struct lge_sbu_switch_instance *inst)
{
	struct lge_sbu_switch *lge_sbu_switch = __lge_sbu_switch;
	unsigned long flag_bit;

	if (!dev || !inst || dev != inst->dev)
		return;

	dev_dbg(lge_sbu_switch->dev, "%s unregister\n", dev_driver_string(dev));

	for_each_flag_bit(flag_bit, inst->flags)
		lge_sbu_switch_put(inst, BIT(flag_bit));

	list_del(&inst->list);
	kfree(inst);
}
EXPORT_SYMBOL(devm_lge_sbu_switch_instance_unregister);

struct lge_sbu_switch_instance *__must_check
devm_lge_sbu_switch_instance_register(struct device *dev,
			       const struct lge_sbu_switch_desc *desc)
{
	struct lge_sbu_switch *lge_sbu_switch = __lge_sbu_switch;
	struct lge_sbu_switch_instance *inst;
	unsigned long flag_bit;

	if (!lge_sbu_switch)
		return NULL;

	if (!dev || desc->flags >= LGE_SBU_SWITCH_FLAG_MAX)
		return ERR_PTR(-EINVAL);

	dev_dbg(lge_sbu_switch->dev, "%s register\n", dev_driver_string(dev));

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return ERR_PTR(-ENOMEM);

	inst->dev = dev;
	inst->desc = desc;

	INIT_LIST_HEAD(&inst->list);
	list_add(&inst->list, &lge_sbu_switch->inst_list);

	for_each_flag_bit(flag_bit, desc->flags) {
		dev_info(lge_sbu_switch->dev, "%s registered \"%s\"\n",
			 dev_driver_string(dev), flag_to_string(BIT(flag_bit)));
	}

	return inst;
}
EXPORT_SYMBOL(devm_lge_sbu_switch_instance_register);

static int lge_sbu_switch_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lge_sbu_switch *lge_sbu_switch;
	struct pinctrl* gpio_pinctrl;
	struct pinctrl_state* gpio_state;
	int i, ret;

	dev_info(&pdev->dev, "%s\n", __func__);

	lge_sbu_switch = devm_kzalloc(dev, sizeof(*lge_sbu_switch), GFP_KERNEL);
	if (!lge_sbu_switch) {
		dev_err(dev, "out of memory\n");
		return -ENOMEM;
	}

	lge_sbu_switch->dev = &pdev->dev;

	lge_sbu_switch->oe = 1;
	lge_sbu_switch->sel = 0;

	lge_sbu_switch->oe_desc = devm_gpiod_get(dev, "lge,oe",
				  lge_sbu_switch->oe ? GPIOD_OUT_HIGH : GPIOD_OUT_LOW);
	if (IS_ERR(lge_sbu_switch->oe_desc)) {
		dev_err(dev, "couldn't get oe gpio_desc\n");
		lge_sbu_switch->oe_desc = NULL;
		lge_sbu_switch->oe = 0;
	}

	lge_sbu_switch->sel_desc = devm_gpiod_get(dev, "lge,sel",
				  lge_sbu_switch->sel ? GPIOD_OUT_HIGH : GPIOD_OUT_LOW);
	if (IS_ERR(lge_sbu_switch->sel_desc)) {
		dev_err(dev, "couldn't get sel gpio_desc\n");
		return PTR_ERR(lge_sbu_switch->sel_desc);
	}

	lge_sbu_switch->ovp_desc = devm_gpiod_get(dev, "lge,ovp", GPIOD_IN);
	if (IS_ERR(lge_sbu_switch->ovp_desc)) {
		dev_err(dev, "couldn't get ovp gpio_desc\n");
		lge_sbu_switch->ovp_desc = NULL;
		goto skip_ovp;
	}
	lge_sbu_switch->ovp_irq = gpiod_to_irq(lge_sbu_switch->ovp_desc);

	ret = request_threaded_irq(lge_sbu_switch->ovp_irq,
		NULL, lge_sbu_switch_ovp_irq_thread,
		IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"lge_sbu_switch-ovp", lge_sbu_switch);
	if (ret) {
		dev_err(dev, "Cannot request ovp irq\n");
		return ret;
	}
	enable_irq_wake(lge_sbu_switch->ovp_irq);
skip_ovp:

	gpio_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(gpio_pinctrl)) {
		dev_err(dev, "Failed to get pinctrl (%ld)\n", PTR_ERR(gpio_pinctrl));
		return PTR_ERR(gpio_pinctrl);
	}

	gpio_state = pinctrl_lookup_state(gpio_pinctrl, "default");
	if (IS_ERR_OR_NULL(gpio_state)) {
		dev_err(dev, "pinstate not found, %ld\n", PTR_ERR(gpio_state));
		return PTR_ERR(gpio_state);
	}

	ret = pinctrl_select_state(gpio_pinctrl, gpio_state);
	if (ret < 0) {
		dev_err(dev, "cannot set pins %d\n", ret);
		return ret;
	}
	mutex_init(&lge_sbu_switch->lock);
	INIT_LIST_HEAD(&lge_sbu_switch->inst_list);

	for (i = 0; i < LGE_SBU_SWITCH_MODE_MAX; i++)
		atomic_set(&lge_sbu_switch->flags[i], 0);

	__lge_sbu_switch = lge_sbu_switch;

	return 0;
}

static const struct of_device_id lge_sbu_switch_match_table[] = {
	{ .compatible = "lge,lge_sbu_switch" },
	{ }
};
MODULE_DEVICE_TABLE(of, lge_sbu_switch_match_table);

static struct platform_driver lge_sbu_switch_driver = {
	.driver = {
		.name = "lge_sbu_switch",
		.of_match_table = lge_sbu_switch_match_table,
	},
	.probe = lge_sbu_switch_probe,
};
module_platform_driver(lge_sbu_switch_driver);

MODULE_DESCRIPTION("LGE CC/SBU Protection Switch driver");
MODULE_LICENSE("GPL v2");

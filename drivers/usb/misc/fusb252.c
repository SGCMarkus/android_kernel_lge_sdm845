/*
 * FUSB252 Port Protection Switch driver
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
#include <linux/usb/fusb252.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>

#ifdef CONFIG_LGE_PM_VENEER_PSY
#include "../../power/supply/lge/veneer-primitives.h"
#endif

static unsigned long turn_on_delay_us = 100000;
module_param(turn_on_delay_us, ulong, 0644);

struct fusb252 {
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
	atomic_t			flags[FUSB252_MODE_MAX];
	unsigned long			cur_flag;

	struct mutex			lock;
	struct list_head		inst_list;
};

static struct fusb252 *__fusb252 = NULL;

static char *flag_to_string(unsigned long flag)
{
	switch (flag) {
	case FUSB252_FLAG_SBU_DISABLE:	return "SBU Disable";
	case FUSB252_FLAG_SBU_MD:	return "SBU Moisture Detected";
	case FUSB252_FLAG_SBU_FACTORY_ID: return "Factory Cable ID";
	case FUSB252_FLAG_SBU_MD_ING:	return "SBU Moisture Detecting";
	case FUSB252_FLAG_EDGE_MD:	return "Edge Moisture Detected";
	case FUSB252_FLAG_SBU_AUX:	return "AUX";
	case FUSB252_FLAG_SBU_UART:	return "UART";
	case FUSB252_FLAG_SBU_USBID:	return "USB ID";
	case FUSB252_FLAG_EDGE_MD_ING:	return "Edge Moisture Detecting";
	default:			return "Unknown";
	}
}

static unsigned long find_next_flag_bit(unsigned long flags,
					unsigned long offset)
{
	unsigned long i;

	if (offset >= FUSB252_MODE_MAX ||
	    (!(flags & GENMASK(FUSB252_MODE_MAX - 1, offset))))
		return FUSB252_MODE_MAX;

	for (i = offset; i < FUSB252_MODE_MAX; i++) {
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
	     (bit) < FUSB252_MODE_MAX;			\
	     (bit) = find_next_flag_bit(flags, (bit) + 1))

static unsigned long update_state(struct fusb252 *fusb252)
{
	int oe, sel;
	unsigned i;

	for (i = 0; i < FUSB252_MODE_MAX; i++) {
		if (atomic_read(&fusb252->flags[i]))
			break;
	}

	if (fusb252->cur_flag == BIT(i))
		return fusb252->cur_flag;

	switch (i) {
	case FUSB252_MODE_SBU_DISABLE:
		oe = 1;
		sel = fusb252->sel;
		break;
	case FUSB252_MODE_SBU_FACTORY_ID:
		oe = 0;
		sel = 1;
		break;
	case FUSB252_MODE_SBU_MD:
	case FUSB252_MODE_SBU_MD_ING:
		oe = 0;
		sel = 1;
		break;
	case FUSB252_MODE_EDGE_MD:
	case FUSB252_MODE_EDGE_MD_ING:
		oe = 0;
		sel = 0;
		break;
	case FUSB252_MODE_SBU_AUX:
	case FUSB252_MODE_SBU_UART:
		oe = 0;
		sel = 0;
		break;
	case FUSB252_MODE_SBU_USBID:
		oe = 0;
		sel = 1;
		break;
	default:
		oe = 1;
		sel = fusb252->sel;
		break;
	}

	if (oe) {
		if (fusb252->oe != oe)
			gpiod_set_value(fusb252->oe_desc, oe);
		if (fusb252->sel != sel)
			gpiod_set_value(fusb252->sel_desc, sel);
	} else {
		if (fusb252->sel != sel)
			gpiod_set_value(fusb252->sel_desc, sel);
		if (fusb252->oe != oe)
			gpiod_set_value(fusb252->oe_desc, oe);

		/*
		 * FUSB252 Turn-On Time, S, /OE to output. Max 400us
		 */
		if (sel && ((fusb252->oe != oe) || (fusb252->sel != sel)))
			usleep_range(turn_on_delay_us, turn_on_delay_us);
	}

	fusb252->oe = oe;
	fusb252->sel = sel;
	fusb252->cur_flag = BIT(i);

	dev_info(fusb252->dev, "/OE(%d), SEL(%d), current flag is \"%s\"\n",
		 oe, sel, flag_to_string(fusb252->cur_flag));

	return BIT(i);
}

int fusb252_get(struct fusb252_instance *inst, unsigned long flag)
{
	struct fusb252 *fusb252 = __fusb252;
	unsigned long flag_bit;

	if (!inst || flag >= FUSB252_FLAG_MAX || !(inst->desc->flags & flag))
		return -EINVAL;

	dev_dbg(fusb252->dev, "%s get \"%s\"\n",
		dev_driver_string(inst->dev), flag_to_string(flag));

	mutex_lock(&fusb252->lock);

	flag_bit = find_first_flag_bit(flag);
	if (test_and_set_bit(flag_bit, &inst->flags))
		goto out;

	atomic_inc(&fusb252->flags[flag_bit]);
	update_state(fusb252);
out:
	mutex_unlock(&fusb252->lock);

	return fusb252->cur_flag == flag ? 0 : -EBUSY;
}
EXPORT_SYMBOL(fusb252_get);

int fusb252_put(struct fusb252_instance *inst, unsigned long flag)
{
	struct fusb252 *fusb252 = __fusb252;
	unsigned long flag_bit;

	if (!inst || flag >= FUSB252_FLAG_MAX || !(inst->desc->flags & flag))
		return -EINVAL;

	dev_dbg(fusb252->dev, "%s put \"%s\"\n",
		dev_driver_string(inst->dev), flag_to_string(flag));

	mutex_lock(&fusb252->lock);

	flag_bit = find_first_flag_bit(flag);
	if (!test_and_clear_bit(flag_bit, &inst->flags))
		goto out;

	atomic_dec(&fusb252->flags[flag_bit]);
	update_state(fusb252);
out:
	mutex_unlock(&fusb252->lock);

	return fusb252->cur_flag == flag ? 0 : -EBUSY;
}
EXPORT_SYMBOL(fusb252_put);

unsigned long fusb252_get_current_flag(struct fusb252_instance *inst)
{
	struct fusb252 *fusb252 = __fusb252;
	unsigned long cur_flag;

	if (!fusb252 || !inst)
		return FUSB252_FLAG_MAX;

	mutex_lock(&fusb252->lock);
	cur_flag = fusb252->cur_flag;
	mutex_unlock(&fusb252->lock);

	dev_dbg(fusb252->dev, "current flag is \"%s\"\n",
		flag_to_string(cur_flag));

	return cur_flag;
}
EXPORT_SYMBOL(fusb252_get_current_flag);

static irqreturn_t fusb252_ovp_irq_thread(int irq, void *data)
{
	struct fusb252 *fusb252 = (struct fusb252 *)data;
	int ovp;
	struct list_head *pos;
	struct list_head *tmp;

	ovp = !gpiod_get_value(fusb252->ovp_desc);
	dev_dbg(fusb252->dev, "ovp old(%d), new(%d)\n", fusb252->ovp, ovp);

	if (ovp == fusb252->ovp)
		return IRQ_HANDLED;

	dev_info(fusb252->dev, "ovp (%d)\n", ovp);
	fusb252->ovp = ovp;

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

	list_for_each_safe(pos, tmp, &fusb252->inst_list) {
		struct fusb252_instance *inst = container_of(pos,
			struct fusb252_instance, list);

		if (inst->desc->ovp_callback)
			inst->desc->ovp_callback(inst, ovp);
	}

	return IRQ_HANDLED;
}

int fusb252_get_ovp_state(struct fusb252_instance *inst)
{
	struct fusb252 *fusb252 = __fusb252;

	if (!fusb252 || !inst)
		return -EINVAL;

	return fusb252->ovp;
}
EXPORT_SYMBOL(fusb252_get_ovp_state);

void devm_fusb252_instance_unregister(struct device *dev,
				      struct fusb252_instance *inst)
{
	struct fusb252 *fusb252 = __fusb252;
	unsigned long flag_bit;

	if (!dev || !inst || dev != inst->dev)
		return;

	dev_dbg(fusb252->dev, "%s unregister\n", dev_driver_string(dev));

	for_each_flag_bit(flag_bit, inst->flags)
		fusb252_put(inst, BIT(flag_bit));

	list_del(&inst->list);
	kfree(inst);
}
EXPORT_SYMBOL(devm_fusb252_instance_unregister);

struct fusb252_instance *__must_check
devm_fusb252_instance_register(struct device *dev,
			       const struct fusb252_desc *desc)
{
	struct fusb252 *fusb252 = __fusb252;
	struct fusb252_instance *inst;
	unsigned long flag_bit;

	if (!fusb252)
		return NULL;

	if (!dev || desc->flags >= FUSB252_FLAG_MAX)
		return ERR_PTR(-EINVAL);

	dev_dbg(fusb252->dev, "%s register\n", dev_driver_string(dev));

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return ERR_PTR(-ENOMEM);

	inst->dev = dev;
	inst->desc = desc;

	list_add(&inst->list, &fusb252->inst_list);

	for_each_flag_bit(flag_bit, desc->flags) {
		dev_info(fusb252->dev, "%s registered \"%s\"\n",
			 dev_driver_string(dev), flag_to_string(BIT(flag_bit)));
	}

	return inst;
}
EXPORT_SYMBOL(devm_fusb252_instance_register);

static int fusb252_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fusb252 *fusb252;
	struct pinctrl* gpio_pinctrl;
	struct pinctrl_state* gpio_state;
	int i, ret;

	dev_info(&pdev->dev, "%s\n", __func__);

	fusb252 = devm_kzalloc(dev, sizeof(*fusb252), GFP_KERNEL);
	if (!fusb252) {
		dev_err(dev, "out of memory\n");
		return -ENOMEM;
	}

	fusb252->dev = &pdev->dev;

	fusb252->oe = 1;
	fusb252->sel = 0;

	fusb252->oe_desc = devm_gpiod_get(dev, "lge,oe",
				  fusb252->oe ? GPIOD_OUT_HIGH : GPIOD_OUT_LOW);
	if (IS_ERR(fusb252->oe_desc)) {
		dev_err(dev, "couldn't get oe gpio_desc\n");
		return PTR_ERR(fusb252->oe_desc);
	}

	fusb252->sel_desc = devm_gpiod_get(dev, "lge,sel",
				  fusb252->sel ? GPIOD_OUT_HIGH : GPIOD_OUT_LOW);
	if (IS_ERR(fusb252->sel_desc)) {
		dev_err(dev, "couldn't get sel gpio_desc\n");
		return PTR_ERR(fusb252->sel_desc);
	}

	fusb252->ovp_desc = devm_gpiod_get(dev, "lge,ovp", GPIOD_IN);
	if (IS_ERR(fusb252->ovp_desc)) {
		dev_err(dev, "couldn't get ovp gpio_desc\n");
		return PTR_ERR(fusb252->ovp_desc);
	}
	fusb252->ovp_irq = gpiod_to_irq(fusb252->ovp_desc);

	ret = request_threaded_irq(fusb252->ovp_irq,
		NULL, fusb252_ovp_irq_thread,
		IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"fusb252-ovp", fusb252);
	if (ret) {
		dev_err(dev, "Cannot request ovp irq\n");
		return ret;
	}
	enable_irq_wake(fusb252->ovp_irq);

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

	mutex_init(&fusb252->lock);
	INIT_LIST_HEAD(&fusb252->inst_list);

	for (i = 0; i < FUSB252_MODE_MAX; i++)
		atomic_set(&fusb252->flags[i], 0);

	__fusb252 = fusb252;

	return 0;
}

static const struct of_device_id fusb252_match_table[] = {
	{ .compatible = "lge,fusb252" },
	{ }
};
MODULE_DEVICE_TABLE(of, fusb252_match_table);

static struct platform_driver fusb252_driver = {
	.driver = {
		.name = "fusb252",
		.of_match_table = fusb252_match_table,
	},
	.probe = fusb252_probe,
};
module_platform_driver(fusb252_driver);

MODULE_DESCRIPTION("FUSB252 Port Protection Switch driver");
MODULE_LICENSE("GPL v2");

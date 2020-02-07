/*
 * Copyright (c) 2018, LG Eletronics. All rights reserved.
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
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#define LRMC_DEV_NAME "lge,lrmc"

// BOB Mode define
#define PMI_BOB_AUTO_MODE 0
#define PMI_BOB_PWM_MODE  1
#define PMI_BOB_PASS_MODE 2

// Default BOB Setting
#define DEFAULT_BOB_MIN_VOLT 3390000
#define DEFAULT_BOB_MAX_VOLT 3410000
#define DEFAULT_BOB_AUTO_THRESHOLD 100000
#define DEFAULT_BOB_PWM_THRESHOLD  200000


struct lrmc_dev {
	struct device *dev;
	struct regulator *bob_vreg;
	int bob_current_mode;
	uint32_t bob_min_voltage;
	uint32_t bob_max_voltage;
	uint32_t bob_auto_threshold;
	uint32_t bob_pwm_threshold;
};

static struct lrmc_dev *lrmcdata;

static ssize_t bob_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int input;
	int ret = 0;

	ret = sscanf(buf, "%d", &input);
	if(ret < 0){
		pr_err("%s Invailed argument\n", __func__);
		return -EINVAL;
	}

	pr_err("%s input = %d\n", __func__, input);

	if(input > 0){
		if(lrmcdata->bob_current_mode != PMI_BOB_PWM_MODE){

			ret = regulator_set_voltage(lrmcdata->bob_vreg,
					lrmcdata->bob_min_voltage, lrmcdata->bob_max_voltage);
			if(ret < 0){
				pr_err("%s regulator_set_voltage err = %d\n",__func__, ret);
				return count;
			}

			ret = regulator_set_load(lrmcdata->bob_vreg, lrmcdata->bob_pwm_threshold);
			if(ret < 0){
				pr_err("%s regulator_set_load err = %d\n",__func__, ret);
				return count;
			}

			ret = regulator_enable(lrmcdata->bob_vreg);
			if(ret < 0){
				pr_err("%s regulator_enable err = %d\n",__func__, ret);
				return count;
			}

			lrmcdata->bob_current_mode = PMI_BOB_PWM_MODE;
		}else{
			pr_err("%s already BOB is PWM mode\n",__func__);
		}
	}else if(input == 0){
		if(lrmcdata->bob_current_mode != PMI_BOB_AUTO_MODE){
			ret = regulator_set_load(lrmcdata->bob_vreg, lrmcdata->bob_auto_threshold);
			if(ret < 0){
				pr_err("%s regulator_set_load 0 err = %d\n",__func__, ret);
				return count;
			}

			ret = regulator_disable(lrmcdata->bob_vreg);
			if(ret < 0){
				pr_err("%s regulator_disable err  = %d\n",__func__, ret);
				return count;
			}

			lrmcdata->bob_current_mode = PMI_BOB_AUTO_MODE;
		}else{
			pr_err("%s already BOB is AUTO mode\n",__func__);
		}

	}else{
		pr_err("%s invalid input = %d\n",__func__, input);
	}

	pr_err("%s Final BOB Mode = %d\n",__func__, lrmcdata->bob_current_mode);
	return count;
}

static ssize_t bob_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	pr_err("%s bob_current_mode = %d\n", __func__, lrmcdata->bob_current_mode);

	return sprintf(buf, "%d\n", lrmcdata->bob_current_mode);
}

static DEVICE_ATTR(bob_mode, 0600, bob_mode_show, bob_mode_store);

static int lge_regulator_mode_change_parse_dt(struct device *dev, struct device_node *node)
{
	int ret = 0;

	ret = of_property_read_u32(node, "bob_min_voltage", &lrmcdata->bob_min_voltage);
	if(ret < 0){
		lrmcdata->bob_min_voltage = DEFAULT_BOB_MIN_VOLT;
		pr_err("%s dt parsing err : set DEFAULT_BOB_MIN_VOLT\n", __func__);
	}

	ret = of_property_read_u32(node, "bob_max_voltage", &lrmcdata->bob_max_voltage);
	if(ret < 0){
		lrmcdata->bob_max_voltage = DEFAULT_BOB_MAX_VOLT;
		pr_err("%s dt parsing err : set DEFAULT_BOB_MAX_VOLT\n", __func__);
	}

	ret = of_property_read_u32(node, "bob_auto_threshold", &lrmcdata->bob_auto_threshold);
	if(ret < 0){
		lrmcdata->bob_auto_threshold = DEFAULT_BOB_AUTO_THRESHOLD;
		pr_err("%s dt parsing err : set DEFAULT_BOB_AUTO_THRESHOLD\n", __func__);
	}

	ret = of_property_read_u32(node, "bob_pwm_threshold", &lrmcdata->bob_pwm_threshold);
	if(ret < 0){
		lrmcdata->bob_pwm_threshold = DEFAULT_BOB_PWM_THRESHOLD;
		pr_err("%s dt parsing err : set DEFAULT_BOB_PWM_THRESHOLD\n", __func__);
	}

	pr_err("%s bob_min_voltage = %u\n", __func__, lrmcdata->bob_min_voltage);
	pr_err("%s bob_max_voltage = %u\n", __func__, lrmcdata->bob_max_voltage);
	pr_err("%s bob_auto_threshold = %u\n", __func__, lrmcdata->bob_auto_threshold);
	pr_err("%s bob_pwm_threshold = %u\n", __func__, lrmcdata->bob_pwm_threshold);

	return 0;
}

void bob_mode_enable(void)
{
	int ret = 0;
	pr_err("%s bob_current_mode = %d\n", __func__, lrmcdata->bob_current_mode); 

	if(lrmcdata->bob_current_mode != PMI_BOB_PWM_MODE){
			ret = regulator_set_voltage(lrmcdata->bob_vreg,
					lrmcdata->bob_min_voltage, lrmcdata->bob_max_voltage);
			if(ret < 0){
				pr_err("%s regulator_set_voltage err = %d\n",__func__, ret);
			}

			ret = regulator_set_load(lrmcdata->bob_vreg, lrmcdata->bob_pwm_threshold);
			if(ret < 0){
				pr_err("%s regulator_set_load err = %d\n",__func__, ret);
			}

			ret = regulator_enable(lrmcdata->bob_vreg);
			if(ret < 0){
				pr_err("%s regulator_enable err = %d\n",__func__, ret);
			}

			lrmcdata->bob_current_mode = PMI_BOB_PWM_MODE;
			pr_err("%s set bob_current_mode = PMI_BOB_PWM_MODE\n", __func__);
		}
}
EXPORT_SYMBOL(bob_mode_enable);

void bob_mode_disable(void)
{
	int ret = 0;
	pr_err("%s bob_current_mode = %d\n", __func__, lrmcdata->bob_current_mode);

	if(lrmcdata->bob_current_mode != PMI_BOB_AUTO_MODE){
		ret = regulator_set_load(lrmcdata->bob_vreg, lrmcdata->bob_auto_threshold);
		if(ret < 0){
			pr_err("%s regulator_set_load AUTO err = %d\n",__func__, ret);
		}
	
		ret = regulator_disable(lrmcdata->bob_vreg);
		if(ret < 0){
			pr_err("%s regulator_disable err  = %d\n",__func__, ret);
		}
	
		lrmcdata->bob_current_mode = PMI_BOB_AUTO_MODE;
		pr_err("%s set bob_current_mode = PMI_BOB_AUTO_MODE\n", __func__);
	}
}
EXPORT_SYMBOL(bob_mode_disable);

static int lge_regulator_mode_change_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;

	pr_err("%s start\n", __func__);

	if (!pdev->dev.of_node){
		pr_err("%s no device of_node\n", __func__);
		return -ENODEV;
	}
	lrmcdata = devm_kzalloc(dev, sizeof(struct lrmc_dev),
			GFP_KERNEL);
	if (!lrmcdata){
		pr_err("%s no driver data\n", __func__);
		return -EIO;
	}

	ret = device_create_file(dev, &dev_attr_bob_mode);
	if(ret){
		pr_err("%s Cannot create sysfs\n", __func__);
		return ret;
	}


	lrmcdata->bob_vreg = devm_regulator_get(dev, "lrmc_bob_vreg");
	if (IS_ERR(lrmcdata->bob_vreg)) {
		pr_err("%s bob_vreg cannot be get\n", __func__);
		return PTR_ERR(lrmcdata->bob_vreg);
	}

	ret = lge_regulator_mode_change_parse_dt(dev, node);
	if(ret < 0){
		pr_err("%s dt parsing error err = %d\n", __func__, ret);
		goto exit;
	}

	lrmcdata->dev = dev;

	pr_err("%s done\n", __func__);
	return ret;

exit:
	return ret;
}

static int lge_regulator_mode_change_suspend(struct device *dev)
{
	int ret = 0;

	if(lrmcdata->bob_current_mode != PMI_BOB_PWM_MODE){
		ret = regulator_set_load(lrmcdata->bob_vreg, 0);
		if(ret < 0){
			pr_err("%s regulator_set_load err = %d\n", __func__, ret);
			pr_err("%s bob mode = %d\n",__func__, lrmcdata->bob_current_mode);
			return 0;
		}

		if(regulator_is_enabled(lrmcdata->bob_vreg)){
			ret = regulator_disable(lrmcdata->bob_vreg);
			if(ret < 0){
				pr_err("%s regulator_disable err = %d\n", __func__, ret);
				pr_err("%s bob mode = %d\n",__func__, lrmcdata->bob_current_mode);
				return 0;
			}
			pr_err("%s regulator_disable\n", __func__);
		}

		lrmcdata->bob_current_mode = PMI_BOB_PASS_MODE;
	}
	pr_err("%s done : bob_mode = %d\n", __func__, lrmcdata->bob_current_mode);

	return 0;
}

static int lge_regulator_mode_change_resume(struct device *dev)
{
	int ret = 0;

	if(lrmcdata->bob_current_mode != PMI_BOB_PWM_MODE){
		ret = regulator_set_load(lrmcdata->bob_vreg, lrmcdata->bob_auto_threshold);
		if(ret < 0){
			pr_err("%s regulator_set_load err = %d\n", __func__, ret);
			pr_err("%s bob mode = %d\n",__func__, lrmcdata->bob_current_mode);
			return 0;
		}
		lrmcdata->bob_current_mode = PMI_BOB_AUTO_MODE;
	}
	pr_err("%s done : bob_mode = %d\n", __func__, lrmcdata->bob_current_mode);

	return 0;
}


static int lge_regulator_mode_change_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id lge_regulator_mode_change_match[] = {
	{.compatible = LRMC_DEV_NAME},
	{},
};

static const struct dev_pm_ops lge_regulator_mode_change_pm_ops = {
	.suspend = lge_regulator_mode_change_suspend,
	.resume  = lge_regulator_mode_change_resume,
};

static struct platform_driver lge_regulator_mode_change_driver = {
	.probe          = lge_regulator_mode_change_probe,
	.remove         = lge_regulator_mode_change_remove,
	.driver         = {
		.name   = LRMC_DEV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(lge_regulator_mode_change_match),
		.pm = &lge_regulator_mode_change_pm_ops,
	},
};

static int __init lge_regulator_mode_change_init(void)
{
	int ret = 0;
	pr_err("%s\n",__func__);
	ret = platform_driver_register(&lge_regulator_mode_change_driver);
	if(ret < 0){
		pr_err("%s err = %d\n",__func__, ret);
	}
	return ret;
}

static void __exit lge_regulator_mode_change_exit(void)
{
	pr_err("%s\n",__func__);
	platform_driver_unregister(&lge_regulator_mode_change_driver);
}
module_init(lge_regulator_mode_change_init);
module_exit(lge_regulator_mode_change_exit);

MODULE_AUTHOR("Hochang Kwon <hochang.kwon@lge.com>");
MODULE_DESCRIPTION("LGE Regulator Mode Change driver");
MODULE_LICENSE("GPL v2");

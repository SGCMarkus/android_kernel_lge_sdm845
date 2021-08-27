/*
 * LGE clock boosting driver for mme
 *
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "lge_mme_bus: %s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/qseecom.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/socinfo.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>
#include <linux/delay.h>
#include <linux/compat.h>

#include <soc/qcom/lge/lge_mme_bus.h>

#define DEVICE_NAME		"lge_mme_bus"

#define MAX_TZ_APPS		8
#define MAX_TZ_NAME_LEN		32

struct lge_mme_bus_device {
	u32 msm_bus_handle;
	struct class *class;
	struct device *dev;
	u32 num_apps;
	char applist[MAX_TZ_APPS][MAX_TZ_NAME_LEN];
};

static int boost_enable;
static struct lge_mme_bus_device lge_mme_bus_dev;

static struct msm_bus_paths lge_mme_usecases[]  =
{
	{
		.vectors = (struct msm_bus_vectors[])
		{
			{
				.src = 1,
				.dst = 512,
				.ab = 0,
				.ib = 0,
			},
		},
		.num_paths = 1,
	},
	{
		.vectors = (struct msm_bus_vectors[])
		{
			{
				.src = 1,
				.dst = 512,
				.ab = 28864000000,
				.ib = 28864000000,
			},
		},
		.num_paths = 1,
	},
};

static struct msm_bus_scale_pdata lge_mme_bus_client_pdata = {
	.usecase = lge_mme_usecases,
	.num_usecases = ARRAY_SIZE(lge_mme_usecases),
	.name = "lge_mme_bus",
};

static ssize_t lge_mme_bus_boost_show (struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", boost_enable);
}

static ssize_t lge_mme_bus_boost_store (struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;

	sscanf(buf, "%d", &boost_enable);
	pr_debug("%s: boost_enable %d\n", __func__, boost_enable);

	if (!lge_mme_bus_dev.msm_bus_handle) {
		pr_err("%s: invalid bus scale handle %d - "
				"maybe by probing failure." , __func__,
				lge_mme_bus_dev.msm_bus_handle);
		return -EINVAL;
	}

	ret = msm_bus_scale_client_update_request(
			lge_mme_bus_dev.msm_bus_handle, boost_enable);
	if (ret)
		WARN_ON(1);

	return count;
}

static DEVICE_ATTR(boost, 0664,
		lge_mme_bus_boost_show,
		lge_mme_bus_boost_store);

static struct attribute *lge_mme_bus_attrs[] = {
	&dev_attr_boost.attr,
	NULL
};

static const struct attribute_group lge_mme_bus_files = {
	.attrs  = lge_mme_bus_attrs,
};

/**
 * lge_mme_bus_ebi_boost - boost ebi ddr clock
 * @enable : 1 - boost enable, 0 - disable
 *
 * A return value is error if fail to boost, otherwise 0.
 */
int lge_mme_bus_ebi_boost(int enable)
{
	int ret = 0;

	if (!lge_mme_bus_dev.msm_bus_handle) {
		pr_err("%s: invalid bus scale handle %d - "
				"maybe by probing failure." , __func__,
				lge_mme_bus_dev.msm_bus_handle);
		return -EINVAL;
	}

	ret = msm_bus_scale_client_update_request(
			lge_mme_bus_dev.msm_bus_handle, enable);
	if (ret)
		WARN_ON(1);

	return ret;
}

/**
 * lge_mme_bus_check_boost_app - check if it is tz app boosting needed
 * @appname : name of tz app
 *
 * A return value is 1 if it is tz app which needs ebi boosting,
 * otherwise 0.
 */
int lge_mme_bus_check_boost_app(char *appname)
{
	int i = 0;
	char *name = NULL;
	int num = lge_mme_bus_dev.num_apps;

	if (!num)
		return 0;

	for (i = 0; i < num; i++) {
		name = lge_mme_bus_dev.applist[i];
		if (!strncmp(name, appname, MAX_TZ_NAME_LEN))
			return 1;
	}

	return 0;
}
static int lge_wmc = -1;

static int __init lge_check_wmc(char *str)
{
	int ret = 0;
	ret = kstrtoint(str, 10, &lge_wmc);
	if (ret)
		lge_wmc = 0;
	return 1;
}
__setup("lge.wmc=", lge_check_wmc);

static int lge_mme_bus_probe(struct platform_device *pdev)
{
	int ret = 0;
	int idx = 0;
	int apps_num = 0;
	struct device_node *node = pdev->dev.of_node;
	struct property *prop;
	const char *app_name;

	pr_debug("%s: probe enter\n", __func__);

	lge_mme_bus_dev.class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(lge_mme_bus_dev.class)) {
		pr_err("%s: class_create() failed ENOMEM\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	lge_mme_bus_dev.dev = device_create(lge_mme_bus_dev.class, NULL,
			MKDEV(0, 0),
			NULL, "mme_bus_ctrl");
	if (IS_ERR(lge_mme_bus_dev.dev)) {
		pr_err("%s: device_create() failed\n", __func__);
		ret = PTR_ERR(lge_mme_bus_dev.dev);
		goto exit;
	}

	/* create /sys/class/lge_mme_bus/mme_bus_ctrl/boost */
	ret = device_create_file(lge_mme_bus_dev.dev, &dev_attr_boost);
	if (ret < 0) {
		pr_err("%s: device create file fail\n", __func__);
		goto exit;
	}

	/* get tz app list to boost */
	apps_num = of_property_count_strings(node, "lge,app-names");
	if (apps_num < 0) {
		pr_err("%s: There is no app to boost. not boost at all\n",
				__func__);
		/* fall throgh to probing pass */
	}

	if (apps_num > 0) {
		lge_mme_bus_dev.num_apps = apps_num;
		of_property_for_each_string(node, "lge,app-names",
				prop, app_name) {
			scnprintf(lge_mme_bus_dev.applist[idx],
					MAX_TZ_NAME_LEN, "%s", app_name);

			pr_debug("%s: tz app to boost - %s\n", __func__,
					lge_mme_bus_dev.applist[idx]);
			idx++;
		}
	}

	lge_mme_bus_dev.msm_bus_handle = msm_bus_scale_register_client(
			&lge_mme_bus_client_pdata);
	if (!lge_mme_bus_dev.msm_bus_handle) {
		pr_err("fail to register with bus mgr!\n");
		return -EINVAL;
	}

	pr_debug("%s: probe done\n", __func__);
	return 0;

exit:
	pr_err("%s: probe fail - %d\n", __func__, ret);
	return ret;
}

static int lge_mme_bus_remove(struct platform_device *pdev)
{
	if (lge_mme_bus_dev.msm_bus_handle) {
		if(msm_bus_scale_client_update_request(
				lge_mme_bus_dev.msm_bus_handle, 0))
			WARN_ON(1);
		msm_bus_scale_unregister_client(lge_mme_bus_dev.msm_bus_handle);
	}
	return 0;
}

static const struct of_device_id lge_mme_bus_match[] = {
	{
		.compatible = "lge,mme_bus",
	},
	{}
};

static struct platform_driver lge_mme_bus_plat_driver = {
	.probe = lge_mme_bus_probe,
	.remove = lge_mme_bus_remove,
	.driver = {
		.name = "lge_mme_bus_drv",
		.owner = THIS_MODULE,
		.of_match_table = lge_mme_bus_match,
	},
};

static int lge_mme_bus_init(void)
{
	if(!lge_wmc) {
		pr_info("%s: This device doesn't support wmc\n", __func__);
		return -EINVAL;
	}

	return platform_driver_register(&lge_mme_bus_plat_driver);
}

static void lge_mme_bus_exit(void)
{
	if(!lge_wmc) {
		pr_info("%s: This device doesn't support wmc\n", __func__);
		return;
	}
	platform_driver_unregister(&lge_mme_bus_plat_driver);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Bus boost driver for mme");

module_init(lge_mme_bus_init);
module_exit(lge_mme_bus_exit);

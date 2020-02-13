#define pr_fmt(fmt)	"[Display][DP:%s:%d] " fmt, __func__, __LINE__

#ifdef CONFIG_LGE_DISPLAY_NOT_SUPPORT_DISPLAYPORT
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include "lge_dp.h"
#include <linux/err.h>

struct class *dp_noti_class;

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct dp_noti_dev *ndev = (struct dp_noti_dev *)
		dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ndev->state);
}

static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);

void dp_noti_set_state(struct dp_noti_dev *ndev, int state)
{
	char *envp[2];
	char name_buf[30];

	if (ndev->state != state) {
		ndev->state = state;
		snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=%s%c", ndev->name, ndev->state?'1':'0');
		envp[0] = name_buf;
		envp[1] = NULL;
		kobject_uevent_env(&ndev->dev->kobj, KOBJ_CHANGE, envp);
		pr_info("check_dp_notify,name_buf = %s state = %d,\n", name_buf, ndev->state);
	}
}

static int create_dp_noti_class(void)
{
	if (!dp_noti_class) {
		dp_noti_class = class_create(THIS_MODULE, "dp_notify");
		if (IS_ERR(dp_noti_class))
			return PTR_ERR(dp_noti_class);
	}

	return 0;
}

int dp_noti_register(struct dp_noti_dev *ndev)
{
	int ret;

	if (!ndev) {
		pr_err("%s : dp register failed\n", __func__);
		return -EINVAL;
	}

	ndev->dev = device_create(dp_noti_class, NULL,
			MKDEV(0, 1), NULL, ndev->name);
       if (IS_ERR(ndev->dev))
	       return PTR_ERR(ndev->dev);

       ret = device_create_file(ndev->dev, &dev_attr_state);
       if (ret < 0)
			goto err1;

       dev_set_drvdata(ndev->dev, ndev);
       ndev->state = 0;
       return 0;

err1:
       device_remove_file(ndev->dev, &dev_attr_state);
       return ret;
}

void dp_noti_unregister(struct dp_noti_dev *ndev)
{
	device_remove_file(ndev->dev, &dev_attr_state);
	dev_set_drvdata(ndev->dev, NULL);
	device_destroy(dp_noti_class, MKDEV(0, 1));
}

static int __init dp_noti_class_init(void)
{
	return create_dp_noti_class();
}

static void __exit dp_noti_class_exit(void)
{
	class_destroy(dp_noti_class);
}

module_init(dp_noti_class_init);
module_exit(dp_noti_class_exit);

#else // DP support
#include <linux/string.h>
#include "lge_dp.h"
#include "lge_dp_def.h"
#include "dp_display.h"
#include "dp_parser.h"

static ssize_t lge_dp_hpd_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dp_display *dp;

	dp = dev_get_drvdata(dev);

	if (!dp) {
		pr_err("dp is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", dp->lge_dp.hpd_state);
}
static DEVICE_ATTR(dp_hpd, S_IRUGO, lge_dp_hpd_show, NULL);

extern struct dp_aux_cfg *lge_dp_display_parser_aux_cfg(struct dp_display *dp_display);

static ssize_t lge_dp_aux_cfg1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dp_display *dp;
	int i = 0;
	struct dp_aux_cfg *cfg = NULL;

	dp = dev_get_drvdata(dev);

	if (!dp) {
		pr_err("dp is NULL\n");
		return -EINVAL;
	}

	cfg = lge_dp_display_parser_aux_cfg(dp);
	if (!cfg) {
		pr_err("aux_cfg is NULL\n");
		return -EINVAL;
	}
	buf[0] = '\0';
	for (i = 0; i < cfg[PHY_AUX_CFG1].cfg_cnt; ++i)
		sprintf(buf+strlen(buf), "0x%02X ", cfg[PHY_AUX_CFG1].lut[i]);
	return strlen(buf);
}

static ssize_t lge_dp_aux_cfg1_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct dp_display *dp;
	int i = 0;
	struct dp_aux_cfg *cfg = NULL;
	u32 lut[DP_AUX_CFG_MAX_VALUE_CNT] = {0, };
	u32 cnt = 0;
	char *tmp1 = NULL;
	char *tmp2 = NULL;
	char *tok = NULL;
	int error = 0;

	dp = dev_get_drvdata(dev);

	if (!dp) {
		pr_err("dp is NULL\n");
		return -EINVAL;
	}

	cfg = lge_dp_display_parser_aux_cfg(dp);
	if (!cfg) {
		pr_err("aux_cfg is NULL\n");
		return -EINVAL;
	}

	tmp1 = tmp2 = kstrdup(buf, GFP_KERNEL);
	while ((tok = strsep(&tmp2, " ")) != NULL) {
		if (kstrtoint(tok, 16, &lut[cnt++])) {
			error = 1;
			break;
		}
	}
	kfree(tmp1);

	if (!error) {
		cfg[PHY_AUX_CFG1].cfg_cnt = cnt>DP_AUX_CFG_MAX_VALUE_CNT?DP_AUX_CFG_MAX_VALUE_CNT:cnt;
		for (i = 0; i < cfg[PHY_AUX_CFG1].cfg_cnt; ++i)
			cfg[PHY_AUX_CFG1].lut[i] = lut[i];
	} else {
		pr_err("invalid parameters");
	}

	return strnlen(buf, PAGE_SIZE);
}

static DEVICE_ATTR(dp_aux_cfg1, S_IRUGO|S_IWUSR, lge_dp_aux_cfg1_show, lge_dp_aux_cfg1_store);

static void lge_dp_create_sysfs(struct dp_display *dp_display)
{
	static struct class *class_dp = NULL;
	static struct device *dp_sysfs_dev = NULL;

	if(!class_dp) {
		class_dp = class_create(THIS_MODULE, "display_port");
		if (IS_ERR(class_dp)) {
			pr_err("Failed to create DisplayPort class\n");
		} else {
			if(!dp_sysfs_dev) {
				dp_sysfs_dev = device_create(class_dp, NULL, 0, dp_display, "common");
				if (IS_ERR(dp_sysfs_dev)) {
					pr_err("Failed to create dev(dp_sysfs_dev)!\n");
				} else {
					if ((device_create_file(dp_sysfs_dev, &dev_attr_dp_hpd)) < 0)
						pr_err("add dp_hpd node failed!\n");
					if ((device_create_file(dp_sysfs_dev, &dev_attr_dp_aux_cfg1)) < 0)
						pr_err("add dp_hpd node failed!\n");
				}
			}
		}
	}

}

void lge_dp_drv_init(struct dp_display *dp_display)
{
	lge_dp_create_sysfs(dp_display);
}

void lge_set_dp_hpd(struct dp_display *dp_display, int value)
{
	dp_display->lge_dp.hpd_state = value;
}
#endif

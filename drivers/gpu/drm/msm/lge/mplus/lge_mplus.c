#define pr_fmt(fmt)	"[Display][lge-mplus:%s:%d] " fmt, __func__, __LINE__

#include <linux/kallsyms.h>
#include <linux/backlight.h>
#include <linux/delay.h>

#include "msm_drv.h"
#include "sde_dbg.h"
#include "sde_kms.h"
#include "sde_connector.h"
#include "sde_encoder.h"
#include "dsi_drm.h"
#include "dsi_display.h"
#include "sde_crtc.h"

#include "lge_mplus.h"
#include "brightness/lge_brightness_def.h"

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DIMMING_BOOT_SUPPORT)
#include "lge_dsi_panel.h"
#endif

extern int dsi_display_set_backlight(void *display, u32 bl_lvl);
extern struct lge_blmap *lge_get_blmap(struct dsi_panel *panel, enum lge_blmap_type type);

/* Hidden Menu Mplus Set */
static ssize_t mplus_hd_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int ret = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return ret;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->mplus_hd_get)
		ret = panel->lge.ddic_ops->mplus_hd_get(panel);
	else
		pr_info("Not support\n");

	return sprintf(buf, "%d\n", ret);
}

static ssize_t mplus_hd_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	unsigned int **addr;
	struct dsi_panel *panel;
	struct dsi_display *display;
	int mp_hd;

	panel = dev_get_drvdata(dev);

	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	if(!dsi_panel_initialized(panel)) {
		pr_err("panel not yet initialized\n");
		mutex_unlock(&panel->panel_lock);
		return -EINVAL;
	}
	mutex_unlock(&panel->panel_lock);

	addr = (unsigned int **)kallsyms_lookup_name("primary_display");
	if (addr) {
		display = (struct dsi_display *)*addr;
	} else {
		pr_err("primary_display not founded.\n");
		return -EINVAL;
	}

	if(!display) {
		pr_err("display is null\n");
		return -EINVAL;
	}

	if(display->is_cont_splash_enabled) {
		pr_err("cont_splash enabled\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &mp_hd);
	pr_debug("mp_hd : %d\n", mp_hd);

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->mplus_hd_set)
		panel->lge.ddic_ops->mplus_hd_set(panel, mp_hd);
	else
		pr_info("Not support\n");

	return ret;
}

static DEVICE_ATTR(mplus_hd, S_IRUGO | S_IWUSR | S_IWGRP,
		mplus_hd_get, mplus_hd_set);

/* Max Brightness Mplus Set */
static ssize_t mplus_max_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int ret = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return ret;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->mplus_max_get)
		ret = panel->lge.ddic_ops->mplus_max_get(panel);
	else
		pr_info("Not support\n");

	return sprintf(buf, "%d\n", ret);
}

static ssize_t mplus_max_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	unsigned int **addr;
	struct dsi_panel *panel;
	struct dsi_display *display;
	int mp_max;

	sscanf(buf, "%d", &mp_max);
	pr_debug("mp_max : %d\n", mp_max);

	panel = dev_get_drvdata(dev);

	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	if(!dsi_panel_initialized(panel)) {
		if (panel->lge.ddic_ops && panel->lge.ddic_ops->mplus_max_set) {
			panel->lge.mp_max = mp_max;
			pr_err("panel not yet initialized, mplus max stored\n");
		} else {
			pr_info("Not support\n");
		}
		mutex_unlock(&panel->panel_lock);
		return -EINVAL;
	}
	mutex_unlock(&panel->panel_lock);

	addr = (unsigned int **)kallsyms_lookup_name("primary_display");
	if (addr) {
		display = (struct dsi_display *)*addr;
	} else {
		pr_err("primary_display not founded.\n");
		return -EINVAL;
	}

	if(!display) {
		pr_err("display is null\n");
		return -EINVAL;
	}

	if(display->is_cont_splash_enabled) {
		pr_err("cont_splash enabled\n");
		return -EINVAL;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->mplus_max_set)
		panel->lge.ddic_ops->mplus_max_set(panel, mp_max);
	else
		pr_info("Not support\n");

	return ret;
}

static DEVICE_ATTR(mplus_max, S_IRUGO | S_IWUSR | S_IWGRP,
		mplus_max_get, mplus_max_set);

/* Advanced Mplus Set */
static ssize_t mplus_mode_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int ret = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return ret;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->mplus_mode_get)
		ret = panel->lge.ddic_ops->mplus_mode_get(panel);
	else
		pr_info("Not support\n");

	return sprintf(buf, "%d\n", ret);
}

static ssize_t mplus_mode_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	unsigned int **addr;
	struct dsi_panel *panel;
	struct dsi_display *display;
	int mp_mode;

	panel = dev_get_drvdata(dev);

	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	if(!dsi_panel_initialized(panel)) {
		pr_err("panel not yet initialized\n");
		mutex_unlock(&panel->panel_lock);
		return -EINVAL;
	}
	mutex_unlock(&panel->panel_lock);

	addr = (unsigned int **)kallsyms_lookup_name("primary_display");
	if (addr) {
		display = (struct dsi_display *)*addr;
	} else {
		pr_err("primary_display not founded.\n");
		return -EINVAL;
	}

	if(!display) {
		pr_err("display is null\n");
		return -EINVAL;
	}

	if(display->is_cont_splash_enabled) {
		pr_err("cont_splash enabled\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &mp_mode);
	pr_debug("mp_mode : %d\n", mp_mode);

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->mplus_mode_set)
		panel->lge.ddic_ops->mplus_mode_set(panel, mp_mode);
	else
		pr_info("Not support\n");

	return ret;
}

static DEVICE_ATTR(mplus_mode, S_IRUGO | S_IWUSR | S_IWGRP,
		mplus_mode_get, mplus_mode_set);

static ssize_t mplus_dim_cnt_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int ret = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return ret;
	}

	ret = panel->lge.mplus_dim_cnt;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t mplus_dim_cnt_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input;

	panel = dev_get_drvdata(dev);

	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);
	panel->lge.mplus_dim_cnt = input;
	pr_info("mplus dim cnt : %d\n", panel->lge.mplus_dim_cnt);

	return ret;
}

static DEVICE_ATTR(mplus_dim_cnt, S_IRUGO | S_IWUSR | S_IWGRP,
		mplus_dim_cnt_get, mplus_dim_cnt_set);

static ssize_t mplus_dim_delay_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int ret = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return ret;
	}

	ret = panel->lge.mplus_dim_delay;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t mplus_dim_delay_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input;

	panel = dev_get_drvdata(dev);

	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);
	panel->lge.mplus_dim_delay = input;
	pr_info("mplus dim delay : %d\n", panel->lge.mplus_dim_delay);

	return ret;
}

static DEVICE_ATTR(mplus_dim_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		mplus_dim_delay_get, mplus_dim_delay_set);

int lge_mplus_backlight_dimming(struct backlight_device *bd,
		enum lge_mplus_mode old_mp_mode, enum lge_mplus_mode cur_mp_mode)
{
	int brightness;
	struct lge_blmap *blmap = NULL;
	struct dsi_panel *panel;
	struct dsi_display *display;
	struct sde_connector *c_conn;
	int bl_lvl, old_bl_lvl, cur_bl_lvl, i;
	int target_bl_lvl, bl_lvl_diff;
	struct drm_event event;
	enum lge_blmap_type bl_type = LGE_BLMAP_DEFAULT;
	int dim_cnt, dim_delay;

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DIMMING_BOOT_SUPPORT)
	if (lge_get_bootreason_with_lcd_dimming() && !is_blank_called()) {
		pr_info("Skip Brightness dimming. boot reason is lcd dimming\n");
		return 0;
	} else if (is_factory_cable()  && !is_blank_called()) {
		pr_info("Skip Brightness dimming. Detect factory cable\n");
		return 0;
	}
#endif

	brightness = bd->props.brightness;

	if ((bd->props.power != FB_BLANK_UNBLANK) ||
			(bd->props.state & BL_CORE_FBBLANK) ||
			(bd->props.state & BL_CORE_SUSPENDED))
		brightness = 0;

	c_conn = bl_get_data(bd);
	display = (struct dsi_display *) c_conn->display;
	panel = display->panel;
	panel->bl_config.bd = bd;

	/* Find OLD Mplus BL MAP */
	if (panel->lge.ddic_ops && panel->lge.ddic_ops->mp_blmap_sub) {
		bl_type = panel->lge.ddic_ops->mp_blmap_sub(panel, old_mp_mode);
		blmap = lge_get_blmap(panel, bl_type);
	}

	/* find OLD BL_LVL */
	if (blmap) {
		// DUMMY panel doesn't have blmap, so this code is mandatory
		if (blmap->size == 0)	return -EINVAL;
		if (brightness >= blmap->size) {
			pr_warn("brightness=%d is bigger than blmap size (%d)\n", brightness, blmap->size);
			brightness = blmap->size-1;
		}

		old_bl_lvl = blmap->map[brightness];
	} else {
		if (brightness > display->panel->bl_config.bl_max_level)
			brightness = display->panel->bl_config.bl_max_level;

		/* map UI brightness into driver backlight level with rounding */
		old_bl_lvl = mult_frac(brightness, display->panel->bl_config.bl_max_level,
				display->panel->bl_config.brightness_max_level);

		if (!old_bl_lvl && brightness)
			old_bl_lvl = 1;
	}


	/* Find NEW Mplus BL MAP */
	if (panel->lge.ddic_ops && panel->lge.ddic_ops->mp_blmap_sub) {
		bl_type = panel->lge.ddic_ops->mp_blmap_sub(panel, cur_mp_mode);
		blmap = lge_get_blmap(panel, bl_type);
	}

	/* find CUR BL_LVL */
	if (blmap) {
		// DUMMY panel doesn't have blmap, so this code is mandatory
		if (blmap->size == 0)	return -EINVAL;
		if (brightness >= blmap->size) {
			pr_warn("brightness=%d is bigger than blmap size (%d)\n", brightness, blmap->size);
			brightness = blmap->size-1;
		}
		cur_bl_lvl = blmap->map[brightness];
	} else {
		if (brightness > display->panel->bl_config.bl_max_level)
			brightness = display->panel->bl_config.bl_max_level;

		/* map UI brightness into driver backlight level with rounding */
		cur_bl_lvl = mult_frac(brightness, display->panel->bl_config.bl_max_level,
				display->panel->bl_config.brightness_max_level);

		if (!cur_bl_lvl && brightness)
			cur_bl_lvl = 1;
	}

	if(old_bl_lvl == cur_bl_lvl) {
		pr_info("skip brightness dimming\n");
		return 0;
	}
	pr_info("old bl_lvl : %d , cur bl_lv : %d \n", old_bl_lvl, cur_bl_lvl);

	dim_cnt = display->panel->lge.mplus_dim_cnt;
	dim_delay = display->panel->lge.mplus_dim_delay;

	if(old_bl_lvl < cur_bl_lvl) {
		bl_lvl_diff = (cur_bl_lvl - old_bl_lvl)/dim_cnt;
		bl_lvl = old_bl_lvl;
		target_bl_lvl = cur_bl_lvl;
		for(i = 0; i < dim_cnt; i++) {
			bl_lvl += bl_lvl_diff;
			if((bl_lvl > target_bl_lvl) || (i == (dim_cnt-1)))
				bl_lvl = target_bl_lvl;

			if (c_conn->ops.set_backlight) {
				event.type = DRM_EVENT_SYS_BACKLIGHT;
				event.length = sizeof(u32);
				msm_mode_object_event_notify(&c_conn->base.base,
						c_conn->base.dev, &event, (u8 *)&brightness);
				c_conn->ops.set_backlight(c_conn->display, bl_lvl);
				pr_info("BR:%d BL:%d %s\n", brightness, bl_lvl, lge_get_blmapname(bl_type));
			}
			msleep(dim_delay);
		}
	} else {
		bl_lvl_diff = (cur_bl_lvl - old_bl_lvl)/dim_cnt;
		bl_lvl = old_bl_lvl;
		target_bl_lvl = cur_bl_lvl;
		for(i = 0; i < dim_cnt; i++) {
			bl_lvl += bl_lvl_diff;
			if((bl_lvl < target_bl_lvl) || (i == (dim_cnt-1)))
				bl_lvl = target_bl_lvl;

			if (c_conn->ops.set_backlight) {
				event.type = DRM_EVENT_SYS_BACKLIGHT;
				event.length = sizeof(u32);
				msm_mode_object_event_notify(&c_conn->base.base,
						c_conn->base.dev, &event, (u8 *)&brightness);
				c_conn->ops.set_backlight(c_conn->display, bl_lvl);
				pr_info("BR:%d BL:%d %s\n", brightness, bl_lvl, lge_get_blmapname(bl_type));
			}
			mdelay(dim_delay);
		}
	}
	return 0;
}

int lge_mplus_create_sysfs(struct device *panel_sysfs_dev)
{
	int rc = 0;
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_mplus_hd)) < 0)
		pr_err("add mplus_mode node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_mplus_max)) < 0)
		pr_err("add mplus_mode node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_mplus_mode)) < 0)
		pr_err("add mplus_mode node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_mplus_dim_cnt)) < 0)
		pr_err("add mplus_dim_cnt node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_mplus_dim_delay)) < 0)
		pr_err("add mplus_dim_delay node fail!");
	return rc;
}

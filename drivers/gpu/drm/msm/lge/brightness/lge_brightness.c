#define pr_fmt(fmt)	"[Display][lge-backlight:%s:%d] " fmt, __func__, __LINE__

#include "msm_drv.h"
#include "sde_dbg.h"
#include "sde_kms.h"
#include "sde_connector.h"
#include "sde_encoder.h"
#include <linux/backlight.h>
#include <linux/leds.h>
#include "dsi_drm.h"
#include "dsi_display.h"
#include "sde_crtc.h"

#include "lge_brightness_def.h"
#include "lge_brightness.h"
#include "lge_dsi_panel.h"

char *blmap_names[] = {
	"lge,blmap",
	"lge,blmap-psm",
	"lge,blmap-gal",
	"lge,blmap-bri",
	"lge,blmap-hqc",
	"lge,blmap-max",
	"lge,blmap-hl",
	"lge,blmap-psm-hl",
	"lge,blmap-gal-hl",
	"lge,blmap-bri-hl",
	"lge,blmap-hqc-hl",
	"lge,blmap-max-hl",
	"lge,blmap-hbm-hd",
	"lge,blmap-ve",
	"lge,blmap-ex",
	"lge,blmap-hdr",
};
static const int blmap_names_num = sizeof(blmap_names)/sizeof(blmap_names[0]);

inline static void lge_blmap_free_sub(struct lge_blmap *blmap)
{
	if (blmap->map) {
		kfree(blmap->map);
		blmap->map = NULL;
		blmap->size = 0;
	}
}

void lge_dsi_panel_blmap_free(struct dsi_panel *panel)
{
	int i;
	if (panel->lge.blmap_list) {
		for (i = 0; i < panel->lge.blmap_list_size; ++i) {
			lge_blmap_free_sub(&panel->lge.blmap_list[i]);
		}
		kfree(panel->lge.blmap_list);
		panel->lge.blmap_list_size = 0;
	}
}

static int lge_dsi_panel_parse_blmap_sub(struct device_node *of_node, const char* blmap_name, struct lge_blmap *blmap)
{
	struct property *data;
	int rc = 0;

	if (!blmap) {
		return -EINVAL;
	}

	blmap->size = 0;
	data = of_find_property(of_node, blmap_name, &blmap->size);
	if (!data) {
		pr_err("can't find %s\n", blmap_name);
		return -EINVAL;
	}
	blmap->size /= sizeof(u32);
	pr_info("%s blmap_size = %d\n", blmap_name, blmap->size);
	blmap->map = kzalloc(sizeof(u32) * blmap->size, GFP_KERNEL);
	if (!blmap->map) {
		blmap->size = 0;
		return -ENOMEM;
	}

	rc = of_property_read_u32_array(of_node, blmap_name, blmap->map,
					blmap->size);
	if (rc) {
		lge_blmap_free_sub(blmap);
	} else {
		pr_info("%s has been successfully parsed. \n", blmap_name);
	}

	return rc;
}


int lge_dsi_panel_parse_brightness(struct dsi_panel *panel,
	struct device_node *of_node)
{
	int rc = 0;

	rc = of_property_read_u32(of_node, "lge,default-brightness", &panel->lge.default_brightness);
	if (rc) {
		return rc;
	} else {
		pr_info("default brightness=%d \n", panel->lge.default_brightness);
	}

	return rc;
};

int lge_dsi_panel_parse_blmap(struct dsi_panel *panel,
	struct device_node *of_node)
{
	int rc = 0;
	int i;

	panel->lge.blmap_list = kzalloc(sizeof(struct lge_blmap) * blmap_names_num, GFP_KERNEL);
	if (!panel->lge.blmap_list)
		return -ENOMEM;
	panel->lge.blmap_list_size = blmap_names_num;

	for (i = 0; i < blmap_names_num; ++i) {
		lge_dsi_panel_parse_blmap_sub(of_node, blmap_names[i], &panel->lge.blmap_list[i]);
	}

	return rc;
};

#if IS_ENABLED(CONFIG_LGE_DISPLAY_USE_FSC)
int lge_set_fsc(struct dsi_panel *panel)
{
	struct dsi_backlight_config *bl = &panel->bl_config;
	struct led_classdev *led_cdev;
	int rc = 0;

	read_lock(&bl->wled->leddev_list_lock);
	list_for_each_entry(led_cdev, &bl->wled->led_cdevs, trig_list)
	{
		if(led_cdev->set_fsc)
			led_cdev->set_fsc(led_cdev, panel->lge.fsc_req);
	}
	read_unlock(&bl->wled->leddev_list_lock);

	return rc;
}
#endif

char *lge_get_blmapname(enum lge_blmap_type type)
{
	if (type >= 0 && type < LGE_BLMAP_TYPE_MAX)
		return blmap_names[type];
	else
		return blmap_names[LGE_BLMAP_DEFAULT];
}

struct lge_blmap *lge_get_blmap(struct dsi_panel *panel, enum lge_blmap_type type)
{
	struct lge_blmap *blmap;

	if (type < 0 || type > panel->lge.blmap_list_size)
		type = LGE_BLMAP_DEFAULT;

	blmap = &panel->lge.blmap_list[type];
	if (!blmap)
		blmap = &panel->lge.blmap_list[LGE_BLMAP_DEFAULT];

	return blmap;
}

int lge_update_backlight(struct dsi_panel *panel)
{
	struct dsi_display *display;
	struct backlight_device *bd;
	int rc = 0;


	if (panel == NULL || panel->host == NULL)
		return -EINVAL;

	bd = panel->bl_config.bd;
	if (bd == NULL)
		return -EINVAL;

	if (panel->lge.allow_bl_update || !lge_dsi_panel_is_power_on_interactive(panel))
		return 0;

	mutex_lock(&bd->ops_lock);
	if (panel->lge.bl_lvl_unset < 0) {
		rc = 0;
		goto exit;
	}

	display = container_of(panel->host, struct dsi_display, host);

	rc = dsi_display_set_backlight(display, panel->lge.bl_lvl_unset);
	if (!rc) {
		pr_info("<--%pS unset=%d\n", __builtin_return_address(0), panel->lge.bl_lvl_unset);
	}
	panel->lge.allow_bl_update = true;
	panel->lge.bl_lvl_unset = -1;

exit:
	mutex_unlock(&bd->ops_lock);
	return rc;
}

int lge_backlight_device_update_status(struct backlight_device *bd)
{
	int brightness;
	struct lge_blmap *blmap;
	struct dsi_panel *panel;
	struct dsi_display *display;
	struct sde_connector *c_conn;
	int bl_lvl;
	struct drm_event event = {0,};
	enum lge_blmap_type bl_type;
	int rc = 0;

	brightness = bd->props.brightness;

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DIMMING_BOOT_SUPPORT)
	if (lge_get_bootreason_with_lcd_dimming() && !is_blank_called() && brightness > 0) {
		brightness = 1;
		pr_info("lcd dimming mode. set value = %d\n", brightness);
	} else if (is_factory_cable()  && !is_blank_called() && brightness > 0) {
		brightness =  1;
		pr_info("Detect factory cable. set value = %d\n", brightness);
	}
#endif

	if ((bd->props.power != FB_BLANK_UNBLANK) ||
			(bd->props.state & BL_CORE_FBBLANK) ||
			(bd->props.state & BL_CORE_SUSPENDED))
		brightness = 0;

	c_conn = bl_get_data(bd);
	display = (struct dsi_display *) c_conn->display;
	panel = display->panel;
	panel->bl_config.bd = bd;

	if (panel->lge.use_color_manager && panel->lge.video_enhancement) {
		bl_type = LGE_BLMAP_VE;
	} else if (panel->lge.ddic_ops && panel->lge.ddic_ops->mp_blmap) {
		bl_type = panel->lge.ddic_ops->mp_blmap(panel);
	} else {
		bl_type = LGE_BLMAP_DEFAULT;
	}
	blmap = lge_get_blmap(panel, bl_type);

	if (blmap) {
		// DUMMY panel doesn't have blmap, so this code is mandatory
		if (blmap->size == 0)	return -EINVAL;
		if (brightness >= blmap->size) {
			pr_warn("brightness=%d is bigger than blmap size (%d)\n", brightness, blmap->size);
			brightness = blmap->size-1;
		}

		bl_lvl = blmap->map[brightness];
	} else {
		if (brightness > display->panel->bl_config.bl_max_level)
			brightness = display->panel->bl_config.bl_max_level;

		/* map UI brightness into driver backlight level with rounding */
		bl_lvl = mult_frac(brightness, display->panel->bl_config.bl_max_level,
				display->panel->bl_config.brightness_max_level);

		if (!bl_lvl && brightness)
			bl_lvl = 1;
	}

	mutex_lock(&display->display_lock);
	if (panel->lge.lp_state == LGE_PANEL_NOLP && panel->lge.allow_bl_update) {
		panel->lge.bl_lvl_unset = -1;
		if (c_conn->ops.set_backlight) {
			event.type = DRM_EVENT_SYS_BACKLIGHT;
			event.length = sizeof(u32);
			msm_mode_object_event_notify(&c_conn->base.base,
					c_conn->base.dev, &event, (u8 *)&brightness);
			rc = c_conn->ops.set_backlight(c_conn->display, bl_lvl);
			pr_info("BR:%d BL:%d %s\n", brightness, bl_lvl, lge_get_blmapname(bl_type));
		}
	} else if (!panel->lge.allow_bl_update) {
		panel->lge.bl_lvl_unset = bl_lvl;
		pr_info("brightness = %d, bl_lvl = %d -> differed (not allow) %s\n", brightness, bl_lvl, lge_get_blmapname(bl_type));
	} else {
		panel->lge.bl_lvl_unset = bl_lvl;
		pr_info("brightness = %d, bl_lvl = %d -> differed %s\n", brightness, bl_lvl, lge_get_blmapname(bl_type));
	}
	mutex_unlock(&display->display_lock);

	return rc;
}

/* @Override */
int dsi_panel_update_backlight(struct dsi_panel *panel,
	u32 bl_lvl)
{
	int rc = 0;
	struct mipi_dsi_device *dsi;

	if (!panel || (bl_lvl > 0xffff)) {
		pr_err("invalid params\n");
		return -EINVAL;
	}

	dsi = &panel->mipi_device;

	if (panel->lge.dcs_brightness_be) {
		bl_lvl = ((bl_lvl & 0xff00)>>8) | ((bl_lvl & 0xff)<<8);
		pr_debug("[byte-order] bl_lvl = 0x%X\n", bl_lvl);
	}

	if (panel->lge.use_dcs_brightness_short) {
		rc = mipi_dsi_dcs_set_display_brightness_short(dsi, bl_lvl);
	} else {
		rc = mipi_dsi_dcs_set_display_brightness(dsi, bl_lvl);
	}
	if (rc < 0)
		pr_err("failed to update dcs backlight:%d\n", bl_lvl);

	return rc;
}

static ssize_t hl_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel = NULL;
	int ret = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return ret;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->hl_mode_get)
		ret = panel->lge.ddic_ops->hl_mode_get(panel);
	else
		pr_info("Not support\n");

	return sprintf(buf, "%d\n", ret);
}

static ssize_t hl_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	struct dsi_display *display;
	int input;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);
	pr_info("input data %d\n", input);

	mutex_lock(&panel->panel_lock);
	if(!dsi_panel_initialized(panel)) {
		panel->lge.hl_mode = input;
		pr_err("panel not yet initialized. hl_mode is stored.\n");
		mutex_unlock(&panel->panel_lock);
		return ret;
	}
	mutex_unlock(&panel->panel_lock);

	display = primary_display;

	if(!display) {
		pr_err("display is null\n");
		return -EINVAL;
	}

	if(display->is_cont_splash_enabled) {
		pr_err("cont_splash enabled\n");
		return -EINVAL;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->hl_mode_set)
		panel->lge.ddic_ops->hl_mode_set(panel, input);
	else
		pr_info("Not support\n");

	return ret;
}

static DEVICE_ATTR(hl_mode, S_IRUGO | S_IWUSR | S_IWGRP,
		hl_mode_show, hl_mode_store);

static ssize_t irc_brighter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int ret = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return ret;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->get_irc_state)
		ret = (int)panel->lge.ddic_ops->get_irc_state(panel);
	else
		pr_info("Not support\n");

	return sprintf(buf, "%d\n", ret);
}

static ssize_t irc_brighter_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	struct dsi_display *display;
	int input;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);
	pr_info("input data %d\n", input);

	mutex_lock(&panel->panel_lock);
	if(!dsi_panel_initialized(panel) &&
			(panel->lge.use_irc_ctrl || panel->lge.use_ace_ctrl)) {
		panel->lge.irc_pending = true;
		panel->lge.irc_request_state = ((input == 1) ? LGE_IRC_OFF : LGE_IRC_ON);
		pr_err("panel not yet initialized. irc_ctrl is stored.\n");
		mutex_unlock(&panel->panel_lock);
		return -EINVAL;
	}
	mutex_unlock(&panel->panel_lock);

	display = primary_display;

	if (!display) {
		pr_err("display is null\n");
		return -EINVAL;
	}

	if (display->is_cont_splash_enabled) {
		pr_err("cont_splash enabled\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	if ((panel->lge.use_irc_ctrl || panel->lge.use_ace_ctrl) &&
			panel->lge.ddic_ops && panel->lge.ddic_ops->set_irc_state) {
		panel->lge.irc_pending = true;
		panel->lge.irc_request_state = ((input == 1) ? LGE_IRC_OFF : LGE_IRC_ON);
		mutex_unlock(&panel->panel_lock);
		panel->lge.ddic_ops->set_irc_state(panel, LGE_GLOBAL_IRC_HBM,
						panel->lge.irc_request_state);
	} else {
		mutex_unlock(&panel->panel_lock);
		pr_info("Not support\n");
	}

	return ret;
}
static DEVICE_ATTR(irc_brighter, S_IRUGO | S_IWUSR | S_IWGRP, irc_brighter_show, irc_brighter_store);

static ssize_t irc_support_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int ret = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return ret;
	}

	if (!(panel->lge.use_irc_ctrl || panel->lge.use_ace_ctrl)) {
		pr_err("irc control is not supported\n");
	} else {
		ret = panel->lge.use_irc_ctrl || panel->lge.use_ace_ctrl;
	}

	return sprintf(buf, "%d\n", ret);
}
static DEVICE_ATTR(irc_support, S_IRUGO, irc_support_show, NULL);

int lge_brightness_create_sysfs(struct dsi_panel *panel,
		struct class *class_panel)
{
	int rc = 0;
	static struct device *brightness_sysfs_dev = NULL;

	if(!brightness_sysfs_dev) {
		brightness_sysfs_dev = device_create(class_panel, NULL, 0, panel, "brightness");
		if(IS_ERR(brightness_sysfs_dev)) {
			pr_err("Failed to create dev(brightness_sysfs_dev)!\n");
		} else {
			if (panel->lge.use_hl_mode) {
				if ((rc = device_create_file(brightness_sysfs_dev,
								&dev_attr_hl_mode)) < 0)
					pr_err("add hl_mode set node fail!");
			}

			if (panel->lge.use_irc_ctrl || panel->lge.use_ace_ctrl) {
				if ((rc = device_create_file(brightness_sysfs_dev,
								&dev_attr_irc_brighter)) < 0)
					pr_err("add irc_mode set node fail!");

				if ((rc = device_create_file(brightness_sysfs_dev,
								&dev_attr_irc_support)) < 0)
					pr_err("add irc_status set node fail!");
			}
		}
	}
	return rc;
}

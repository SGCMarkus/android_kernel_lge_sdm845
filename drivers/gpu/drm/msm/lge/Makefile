ccflags-y += -Iinclude/drm -I$(src)/../ -I$(src)/../dsi-staging/
ccflags-y += -Idrivers/gpu/drm/msm/sde
ccflags-y += -Idrivers/gpu/drm/msm/dp

lge_display_common-objs := lge_dsi_panel.o
lge_display_common-objs += lge_panel_reg.o
lge_display_common-objs += lge_ddic_ops.o
lge_display_common-objs += lge_ddic_ops_sw43408.o
lge_display_common-objs += lge_ddic_ops_sw43408a.o
lge_display_common-objs += lge_ddic_ops_sw43402.o
lge_display_common-objs += lge_ddic_ops_sw49409.o
lge_display_common-objs += lge_ddic_ops_sw49410.o
lge_display_common-objs += lge_ddic_ops_sw49410_rev1.o
lge_display_common-objs += lge_ddic_ops_rm69299.o
lge_display_common-objs += lge_ddic_ops_sw43410.o
lge_display_common-objs += lge_ddic_ops_sw43103.o
lge_display_common-objs += ambient/lge_backlight_ex.o
lge_display_common-objs += ambient/lge_ambient.o
lge_display_common-objs += cm/lge_color_manager.o
lge_display_common-objs += drs/lge_drs_mngr.o
lge_display_common-objs += notifier/lge_panel_notifier.o
lge_display_common-objs += mplus/lge_mplus.o
lge_display_common-objs += brightness/lge_brightness.o
lge_display_common-objs += factory/lge_factory.o
lge_display_common-objs += err_detect/lge_err_detect.o
lge_display_common-objs += dp/lge_dp.o
ifeq ($(CONFIG_LGE_DUAL_SCREEN),y)
lge_display_common-objs += ds3/lge_backlight_ds3.o
lge_display_common-objs += cover/lge_cover_ctrl.o
lge_display_common-objs += cover/lge_dsi_panel_cover.o
endif
obj-$(CONFIG_LGE_DISPLAY_COMMON) += lge_display_common.o

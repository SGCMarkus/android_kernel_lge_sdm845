#
# Makefile for the vl53L0 drivers.
#

# Each configuration option enables a list of files.
#FEATURE_USE_CCI := false
FEATURE_USE_CCI := true

ifeq ($(FEATURE_USE_CCI), true)
ccflags-y	+= -Idrivers/input/misc/vl53L0/inc -DCAMERA_CCI
else
ccflags-y	+= -Idrivers/input/misc/vl53L0/inc -Idrivers/input/misc/vl53L0 -DSTM_TEST
endif

ccflags-y	+= -I$(src)/inc -I$(src)/ipp -I$(src)
ccflags-y   += -Idrivers/media/platform/msm/camera/cam_utils
ccflags-y   += -Idrivers/media/platform/msm/camera/cam_cpas/include
ccflags-y   += -Idrivers/media/platform/msm/camera/cam_sensor_module/cam_sensor_io
ccflags-y   += -Idrivers/media/platform/msm/camera/cam_sensor_module/cam_res_mgr
ccflags-y   += -Idrivers/media/platform/msm/camera/cam_sensor_module/cam_sensor_utils
ccflags-y   += -Idrivers/media/platform/msm/camera/cam_req_mgr
ccflags-y   += -Idrivers/media/platform/msm/camera/cam_sensor_module/cam_cci
ccflags-y   += -Idrivers/media/platform/msm/camera/cam_smmu/

obj-$(CONFIG_STMVL53L0)			+= stmvl53l0.o
stmvl53l0-objs				:= stmvl53l0_module.o stmvl53l0_module-i2c.o stmvl53l0_module-cci.o src/vl53l0_api_calibration.o src/vl53l0_api_core.o src/vl53l0_api_histogram.o src/vl53l0_api_ranging.o src/vl53l0_api_strings.o src/vl53l0_api.o src/vl53l0_platform.o src/vl53l0_i2c_platform.o src/vl53l0_port_i2c.o src/vl53l010_api.o src/vl53l010_tuning.o

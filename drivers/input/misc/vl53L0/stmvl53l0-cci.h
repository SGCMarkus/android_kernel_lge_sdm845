/*
 *  stmvl53l0-cci.h - Linux kernel modules for STM VL53L0 FlightSense TOF sensor
 *
 *  Copyright (C) 2016 STMicroelectronics Imaging Division
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
/*
 * Defines
 */
#ifndef STMVL53L0_CCI_H
#define STMVL53L0_CCI_H
#include <linux/types.h>

#ifdef CAMERA_CCI
#include <cam_sensor_io.h>
#include <cam_req_mgr_interface.h>
#include <cam_subdev.h>
#include "cam_soc_util.h"

enum cam_ldaf_state {
	CAM_LDAF_INIT,
	CAM_LDAF_ACQUIRE,
	CAM_LDAF_START,
};

struct cam_ldaf_registered_driver_t {
	bool platform_driver;
	bool i2c_driver;
};

struct cam_ldaf_i2c_info_t {
	uint16_t slave_addr;
	uint8_t i2c_freq_mode;
};

struct cam_ldaf_soc_private {
	const char *ldaf_name;
	struct cam_ldaf_i2c_info_t i2c_info;
	struct cam_sensor_power_ctrl_t power_info;
};

struct cam_ldaf_intf_params {
	int32_t device_hdl;
	int32_t session_hdl;
	int32_t link_hdl;
	struct cam_req_mgr_kmd_ops ops;
	struct cam_req_mgr_crm_cb *crm_cb;
};

struct cam_ldaf_ctrl_t {
	struct platform_device *pdev;
	struct mutex ldaf_mutex;
	struct cam_hw_soc_info soc_info;
	struct camera_io_master io_master_info;
	enum cci_i2c_master_t cci_i2c_master;
	struct cam_subdev v4l2_dev_str;
	struct cam_ldaf_intf_params bridge_intf;
	enum msm_camera_device_type_t device_type;
	enum cam_ldaf_state cam_ldaf_state;
	char device_name[20];
	char ldaf_name[32];

	struct stmvl53l0_data *vl53l0_data;

	int irq;

	uint8_t power_up;
};

int stmvl53l0_init_cci(void);
void stmvl53l0_exit_cci(void);
int stmvl53l0_power_down_cci(void);
int stmvl53l0_power_up_cci(void);
int stmvl53l0_cci_power_status(void);
#endif /* CAMERA_CCI */
#endif /* STMVL53L0_CCI_H */

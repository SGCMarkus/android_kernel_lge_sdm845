/*
 *  stmvl53l0_module-cci.c - Linux kernel modules for STM VL53L0 FlightSense TOF
 *							sensor
 *
 *  Copyright (C) 2016 STMicroelectronics Imaging Division.
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
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
/*
 * power specific includes
 */
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
/*
 * API includes
 */
#include "vl53l0_api.h"
#include "vl53l0_def.h"
#include "vl53l0_platform.h"
#include "stmvl53l0-cci.h"
#include "stmvl53l0-i2c.h"
#include "stmvl53l0.h"

#include <cam_sensor_cmn_header.h>
#include <cam_sensor_util.h>
#include <cam_sensor_io.h>
#include <cam_req_mgr_util.h>
#include "msm_ldaf_i2c.h"

#define STMVL53L0_SLAVE_ADDR_8bit 0x52
#define E2P_SLAVE_ADDR_8bit       0xA8


#ifdef CAMERA_CCI

struct cam_ldaf_ctrl_t *local_cam_ldaf_t = NULL;
struct stmvl53l0_data *vl53l0_data = NULL;

static int32_t stmvl53l0_update_i2c_info(struct cam_ldaf_ctrl_t *p_ctrl,
		struct cam_ldaf_i2c_info_t *i2c_info)
{
	struct cam_sensor_cci_client        *cci_client = NULL;



	if (p_ctrl->io_master_info.master_type == CCI_MASTER) {
		cci_client = p_ctrl->io_master_info.cci_client;
		if (!cci_client) {
			vl53l0_dbgmsg("failed: cci_client %pK",
					cci_client);
			return -EINVAL;
		}
		cci_client->cci_i2c_master = p_ctrl->cci_i2c_master;
		cci_client->sid = STMVL53L0_SLAVE_ADDR_8bit >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;//i2c_info->i2c_freq_mode;
	}

	return 0;
}

static int stmvl53l0_get_dt_data(struct cam_ldaf_ctrl_t *p_ctrl)
{
	int                             i, rc = 0;
	struct cam_hw_soc_info         *soc_info = &p_ctrl->soc_info;
	struct cam_ldaf_soc_private     *soc_private =
		(struct cam_ldaf_soc_private *)p_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;
	struct device_node             *of_node = NULL;

	of_node = soc_info->dev->of_node;

	if (!of_node) {
		vl53l0_dbgmsg("of_node is NULL, device type %d",
			p_ctrl->device_type);
		return -EINVAL;
	}
	rc = cam_soc_util_get_dt_properties(soc_info);
	if (rc < 0) {
		vl53l0_dbgmsg("cam_soc_util_get_dt_properties rc %d",
			rc);
		return rc;
	}

	if (!soc_info->gpio_data) {
		vl53l0_dbgmsg("No GPIO found");
		return 0;
	}

	if (!soc_info->gpio_data->cam_gpio_common_tbl_size) {
		vl53l0_dbgmsg("No GPIO found");
		return -EINVAL;
	}

	rc = cam_sensor_util_init_gpio_pin_tbl(soc_info,
		&power_info->gpio_num_info);
	if ((rc < 0) || (!power_info->gpio_num_info)) {
		vl53l0_dbgmsg("No/Error PROXY GPIOs");
		return -EINVAL;
	}

	for (i = 0; i < soc_info->num_clk; i++) {
		soc_info->clk[i] = devm_clk_get(soc_info->dev,
			soc_info->clk_name[i]);
		if (!soc_info->clk[i]) {
			vl53l0_dbgmsg("get failed for %s",
				soc_info->clk_name[i]);
			rc = -ENOENT;
			return rc;
		}
	}

	return rc;
}


int stmvl53l0_driver_soc_init(struct cam_ldaf_ctrl_t *p_ctrl)
{
	int                             rc = 0;
	struct cam_hw_soc_info         *soc_info = &p_ctrl->soc_info;
	struct device_node             *of_node = NULL;

	if (!soc_info->dev) {
		vl53l0_dbgmsg("soc_info is not initialized");
		return -EINVAL;
	}

	of_node = soc_info->dev->of_node;
	if (!of_node) {
		vl53l0_dbgmsg("dev.of_node NULL");
		return -EINVAL;
	}

	if (p_ctrl->device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = of_property_read_u32(of_node, "cci-master",
			&p_ctrl->cci_i2c_master);
		if (rc < 0) {
			vl53l0_dbgmsg("failed rc %d", rc);
			return rc;
		}
	}

	rc = stmvl53l0_get_dt_data(p_ctrl);
	if (rc < 0)
		vl53l0_dbgmsg("failed: ldaf get dt data rc %d", rc);

	return rc;
}
#if 0
static int msm_tof_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int rc = 0;
/*
 *	struct msm_tof_ctrl_t *tof_ctrl =  v4l2_get_subdevdata(sd);
 *	if (!tof_ctrl) {
 *		pr_err("failed\n");
 *		return -EINVAL;
 *	}
 *	if (tof_ctrl->tof_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
 *		rc = tof_ctrl->i2c_client.i2c_func_tbl->i2c_util(
 *			&tof_ctrl->i2c_client, MSM_CCI_RELEASE);
 *		if (rc < 0)
 *			pr_err("cci_init failed\n");
 *	}
 *    tof_ctrl->i2c_state = TOF_I2C_RELEASE;
 */
	return rc;
}

static const struct v4l2_subdev_internal_ops msm_tof_internal_ops = {
	.close = msm_tof_close,
};

static long msm_tof_subdev_ioctl(struct v4l2_subdev *sd,
				 unsigned int cmd, void *arg)
{
	vl53l0_dbgmsg("Subdev_ioctl not handled\n");
	return 0;
}

static int32_t msm_tof_power(struct v4l2_subdev *sd, int on)
{
	vl53l0_dbgmsg("TOF power called\n");
	return 0;
}

static struct v4l2_subdev_core_ops msm_tof_subdev_core_ops = {
	.ioctl = msm_tof_subdev_ioctl,
	.s_power = msm_tof_power,
};

static struct v4l2_subdev_ops msm_tof_subdev_ops = {
	.core = &msm_tof_subdev_core_ops,
};

static int stmvl53l0_cci_init(struct cci_data *data)
{
	int rc = 0;
	struct msm_camera_cci_client *cci_client = data->client->cci_client;

	if (data->subdev_initialized == FALSE) {
		data->client->i2c_func_tbl = &msm_sensor_cci_func_tbl;
		data->client->cci_client =
		    kzalloc(sizeof(struct msm_camera_cci_client), GFP_KERNEL);
		if (!data->client->cci_client) {
			vl53l0_errmsg("%d, failed no memory\n", __LINE__);
			return -ENOMEM;
		}
		cci_client = data->client->cci_client;
		cci_client->cci_subdev = msm_cci_get_subdev();
		if (cci_client->cci_subdev == NULL) {
			vl53l0_errmsg("CCI subdev is not initialized!!\n");
			return -ENODEV;
		}

		cci_client->cci_i2c_master = data->cci_master;
		v4l2_subdev_init(&data->msm_sd.sd, data->subdev_ops);
		v4l2_set_subdevdata(&data->msm_sd.sd, data);
		data->msm_sd.sd.internal_ops = &msm_tof_internal_ops;
		data->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
		snprintf(data->msm_sd.sd.name,
			 ARRAY_SIZE(data->msm_sd.sd.name), "msm_tof");
		media_entity_init(&data->msm_sd.sd.entity, 0, NULL, 0);
		data->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
		data->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_TOF;
		data->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
		msm_sd_register(&data->msm_sd);
		msm_tof_v4l2_subdev_fops = v4l2_subdev_fops;
		data->msm_sd.sd.devnode->fops = &msm_tof_v4l2_subdev_fops;
		data->subdev_initialized = TRUE;
	}

	cci_client->sid = 0x29;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->cci_i2c_master = data->cci_master;
	rc = data->client->i2c_func_tbl->i2c_util(data->client, MSM_CCI_INIT);
	if (rc < 0) {
		vl53l0_errmsg("%d: CCI Init failed\n", __LINE__);
		return rc;
	}
	vl53l0_dbgmsg("CCI Init Succeeded\n");

	data->client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;

	return 0;
}
#endif
int32_t stmvl53l0_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	vl53l0_dbgmsg("Enter");
	power_info->power_setting_size = 3;
	power_info->power_setting =
		(struct cam_sensor_power_setting *)
		kzalloc(sizeof(struct cam_sensor_power_setting)
				* power_info->power_setting_size,
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VIO;
	power_info->power_setting[0].seq_val = CAM_VIO;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 0;

	power_info->power_setting[1].seq_type = SENSOR_VAF;
	power_info->power_setting[1].seq_val = CAM_VAF;
	power_info->power_setting[1].config_val = 1;
	power_info->power_setting[1].delay = 0;

	power_info->power_setting[2].seq_type = SENSOR_RESET;
//	power_info->power_setting[2].seq_val = CAM_VAF;
	power_info->power_setting[2].config_val = 1;
	power_info->power_setting[2].delay = 0;

	power_info->power_down_setting_size = 3;
	power_info->power_down_setting =
		(struct cam_sensor_power_setting *)
		kzalloc(sizeof(struct cam_sensor_power_setting)
				* power_info->power_down_setting_size,
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}
	power_info->power_down_setting[0].seq_type = SENSOR_VIO;
	power_info->power_down_setting[0].seq_val = CAM_VIO;
	power_info->power_down_setting[0].config_val = 0;
	power_info->power_down_setting[0].delay = 0;

	power_info->power_down_setting[1].seq_type = SENSOR_VAF;
	power_info->power_down_setting[1].seq_val = CAM_VAF;
	power_info->power_down_setting[1].config_val = 0;
	power_info->power_down_setting[1].delay = 0;

	power_info->power_down_setting[2].seq_type = SENSOR_RESET;
//	power_info->power_down_setting[2].seq_val = CAM_VAF;
	power_info->power_down_setting[2].config_val = 0;
	power_info->power_down_setting[2].delay = 0;

	vl53l0_dbgmsg("End rc:%d",rc);
	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	vl53l0_dbgmsg("End rc:%d",rc);
	return rc;
}

static int32_t stmvl53l0_platform_probe(struct platform_device *pdev)
{
	int32_t                         rc = 0;
	struct cam_ldaf_ctrl_t          *p_ctrl = NULL;
	struct cam_ldaf_soc_private     *soc_private = NULL;
	vl53l0_dbgmsg("%s Enter",__func__);

	p_ctrl = kzalloc(sizeof(struct cam_ldaf_ctrl_t), GFP_KERNEL);
	if (!p_ctrl)
		return -ENOMEM;

	p_ctrl->soc_info.pdev = pdev;
	p_ctrl->pdev = pdev;
	p_ctrl->soc_info.dev = &pdev->dev;
	p_ctrl->soc_info.dev_name = pdev->name;
	vl53l0_dbgmsg("%s::pdev->name:%s",__func__,pdev->name);

	p_ctrl->device_type = MSM_CAMERA_PLATFORM_DEVICE;

	p_ctrl->io_master_info.master_type = CCI_MASTER;
	p_ctrl->io_master_info.cci_client = kzalloc(
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!p_ctrl->io_master_info.cci_client)
		goto free_p_ctrl;

	soc_private = kzalloc(sizeof(struct cam_ldaf_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto free_cci_client;
	}
	p_ctrl->soc_info.soc_private = soc_private;
	soc_private->power_info.dev  = &pdev->dev;

	mutex_init(&(p_ctrl->ldaf_mutex));

	rc = stmvl53l0_driver_soc_init(p_ctrl);
	if (rc) {
		vl53l0_dbgmsg("failed: soc init rc %d", rc);
		goto free_soc;
	}
/*
	rc = stmvl53l0_init_subdev_param(p_ctrl);
	if (rc)
		goto free_soc;
*/
	rc = stmvl53l0_update_i2c_info(p_ctrl, &soc_private->i2c_info);
	if (rc) {
		vl53l0_dbgmsg("failed: to update i2c info rc %d", rc);
		goto unreg_subdev;
	}
	platform_set_drvdata(pdev, p_ctrl);
	v4l2_set_subdevdata(&p_ctrl->v4l2_dev_str.sd, p_ctrl);

	p_ctrl->cam_ldaf_state = CAM_LDAF_INIT;

/*
	rc = stmvl53l0_parse_tree(&i2c_data->client->dev, i2c_data);
	if (rc)
		goto done_freemem;
*/

	rc = stmvl53l0_default_power_setting(&soc_private->power_info);
	local_cam_ldaf_t = p_ctrl;

	if(vl53l0_data == NULL){
		int rc;
		vl53l0_dbgmsg("msm_init_ldaf_VL53L0\n");
		vl53l0_data = kzalloc(sizeof(struct stmvl53l0_data), GFP_KERNEL);
		if (!vl53l0_data) {
			rc = -ENOMEM;
			kfree(vl53l0_data);
			return rc;
		}
		//Interrupt Setting
		/*
		vl53l0_data->poll_mode = 0;
		stmvl53l0_module_func_tbl.start_intr(msm_proxy_t.i2c_driver,
		&vl53l0_data->poll_mode);
		*/

		vl53l0_dbgmsg("vl53l0_data :%p\n",vl53l0_data);
		p_ctrl->vl53l0_data = vl53l0_data;

		#if 0
		rc = request_threaded_irq(p_ctrl->irq, NULL,
		vl53l0_irq_handler_i2c,
		IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
		"vl53l0_interrupt",
		(void *)vl53l0_data);
		vl53l0_dbgmsg("request_threaded_irq rc = %d\n",rc);
		if(rc)
			return rc;
		#endif

		/* setup device name */
		vl53l0_data->dev_name = dev_name(&pdev->dev);

		/* setup other stuff */
		rc = stmvl53l0_setup(vl53l0_data);

		/* init default value */
		p_ctrl->power_up = 0;
	}else
		vl53l0_dbgmsg("vl53l0_data already initailzed\n");

	return rc;
unreg_subdev:
	cam_unregister_subdev(&(p_ctrl->v4l2_dev_str));
free_soc:
	kfree(soc_private);
free_cci_client:
	kfree(p_ctrl->io_master_info.cci_client);
free_p_ctrl:
	kfree(p_ctrl);
	return rc;
}

static int32_t stmvl53l0_platform_remove(struct platform_device *pdev)
{
	int32_t                         rc = 0;
	int                             i;
	struct cam_ldaf_ctrl_t          *p_ctrl;
	struct cam_ldaf_soc_private     *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_hw_soc_info         *soc_info;

	vl53l0_dbgmsg("Enter\n");
	p_ctrl = platform_get_drvdata(pdev);
	if (!p_ctrl) {
		vl53l0_dbgmsg("ldaf device is NULL");
		return -EINVAL;
	}

	soc_info = &p_ctrl->soc_info;
	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	soc_private =
		(struct cam_ldaf_soc_private *)p_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	kfree(p_ctrl->soc_info.soc_private);
	kfree(p_ctrl->io_master_info.cci_client);
	kfree(p_ctrl);
	kfree(vl53l0_data);

	rc = stmvl53l0_power_down_cci();
		vl53l0_dbgmsg("stmvl53l0_power_down_cci rc = %d\n",rc);

	vl53l0_dbgmsg("End\n");

	return 0;
}

static const struct of_device_id st_stmvl53l0_dt_match[] = {
	{.compatible = "st,stmvl53l0",},
	{},
};
MODULE_DEVICE_TABLE(of, st_stmvl53l0_dt_match);

static struct platform_driver stmvl53l0_platform_driver = {
	.probe = stmvl53l0_platform_probe,
	.remove = stmvl53l0_platform_remove,
	.driver = {
		   .name = STMVL53L0_DRV_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = st_stmvl53l0_dt_match,
		   },
};

int stmvl53l0_power_up_cci(void)
{
	int                             rc = 0;
	struct cam_hw_soc_info          *soc_info =
		&local_cam_ldaf_t->soc_info;
	struct cam_ldaf_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t  *power_info;

	vl53l0_dbgmsg( "%s Enter",__func__);

	soc_private =
		(struct cam_ldaf_soc_private *)local_cam_ldaf_t->soc_info.soc_private;
	power_info = &soc_private->power_info;

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		&local_cam_ldaf_t->soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		vl53l0_dbgmsg(
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		&local_cam_ldaf_t->soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		vl53l0_dbgmsg(
			"failed to fill vreg params power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		vl53l0_dbgmsg( "failed in ldaf power up rc %d", rc);
		return rc;
	}

	local_cam_ldaf_t->power_up=1;
	/* VREG needs some delay to power up */
	usleep_range(3000, 3500);

	rc = camera_io_init(&local_cam_ldaf_t->io_master_info);
	if (rc)
		vl53l0_dbgmsg( "cci_init failed: rc: %d", rc);

	vl53l0_dbgmsg( "%s End",__func__);

	return rc;
}

int stmvl53l0_power_down_cci(void)
{
	int32_t                         rc = 0;
	struct cam_sensor_power_ctrl_t  *power_info;
	struct cam_hw_soc_info          *soc_info =
		&local_cam_ldaf_t->soc_info;
	struct cam_ldaf_soc_private *soc_private;
	vl53l0_dbgmsg( "%s Enter",__func__);
	if (!local_cam_ldaf_t) {
		vl53l0_dbgmsg( "failed: local_cam_ldaf_t %pK", local_cam_ldaf_t);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ldaf_soc_private *)local_cam_ldaf_t->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &local_cam_ldaf_t->soc_info;

	if (!power_info) {
		vl53l0_dbgmsg( "failed: power_info %pK", power_info);
		return -EINVAL;
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		vl53l0_dbgmsg( "power down the core is failed:%d", rc);
		return rc;
	}

	local_cam_ldaf_t->power_up=0;
	camera_io_release(&local_cam_ldaf_t->io_master_info);
	vl53l0_dbgmsg( "%s End",__func__);
	return rc;
}

int stmvl53l0_cci_power_status(void)
{
	return local_cam_ldaf_t->power_up;
}

int32_t ldaf_i2c_read(uint32_t RegAddr, uint16_t *RegData, enum camera_sensor_i2c_type data_type)
{
	int32_t ret = 0;
	uint32_t data = 0;
/*
	vl53l0_dbgmsg("p_ctrl->io_master_info.master_type : %d, sid = 0x%x, sid << 1 = 0x%x \n",
		local_cam_ldaf_t->io_master_info.master_type,local_cam_ldaf_t->io_master_info.cci_client->sid,(local_cam_ldaf_t->io_master_info.cci_client->sid << 1));
*/
	ret = camera_io_dev_read(
		&(local_cam_ldaf_t->io_master_info),
		RegAddr,
		&data,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		data_type);

	*RegData = (uint16_t)data;
	return ret;
}
int32_t ldaf_i2c_write(uint32_t RegAddr, uint16_t RegData, enum camera_sensor_i2c_type data_type)
{
	int32_t ret = 0;

	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	uint32_t data = (uint32_t)RegData;
	//vl53l0_dbgmsg("%s :: sid=0x%x", __func__, local_cam_ldaf_t->io_master_info.cci_client->sid);
	//vl53l0_dbgmsg("%s Enter addr:0x%x, data:0x%x \n",__func__,RegAddr,RegData);

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = data_type;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * 1, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL)
	{
		vl53l0_errmsg("kzalloc failed\n");
		return -EINVAL;
	}
	i2c_reg_setting.reg_setting->reg_addr = RegAddr;
	i2c_reg_setting.reg_setting->reg_data = data;
	i2c_reg_setting.reg_setting->delay = 0;
	i2c_reg_setting.reg_setting->data_mask = 0;
	ret = camera_io_dev_write(
		&(local_cam_ldaf_t->io_master_info),
		&i2c_reg_setting);

	kfree(i2c_reg_setting.reg_setting);
	return ret;
}

int32_t ldaf_i2c_write_seq(uint32_t addr, uint8_t *data, uint16_t num_byte)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	uint16_t cnt;
	uint8_t *ptr = NULL;
	int32_t ret = 0;
	//vl53l0_dbgmsg("%s Enter num_byte:%d \n",__func__,num_byte);

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = num_byte;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * num_byte, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL)
	{
		vl53l0_errmsg("kzalloc failed\n");
		return -EINVAL;
	}

	for (cnt = 0, ptr = (uint8_t *)data; cnt < num_byte;
	cnt++, ptr++, addr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr = addr;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
		//vl53l0_dbgmsg("%s [%d] addr:0x%x, data:0x%x \n",__func__,cnt,addr,*ptr);

		}

	ret = camera_io_dev_write_continuous(&(local_cam_ldaf_t->io_master_info),
		&i2c_reg_setting, 0);

	if (ret < 0) {
		vl53l0_errmsg("ldaf fail %d", ret);
	}

	kfree(i2c_reg_setting.reg_setting);

	return ret;
}

int32_t ldaf_i2c_read_seq(uint32_t addr, uint8_t *data, uint16_t num_byte)
{
	int32_t ret = 0;

	ret = camera_io_dev_read_seq(
		&(local_cam_ldaf_t->io_master_info),
		addr,
		&data[0],
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		num_byte);

	return ret;
}

int32_t ldaf_i2c_e2p_read(uint32_t RegAddr, uint8_t *RegData, enum camera_sensor_i2c_type data_type)
{
	int32_t ret = 0;
	uint32_t data = 0;
	uint16_t temp_sid = 0;

	temp_sid = local_cam_ldaf_t->io_master_info.cci_client->sid;
	local_cam_ldaf_t->io_master_info.cci_client->sid = E2P_SLAVE_ADDR_8bit >> 1;

	ret = camera_io_dev_read(
		&(local_cam_ldaf_t->io_master_info),
		RegAddr,
		&data,
		CAMERA_SENSOR_I2C_TYPE_WORD,
		data_type);

	local_cam_ldaf_t->io_master_info.cci_client->sid = temp_sid;

	*RegData = (uint16_t)data;
	return ret;
}


int32_t ldaf_i2c_e2p_write(uint32_t RegAddr, uint8_t RegData, enum camera_sensor_i2c_type data_type)
{
	int32_t ret = 0;
	uint16_t temp_sid = 0;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	uint32_t data = (uint32_t)RegData;
	//vl53l0_dbgmsg("%s :: sid=0x%x", __func__, local_cam_ldaf_t->io_master_info.cci_client->sid);

	temp_sid = local_cam_ldaf_t->io_master_info.cci_client->sid;
	local_cam_ldaf_t->io_master_info.cci_client->sid = E2P_SLAVE_ADDR_8bit >> 1;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = data_type;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * 1, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL)
	{
		vl53l0_errmsg("kzalloc failed\n");
		local_cam_ldaf_t->io_master_info.cci_client->sid = temp_sid;
		return -EINVAL;
	}
	i2c_reg_setting.reg_setting->reg_addr = RegAddr;
	i2c_reg_setting.reg_setting->reg_data = data;
	i2c_reg_setting.reg_setting->delay = 0;
	i2c_reg_setting.reg_setting->data_mask = 0;
	ret = camera_io_dev_write(
		&(local_cam_ldaf_t->io_master_info),
		&i2c_reg_setting);

	local_cam_ldaf_t->io_master_info.cci_client->sid = temp_sid;

	kfree(i2c_reg_setting.reg_setting);
	return ret;
}

int32_t ldaf_i2c_e2p_write_seq(uint32_t addr, uint8_t *data, uint16_t num_byte)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	uint16_t cnt;
	uint8_t *ptr = NULL;
	int32_t ret = 0;
	uint16_t temp_sid = 0;

	//vl53l0_dbgmsg("%s Enter num_byte:%d \n",__func__,num_byte);
	temp_sid = local_cam_ldaf_t->io_master_info.cci_client->sid;
	local_cam_ldaf_t->io_master_info.cci_client->sid = E2P_SLAVE_ADDR_8bit >> 1;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = num_byte;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * num_byte, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL)
	{
		vl53l0_errmsg("kzalloc failed\n");
		local_cam_ldaf_t->io_master_info.cci_client->sid = temp_sid;
		return -EINVAL;
	}

	for (cnt = 0, ptr = (uint8_t *)data; cnt < num_byte;
	cnt++, ptr++, addr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr = addr;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
		//vl53l0_dbgmsg("%s [%d] addr:0x%x, data:0x%x \n",__func__,cnt,addr,*ptr);

		}

	ret = camera_io_dev_write_continuous(&(local_cam_ldaf_t->io_master_info),
		&i2c_reg_setting, 0);
	if (ret < 0) {
		vl53l0_errmsg("ldaf fail %d", ret);
	}
	local_cam_ldaf_t->io_master_info.cci_client->sid = temp_sid;

	kfree(i2c_reg_setting.reg_setting);

	return ret;
}


int stmvl53l0_init_cci(void)
{
	int ret = 0;

	vl53l0_dbgmsg("Enter\n");

	/* register as a platform device */
	ret = platform_driver_register(&stmvl53l0_platform_driver);
	if (ret)
		vl53l0_errmsg("%d, error ret:%d\n", __LINE__, ret);

	vl53l0_dbgmsg("End\n");

	return ret;
}

void stmvl53l0_exit_cci(void)
{
//	struct cci_data *data = (struct cci_data *)cci_object;

	vl53l0_dbgmsg("Enter \n");
/*
	if (data && data->client->cci_client)
		kfree(data->client->cci_client);
*/
	vl53l0_dbgmsg("End\n");
}
#endif				/* end of CAMERA_CCI */

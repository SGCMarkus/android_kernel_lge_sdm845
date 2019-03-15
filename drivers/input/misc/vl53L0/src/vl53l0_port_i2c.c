/*
 * vl53l0_port_i2c.c
 *
 *  Created on: July, 2015
 *      Author:  Teresa Tao
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include "stmvl53l0-i2c.h"
#include "stmvl53l0-cci.h"
#include "vl53l0_platform.h"
#include "vl53l0_i2c_platform.h"
#include "stmvl53l0.h"
#include "vl53l0_types.h"
#include "vl53l0_def.h"
#include "msm_ldaf_i2c.h"

#define I2C_M_WR			0x00
#define STATUS_OK			0x00
#define STATUS_FAIL			(-1)
/** int  VL53L0_I2CWrite(VL53L0_Dev_t dev, void *buff, uint8_t len);
 * @brief       Write data buffer to VL53L0 device via i2c
 * @param dev   The device to write to
 * @param buff  The data buffer
 * @param len   The length of the transaction in byte
 * @return      0 on success
 */
int VL53L0_I2CWrite(VL53L0_DEV dev, uint8_t *buff, uint8_t len)
{
	// the index is always at least 16bit
	uint32_t index = 0;
	VL53L0_Error Status = VL53L0_ERROR_NONE;

	// check for no data as the first 2 bytes are the index
	if (len<2)
		return 0;

	index = buff[0]&0xFF;

	// increment buffer past the index byte
	buff++;
	//pr_err("ldaf_i2c_write_seq(), index = 0x%X, data = %d", index,buff[0]);
	Status = ldaf_i2c_write_seq( index,buff,len-1 );

	return Status;
}


/** int VL53L0_I2CRead(VL53L0_Dev_t dev, void *buff, uint8_t len);
 * @brief       Read data buffer from VL53L0 device via i2c
 * @param dev   The device to read from
 * @param buff  The data buffer to fill
 * @param len   The length of the transaction in byte
 * @return      transaction status
 */
int VL53L0_I2CRead(VL53L0_DEV dev, uint8_t *buff, uint8_t len)
{
	// the index is always at least 16bit
	uint32_t index = 0;
	VL53L0_Error Status = VL53L0_ERROR_NONE;

	index = buff[0]&0x000000FF;

	Status = ldaf_i2c_read_seq( index, (uint8_t *)buff, len );

    //pr_err("VL53L0_I2CRead = %d, index = %d, len = %d", (uint8_t) *buff,index,len );

	return Status;
}

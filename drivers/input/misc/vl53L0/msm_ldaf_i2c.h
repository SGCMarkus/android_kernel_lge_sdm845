#ifndef LDAF_I2C_COMMON_H
#define LDAF_I2C_COMMON_H

#include <linux/i2c.h>
#include <cam_sensor_cmn_header.h>

int32_t ldaf_i2c_write(uint32_t addr, uint16_t data, enum camera_sensor_i2c_type data_type);
int32_t ldaf_i2c_read(uint32_t addr, uint16_t *data, enum camera_sensor_i2c_type data_type);
int32_t ldaf_i2c_write_seq(uint32_t addr, uint8_t *data, uint16_t num_byte);
int32_t ldaf_i2c_read_seq(uint32_t addr, uint8_t *data, uint16_t num_byte);
int32_t ldaf_i2c_read_seq_byte(uint32_t addr, uint8_t *data, uint16_t num_byte);
int32_t ldaf_i2c_e2p_write(uint32_t addr, uint8_t data, enum camera_sensor_i2c_type data_type);
int32_t ldaf_i2c_e2p_read(uint32_t addr, uint8_t *data, enum camera_sensor_i2c_type data_type);
int32_t ldaf_i2c_e2p_write_seq(uint32_t addr, uint8_t *data, uint16_t num_byte);

#endif

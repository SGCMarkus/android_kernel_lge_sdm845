#ifndef LGIT_ROHM_OIS_H
#define LGIT_ROHM_OIS_H

//OIS ERROR CODE
#define OIS_SUCCESS	 0
#define OIS_FAIL	-1
#define OIS_INIT	-9
#define OIS_INIT_OLD_MODULE		 1
#define OIS_INIT_NOT_SUPPORTED  -2
#define OIS_INIT_CHECKSUM_ERROR -3
#define OIS_INIT_EEPROM_ERROR   -4
#define OIS_INIT_I2C_ERROR      -5
#define OIS_INIT_TIMEOUT		-6
#define OIS_INIT_LOAD_BIN_ERROR -7
#define OIS_INIT_NOMEM			-8
#define OIS_INIT_GYRO_ADJ_FAIL	 2
#define OIS_INIT_SRV_GAIN_FAIL	 4

int32_t ois_i2c_e2p_write(uint32_t e2p_addr, uint16_t e2p_data, enum camera_sensor_i2c_type data_type);
int32_t ois_i2c_e2p_read(uint32_t e2p_addr, uint16_t *e2p_data, enum camera_sensor_i2c_type data_type);

int32_t RamWriteA(uint32_t RamAddr, uint16_t RamData);
int32_t RamReadA(uint32_t RamAddr, uint16_t *ReadData);
int32_t RamRead32A(uint32_t RamAddr, uint32_t *ReadData);
int32_t RegWriteA(uint32_t RegAddr, uint8_t RegData);
int32_t RegWriteB(uint32_t RegAddr, uint16_t RegData);
int32_t RegReadA(uint32_t RegAddr, uint8_t *RegData);
int32_t RegReadB(uint32_t RegAddr, uint16_t *RegData);
static int8_t ois_selftest(void);
static int8_t ois_selftest2(void);

void msm_ois_create_sysfs(void);
void msm_ois_destroy_sysfs(void);

int32_t ois_i2c_read_seq(uint32_t addr, uint8_t *data, uint16_t num_byte);

#define MAX_OIS_BIN_FILENAME 128
#define MAX_OIS_BIN_BLOCKS 4
#define MAX_OIS_BIN_FILES 3

struct ois_i2c_bin_addr {
	uint32_t bin_str_addr;
	uint32_t bin_end_addr;
	uint32_t reg_str_addr;
};

struct ois_i2c_bin_entry {
	char  filename[MAX_OIS_BIN_FILENAME];
	uint32_t filesize;
	uint16_t blocks;
	struct ois_i2c_bin_addr addrs[MAX_OIS_BIN_BLOCKS];
};

struct ois_i2c_bin_list {
	uint16_t files;
	struct ois_i2c_bin_entry entries[MAX_OIS_BIN_FILES];
	uint32_t checksum;
};

typedef struct {
    char ois_provider[32];
    int16_t gyro[2];
    int16_t target[2];
    int16_t hall[2];
    uint8_t is_stable;
} sensor_ois_stat_t;

#define MAX_GYRO_QUERY_SIZE 15
#define READ_OUT_TIME 5000000 /*5ms*/
#define MAX_FAIL_CNT 3

enum msm_ois_timer_state_t {
	OIS_TIME_INIT,
	OIS_TIME_ACTIVE,
	OIS_TIME_INACTIVE,
	OIS_TIME_ERROR,
};

enum OISUserData {
	NO_SELECTION,
	HALL_FEEDING,
	USERDATAEND
};

struct ois_timer {
	struct hrtimer hr_timer;
	struct workqueue_struct *ois_wq;
	struct work_struct g_work;
	enum msm_ois_timer_state_t ois_timer_state;
	struct cam_ois_ctrl_t *o_ctrl;
	int i2c_fail_count;
};

int lgit_imx351_rohm_ois_poll_ready(int limit);
int32_t ois_i2c_load_and_write_e2prom_data(uint32_t e2p_str_addr, uint16_t e2p_data_length, uint16_t reg_str_addr);
int32_t ois_i2c_load_and_write_bin_list(struct ois_i2c_bin_list bin_list);

#endif

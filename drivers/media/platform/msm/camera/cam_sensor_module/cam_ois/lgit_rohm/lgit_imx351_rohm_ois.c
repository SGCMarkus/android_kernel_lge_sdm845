/* ================================================================== */
/*  OIS firmware */
/* ================================================================== */
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "lgit_imx351_rohm_ois.h"
#include <linux/device.h>
#include <linux/spinlock.h>
#include <asm/arch_timer.h>

#define LAST_UPDATE "18-01-23, OIS bu24235"

#define I2C_NUM_BYTE        (6)  //QCT maximum size is 6!
#define END_FIRST_BLOCK_CAL_DATA  0x1A
#define STR_SECOND_BLOCK_CAL_DATA 0x1C

/*If changed FW, change below FW bin and Checksum information*/
#define JUDY_1023_LGIT_OLAF_ACTUATOR_FIRMWARE_VER_0_BIN_1 "bu24235_dl_program_JUDY_JOAN_LGITAct_ICG1020S_rev0_S_data1.bin"
#define JUDY_1023_LGIT_OLAF_ACTUATOR_FIRMWARE_VER_0_BIN_2 "bu24235_dl_program_JUDY_JOAN_LGITAct_ICG1020S_rev0_S_data2.bin"
#define JUDY_1226_JUDY_NEO_FIRMWARE_VER_0_BIN_1 "bu24235_dl_program_Judy_LGITAct_ICG1020S_rev0_S_data1.bin"
#define JUDY_1226_JUDY_NEO_FIRMWARE_VER_0_BIN_2 "bu24235_dl_program_Judy_LGITAct_ICG1020S_rev0_S_data2.bin"
#define JUDY_0119_JUDY_EMMA_FIRMWARE_VER_1_BIN_1 "bu24235_dl_program_Judy_emma_LGIT_ICG1020S_rev1_S_data1.bin"
#define JUDY_0119_JUDY_EMMA_FIRMWARE_VER_1_BIN_2 "bu24235_dl_program_Judy_emma_LGIT_ICG1020S_rev1_S_data2.bin"

#define JUDY_1023_LGIT_OLAF_ACTUATOR_FIRMWARE_VER_0_CHECKSUM	0x00014523
#define JUDY_1226_JUDY_NEO_FIRMWARE_VER_0_CHECKSUM	0x0001E8FE
#define JUDY_0119_JUDY_EMMA_FIRMWARE_VER_1_CHECKSUM	0x0001E83D

/*If changed FW, change above FW bin and Checksum information*/
#define E2P_SID					(0xA8)
#define E2P_FIRST_ADDR			(0xA90)
#define E2P_DATA_BYTE_FIRST		(48)
#define E2P_MAP_VER_ADDR        (0xBE2)
#define CAL_VER					(0xAC2)

#define CTL_END_ADDR_FOR_FIRST_E2P_DL	(0x1DC0)
#define CTL_END_ADDR_FOR_SECOND_E2P_DL	(0x1DDA)

#define OIS_START_DL_ADDR		(0xF010)
#define OIS_COMPLETE_DL_ADDR	(0xF006)
#define OIS_READ_STATUS_ADDR	(0x6024)
#define OIS_CHECK_SUM_ADDR		(0xF008)

#define LIMIT_STATUS_POLLING	(15)
#define LIMIT_OIS_ON_RETRY		(5)

#define GYRO_SCALE_FACTOR 175
#define HALL_SCALE_FACTOR 8192
#define GYRO_GAIN_LGIT 6900

#define NUM_GYRO_SAMLPING (10)

#define PROPERTY_VALUE_MAX 92

#define OIS_HALL_SEQ_ADDR       (0x6140)//(0xE001)
#define OIS_HALL_X_ADDR         (0x6142)//(0x01B0)
#define OIS_HALL_Y_ADDR         (0x6144)//(0x01B4)
#define OIS_GYRO_ADDR           (0xE002)
#define OIS_HALL_X_OFFSET       (0x0110)
#define OIS_HALL_Y_OFFSET       (0x0160)

/***********************************
614Eh : Decrement Gyro Gain X
        unsigned 8bit
        [7:0] Decrement of calibrated gyro gain Xch [%]
              (0~100d. Over 100d input is treated as 100d)

614Fh : Decrement Gyro Gain Y
        unsigned 8bit
        [7:0] Decrement of calibrated gyro gain Ych [%]
              (0~100d. Over 100d input is treated as 100d)

Writing to these registers is valid only when Servo ON(6020h = 01h) or OIS ON (6020h = 02h).
***********************************/
//#define GYRO_GAIN_X 0x46
//#define GYRO_GAIN_Y 0x46

extern struct class* get_camera_class(void);
static struct class *camera_ois_hall_class = NULL;
static struct ois_timer ois_timer_t;
static void msm_ois_read_work(struct work_struct *work);
static bool msm_ois_data_enqueue(uint64_t x_readout_time, int16_t x_shift,
		uint64_t y_readout_time, int16_t y_shift, struct msm_ois_readout_buffer *o_buf);
static int msm_startGyroThread(struct cam_ois_ctrl_t *o_ctrl);
extern int msm_stopGyroThread(void);
extern int parse_ois_userdata(struct cam_cmd_ois_userdata *ois_userdata,
		struct cam_ois_ctrl_t *o_ctrl);

static struct class *ois_aat_result_class = NULL;
struct device*	ois_aat_result1;
struct device*	ois_aat_result2;
static char aat_selftest_result[PROPERTY_VALUE_MAX] = "000000000000";
static char aat_vendor_name[PROPERTY_VALUE_MAX] = "UNKNOWN";

extern int cam_ois_power_up(struct cam_ois_ctrl_t *o_ctrl);
extern int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl);

static ssize_t show_ois_aat_selftest_result(struct device *dev, struct device_attribute *attr, char *buf)
{
	CAM_ERR(CAM_OIS, "show_ois_aat_selftest_result: [%s] \n", aat_selftest_result);
	return sprintf(buf, "%s\n", aat_selftest_result);
}
static DEVICE_ATTR(ois_aat_selftest_result, S_IRUGO, show_ois_aat_selftest_result, NULL);

static ssize_t show_ois_aat_vendor_name(struct device *dev,struct device_attribute *attr, char *buf)
{
	CAM_ERR(CAM_OIS, "show_ois_aat_vendor_name: [%s]\n", aat_vendor_name);
	return sprintf(buf, "%s\n", aat_vendor_name);
}
static DEVICE_ATTR(ois_aat_vendor_name, S_IRUGO, show_ois_aat_vendor_name, NULL);

static struct cam_ois_ctrl_t *local_cam_ois_t;
static int16_t g_gyro_offset_value_x, g_gyro_offset_value_y;
static uint16_t cal_ver = 0;

/*If changed FW, change below FW bin and Checksum information*/

static struct ois_i2c_bin_list JUDY_1023_LGIT_OLAF_ACTUATOR_FIRMWARE_VER_0_BIN_DATA = {
	.files = 2,
	.entries = {
		{
			.filename = JUDY_1023_LGIT_OLAF_ACTUATOR_FIRMWARE_VER_0_BIN_1,
			.filesize = 0x0338, //824byte
			.blocks = 1,
			.addrs = {
				{0x0000, 0x0337, 0x0000},
			}
		},
		{
			.filename = JUDY_1023_LGIT_OLAF_ACTUATOR_FIRMWARE_VER_0_BIN_2,
			.filesize = 0x01C0,
			.blocks = 1,
			.addrs = {
				{0x0000, 0x01BF, 0x1C00},
			}
		},
	},
	.checksum = JUDY_1023_LGIT_OLAF_ACTUATOR_FIRMWARE_VER_0_CHECKSUM
};

static struct ois_i2c_bin_list JUDY_1226_JUDY_NEO_FIRMWARE_VER_0_BIN_DATA = {
	.files = 2,
	.entries = {
		{
			.filename = JUDY_1226_JUDY_NEO_FIRMWARE_VER_0_BIN_1,
			.filesize = 0x0574, //1396byte
			.blocks = 1,
			.addrs = {
				{0x0000, 0x0573, 0x0000},
			}
		},
		{
			.filename = JUDY_1226_JUDY_NEO_FIRMWARE_VER_0_BIN_2,
			.filesize = 0x01C0, //448byte
			.blocks = 1,
			.addrs = {
				{0x0000, 0x01BF, 0x1C00},
			}
		},
	},
	.checksum = JUDY_1226_JUDY_NEO_FIRMWARE_VER_0_CHECKSUM
};

static struct ois_i2c_bin_list JUDY_0119_JUDY_EMMA_FIRMWARE_VER_1_BIN_DATA = {
	.files = 2,
	.entries = {
		{
			.filename = JUDY_0119_JUDY_EMMA_FIRMWARE_VER_1_BIN_1,
			.filesize = 0x0574, //1396byte
			.blocks = 1,
			.addrs = {
				{0x0000, 0x0573, 0x0000},
			}
		},
		{
			.filename = JUDY_0119_JUDY_EMMA_FIRMWARE_VER_1_BIN_2,
			.filesize = 0x01C0, //448byte
			.blocks = 1,
			.addrs = {
				{0x0000, 0x01BF, 0x1C00},
			}
		},
	},
	.checksum = JUDY_0119_JUDY_EMMA_FIRMWARE_VER_1_CHECKSUM
};

/*If changed FW, change above FW bin and Checksum information*/

int lgit_imx351_rohm_ois_poll_ready(int limit)
{
	uint8_t ois_status = 0;
	int read_byte = 0;
	int rc = OIS_SUCCESS;

	usleep_range(1000, 1000 + 10); /* wait 1ms */

	while ((ois_status != 0x01) && (read_byte < limit)) {
		rc = RegReadA(OIS_READ_STATUS_ADDR, &ois_status); /* polling status ready */
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS_I2C_ERROR");
			return OIS_INIT_I2C_ERROR;
		}
		usleep_range(5* 1000, 5* 1000 + 10); /* wait 5ms */
		read_byte++;
	}

	if (ois_status == 0x01) {
		return OIS_SUCCESS;
	}
	else {
		CAM_ERR(CAM_OIS, "OIS_TIMEOUT_ERROR");
		return OIS_INIT_TIMEOUT;
	}
}

static ssize_t show_ois_hall(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;

	if(ois_timer_t.ois_timer_state == OIS_TIME_ACTIVE) {
		// Put Hall data
		struct msm_ois_readout gyro_data[MAX_GYRO_QUERY_SIZE];
		uint8_t query_size = MAX_GYRO_QUERY_SIZE;
		uint8_t data_get_count = 0;
		uint16_t counter = 0;
		int i;

		memset(gyro_data, 0, sizeof(gyro_data));

		spin_lock(&local_cam_ois_t->gyro_lock);

		counter = local_cam_ois_t->buf.buffer_tail;
		for (i = query_size-1; i >= 0; i-- )
		{
			counter = (counter == 0) ? MSM_OIS_DATA_BUFFER_SIZE-1 : counter-1;
			gyro_data[i].ois_x_shift = local_cam_ois_t->buf.buffer[counter].ois_x_shift;
			gyro_data[i].ois_y_shift = local_cam_ois_t->buf.buffer[counter].ois_y_shift;
			gyro_data[i].x_readout_time = local_cam_ois_t->buf.buffer[counter].x_readout_time;
			gyro_data[i].y_readout_time = local_cam_ois_t->buf.buffer[counter].y_readout_time;
			data_get_count++;
		}
		spin_unlock(&local_cam_ois_t->gyro_lock);

		if (data_get_count != 0 && data_get_count < MAX_GYRO_QUERY_SIZE + 1) {
			for(i = 0; i < data_get_count; i++) {
				count += sprintf(buf + strlen(buf),"%d,%016llx,%d,%016llx,\n",gyro_data[i].ois_x_shift, gyro_data[i].x_readout_time, gyro_data[i].ois_y_shift, gyro_data[i].y_readout_time);
			}
		}

		// Check Gyro data count
		CAM_DBG(CAM_OIS, "[OIS]:data_get_count = %d count : %d", data_get_count,count);
	}
	else {
		CAM_ERR(CAM_OIS, "OIS is not working");
	}

	return count;
}
static DEVICE_ATTR(ois_hall, S_IRUGO, show_ois_hall, NULL);

/*Get OIS gyro data via i2c.*/
static void msm_ois_read_work(struct work_struct *work)
{
	int32_t rc = 0;
	bool result;
	uint8_t buf[6] = {0,};
	uint64_t x_readout_time;
	uint64_t y_readout_time;
	int16_t x_shift = 0;
	int16_t y_shift = 0;
	struct ois_timer *ois_timer_in_t = NULL;

	ois_timer_in_t = container_of(work, struct ois_timer, g_work);

	// Read Hall data sequencelly (Timestamp -> Hall X & Y)
	rc = ois_i2c_read_seq(OIS_HALL_SEQ_ADDR, buf, 6);
	x_readout_time = y_readout_time  = arch_counter_get_cntvct();

	x_shift = (int16_t)(((buf[2] & 0x0F) << 8) | (buf[3]));
	y_shift = (int16_t)(((buf[4] & 0x0F) << 8) | (buf[5]));
//	CAM_ERR(CAM_OIS, "[OIS] Hall x : %hd y : %hd", x_shift, y_shift);

	if (rc != 0) {
		ois_timer_t.i2c_fail_count++;
		CAM_ERR(CAM_OIS, "[OIS] %s : i2c_read_seq fail. cnt = %d",
				__func__, ois_timer_t.i2c_fail_count);
		if (ois_timer_t.i2c_fail_count == MAX_FAIL_CNT) {
			CAM_ERR(CAM_OIS, "[OIS] %s : Too many i2c failed. Stop timer.",
					__func__);
			ois_timer_t.ois_timer_state = OIS_TIME_ERROR;
		}
	} else {
		ois_timer_t.i2c_fail_count = 0;
		result = msm_ois_data_enqueue(x_readout_time, x_shift,
				y_readout_time, y_shift, &ois_timer_in_t->o_ctrl->buf);

		if (!result)
			CAM_ERR(CAM_OIS, "%s %d ois data enqueue ring buffer failed",
					__func__, __LINE__);
	}
}

static bool msm_ois_data_enqueue(uint64_t x_readout_time,
		int16_t x_shift,
		uint64_t y_readout_time,
		int16_t y_shift,
		struct msm_ois_readout_buffer *o_buf)
{
	bool rc;

	spin_lock(&local_cam_ois_t->gyro_lock);

	if (o_buf->buffer_tail >= 0 && o_buf->buffer_tail <
			MSM_OIS_DATA_BUFFER_SIZE) {
		o_buf->buffer[o_buf->buffer_tail].ois_x_shift = x_shift;
		o_buf->buffer[o_buf->buffer_tail].ois_y_shift = y_shift;
		o_buf->buffer[o_buf->buffer_tail].x_readout_time = x_readout_time;
		o_buf->buffer[o_buf->buffer_tail].y_readout_time = y_readout_time;

		o_buf->buffer_tail++;
		if (o_buf->buffer_tail >= MSM_OIS_DATA_BUFFER_SIZE)
			o_buf->buffer_tail -= MSM_OIS_DATA_BUFFER_SIZE;

		rc = true;
	} else {
		rc = false;
	}

	spin_unlock(&local_cam_ois_t->gyro_lock);

	return rc;
}

static enum hrtimer_restart msm_gyro_timer(struct hrtimer *timer)
{
	ktime_t currtime, interval;
	struct ois_timer *ois_timer_in_t;

	ois_timer_in_t = container_of(timer, struct ois_timer, hr_timer);
	if ((ois_timer_in_t->o_ctrl->cam_ois_state >= CAM_OIS_CONFIG)
			&& (ois_timer_t.ois_timer_state != OIS_TIME_ERROR)) {
		queue_work(ois_timer_in_t->ois_wq, &ois_timer_in_t->g_work);
		currtime  = ktime_get();
		interval = ktime_set(0, READ_OUT_TIME);
		hrtimer_forward(timer, currtime, interval);

		return HRTIMER_RESTART;
	} else {
		CAM_ERR(CAM_OIS, "[OIS] %s HRTIMER_NORESTART ois_state : %d timer_state : %d",
				__func__, ois_timer_in_t->o_ctrl->cam_ois_state, ois_timer_t.ois_timer_state);
		return HRTIMER_NORESTART;
	}
}

static int msm_startGyroThread(struct cam_ois_ctrl_t *o_ctrl)
{
	ktime_t  ktime;

	CAM_INFO(CAM_OIS, "[OIS] %s:E", __func__);

	if (ois_timer_t.ois_timer_state == OIS_TIME_ERROR) {
		CAM_ERR(CAM_OIS, "[OIS] %s:Timer error, close befoe create :%d.",
				__func__, ois_timer_t.ois_timer_state);
		msm_stopGyroThread();
	}

	ois_timer_t.i2c_fail_count = 0;
	if (ois_timer_t.ois_timer_state != OIS_TIME_ACTIVE) {
		ois_timer_t.o_ctrl = o_ctrl;
        INIT_WORK(&ois_timer_t.g_work, msm_ois_read_work);
		ois_timer_t.ois_wq = create_workqueue("ois_wq");
		if (!ois_timer_t.ois_wq) {
			CAM_ERR(CAM_OIS, "[OIS]:%s ois_wq create failed.", __func__);
			return -EFAULT;
		}
		ktime = ktime_set(0, READ_OUT_TIME);
		hrtimer_init(&ois_timer_t.hr_timer, CLOCK_MONOTONIC,
				HRTIMER_MODE_REL);
		ois_timer_t.hr_timer.function = &msm_gyro_timer;
		hrtimer_start(&ois_timer_t.hr_timer, ktime,
				HRTIMER_MODE_REL);
		ois_timer_t.ois_timer_state = OIS_TIME_ACTIVE;
	} else
		CAM_ERR(CAM_OIS, "[OIS] invalid timer state = %d.",
				ois_timer_t.ois_timer_state);
	CAM_DBG(CAM_OIS, "[OIS] %s:X", __func__);
	return 0;
}
extern int msm_stopGyroThread(void)
{
	CAM_INFO(CAM_OIS, "[OIS] %s:E", __func__);

	if ((ois_timer_t.ois_timer_state == OIS_TIME_ACTIVE) ||
			(ois_timer_t.ois_timer_state == OIS_TIME_ERROR)) {
		CAM_INFO(CAM_OIS, "[OIS] %s:timer cancel.", __func__);
		hrtimer_cancel(&ois_timer_t.hr_timer);
		destroy_workqueue(ois_timer_t.ois_wq);
		ois_timer_t.ois_timer_state = OIS_TIME_INACTIVE;
	} else
		CAM_ERR(CAM_OIS, "[OIS] invalid timer state = %d",
				ois_timer_t.ois_timer_state);
	CAM_DBG(CAM_OIS, "[OIS] %s:X", __func__);
	return 0;
}

extern int parse_ois_userdata(
        struct cam_cmd_ois_userdata *ois_userdata,
        struct cam_ois_ctrl_t *o_ctrl)
{
    struct cam_ois_ctrl_t *ctrl = o_ctrl;
    int rc = -1;
    switch(ois_userdata->select)
    {
        case HALL_FEEDING:
            if(ois_userdata->oisThreadNeeded == 1) {
                rc = msm_startGyroThread(ctrl);
                ctrl->ois_thread_running = true;
            } else {
                rc = msm_stopGyroThread();
                ctrl->ois_thread_running = false;
            }
            break;
        default:
            break;
    }

    return rc;
}

int lgit_imx351_rohm_ois_bin_download(struct ois_i2c_bin_list bin_list)
{
	int rc = 0;
	int32_t read_value_32t;

	/* check OIS ic is alive */
	rc = lgit_imx351_rohm_ois_poll_ready(LIMIT_STATUS_POLLING);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS_POLLING_ERROR");
		return rc;
	}

	/* Send command ois start DownLoad */
	rc = RegWriteA(OIS_START_DL_ADDR, 0x00);

#if 0 /* request by rohm */
	while (rc < 0 && cnt < LIMIT_STATUS_POLLING) {
		usleep_range(2000, 2010);
		rc = RegWriteA(OIS_START_DL_ADDR, 0x00);
		cnt++;
	}
#endif

	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS_I2C_ERROR");
		rc = OIS_INIT_I2C_ERROR;
		return rc;
	}

	/* OIS program downloading */
	rc = ois_i2c_load_and_write_bin_list(bin_list);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS_BIN_DOWNLOAD_FAIL");
		return rc;
	}

#if 1
	/* Check sum value!*/
	RamRead32A(OIS_CHECK_SUM_ADDR, &read_value_32t);
	if (read_value_32t != bin_list.checksum) {
		CAM_ERR(CAM_OIS, "saved sum = 0x%x, read sum = 0x%x\n", bin_list.checksum, read_value_32t);
		rc = OIS_INIT_CHECKSUM_ERROR;
		return rc;
	}
#else
	RamRead32A(OIS_CHECK_SUM_ADDR, &read_value_32t);
	CAM_ERR(CAM_OIS, "saved sum = 0x%x, read sum = 0x%x\n", bin_list.checksum, read_value_32t);
#endif
	/* If Change EEPROM MAP, change below */
	rc = ois_i2c_load_and_write_e2prom_data(E2P_FIRST_ADDR, E2P_DATA_BYTE_FIRST, CTL_END_ADDR_FOR_FIRST_E2P_DL);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "ois_i2c_load_and_write_e2prom_data failed (rc: %d)", rc);
		return rc;
	}
	/* Send command ois complete dl */
	RegWriteA(OIS_COMPLETE_DL_ADDR, 0x00);

	/* Read ois status */
	rc = lgit_imx351_rohm_ois_poll_ready(LIMIT_STATUS_POLLING);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS_POLLING_ERROR");
		return rc;
	}
	return rc;
}

int lgit_imx351_rohm_ois_init_cmd(int limit)
{
	int rc = 0;
	uint8_t ois_status = 0;

	RegReadA(0x6020, &ois_status);
	CAM_DBG(CAM_OIS, "0x6020 status : 0x%d", ois_status);
	if(ois_status != 0x01) {
		RegWriteA(0x6020, 0x01);
//		RegWriteA(0x614E, GYRO_GAIN_X); //for new FW
//		RegWriteA(0x614F, GYRO_GAIN_Y); //for new FW
	}
	rc = lgit_imx351_rohm_ois_poll_ready(LIMIT_STATUS_POLLING + limit);
	//do rc check after VCM Init for AF

	//VCM Init Code
	RegWriteA(0x60F1, 0x02);
	//vcm_check = 1;

#if 0 //VCM Test Code
	RegWriteB(0x60E4, 0x00); // 0x1FF: 511, 0x200: -512
#endif

	//do rc check after VCM Init for AF
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS_POLLING_ERROR");
		return rc;
	}

	return OIS_SUCCESS;
}

int lgit_imx351_rohm_ois_calibration(void)
{
	int16_t gyro_offset_value_x, gyro_offset_value_y = 0;
	uint8_t ois_status = 0;
	int rc = 0;

	CAM_ERR(CAM_OIS, "%s: lgit_ois_calibration start\n", __func__);
	/* Gyro Zero Calibration Starts. */
	//20151127 Rohm_LeeDJ
	/* Read ois status */
	RegReadA(0x6020, &ois_status);

	CAM_DBG(CAM_OIS, "0x6020 status : 0x%d", ois_status);
	if(ois_status != 0x01) {
			RegWriteA(0x6020, 0x01);
//			RegWriteA(0x614E, GYRO_GAIN_X); //for new FW
//			RegWriteA(0x614F, GYRO_GAIN_Y); //for new FW
			usleep_range(100 * 1000, 100 * 1000 + 10); // 20151127 Rohm_LeeDJ_delay 100ms
	}

	rc = lgit_imx351_rohm_ois_poll_ready(LIMIT_STATUS_POLLING);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS_POLLING_ERROR");
		return rc;
	}

	/* Gyro On */
	RegWriteA(0x6023, 0x00);
	rc = lgit_imx351_rohm_ois_poll_ready(LIMIT_STATUS_POLLING);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS_POLLING_ERROR");
		return rc;
	}

	RegReadA(0x6023, &ois_status);
	CAM_ERR(CAM_OIS, "2. 0x6023 0x%x", ois_status);

	/* Select Xch Gyro */
	RegWriteA(0x6088, 0);

	rc = lgit_imx351_rohm_ois_poll_ready(LIMIT_STATUS_POLLING);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS_POLLING_ERROR");
		return rc;
	}

	/* Read Xch Gyro Offset */
	RegReadB(0x608A, &gyro_offset_value_x);

	/* Select Ych Gyro */
	RegWriteA(0x6088, 1);

	rc = lgit_imx351_rohm_ois_poll_ready(LIMIT_STATUS_POLLING);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS_POLLING_ERROR");
		return rc;
	}


	/* Read Ych Gyro Offset */
	RegReadB(0x608A, &gyro_offset_value_y);

	/* If Change EEPROM MAP, change below */
	/* Cal. Data to eeprom */
	ois_i2c_e2p_write(0xA98, (uint16_t)(0xFFFF & gyro_offset_value_x), CAMERA_SENSOR_I2C_TYPE_WORD);
	usleep_range(100 * 1000, 100 * 1000 + 10);
	ois_i2c_e2p_write(0xA9A, (uint16_t)(0xFFFF & gyro_offset_value_y), CAMERA_SENSOR_I2C_TYPE_WORD);
	/* gyro_offset_value_x -> gyro_offset_value_y*/

	/* Cal. Data to OIS Driver */
	RegWriteA(0x609C, 0x00); //Changed rohm 0926 LeeDJ
	RamWriteA(0x609D, gyro_offset_value_x); /* 16 */

	RegWriteA(0x609C, 0x01); //Changed rohm 0926 LeeDJ
	RamWriteA(0x609D, gyro_offset_value_y); /* 16 */

	RegWriteA(0x6023, 0x00); //added rohm 0926 LeeDJ

	/* Gyro Zero Calibration Ends. */
	CAM_ERR(CAM_OIS, "gyro_offset_value_x %d gyro_offset_value_y %d", gyro_offset_value_x, gyro_offset_value_y);
	g_gyro_offset_value_x = gyro_offset_value_x;
	g_gyro_offset_value_y = gyro_offset_value_y;

	CAM_ERR(CAM_OIS, "lgit_ois_calibration end");
	return OIS_SUCCESS;
}

int32_t lgit_imx351_init_set_rohm_ois(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t rc = OIS_INIT;
	int cnt = 0;
	local_cam_ois_t = o_ctrl;

	CAM_ERR(CAM_OIS, "Enter, %s", LAST_UPDATE);

	ois_i2c_e2p_read(CAL_VER, &cal_ver, CAMERA_SENSOR_I2C_TYPE_WORD);

	CAM_ERR(CAM_OIS, "cal_ver 0x%x slave_addr 0x%x", cal_ver, local_cam_ois_t->io_master_info.cci_client->sid);

	while ((rc < 0) && (cnt < LIMIT_OIS_ON_RETRY)) {
		if (rc != OIS_INIT) {
			rc = cam_ois_power_down(o_ctrl);
			if (rc < 0)	{
				CAM_ERR(CAM_OIS, "OIS Power down failed");
				return rc;
			}
			rc = cam_ois_power_up(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, "failed in ois power up rc %d", rc);
				return rc;
			}
		}
		switch (cal_ver) {
			case 0x011:
				CAM_ERR(CAM_OIS, "[CHECK] JUDY_1226_JUDY_NEO_FIRMWARE_VER_0_BIN_DATA, 0M_0S");
				rc = lgit_imx351_rohm_ois_bin_download(JUDY_1226_JUDY_NEO_FIRMWARE_VER_0_BIN_DATA);
				break;
			case 0x111:
				CAM_ERR(CAM_OIS, "[CHECK] JUDY_0119_JUDY_EMMA_FIRMWARE_VER_1_BIN_DATA, 1M_1S");
				rc = lgit_imx351_rohm_ois_bin_download(JUDY_0119_JUDY_EMMA_FIRMWARE_VER_1_BIN_DATA);
				break;
			case 0x0301:
				CAM_ERR(CAM_OIS, "[CHECK] JUDY_1023_LGIT_OLAF_ACTUATOR_FIRMWARE_VER_0_BIN_DATA, 3M_5S");
				rc = lgit_imx351_rohm_ois_bin_download(JUDY_1023_LGIT_OLAF_ACTUATOR_FIRMWARE_VER_0_BIN_DATA);
				break;
			default:
				CAM_ERR(CAM_OIS, "[CHECK] Apply Default : No Download BIN_DATA cal_ver:0x%x", cal_ver);
				rc = -EFAULT;
				break;
		}
		cnt++;
	}
	if (rc < 0)	{
		CAM_ERR(CAM_OIS, "init fail");
		return rc;
	}

	if (o_ctrl->is_ois_aat) {
		CAM_ERR(CAM_OIS, "OIS_VER_calibration");
		lgit_imx351_rohm_ois_calibration();
		rc = lgit_imx351_rohm_ois_poll_ready(LIMIT_STATUS_POLLING);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS_POLLING_ERROR");
			return rc;
			}

			/* OIS ON */
			RegWriteA(0x6021, 0x03);/* LGIT STILL & PAN ON MODE */
			usleep_range(10 * 1000, 10 * 1000 + 10); // 20151127 Rohm_LeeDJ_delay 10ms
			RegWriteA(0x6020, 0x02);/* OIS ON */
//			RegWriteA(0x614E, GYRO_GAIN_X); //for new FW
//			RegWriteA(0x614F, GYRO_GAIN_Y); //for new FW

			rc = lgit_imx351_rohm_ois_poll_ready(LIMIT_STATUS_POLLING);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "OIS_POLLING_ERROR");
				return rc;
				}
			ois_selftest();
			ois_selftest2();
			}
	else {
		CAM_ERR(CAM_OIS, "OIS_VER_RELEASE");
		lgit_imx351_rohm_ois_init_cmd(LIMIT_OIS_ON_RETRY);
		}
	CAM_ERR(CAM_OIS, "done.");
	return rc;
}

#if 0
int32_t	lgit_imx351_rohm_ois_on(struct cam_ois_ctrl_t *o_ctrl,
					 struct msm_ois_set_info_t *set_info)
{
	int32_t rc = OIS_SUCCESS;
	CAM_ERR(CAM_OIS, "Enter");
#if 0
	RegWriteA(0x6020, 0x01);
	RegWriteA(0x6021, 0x79); /* LGIT STILL & PAN ON MODE */
	RegWriteA(0x613F, 0x01);
	RegWriteA(0x6020, 0x02); /* OIS ON */
#endif
	CAM_ERR(CAM_OIS, "End");
	return rc;
}
#endif

#if 0
int32_t	lgit_imx351_rohm_ois_off(struct cam_ois_ctrl_t *o_ctrl)//,
//					  struct msm_ois_set_info_t *set_info)
{
	int rc = OIS_SUCCESS;

	RegWriteA(0x6020, 0x01);
	rc = lgit_imx351_rohm_ois_poll_ready(LIMIT_STATUS_POLLING);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS_POLLING_ERROR");
		return rc;
	}

	RegWriteA(0x6020, 0x00); //Standby Mode
	vcm_check = 0;
	return rc;
}
#endif

extern void oeis_create_sysfs(void) {
	struct device*  camera_ois_hall_dev;
	if(!camera_ois_hall_class) {
		CAM_INFO(CAM_OIS, "create ois_hall_class!!");
		camera_ois_hall_class = get_camera_class();
		camera_ois_hall_dev   = device_create(camera_ois_hall_class, NULL,
				0, NULL, "ois_hall");
		device_create_file(camera_ois_hall_dev, &dev_attr_ois_hall);
	}
}

extern void oeis_destroy_sysfs(void) {
	if(camera_ois_hall_class) {
		class_destroy(camera_ois_hall_class);
		camera_ois_hall_class = NULL;
	}
}

void msm_ois_create_sysfs(void){
	if(!ois_aat_result_class){
	  CAM_ERR(CAM_OIS, "create ois_aat_result_class!!");
      ois_aat_result_class = class_create(THIS_MODULE, "ois");
      ois_aat_result1 = device_create(ois_aat_result_class, NULL,0, NULL, "ois_aat_selftest_result");
      device_create_file(ois_aat_result1, &dev_attr_ois_aat_selftest_result);
      ois_aat_result2 = device_create(ois_aat_result_class, NULL,0, NULL, "ois_aat_vendor_name");
      device_create_file(ois_aat_result2, &dev_attr_ois_aat_vendor_name);
	}
}

void msm_ois_destroy_sysfs(void){
	if(ois_aat_result_class){
        device_remove_file(ois_aat_result1, &dev_attr_ois_aat_selftest_result);
        device_remove_file(ois_aat_result2, &dev_attr_ois_aat_vendor_name);
		device_destroy(ois_aat_result_class, 0);
		device_destroy(ois_aat_result_class, 0);
		class_destroy(ois_aat_result_class);
	    ois_aat_result_class = NULL;
		CAM_ERR(CAM_OIS, "del ois_aat_result_class!!");
	}
}

int32_t lgit_imx351_rohm_ois_stat(sensor_ois_stat_t *data)
{
	sensor_ois_stat_t ois_stat;
	int rc = 0;

	int16_t val_hall_x;
	int16_t val_hall_y;

	/* float gyro_scale_factor_idg2020 = (1.0)/(262.0); */
	int16_t val_gyro_x;
	int16_t val_gyro_y;
	/* Hall Fail Spec. */
	int16_t spec_hall_x_lower = 1467;/* for get +-0.65deg, need 45.0um. */
	int16_t spec_hall_x_upper = 2629;
	int16_t spec_hall_y_lower = 1467;
	int16_t spec_hall_y_upper = 2629;

	memset(&ois_stat, 0, sizeof(ois_stat));
	snprintf(ois_stat.ois_provider, ARRAY_SIZE(ois_stat.ois_provider), "LGIT_ROHM");

	/* Gyro Read by reg */
	RegWriteA(0x609C, 0x02); //Changed rohm 0926 LeeDJ
	RamReadA(0x609D, &val_gyro_x);
	RegWriteA(0x609C, 0x03); //Changed rohm 0926 LeeDJ
	RamReadA(0x609D, &val_gyro_y);

	/* Hall Fail */
	/* Read Hall X */
	RegWriteA(0x6060, 0x00);
	rc = lgit_imx351_rohm_ois_poll_ready(LIMIT_STATUS_POLLING);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS_POLLING_ERROR");
		return rc;
	}

	RegReadB(0x6062, &val_hall_x);

	/* Read Hall Y */
	RegWriteA(0x6060, 0x01);
	rc = lgit_imx351_rohm_ois_poll_ready(LIMIT_STATUS_POLLING);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS_POLLING_ERROR");
		return rc;
	}

	RegReadB(0x6062, &val_hall_y);

	ois_stat.gyro[0] = (int16_t)val_gyro_x;
	ois_stat.gyro[1] = (int16_t)val_gyro_y;
	ois_stat.hall[0] = (int16_t)val_hall_x;
	ois_stat.hall[1] = (int16_t)val_hall_y;
	ois_stat.is_stable = 1;

	CAM_ERR(CAM_OIS, "val_hall_x : (%d), val_gyro_x : (0x%x), g_gyro_offset_value_x (%d)",
		val_hall_x, val_gyro_x, g_gyro_offset_value_x);
	CAM_ERR(CAM_OIS, "val_hall_y : (%d), val_gyro_y : (0x%x), g_gyro_offset_value_y (%d)",
		val_hall_y, val_gyro_y, g_gyro_offset_value_y);

	if (abs(val_gyro_x) > (5 * GYRO_SCALE_FACTOR) ||
		abs(val_gyro_y) > (5 * GYRO_SCALE_FACTOR)) {
		CAM_ERR(CAM_OIS, "Gyro Offset X is FAIL!!! (%d)", val_gyro_x);
		CAM_ERR(CAM_OIS, "Gyro Offset Y is FAIL!!! (%d)", val_gyro_y);
		ois_stat.is_stable = 0;
	}

	/* Hall Spec. Out? */
	if (val_hall_x > spec_hall_x_upper || val_hall_x < spec_hall_x_lower) {
		CAM_ERR(CAM_OIS, "val_hall_x is FAIL!!! (%d) 0x%x", val_hall_x,
			 val_hall_x);
		ois_stat.is_stable = 0;
	}

	if (val_hall_y > spec_hall_y_upper || val_hall_y < spec_hall_y_lower) {
		CAM_ERR(CAM_OIS, "val_hall_y is FAIL!!! (%d) 0x%x", val_hall_y,
			 val_hall_y);
		ois_stat.is_stable = 0;
	}
	*data = ois_stat;

	return 0;
}

int32_t lgit_imx351_rohm_ois_move_lens(void *data)
{
	int8_t hallx = 0;/* target_x / HALL_SCALE_FACTOR; */
	int8_t hally = 0;/* target_y / HALL_SCALE_FACTOR; */
	uint8_t result = 0;
	int32_t rc = OIS_SUCCESS;
	int16_t offset[2];

	memcpy(offset, data, sizeof(offset));

	switch (cal_ver) {
		/* LGIT Actuator */
		case 0x011:
		case 0x111:
		case 0x0301:
			hallx =  offset[0] * GYRO_GAIN_LGIT / HALL_SCALE_FACTOR;
			hally =  offset[1] * GYRO_GAIN_LGIT / HALL_SCALE_FACTOR;
			break;
		default:
			CAM_ERR(CAM_OIS, "[CHECK] OIS NOT SUPPORTED");
			rc = OIS_FAIL;
			break;
	}

	/* check ois mode & change to suitable mode */
	RegReadA(0x6020, &result);
	if (result != 0x01) {
		RegWriteA(0x6020, 0x01);
//		RegWriteA(0x614E, GYRO_GAIN_X); //for new FW
//		RegWriteA(0x614F, GYRO_GAIN_Y); //for new FW

		rc = lgit_imx351_rohm_ois_poll_ready(LIMIT_STATUS_POLLING);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS_POLLING_ERROR");
			return rc;
		}
	}

	CAM_ERR(CAM_OIS, "target : %d(0x%x), %d(0x%x)", hallx, hallx, hally, hally);

	RegWriteA(0x6026, 0xFF & hallx); /* target x position input */ //Changed rohm 0926 LeeDJ
	RegWriteA(0x6027, 0xFF & hally); /* target y position input */ //Changed rohm 0926 LeeDJ
	RegWriteA(0x6020, 0x03);/* order to move. */ //Changed rohm 0926 LeeDJ

	/* wait 100ms */
	usleep_range(100000, 100010); //added rohm 0926 LeeDJ
	rc = lgit_imx351_rohm_ois_poll_ready(LIMIT_STATUS_POLLING);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS_POLLING_ERROR");
		return rc;
	}

	RegReadA(0x609B, &result);

	RegWriteA(0x6020, 0x01); //added rohm 0926 LeeDJ
//	RegWriteA(0x614E, GYRO_GAIN_X); //for new FW
//	RegWriteA(0x614F, GYRO_GAIN_Y); //for new FW

	if (result == 0x03)
		return  OIS_SUCCESS;

	CAM_ERR(CAM_OIS, "move fail : 0x%x", result);
	return OIS_FAIL;
}

/*===========================================================================
 * FUNCTION    - ois_selftest_get -
 *
 * DESCRIPTION: ois self-test routine for all-auto-test
 *==========================================================================*/
static uint32_t ois_selftest_get(void)
{
  uint32_t out = 0;
  int i;

  for (i = 0; i < 12; i++)
	if (aat_selftest_result[11 - i] == '1')
		out |= (1 << i);

  CAM_ERR(CAM_OIS," result = %x %s", out, aat_selftest_result);
  return out;
}

/*===========================================================================
 * FUNCTION    - ois_selftest_set -
 *
 * DESCRIPTION: ois self-test routine for all-auto-test
 *==========================================================================*/
static void ois_selftest_set(uint32_t result)
{
  uint8_t rc[12];
  int i;

  for (i=0; i < 12; i++)
    rc[i] = (result & (1 << i)) >> i;

  sprintf(aat_selftest_result, "%d%d%d%d%d%d%d%d%d%d%d%d", rc[11], rc[10], rc[9], rc[8], rc[7],
    rc[6], rc[5], rc[4], rc[3], rc[2], rc[1], rc[0]);

  CAM_ERR(CAM_OIS," result = %x %s",result, aat_selftest_result);
}

/*===========================================================================
 * FUNCTION    - ois_selftest -
 *
 * DESCRIPTION: ois self-test routine for all-auto-test
 *==========================================================================*/
#define  HALL_LIMIT        65
#define  GYRO_LIMIT        10   // 10[dps]*175
#define  GYRO_OFFSET_LIMIT 30   // 30[dps]*175
#define  GYRO_SCALE_FACTOR 175

#define GYRO_X_VALUE ois_stat.gyro[0] / GYRO_SCALE_FACTOR
#define GYRO_Y_VALUE ois_stat.gyro[1] / GYRO_SCALE_FACTOR

#define  UNSTABLE_RATIO 40 //0.4 * 100

static int8_t ois_selftest(void)
{
  sensor_ois_stat_t ois_stat;
  uint32_t result = 0;
  int i = 0;

  int16_t hall_target[][2] = {
    {HALL_LIMIT, 0},
    {HALL_LIMIT, HALL_LIMIT}, {0, HALL_LIMIT}, {-HALL_LIMIT, HALL_LIMIT},
    {-HALL_LIMIT, 0}, {-HALL_LIMIT, -HALL_LIMIT}, {0, -HALL_LIMIT},
    {HALL_LIMIT, -HALL_LIMIT}, {HALL_LIMIT, 0}
   };

  CAM_ERR(CAM_OIS, " enter");
  //0. is ois init success ?
  result |= (1 << 11); // <- init success.

  //1. get first stat value.
  lgit_imx351_rohm_ois_stat(&ois_stat);

  //2. check ois module is alive.
  if (ois_stat.gyro[0] == 0 && ois_stat.gyro[1] == 0 &&
    ois_stat.hall[0] == 0 && ois_stat.hall[1] == 0) goto END;

  //3. check gyro initial dps.
  if (abs(GYRO_X_VALUE) <= GYRO_OFFSET_LIMIT)
	{
    result |= (1 << 10);
	CAM_ERR(CAM_OIS, "(GYRO_X_VALUE) <= GYRO_OFFSET_LIMIT)");
	}
  if (abs(GYRO_Y_VALUE) <= GYRO_OFFSET_LIMIT)
	{
	result |= (1 << 9);
	CAM_ERR(CAM_OIS, "(GYRO_Y_VALUE) <= GYRO_OFFSET_LIMIT)");
	}
  //4. check lens movement range
  for (i = 0; i <= 8; i++) {
	  if (lgit_imx351_rohm_ois_move_lens(&hall_target[i])>= 0)
      result |= (1 << (8 - i));
  }

  //5. reset lens position before ois turn-on.
  {
    uint16_t offset[2] = {0, 0};

	lgit_imx351_rohm_ois_move_lens(&offset);
  }

  END:
  strcpy(aat_vendor_name, ois_stat.ois_provider);
  ois_selftest_set(result);
  CAM_ERR(CAM_OIS," exit");
  return OIS_SUCCESS;
}

/*===========================================================================
 * FUNCTION    - ois_selftest2 -
 *
 * DESCRIPTION: ois self-test routine #2 for all-auto-test
 *==========================================================================*/
static int8_t ois_selftest2(void)
{
  sensor_ois_stat_t ois_stat;
  int i;
  int unstable = 0;
  uint32_t result = ois_selftest_get();
  int32_t rc = 0;

  CAM_ERR(CAM_OIS," enter");
  memset(&ois_stat, 0, sizeof(ois_stat));

  // if ois init was failed, skip test
  if (result == 0) {
    CAM_ERR(CAM_OIS,"ois init was failed, skip test\n");
    goto END;
  }

  //wait 750ms
  usleep_range(750000, 750010);

  //check gyro validity, servo stability at least 100 times (~ about 250ms)
  for (i = 0; i < 100; i++) {
    rc = lgit_imx351_rohm_ois_stat(&ois_stat);
	if (rc < 0 ) {
		CAM_ERR(CAM_OIS,"i:%d ois_get_stat error\n", i);
		ois_stat.is_stable = 0;
	}
    if (abs(GYRO_X_VALUE) > GYRO_LIMIT) {
      CAM_ERR(CAM_OIS," %d spec over GYRO_X_VALUE %d\n", i, GYRO_X_VALUE);
      result &= ~(1 << 10);
      ois_stat.is_stable = 0;
    }
    if (abs(GYRO_Y_VALUE) > GYRO_LIMIT) {
			CAM_ERR(CAM_OIS," %d spec over GYRO_Y_VALUE %d\n", i, GYRO_Y_VALUE);
      result &= ~(1 << 9);
			ois_stat.is_stable = 0;
    }
    if (!ois_stat.is_stable)
      unstable++;
  }

  if (unstable > UNSTABLE_RATIO)
    result &= ~(1 << 11);

  CAM_ERR(CAM_OIS," unstable -> %d \n", unstable);

  CAM_ERR(CAM_OIS," lens position -> hall_x:%d, hall_y:%d\n", ois_stat.hall[0], ois_stat.hall[1]);
  END:
  ois_selftest_set(result);
  CAM_ERR(CAM_OIS," exit");
  return OIS_SUCCESS;
}

int32_t ois_i2c_read_seq(uint32_t addr, uint8_t *data, uint16_t num_byte)
{
	int32_t ret = 0;

	ret = camera_io_dev_read_seq(
		&(local_cam_ois_t->io_master_info),
		addr,
		data,
		CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		num_byte);

	return ret;
}
int32_t RamWriteA(uint32_t RamAddr, uint16_t RamData)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	int32_t ret = 0;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * 1, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL)
	{
		CAM_ERR(CAM_OIS,"kzalloc failed");
		ret = OIS_FAIL;
		return ret;
	}
	i2c_reg_setting.reg_setting->reg_addr = RamAddr;
	i2c_reg_setting.reg_setting->reg_data = RamData;
	i2c_reg_setting.reg_setting->delay = 0;
	i2c_reg_setting.reg_setting->data_mask = 0;

	ret = camera_io_dev_write(
		&(local_cam_ois_t->io_master_info),
		&i2c_reg_setting);
	kfree(i2c_reg_setting.reg_setting);
	return ret;
}

int32_t RamReadA(uint32_t RamAddr, uint16_t *ReadData)
{
	int32_t ret = 0;
	uint32_t data = 0;
	ret = camera_io_dev_read(
		&(local_cam_ois_t->io_master_info),
		RamAddr,
		&data,
		CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_WORD);
	*ReadData = (uint16_t)data;

	return ret;
}

int32_t RamRead32A(uint32_t RamAddr, uint32_t *ReadData)
{
	int32_t ret = 0;
	uint8_t buf[4];

	ret = camera_io_dev_read_seq(
		&(local_cam_ois_t->io_master_info),
		RamAddr,
		buf,
		CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_WORD,
		4);
	*ReadData = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];

	return ret;
}

int32_t RegWriteA(uint32_t RegAddr, uint8_t RegData)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	uint32_t data = (uint32_t)RegData;
	int32_t ret = 0;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * 1, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL)
	{
		CAM_ERR(CAM_OIS,"kzalloc failed");
		ret = OIS_FAIL;
		return ret;
	}
	i2c_reg_setting.reg_setting->reg_addr = RegAddr;
	i2c_reg_setting.reg_setting->reg_data = data;
	i2c_reg_setting.reg_setting->delay = 0;
	i2c_reg_setting.reg_setting->data_mask = 0;
	ret = camera_io_dev_write(
		&(local_cam_ois_t->io_master_info),
		&i2c_reg_setting);

	kfree(i2c_reg_setting.reg_setting);
	return ret;
}

int32_t RegWriteB(uint32_t RegAddr, uint16_t RegData)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	int32_t ret = 0;
	uint32_t data = (uint32_t)RegData;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * 1, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL)
	{
		CAM_ERR(CAM_OIS,"kzalloc failed");
		ret = OIS_FAIL;
		return ret;
	}
	i2c_reg_setting.reg_setting->reg_addr = RegAddr;
	i2c_reg_setting.reg_setting->reg_data = data;
	i2c_reg_setting.reg_setting->delay = 0;
	i2c_reg_setting.reg_setting->data_mask = 0;

	ret = camera_io_dev_write(
		&(local_cam_ois_t->io_master_info),
		&i2c_reg_setting);

	kfree(i2c_reg_setting.reg_setting);
	return ret;
}

int32_t RegReadA(uint32_t RegAddr, uint8_t *RegData)
{
	int32_t ret = 0;
	uint32_t data = 0;

	ret = camera_io_dev_read(
		&(local_cam_ois_t->io_master_info),
		RegAddr,
		&data,
		CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_BYTE);

	*RegData = (uint8_t)data;
	return ret;
}

int32_t RegReadB(uint32_t RegAddr, uint16_t *RegData)
{
	int32_t ret = 0;
	uint32_t data = 0;

	ret = camera_io_dev_read(
		&(local_cam_ois_t->io_master_info),
		RegAddr,
		&data,
		CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_WORD);

	*RegData = (uint16_t)data;
	return ret;
}
int32_t ois_i2c_e2p_write(uint32_t e2p_addr, uint16_t e2p_data, enum camera_sensor_i2c_type data_type)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	int32_t ret = 0;
	uint32_t data = (uint32_t)e2p_data;
	uint16_t temp_sid = 0;

	temp_sid = local_cam_ois_t->io_master_info.cci_client->sid;
	local_cam_ois_t->io_master_info.cci_client->sid = E2P_SID >> 1;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = data_type;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * 1, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL)
	{
		CAM_ERR(CAM_OIS,"kzalloc failed");
		ret = OIS_FAIL;
		local_cam_ois_t->io_master_info.cci_client->sid = temp_sid;
		return ret;
	}
	i2c_reg_setting.reg_setting->reg_addr = e2p_addr;
	i2c_reg_setting.reg_setting->reg_data = data;
	i2c_reg_setting.reg_setting->delay = 0;
	i2c_reg_setting.reg_setting->data_mask = 0;

	ret = camera_io_dev_write(
		&(local_cam_ois_t->io_master_info),
		&i2c_reg_setting);

	local_cam_ois_t->io_master_info.cci_client->sid = temp_sid;

	kfree(i2c_reg_setting.reg_setting);
	return ret;
}
int32_t ois_i2c_e2p_read(uint32_t e2p_addr, uint16_t *e2p_data, enum camera_sensor_i2c_type data_type)
{
	int32_t ret = 0;
	uint32_t data = 0;
	uint16_t temp_sid = 0;

	temp_sid = local_cam_ois_t->io_master_info.cci_client->sid;
	local_cam_ois_t->io_master_info.cci_client->sid = E2P_SID >> 1;

	ret = camera_io_dev_read(
		&(local_cam_ois_t->io_master_info),
		e2p_addr,
		&data,
		CAMERA_SENSOR_I2C_TYPE_WORD,
		data_type);

	local_cam_ois_t->io_master_info.cci_client->sid = temp_sid;
	*e2p_data = (uint16_t)data;

	return ret;
}

int32_t load_bin(uint8_t *ois_bin, uint32_t filesize, char *filename)
{
	int fd1;
	uint8_t fs_name_buf1[256];
	uint32_t cur_fd_pos;
	int32_t rc = OIS_SUCCESS;

	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	sprintf(fs_name_buf1, "/vendor/lib/camera/fw/%s", filename);

	fd1 = sys_open(fs_name_buf1, O_RDONLY, 0);
	if (fd1 < 0) {
		CAM_ERR(CAM_OIS,"File not exist (filename: %s)", fs_name_buf1);
		rc = OIS_INIT_LOAD_BIN_ERROR;
		goto END;
	}

	if ((unsigned)sys_lseek(fd1, (off_t)0, 2) != filesize) {
		CAM_ERR(CAM_OIS,"File size error");
		rc = OIS_INIT_LOAD_BIN_ERROR;
		goto END;
	}

	cur_fd_pos = (unsigned)sys_lseek(fd1, (off_t)0, 0);

	memset(ois_bin, 0x00, filesize);

	if ((unsigned)sys_read(fd1, (char __user *)ois_bin, filesize) != filesize) {
		CAM_ERR(CAM_OIS,"File read error");
		rc = OIS_INIT_LOAD_BIN_ERROR;
	}

END:
	sys_close(fd1);
	set_fs(old_fs);

	return rc;
}

#define I2C_WRITE_MAX_BYTE 6
int32_t ois_i2c_bin_seq_write(uint32_t src_saddr, uint32_t src_eaddr, uint32_t dst_addr, uint8_t *ois_bin)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	uint16_t cnt;
	int16_t total_byte;
	uint8_t	*ptr = NULL;
	int32_t rc = 0;
	uint16_t count = 0;

	total_byte = src_eaddr - src_saddr + 1;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = 4/*total_byte*/;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * 4/*total_byte*/, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL)
	{
		CAM_ERR(CAM_OIS,"kzalloc failed");
		rc = OIS_FAIL;
		return rc;
	}

	for (cnt = 0, ptr = (uint8_t *)ois_bin; cnt < total_byte; cnt++, ptr++, dst_addr++) {
		i2c_reg_setting.reg_setting[count].reg_addr = dst_addr;
		i2c_reg_setting.reg_setting[count].reg_data = *ptr;
		i2c_reg_setting.reg_setting[count].delay = 0;
		i2c_reg_setting.reg_setting[count].data_mask = 0;
		if(count == 3) {
			rc = camera_io_dev_write_continuous(&(local_cam_ois_t->io_master_info),
			&i2c_reg_setting, 0);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
					return rc;
			}
			count = 0;
		} else {
			count++;
		}
	}
#if 0
	rc = camera_io_dev_write_continuous(&(local_cam_ois_t->io_master_info),
	&i2c_reg_setting, 0);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
			return rc;
	}
#endif
	kfree(i2c_reg_setting.reg_setting);

	return rc;

}

int32_t ois_i2c_load_and_write_bin(struct ois_i2c_bin_entry bin_entry)
{
	int32_t rc = OIS_SUCCESS;
	uint8_t *ois_bin_data = NULL;
	int i;
	struct ois_i2c_bin_addr addr;

	ois_bin_data = kmalloc(bin_entry.filesize, GFP_KERNEL);

	if (!ois_bin_data) {
		CAM_ERR(CAM_OIS,"Can not alloc bin data");
		rc = OIS_INIT_NOMEM;
		goto END2;
	}

	rc = load_bin(ois_bin_data, bin_entry.filesize, bin_entry.filename);
	if (rc < 0) {
		CAM_ERR(CAM_OIS,"load bin error, rc: %d", rc);
		goto END1;
	}

	for (i = 0; i < bin_entry.blocks; i++) {
		addr = bin_entry.addrs[i];
		rc = ois_i2c_bin_seq_write(addr.bin_str_addr, addr.bin_end_addr, addr.reg_str_addr, ois_bin_data);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,"program %d down error", i);
			rc = OIS_INIT_I2C_ERROR;
			goto END1;
		}
	}

END1:
	kfree(ois_bin_data);
END2:
	return rc;

}

int32_t ois_i2c_load_and_write_bin_list(struct ois_i2c_bin_list bin_list)
{
	int i;
	int length = bin_list.files;
	int32_t rc = OIS_SUCCESS;
	for (i = 0; i < length; i++) {
		rc = ois_i2c_load_and_write_bin(bin_list.entries[i]);
		if (rc <0) {
			goto END;
		}
	}

END:
	return rc;
}
#if 1
int32_t ois_i2c_load_and_write_e2prom_data(uint32_t e2p_str_addr,
		uint16_t e2p_data_length, uint16_t reg_str_addr)
{
	int32_t rc = OIS_SUCCESS;
	uint8_t *e2p_cal_data = NULL;
	uint16_t temp_sid = 0;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;

	temp_sid = local_cam_ois_t->io_master_info.cci_client->sid;
	local_cam_ois_t->io_master_info.cci_client->sid = E2P_SID >> 1;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = e2p_data_length;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * e2p_data_length, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL)
	{
		CAM_ERR(CAM_OIS,"kzalloc failed");
		rc = OIS_FAIL;
		local_cam_ois_t->io_master_info.cci_client->sid = temp_sid;
		return rc;
	}
	i2c_reg_setting.reg_setting->delay = 0;
	i2c_reg_setting.reg_setting->data_mask = 0;


	/* Read E2P data!*/
	e2p_cal_data = kmalloc(e2p_data_length + 2, GFP_KERNEL);

	if (!e2p_cal_data) {
		CAM_ERR(CAM_OIS,"Can not alloc e2p data");
		rc = OIS_INIT_NOMEM;
		local_cam_ois_t->io_master_info.cci_client->sid = temp_sid;
		goto END2;
	}

	memset(e2p_cal_data, 0x00, e2p_data_length);

	camera_io_dev_read_seq(&(local_cam_ois_t->io_master_info), e2p_str_addr,
		e2p_cal_data, CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD, END_FIRST_BLOCK_CAL_DATA);
	camera_io_dev_read_seq(&(local_cam_ois_t->io_master_info), e2p_str_addr + STR_SECOND_BLOCK_CAL_DATA,
		e2p_cal_data + END_FIRST_BLOCK_CAL_DATA, CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD,(e2p_data_length - END_FIRST_BLOCK_CAL_DATA));

	local_cam_ois_t->io_master_info.cci_client->sid = temp_sid;

	rc = ois_i2c_bin_seq_write(0, e2p_data_length - 1, reg_str_addr, e2p_cal_data);

	if (rc < 0) {
		CAM_ERR(CAM_OIS, "e2p down error");
		goto END1;
	}

END1:
	kfree(e2p_cal_data);
END2:
	kfree(i2c_reg_setting.reg_setting);
	return rc;

}
#endif

/*
** =============================================================================
**
** File: ImmVibeSPI.c
**
** Description:
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
**
** Copyright (c) 2012-2017 Immersion Corporation. All Rights Reserved.
**
** This file contains Original Code and/or Modifications of Original Code
** as defined in and that are subject to the GNU Public License v2 -
** (the 'License'). You may not use this file except in compliance with the
** License. You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES,
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see
** the License for the specific language governing rights and limitations
** under the License.
**
** =============================================================================
*/

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>


#include <linux/init.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/ioctl.h>

#include <asm/uaccess.h>
#include <linux/pm_wakeup.h>
#include <linux/of.h>

#define DW7912_MAX_PATTERN_DATA (80) /* max 16kbytes data */

#if 1
	#define gprintk(fmt, x... ) printk( "%s: " fmt, __FUNCTION__ , ## x)
#else
	#define gprintk(x...) do { } while (0)
#endif

/*
** This SPI supports only one actuator.
*/
#define DEVICE_BUS  6
#define DEVICE_ADDR 0x59
#define NUM_ACTUATORS       1

/*
** Called to disable amp (disable output force)
*/
#define DEVICE_NAME "LG JUDY"
/*
** Manage amplifier buffer using ImmVibeSPI_ForceOut_BufferFull
*/
//#define IMMVIBESPI_USE_BUFFERFULL

struct dw7912_priv {
	int use_en_pin; /* en gpio enable/disable */
	int enable_pin; /* if use_en_pin is 1, this pin used */
	struct i2c_client *dwclient;
	struct hrtimer timer;
//	struct timed_output_dev dev;
	struct work_struct work;
	struct mutex lock;
	struct wakeup_source wklock;
	int plen; /* just pattern data size, not included register address size(1) */
	u8  pdat[DW7912_MAX_PATTERN_DATA+sizeof(int)]; /* with register address */
	bool running; 
	int timeout;  /* timeout time in ms */
};

static struct dw7912_priv *dw7912p;


#define DW7912_SYSFS // use this feature only for user debug, not release
#define DW7912_REG_00 0
#define DW7912_REG_01 1
#define DW7912_REG_02 2
#define DW7912_REG_03 3
#define DW7912_REG_04 4
#define DW7912_REG_05 5
#define DW7912_REG_06 6
#define DW7912_REG_07 7
#define DW7912_REG_08 8
#define DW7912_REG_09 9
#define DW7912_REG_0A 10
#define DW7912_REG_0B 11
#define DW7912_REG_0C 12
#define DW7912_REG_0D 13
#define DW7912_REG_0E 14
#define DW7912_REG_0F 15
#define DW7912_REG_10 16
#define DW7912_REG_11 17
#define DW7912_REG_12 18
#define DW7912_REG_2F 47

#define DW7912_INFO             0x00
#define DW7912_VERSION          0x00
#define DW7912_STATUS           0x01
#define DW7912_FIFOFULL         0x01
#define DW7912_MODE				0x03
#define DW7912_DATA             0x0A
#define DW7912_SWRESET          0x2F
#define DW7912_LDO              0x08
#define SW_RESET_COMMAND        0x01 /* Reset */
#define HW_RESET_ENABLE_COMMAND 0x00 /* Enable hardware */
#define RTP_MODE				0x00
#define MEM_MODE				0x01

/*
** Value between 1 and 86. Higher values buffer more future samples.
*/
#define dw7912_FIFOFULL_TARGET 1024

/*
** Number of 5ms blocks of data to preload at effect start.
** Use a value of '0' when using IMMVIBESPI_USE_BUFFERFULL
** since a different mechanism manages the buffer.
*/
#define NUM_EXTRA_BUFFERS  0

VibeStatus Dw7912I2CWrite(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer);
VibeStatus Dw7912I2CWriteWithResendOnError(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer);
VibeStatus Dw7912I2CWriteWithResendOnError_DW7912(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer);

/*
** I2C Driver
*/
static int dw7912_i2c_probe(struct i2c_client* client, const struct i2c_device_id* id);
static int dw7912_i2c_remove(struct i2c_client* client);

#ifdef CONFIG_PM
static int dw7912_suspend(struct device *dev)
{
	return 0;
}

static int dw7912_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(dw7912_pm_ops,
			 dw7912_suspend, dw7912_resume);

#define DW7912_VIBRATOR_PM_OPS (&dw7912_pm_ops)
#else
#define DW7912_VIBRATOR_PM_OPS NULL
#endif

#ifdef CONFIG_OF
static struct of_device_id dw7912_i2c_dt_ids[] = {
	{ .compatible = "dwanatech,dw7912"},
	{ }
};
#endif

static const struct i2c_device_id dw7912_i2c_id[] = {
	{"dw7912", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, dw7912_i2c_id);

static struct i2c_driver dw7912_i2c_driver = {
	.probe = dw7912_i2c_probe,
	.remove = dw7912_i2c_remove,
	.id_table = dw7912_i2c_id,
	.driver = {
		.name = "dw7912",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(dw7912_i2c_dt_ids),
#endif
#ifdef CONFIG_PM
		.pm	= DW7912_VIBRATOR_PM_OPS,
#endif
		},
};

#ifdef DW7912_SYSFS
struct dw7912_regmap {
    const char *name;
    uint8_t reg;
    int writeable;
} dw7912_regs[] = {
    { "00_IC_INFO",          DW7912_REG_00, 0 },
    { "01_IC_VERSION",       DW7912_REG_01, 0 },
    { "02_STATUS",           DW7912_REG_02, 0 },
    { "03_FIFO_FULLNESS",    DW7912_REG_03, 0 },
    { "04_HAPTIC_DATA",      DW7912_REG_04, 1 },
    { "05_SOFTWARE_RESET",   DW7912_REG_05, 1 },
    { "06_TIMING_SET",       DW7912_REG_06, 1 },
    { "07_LDO_LEVEL",        DW7912_REG_07, 1 },
    { "08_HW_RESET_DISABLE", DW7912_REG_08, 1 },
    { "09_PACKET_SIZE",      DW7912_REG_09, 0 },
};

unsigned char dw7912_moisture_data_buf[166] = {
    0x04,
    0x00, 0x0E, 0x1D, 0x2B, 0x39, 0x46, 0x52, 0x5D, 0x66, 0x6E, 0x75, 0x7A, 0x7D, 0x7E, 0x7E, 0x7C, 0x78, 0x73, 0x6C, 0x63,
    0x59, 0x4E, 0x42, 0x35, 0x27, 0x18, 0x09, 0xFC, 0xED, 0xDE, 0xD0,

    0x04,
    0xC2, 0xB6, 0xAA, 0xA0, 0x97, 0x8F, 0x89, 0x85, 0x82, 0x81, 0x82, 0x85, 0x89, 0x8F, 0x97, 0xA0, 0xAA, 0xB6, 0xC2, 0xD0,
    0xDE, 0xED, 0xFC, 0x09, 0x18, 0x27, 0x35, 0x42, 0x4E, 0x59, 0x63,

    0x04,
    0x6C, 0x73, 0x78, 0x7C, 0x7E, 0x7E, 0x7D, 0x7A, 0x75, 0x6E, 0x66, 0x5D, 0x52, 0x46, 0x39, 0x2B, 0x1D, 0x0E, 0x00, 0xF2,
    0xE3, 0xD5, 0xC7, 0xBA, 0xAE, 0xA3, 0x9A, 0x92, 0x8B, 0x86, 0x83,

    0x04,
    0x82, 0x82, 0x84, 0x88, 0x8D, 0x94, 0x9D, 0xA7, 0xB2, 0xBE, 0xCB, 0xD9, 0xE8, 0xF7, 0x04, 0x13, 0x22, 0x30, 0x3E, 0x4A,
    0x56, 0x60, 0x69, 0x71, 0x77, 0x7B, 0x7E, 0x7F, 0x7E, 0x7B, 0x77,

    0x04,
    0x71, 0x69, 0x60, 0x56, 0x4A, 0x3E, 0x30, 0x22, 0x13, 0x04, 0xF7, 0xE8, 0xD9, 0xCB, 0xBE, 0xB2, 0xA7, 0x9D, 0x94, 0x8D,
    0x88, 0x84, 0x82, 0x82, 0x83, 0x86, 0x8B, 0x92, 0x9A, 0xA3, 0xAE,

    0x04,
    0xBA, 0xC7, 0xD5, 0xE3, 0xF2
};

static ssize_t set_moisture_vib(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
    int i = 0;
    int j = 0;
    int res = 0;
    unsigned char reg = 0x3;
    unsigned char fifo = 0;
    struct i2c_msg send_msg;
    struct i2c_msg recv_msg[2];

    send_msg.addr = dw7912p->dwclient->addr;
    send_msg.flags = 0;
    send_msg.len = 32;

    recv_msg[0].addr = dw7912p->dwclient->addr;
    recv_msg[0].flags = 0;
    recv_msg[0].len = 1;
    recv_msg[0].buf = &reg;

    recv_msg[1].addr = dw7912p->dwclient->addr;
    recv_msg[1].flags = I2C_M_RD;
    recv_msg[1].len = 1;
    recv_msg[1].buf = &fifo;

    for(j = 0; j < 5; j++) {
        for(i = 0; i < 6; i++) {
            send_msg.buf = dw7912_moisture_data_buf + (32*i);
            res = i2c_transfer(dw7912p->dwclient->adapter, &send_msg, 1);

            while(1) {
                res = i2c_transfer(dw7912p->dwclient->adapter, recv_msg, 2);
                if(fifo < 80)
                    break;

                usleep_range(1000, 1100);
            }
        }
    }

    return res;
}

static DEVICE_ATTR(moisture_vib, S_IWUSR | S_IRUGO, NULL, set_moisture_vib);

static struct attribute *dw7912_attrs[] = {
    &dev_attr_moisture_vib.attr,
    NULL
};

static const struct attribute_group dw7912_attr_group = {
    .attrs = dw7912_attrs,
};

#endif // End of #ifdef DW7912_SYSFS
#if 0
static int i2c_recv_buf(struct i2c_client *i2c, unsigned char reg, unsigned char *buf, int count)
{
    struct i2c_msg msg[2];
    int res = 0;

    msg[0].addr = i2c->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &reg;

    msg[1].addr = i2c->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = count;
    msg[1].buf = buf;

    res = i2c_transfer(i2c->adapter, msg, 2);
    //DbgOut((DBL_INFO, "dw7912: i2c recv res=%d\n", res));

    return res;
}
#endif

static bool i2c_send_buf(struct i2c_client *i2c, unsigned char *buf, int count)
{
    struct i2c_msg msg;
    int res = 0;

    msg.addr = i2c->addr;
    msg.flags = 0;
    msg.len = count;
    msg.buf = buf;

    res = i2c_transfer(i2c->adapter, &msg, 1);
    //DbgOut((DBL_INFO, "dw7912: i2c send res=%d\n", res));

    return res;
}

/*
** TouchSense SPI Functions
*/

#define NAK_RESEND_ATTEMPT 3
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
    //DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_AmpDisable.\n"));

    /* Nothing to do. dw7912 enters standby when FIFO is empty. */

    return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
    //DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_AmpEnable.\n"));

    /* Set duty cycle to 50% */
    /* To be implemented with appropriate hardware access macros */

    /* Enable amp */
    /* To be implemented with appropriate hardware access macros */

    return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
    int retVal = 0;

//hoseong.kang   retVal = i2c_add_driver(&dw7912_driver);
    if (retVal) {
		        //DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_Initialize: Cannot add driver.\n"));
		        return VIBE_E_FAIL;
    }

    return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
    /* Remove driver */

    /* Set PWM frequency */
    /* To be implemented with appropriate hardware access macros */

    /* Set duty cycle to 50% */
    /* To be implemented with appropriate hardware access macros */

    //DbgOut((DBL_INFO, "ImmVibeSPI_ForceOut_Terminate.\n"));
    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set force output, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
//	unsigned char data;
    //DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_SetSamples enter\n"));

#ifdef IMMVIBESPI_USE_BUFFERFULL
    /* write zeros to keep hardware buffer full */
    if (nBufferSizeInBytes == 0) {
        static VibeInt8 buffer[40] = {0,};
        pForceOutputBuffer = buffer;
        nBufferSizeInBytes = sizeof(buffer);
        nOutputSignalBitDepth = 8;
    }
#endif

/* hoseong.kang */

/* 	if (VIBE_S_SUCCESS != Dw7912I2CWrite(0x09, 1, &data))
	{
		//DbgOut((DBL_ERROR, "Dw7912I2CWrite failed to send GO\n"));
	}
    //DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_SetSamples enter 2\n")); 
*/

    if (VIBE_S_SUCCESS != Dw7912I2CWriteWithResendOnError_DW7912(DW7912_DATA, nBufferSizeInBytes, pForceOutputBuffer)) {
        //DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_SetSamples: i2c write failed\n"));
        return VIBE_E_FAIL;
    }

    return VIBE_S_SUCCESS;
}

VibeStatus Dw7912I2CWrite(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
#if 1
    //unsigned char buf[VIBE_OUTPUT_SAMPLE_SIZE+1];
    char buf[VIBE_OUTPUT_SAMPLE_SIZE+1];

    //DbgOut((DBL_ERROR, "Dw7912I2CWrite enter\n"));
    if (!dw7912p->dwclient) {
		//pr_err("%s, dwclient is null\n", __func__);

        return VIBE_E_FAIL;
	}
    if (nBufferSizeInBytes > VIBE_OUTPUT_SAMPLE_SIZE) {
		//pr_err("%s, buffer size error\n", __func__);

        return VIBE_E_FAIL;
	}

    buf[0] = address;
    memcpy(buf + 1, pForceOutputBuffer, nBufferSizeInBytes);
    if (1 != i2c_send_buf(dw7912p->dwclient, buf, nBufferSizeInBytes+1)) {
		//pr_err("%s, i2c_send_buf failed\n", __func__);
        return VIBE_E_FAIL;
    }
#endif
    return VIBE_S_SUCCESS;
}



VibeStatus Dw7912I2CWriteWithResendOnError_DW7912(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
	
//    int nResendAttempt = 1000;
    char pktsize = 0;
	//char status_msg, cmd;
	char cmd;
    char buf[VIBE_OUTPUT_SAMPLE_SIZE*3 + 1];	
	int cv, sv, buf_cnt, out_rate;
	int ret;
	
	buf_cnt = 0;
	out_rate = 3;

	//DbgOut((DBL_ERROR, "Dw7912I2CWriteWithResendOnError_DW7912 enter\n"));

    if (!dw7912p->dwclient) {
		//pr_err("%s : dw7912p->dwclient is null\n", __func__);
        return VIBE_E_FAIL;
	}

    if (nBufferSizeInBytes > VIBE_OUTPUT_SAMPLE_SIZE) {
		//pr_err("%s : output sample size error\n", __func__);
        return VIBE_E_FAIL;
	}


	if (!pktsize) {

		/* First byte is i2c destination register address */
		buf[0] = address;

		/* Conversion data 8khz to 48khz */
		for(cv=0; cv < nBufferSizeInBytes; cv++)
		{
			for(sv=0; sv < out_rate; sv++) 
			{
				buf[1 + buf_cnt++] = pForceOutputBuffer[cv];
				//pr_err("%s, hoseong.kang buf[%d] : %d, pForceOutputBuffer[%d] : %d\n",__func__, buf_cnt, buf[buf_cnt], cv, pForceOutputBuffer[cv]);
			}
		}
		/* Send remaining bytes */
		/* The return value of this function is unreliable so ignoring it */

		cmd = 0x01;
		Dw7912I2CWrite(0x09, 1, &cmd);
		ret = i2c_send_buf(dw7912p->dwclient, buf, nBufferSizeInBytes * out_rate + 1);
	}

	/* Check how many bytes actually received by dw7912 */
#if 0
	pktsize = (2 != i2c_recv_buf(dw7912p->dwclient, 0x01, &status_msg, 1));
	
	if (pktsize) {
		//DbgOut((DBL_WARNING, "dw7912_status register not be read\n"));
		udelay(250);
		//DbgOut((DBL_ERROR, "Dw7912I2CWriteWithResendOnError failed\n"));
		return VIBE_E_FAIL;
	}
#endif
	
	#if 0
	if(status_msg & 0x40) {	// fifo full
		while(nResendAttempt--) {
			udelay(1000);
			i2c_recv_buf(dw7912p->dwclient, 0x01, &status_msg, 1);
			if(status_msg & 0x10) break;
		}
	}
	#endif

		
	//pr_err("%s : success\n", __func__);
	return VIBE_S_SUCCESS;
}



VibeStatus Dw7912I2CWriteWithResendOnError(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{

//#if 1 //hoseong.kang
    int nDataIndex = 0;
    int waitDuration = 0;
    int nResendAttempt = NAK_RESEND_ATTEMPT;
    char datacnt = 0;
    char pktsize = 0;
    unsigned char buf[VIBE_OUTPUT_SAMPLE_SIZE+1];
	//int i;

	//pr_err("%s : enter\n", __func__);
	
	//DbgOut((DBL_ERROR, "Dw7912I2CWriteWithResendOnError enter\n"));

    if (!dw7912p->dwclient) {
		//pr_err("%s : dw7912p->dwclient is null\n", __func__);

        return VIBE_E_FAIL;
	}

    if (nBufferSizeInBytes > VIBE_OUTPUT_SAMPLE_SIZE) {
		//pr_err("%s : output sample size error\n", __func__);
        return VIBE_E_FAIL;
	}

    while (nResendAttempt--) {

        if (!pktsize) {

            /* First byte is i2c destination register address */
            buf[0] = address;

            /* Copy remaining data */
            memcpy(buf + 1, pForceOutputBuffer + nDataIndex, nBufferSizeInBytes - nDataIndex);

            /* Send remaining bytes */
            /* The return value of this function is unreliable so ignoring it */
			//for(i=1; i<VIBE_OUTPUT_SAMPLE_SIZE+1;i++)
				//pr_err("%s : buf[%d] : %d\n", __func__, i, buf[i]);

            i2c_send_buf(dw7912p->dwclient, buf, nBufferSizeInBytes - nDataIndex + 1);
        }

        /* Check how many bytes actually received by dw7912 */
#if 0
        datacnt = 0;
        pktsize = (2 != i2c_recv_buf(dw7912p->dwclient, DW7912_PKTSIZE, &datacnt, 1));
        if (pktsize) {
            //DbgOut((DBL_WARNING, "dw7912_pktsize could not be read\n"));
            udelay(250);
            continue;
        }
#endif
        /* Advance data pointer */
		datacnt = 40;
        nDataIndex += datacnt;
		//pr_err("%s : 1. nDataIndex : %d, nBufferSizeInBytes : %d\n", __func__, nDataIndex, nBufferSizeInBytes);

        if (nDataIndex < nBufferSizeInBytes)
        {
            /* wait small amount to avoid underrun */
			//pr_err("%s : 2. nDataIndex : %d, nBufferSizeInBytes : %d\n", __func__, nDataIndex, nBufferSizeInBytes);
            waitDuration = nDataIndex > 0 ? nDataIndex * 50 : 50;
            udelay(waitDuration);
        }
        else
        {
			//pr_err("%s : success\n", __func__);
            return VIBE_S_SUCCESS;
        }
    }

    //DbgOut((DBL_ERROR, "Dw7912I2CWriteWithResendOnError failed\n"));
//#endif
    return VIBE_E_FAIL;
}


/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(
        VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID,
        VibeUInt32 nFrequencyParameterValue) {
    return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
#if 0 //hoseong.kang
    char szRevision[24];
    if ((!szDevName) || (nSize < 1)) return VIBE_E_FAIL;

    //DbgOut((DBL_VERBOSE, "ImmVibeSPI_Device_GetName.\n"));

    /* Append revision number to the device name */
    sprintf(szRevision, "%s-dw7912-%02x.%02x", DEVICE_NAME, (dw7912.id << 4) | dw7912.model, dw7912.design);
    if (strlen(szRevision) + strlen(szDevName) < nSize - 1)
        strcat(szDevName, szRevision);

    /* Make sure the string is NULL terminated */
    szDevName[nSize - 1] = '\0';

    dw7912_dump_registers(&dw7912);
#endif
    return VIBE_S_SUCCESS;
}

/*
** Called by the TouchSense Player Service/Daemon when an application calls
** ImmVibeGetDeviceCapabilityInt32 with VIBE_DEVCAPTYPE_DRIVER_STATUS.
** The TouchSense Player does not interpret the returned status.
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetStatus(VibeUInt8 nActuatorIndex)
{
#if 0//hoseong.kang
    char ic_info = 0;

    if( 2 != i2c_recv_buf(dw7912p->dwclient, DW7912_INFO, &ic_info, 1) )
    {
        return VIBE_E_FAIL;
    }

    if( ic_info != 0xF8 )
    {
        return VIBE_E_FAIL;
    }
#endif
    return VIBE_S_SUCCESS;
}

#ifdef IMMVIBESPI_USE_BUFFERFULL
/*
** Check if the amplifier sample buffer is full (not ready for more data).
*/
IMMVIBESPIAPI int ImmVibeSPI_ForceOut_BufferFull(void)
{
	static int nResendAttempt = NAK_RESEND_ATTEMPT;
	static unsigned char fifo = 0;
	static unsigned char full = 0;
	
	//pr_err("%s, hoseong.kang enter\n",__func__);

	if(2 != i2c_recv_buf(dw7912p->dwclient, 0x01, &fifo, 1)) {
		/* failed too many consecutive times: stop managing the buffer */
        if (nResendAttempt < 0) {
            return -1;
        }
        /* try again later */
        DbgOut((DBL_WARNING, "dw7912_pktsize could not be read\n"));
        nResendAttempt -= 1;
        return 1;		
	}
	pr_err("%s, hoseong.kang fifo = %x\n",__func__, fifo);

	nResendAttempt = NAK_RESEND_ATTEMPT;
	
	//pr_err("%s, hoseong.kang fifo = %x\n",__func__, fifo);
	
	if(fifo & 0x40) {	// fifo full
		full = 1;
		return 1;
	}
	
	if(full == 1) {
		if(fifo & 0x10) {	// process done
			full = 0;
			return 0;
		}
		else {	// wait done flag
			//fifo = 0x01;
			//Dw7912I2CWrite(0x09, 1, &fifo);	
			return 1;
		}		
	}

	return 0;
	
	
#if 0//hoseong.kang
    static int nResendAttempt = NAK_RESEND_ATTEMPT;

    /* if the hardware is too busy to respond, assume it is full */
    if (2 != i2c_recv_buf(dw7912p->dwclient, DW7912_FIFOFULL, &fifo, 1)) {

        /* failed too many consecutive times: stop managing the buffer */
        if (nResendAttempt < 0) {
            return -1;
        }

        /* try again later */
        //DbgOut((DBL_WARNING, "dw7912_pktsize could not be read\n"));
        nResendAttempt -= 1;
        return 1;
    }

    nResendAttempt = NAK_RESEND_ATTEMPT;

    return fifo > DW79912_FIFOFULL_TARGET;
#else
	return 0;
#endif
}
#endif



static void dw7912_poweron(int mode)
{	
    unsigned char data;
    msleep(5);

	if(!mode) {
		// Software reset
		data = 0x01;
		if (VIBE_S_SUCCESS != Dw7912I2CWrite(DW7912_SWRESET, 1, &data))
		{
			//DbgOut((DBL_ERROR, "Dw7912I2CWrite failed to send SW_RESET_COMMAND\n"));
		}
		msleep(1);	

		data = RTP_MODE; // rtp mode set
		if (VIBE_S_SUCCESS != Dw7912I2CWrite(DW7912_MODE, 1, &data))
		{
			//DbgOut((DBL_ERROR, "Dw7912I2CWrite failed to send RTP_COMMAND\n"));
		}
		msleep(1);		

		// Send VD_Clamp level
		data = 0x46;	// set 2.8V clamp
		if (VIBE_S_SUCCESS != Dw7912I2CWrite(DW7912_LDO, 1, &data))
		{
			//DbgOut((DBL_ERROR, "Dw7912I2CWrite failed to send vd clamp data\n"));
		}
		msleep(1);
	}
	
	else {
		data = 0x01;
		if (VIBE_S_SUCCESS != Dw7912I2CWrite(DW7912_SWRESET, 1, &data))
		{
			//DbgOut((DBL_ERROR, "Dw7912I2CWrite failed to send SW_RESET_COMMAND\n"));
		}
		msleep(1);	
		
		data = MEM_MODE; // meomory mode set
		if (VIBE_S_SUCCESS != Dw7912I2CWrite(DW7912_MODE, 1, &data))
		{
			//DbgOut((DBL_ERROR, "Dw7912I2CWrite failed to send RTP_COMMAND\n"));
		}
		msleep(1);					
	}
    //dw7912_dump_registers(&dw7912);
}

/* 
 * sysfs device functions 
 * ---
 */
static ssize_t dw7912_show_regs(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *c = dw7912p->dwclient;
	int i, cnt = 0;

	for (i = 0; i < 0x20; i+=16) {
		cnt += sprintf(buf + cnt, "%02x: %02x", i, i2c_smbus_read_byte_data(c, i));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+1));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+2));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+3));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+4));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+5));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+6));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+7));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+8));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+9));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+10));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+11));
		cnt += sprintf(buf + cnt, " %02x",   i2c_smbus_read_byte_data(c, i+12));
		cnt += sprintf(buf + cnt, " %02x",   i2c_smbus_read_byte_data(c, i+13));
		cnt += sprintf(buf + cnt, " %02x",	 i2c_smbus_read_byte_data(c, i+14));
		cnt += sprintf(buf + cnt, " %02x\n", i2c_smbus_read_byte_data(c, i+15));
	}
	
	cnt += sprintf(buf + cnt, "23: %02x\n\n", i2c_smbus_read_byte_data(c, 0x23) ); // boost_option

	return cnt;
}


static ssize_t dw7912_store_regs(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t cnt)
{
	struct i2c_client *c = dw7912p->dwclient;
	unsigned int reg = 0, val = 0;
	char tmp[5];
	int i;

	memset(tmp,0x00,5);
	for(i = 0; i < cnt; i++) {
		if ( buf[i] != ' ' )
			tmp[i] = buf[i];
		else {
			buf += i+1;
			reg = simple_strtoul(tmp, NULL, 16);
			val = simple_strtoul(buf, NULL, 16);
			break;
		}
	}
	printk(KERN_INFO "writing: 0x%x: 0x%02X --> 0x%02X\n",
			reg, i2c_smbus_read_byte_data(c, reg), val);
	i2c_smbus_write_byte_data(c, reg, val);

	return cnt;
}

static DEVICE_ATTR(index_reg, 0660,
		dw7912_show_regs, dw7912_store_regs);

/* 
 * i2c driver functions 
 * ---
 */

#ifdef CONFIG_OF
static int dw7912_i2c_parse_dt(struct i2c_client *i2c, struct dw7912_priv *p)
{
	struct device *dev = &i2c->dev;
	struct device_node *np = dev->of_node;
	int ret = -1;
	u32 value;

	if (!np)
		return -1;


    ret = of_property_read_u32(np, "use_en_pin", &value);
    if (ret < 0) {
        dev_err(&i2c->dev, "use_en_pin read error\n");
        p->use_en_pin = -1;
    }
    p->use_en_pin = value;
		
	/* If you want to use en gpio pin */
	if (p->use_en_pin ) {
		p->enable_pin = of_get_named_gpio(np, "dw7912,en-gpio", 0);
		if (p->enable_pin < 0) {
			printk(KERN_ERR "Looking up %s property in node %s failed %d\n",
					"dw7912,en-gpio", dev->of_node->full_name,
					p->enable_pin);
			p->enable_pin = -1;
        
			goto error;
		}
		
		if( !gpio_is_valid(p->enable_pin) ) {
			printk(KERN_ERR "dw7912 enable_pin pin(%u) is invalid\n", p->enable_pin);
			goto error;
		}
	}

	gprintk("p->use_en_pin = %d\n", p->use_en_pin);
	gprintk("p->enable_pin = %d\n", p->enable_pin);
	return 0;

error:
	gprintk("p->use_en_pin = %d\n", p->use_en_pin);
	gprintk("p->enable_pin = %d\n", p->enable_pin);
	return -1;	
}
#else
static int dw7912_i2c_parse_dt(struct i2c_client *i2c, struct dw7912_priv *p)
{
	return NULL;
}
#endif


static int dw7912_i2c_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{
	//struct dw7912_priv *dw7912;
	int ret = -1;
	char cmd;

	gprintk("\n");

	dw7912p = kzalloc(sizeof(struct dw7912_priv), GFP_KERNEL);
	if (!dw7912p)
		return -ENOMEM;


	if (client->dev.of_node) {
		gprintk("Read enable pin from device tree\n");
		ret = dw7912_i2c_parse_dt(client, dw7912p);
		if( ret < 0 ) {
			dev_err(&client->dev, "%s: dt parse error, use_en_pin = %d,"\
			                      "enable_pin = %d\n", __func__, 
			                      dw7912p->use_en_pin, dw7912p->enable_pin);
		}
	} else {
		dw7912p->use_en_pin = -1;
	}

	if (dw7912p->use_en_pin == -1) {
		dev_err(&client->dev, "%s: use_en_pin error\n", __func__);
		ret = -EINVAL;
		goto error1;
	} 

	
	if (dw7912p->use_en_pin > 0) { 
		ret = gpio_request(dw7912p->enable_pin, "dw7912_en");
		if (ret < 0) {
			dev_err(&client->dev, "%s: dw7912_en gpio pin request error\n", __func__);
			ret = -EINVAL;
			goto error1;
		}
		
		ret = gpio_direction_output(dw7912p->enable_pin, 1);
		if (ret < 0) {
			dev_err(&client->dev, "enable pin level set failed");
			ret = -EIO;
			goto error2;
		}
	}

	i2c_set_clientdata(client, dw7912p);
	dw7912p->dwclient = client; /* i2c client pointer save */
	
	// code here!!
	dw7912_poweron(RTP_MODE);	// rtp mode setting
	
	
	ret = device_create_file(&client->dev, &dev_attr_index_reg);
	if (ret < 0) {
		dev_err(&client->dev, "create sysfs index_reg file --> failed");
		ret = -EIO;
		goto error2;
	}
	gprintk("create index_reg file\n");

	mutex_init(&dw7912p->lock);
	wakeup_source_init(&dw7912p->wklock,"vibrator");
	cmd = 0x00;
	Dw7912I2CWrite(DW7912_REG_04, 1, &cmd);

	printk("dw7912 probe success\n");

#ifdef DW7912_SYSFS
    ret = sysfs_create_group(&client->dev.kobj, &dw7912_attr_group);
#endif

	return 0;

error1:
	kfree(dw7912p);
	return ret;

error2:
	if (dw7912p->use_en_pin > 0)
		gpio_free(dw7912p->enable_pin);
	kfree(dw7912p);
	return ret;

#if 0
error3:
	mutex_destroy(&dw7912p->lock);
	wake_lock_destroy(&dw7912p->wklock);
	if (dw7912p->use_en_pin > 0)
		gpio_free(dw7912p->enable_pin);
	kfree(dw7912p);
	return ret;

error4:
//	timed_output_dev_unregister(&dw7912p->dev);
	mutex_destroy(&dw7912p->lock);
	wake_lock_destroy(&dw7912p->wklock);
	if (dw7912p->use_en_pin > 0)
		gpio_free(dw7912p->enable_pin);
	kfree(dw7912p);
	return ret;
#endif

}

static int dw7912_i2c_remove(struct i2c_client *client)
{
	struct dw7912_priv *p = i2c_get_clientdata(client);

	gprintk("1\n");
//	misc_deregister(&dw7912f_miscdev);
	
	if (dw7912p->use_en_pin > 0)
		gpio_free(p->enable_pin);
		
	device_remove_file(&client->dev, &dev_attr_index_reg);

	i2c_set_clientdata(client, NULL);

//	timed_output_dev_unregister(&p->dev);
//	hrtimer_cancel(&p->timer);
	mutex_destroy(&p->lock);

	wakeup_source_trash(&p->wklock);
	kfree(p);

	return 0;
}

static int __init dw7912_modinit(void)
{
	int ret;

	gprintk("\n");
	ret = i2c_add_driver(&dw7912_i2c_driver);
	//if (ret)
		//pr_err("Failed to register dw7912 I2C driver: %d\n", ret);

	return ret;
}

module_init(dw7912_modinit);

static void __exit dw7912_exit(void)
{
	i2c_del_driver(&dw7912_i2c_driver);
}

module_exit(dw7912_exit);

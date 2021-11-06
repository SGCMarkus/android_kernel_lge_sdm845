/*
 * LGE FUSB251 Moisture Detection Driver
 *
 * Copyright (C) 2020 LG Electronics, Inc.
 * Author: Jayci Choi <jayci.choi@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __FUSB251_H
#define __FUSB251_H

#include <linux/bitops.h>

enum moisture_state {
	FUSB251_STATE_DRY = 0,
	FUSB251_STATE_CC_DETECTING,
	FUSB251_STATE_CC_FLOATING,
	FUSB251_STATE_SBU_FLOATING,
	FUSB251_STATE_CC_SBU_AUTO_DETECTING,
	FUSB251_STATE_CC_MOS_DETECTED,
	FUSB251_STATE_SBU_MOS_DETECTED,
	FUSB251_STATE_TYPEC_CONNECTED,
	FUSB251_STATE_DS_CONNECTED,
	FUSB251_STATE_ERROR_RECOVERY,
};

enum {
	FUSB251_OVP_NONE = 0,
	FUSB251_CC_OVP_DETECTED,
	FUSB251_SBU_OVP_DETECTED,
	FUSB251_BOTH_OVP_DETECTED,
};

/* Register map */
#define FUSB251_PRODUCT_ID		0x01
#define REVISION_ID_MASK		GENMASK(1, 0)
#define PRODUCT_ID_MASK			GENMASK(3, 2)
#define DEVICE_ID_MASK			GENMASK(7, 4)

#define FUSB251_CONTROL			0x02
#define EN_MAN_SW_CTRL_BIT		BIT(0)
#define EN_CC_MOS_BIT			BIT(1)
#define EN_SBU_MOS_BIT			BIT(2)
#define EN_SBUFT_BIT			BIT(3)
#define EN_AUTO_SBU_BIT			BIT(4)
#define EN_DRY_BIT			BIT(5)
#define EN_DEBUG_ACC_DET_BIT		BIT(7)

#define FUSB251_INTERRUPT		0x03
#define OVP_INT_BIT			BIT(0)
#define OVP_REC_INT_BIT			BIT(1)
#define MOS_CHG_INT_BIT			BIT(2)
#define DRY_CHG_INT_BIT			BIT(3)
#define CC1_TIMER_INT_BIT		BIT(4)
#define CC2_TIMER_INT_BIT		BIT(5)

#define FUSB251_INTERRUPT_MASK		0x04
#define MASK_OVP_BIT			BIT(0)
#define MASK_OVP_REC_BIT		BIT(1)
#define MASK_MOS_DETECT_BIT		BIT(2)
#define MASK_DRY_DETECT_BIT		BIT(3)
#define MASK_CC1_TIMER_BIT		BIT(4)
#define MASK_CC2_TIMER_BIT		BIT(5)

#define FUSB251_STATUS			0x05
#define OVP_CC_BIT			BIT(0)
#define OVP_SBU_BIT			BIT(1)
#define LOOK4DRY_BIT			BIT(5)
#define LOOK4SBU_BIT			BIT(6)
#define LOOK4CC_BIT			BIT(7)

#define FUSB251_MOISTURE_STATUS		0x06
#define CC1_MOS_BIT			BIT(0)
#define CC2_MOS_BIT			BIT(1)
#define SBU1_MOS_BIT			BIT(2)
#define SBU2_MOS_BIT			BIT(3)
#define SBU1_FT_BIT			BIT(4)
#define SBU2_FT_BIT			BIT(5)
#define FAULT_MASK			GENMASK(7, 6)
#define NO_MOISTURE_ON_SBU             0x00
#define CC_MOS_BUT_NOT_SBU             0x40
#define SBU_FLOAT                      0x80
#define NO_MOISTURE_ON_BOTH            0xC0

#define FUSB251_SWITCH_CONTROL		0x07
#define CC_SW_CTRL_BIT			BIT(0)
#define SBU_SW_CTRL_MASK		GENMASK(2, 1)
#define OPEN_SBU1_SBU2			0x0
#define CLOSE_SBU1_SBU2			0x2
#define SBU2_FM_SBU1_OPEN		0x4
#define SBU1_FM_SBU2_OPEN		0x6

#define FUSB251_THRESHOLD1		0x08
#define CC_MOS_DET_THR_MASK		GENMASK(3, 0)
#define SBU_MOS_DET_THR_MASK		GENMASK(7, 4)
#define CC_MOS_THR_320K			0x9
#define CC_MOS_THR_480K			0xB
#define CC_MOS_THR_747K			0xD
#define CC_MOS_THR_960K			0xE
#define CC_MOS_THR_1280K		0xF
#define SBU_MOS_THR_320K		0x90
#define SBU_MOS_THR_480K		0xB0
#define SBU_MOS_THR_747K		0xD0
#define SBU_MOS_THR_960K		0xE0
#define SBU_MOS_THR_1280K		0xF0

#define FUSB251_THRESHOLD2		0x09
#define VDRY_THR_MASK			GENMASK(3, 0)
#define SBU_FLOAT_DET_THR_MASK		GENMASK(7, 4)
#define VDRY_THR_594K			0xC
#define VDRY_THR_747K			0xD
#define VDRY_THR_960K			0xE
#define VDRY_THR_1280K			0xF
#define SBU_FT_DET_THR_200MV		0x10
#define SBU_FT_DET_THR_300MV		0x20
#define SBU_FT_DET_THR_500MV		0x40
#define SBU_FT_DET_THR_600MV		0x50
#define SBU_FT_DET_THR_700MV		0x60

#define FUSB251_TIMER			0x0A
#define TDRY_MASK			GENMASK(2, 0)
#define TDRY_TIME_4S			0x5
#define TDRY_TIME_8S			0x6
#define TDRY_TIME_10S			0x7

#define FUSB251_RESET			0x0B
#define RESET_BIT			BIT(0)
#define MOS_RESET_BIT			BIT(1)

#define FUSB251_TIMER2			0x0C
#define CC_SETTLE_TIME_MASK		GENMASK(1, 0)
#define NUM_ADC_READ_MASK		GENMASK(3, 2)
#define CC_SETTLE_TIME_400US		0x0
#define CC_SETTLE_TIME_300US		0x1
#define CC_SETTLE_TIME_500US		0x2
#define CC_SETTLE_TIME_600US		0x3
#define ADC_READ_1_TIME			0x0
#define ADC_READ_2_TIMES		0x4
#define ADC_READ_3_TIMES		0x8

#endif /* __FUSB251_H */

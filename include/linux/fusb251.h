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
	FUSB251_STATE_MOS_DETECTECT,
};

enum {
	FUSB251_CC_MOISTURE_NONE = 0,
	FUSB251_CC1_MOISTURE_DETECTED,
	FUSB251_CC2_MOISTURE_DETECTED,
	FUSB251_BOTH_CC_MOISTURE_DETECTED,
};

enum {
	FUSB251_SBU_FLOATED_NONE = 0,
	FUSB251_SBU1_FLOATED,
	FUSB251_SBU2_FLOATED,
	FUSB251_BOTH_SBU_FLOATED,
};

enum {
	FUSB251_SBU1_MOISTURE_NONE = 0,
	FUSB251_SBU1_MOISTURE_DETECTED,
	FUSB251_SBU2_MOISTURE_DETECTED,
	FUSB251_BOTH_SBU_MOISTURE_DETECTED,
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

#define FSUB251_MOISTURE_STATUS		0x06
#define CC1_MOS_BIT			BIT(0)
#define CC2_MOS_BIT			BIT(1)
#define SBU1_MOS_BIT			BIT(2)
#define SBU2_MOS_BIT			BIT(3)
#define SBU1_FT_BIT			BIT(4)
#define SBU2_FT_BIT			BIT(5)
#define FAULT_MASK			GENMASK(7, 6)

#define FUSB251_SWITCH_CONTROL		0x07
#define CC_SW_CTRL_BIT			BIT(0)
#define SBU_SW_CTRL_MASK		GENMASK(2, 1)

#define FUSB251_THRESHOLD1		0x08
#define CC_MOS_DET_THR_MASK		GENMASK(3, 0)
#define SBU_MOS_DET_THR_MASK		GENMASK(7, 4)

#define FUSB251_THRESHOLD2		0x09
#define VDRY_THR_MASK			GENMASK(3, 0)
#define SBU_FLOAT_DET_THR_MASK		GENMASK(7, 4)

#define FUSB251_TIMER			0x0A
#define TDRY_MASK			GENMASK(2, 0)

#define FUSB251_RESET			0x0B
#define RESET_BIT			BIT(0)
#define MOS_RESET_BIT			BIT(1)

#define FUSB251_TIMER2			0x0C
#define CC_SETTLE_TIME_MASK		GENMASK(1, 0)
#define NUM_ADC_READ_MASK		GENMASK(3, 2)

#endif /* __FUSB251_H */

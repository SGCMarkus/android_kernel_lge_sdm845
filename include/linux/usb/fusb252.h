/*
 * FUSB252 Port Protection Switch driver
 *
 * Copyright (C) 2018 LG Electronics, Inc.
 * Author: Hansun Lee <hansun.lee@lge.com>
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
#ifndef __FUSB252_H__
#define __FUSB252_H__

struct device;

/*
 * FUSB252 modes & flags
 *
 * Must be declared in priority order. Small value have a high priority.
 * That is, 0 is the highest priority.
 */
enum fusb252_modes {
	FUSB252_MODE_SBU_DISABLE = 0,
	FUSB252_MODE_SBU_MD,
	FUSB252_MODE_SBU_FACTORY_ID,
	FUSB252_MODE_SBU_MD_ING,
	FUSB252_MODE_EDGE_MD,
	FUSB252_MODE_SBU_AUX,
	FUSB252_MODE_SBU_UART,
	FUSB252_MODE_SBU_USBID,
	FUSB252_MODE_EDGE_MD_ING,
	FUSB252_MODE_MAX,
};

#define FUSB252_FLAG_SBU_DISABLE	BIT(FUSB252_MODE_SBU_DISABLE)
#define FUSB252_FLAG_SBU_MD		BIT(FUSB252_MODE_SBU_MD)
#define FUSB252_FLAG_SBU_FACTORY_ID     BIT(FUSB252_MODE_SBU_FACTORY_ID)
#define FUSB252_FLAG_SBU_MD_ING		BIT(FUSB252_MODE_SBU_MD_ING)
#define FUSB252_FLAG_EDGE_MD		BIT(FUSB252_MODE_EDGE_MD)
#define FUSB252_FLAG_SBU_AUX		BIT(FUSB252_MODE_SBU_AUX)
#define FUSB252_FLAG_SBU_UART		BIT(FUSB252_MODE_SBU_UART)
#define FUSB252_FLAG_SBU_USBID		BIT(FUSB252_MODE_SBU_USBID)
#define FUSB252_FLAG_EDGE_MD_ING	BIT(FUSB252_MODE_EDGE_MD_ING)
#define FUSB252_FLAG_MAX		BIT(FUSB252_MODE_MAX)

struct fusb252_instance;

struct fusb252_desc {
	unsigned long flags;
	void (*ovp_callback)(struct fusb252_instance *inst, int ovp);
};

struct fusb252_instance {
	struct device			*dev;
	const struct fusb252_desc	*desc;
	unsigned long			flags;
	struct list_head		list;

	/* Driver private data */
	void				*drv_data;
};

int fusb252_get(struct fusb252_instance *inst, unsigned long flag);
int fusb252_put(struct fusb252_instance *inst, unsigned long flag);
unsigned long fusb252_get_current_flag(struct fusb252_instance *inst);
int fusb252_get_ovp_state(struct fusb252_instance *inst);

struct fusb252_instance *__must_check
devm_fusb252_instance_register(struct device *dev,
			       const struct fusb252_desc *desc);
void devm_fusb252_instance_unregister(struct device *dev,
				      struct fusb252_instance *inst);
#endif /* __FUSB252_H__ */

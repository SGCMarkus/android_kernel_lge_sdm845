#ifndef __LINUX_TOUCH_NOTIFY_H
#define __LINUX_TOUCH_NOTIFY_H

#include <linux/notifier.h>

#ifdef CONFIG_LGE_TOUCH_CORE_QOS
#define NOTIFY_NO_EVENT			0x00
#define NOTIFY_TOUCH_RESET		0x01
#define NOTIFY_CONNECTION		0x02
#define NOTIFY_WIRELESS			0x03
#define NOTIFY_IME_STATE		0x04
#define NOTIFY_DEBUG_TOOL		0x05
#define NOTIFY_CALL_STATE		0x06
#define NOTIFY_FB			0x07
#define NOTIFY_EARJACK			0x08
#define NOTIFY_DEBUG_OPTION		0x09
#define NOTIFY_ONHAND_STATE		0x0A
#define NOTIFY_QMEMO_STATE		0x0B
#define NOTIFY_PMEMO_STATE		0x0C
#define NOTIFY_FILM_STATE		0x0D
#define NOTIFY_ACTIVE_PEN_STATE		0x0E
#define NOTIFY_DUALSCREEN_STATE         0x0F
#define NOTIFY_FM_RADIO		        0x10

#define LCD_EVENT_LCD_MODE		0xD0
#define LCD_EVENT_READ_REG		0xD1
#define LCD_EVENT_TOUCH_RESET_START	0xD2
#define LCD_EVENT_TOUCH_RESET_END	0xD3
#define LCD_EVENT_LCD_BLANK		0xD4
#define LCD_EVENT_LCD_UNBLANK		0xD5
#define LCD_EVENT_TOUCH_LPWG_ON		0xD6
#define LCD_EVENT_TOUCH_LPWG_OFF	0xD7

enum {
	ATOMIC_NOTIFY_CONNECTION = 0,
	ATOMIC_NOTIFY_WIRELESS,
	ATOMIC_NOTIFY_EARJACK,
	ATOMIC_NOTIFY_FM_RADIO,
	ATOMIC_NOTIFY_EVENT_SIZE,
};

struct atomic_notify_event {
	unsigned long event;
	int data;
};

int touch_blocking_notifier_register(struct notifier_block *nb);
int touch_blocking_notifier_unregister(struct notifier_block *nb);
int touch_blocking_notifier_call(unsigned long val, void *v);

int touch_atomic_notifier_register(struct notifier_block *nb);
int touch_atomic_notifier_unregister(struct notifier_block *nb);
int touch_atomic_notifier_call(unsigned long val, void *v);

int touch_register_client(struct notifier_block *nb);
int touch_unregister_client(struct notifier_block *nb);
int touch_notifier_call_chain(unsigned long val, void *v);

extern int ignore_compared_event;

#else
#define NO_EVENT			0x00

/* the dsv on */
#define LCD_EVENT_TOUCH_LPWG_ON		0x01
#define LCD_EVENT_TOUCH_LPWG_OFF	0x02

#define LCD_EVENT_TOUCH_PWR_OFF      0XFF
/* to let lcd-driver know touch-driver's status */
#define LCD_EVENT_TOUCH_DRIVER_REGISTERED	0x03
/* For notifying proxy status to operate ENA control in lcd driver*/
#define LCD_EVENT_TOUCH_PROXY_STATUS 0X04
#define LCD_EVENT_TOUCH_SLEEP_STATUS 0X05
#define LCD_EVENT_TOUCH_SWIPE_STATUS 0X06
#define LCD_EVENT_TOUCH_PANEL_INFO_READ		0x07
#define LCD_EVENT_TOUCH_PANEL_INFO_WRITE	0x08

/* For PPlus */
#define NOTIFY_TOUCH_RESET			0x07
#define NOTIFY_CONNECTION			0x09
#define NOTIFY_WIRELEES				0x0A
#define NOTIFY_IME_STATE			0x0B
#define NOTIFY_DEBUG_TOOL			0x0C
#define NOTIFY_CALL_STATE			0x0D
#define NOTIFY_FB				0x0E
#define NOTIFY_EARJACK				0x0F
#define NOTIFY_DEBUG_OPTION			0x10
#define NOTIFY_ONHAND_STATE			0x12
#define NOTIFY_TOUCH_IRQ			0x13
#define NOTIFY_FILM_STATE			0x14
#define LCD_EVENT_HW_RESET			(NOTIFY_TOUCH_RESET)
#define LCD_EVENT_LCD_MODE			0x08
#define LCD_EVENT_READ_REG			0x11
#define NOTIFY_DUALSCREEN_STATE			0x00

struct touch_event {
	void *data;
};

enum {
	ATOMIC_NOTIFY_CONNECTION = 0,
	ATOMIC_NOTIFY_WIRELESS,
	ATOMIC_NOTIFY_EARJACK,
	ATOMIC_NOTIFY_EVENT_SIZE,
};

struct atomic_notify_event {
	unsigned long event;
	int data;
};

int touch_blocking_notifier_register(struct notifier_block *nb);
int touch_blocking_notifier_unregister(struct notifier_block *nb);
int touch_blocking_notifier_call(unsigned long val, void *v);

int touch_atomic_notifier_register(struct notifier_block *nb);
int touch_atomic_notifier_unregister(struct notifier_block *nb);
int touch_atomic_notifier_call(unsigned long val, void *v);

int touch_register_client(struct notifier_block *nb);
int touch_unregister_client(struct notifier_block *nb);
int touch_notifier_call_chain(unsigned long val, void *v);

#endif /*CONFIG_LGE_TOUCH_CORE_QOS */
#endif /* _LINUX_TOUCH_NOTIFY_H */
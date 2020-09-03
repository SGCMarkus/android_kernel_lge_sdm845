#ifndef _NAVI_INPUT_H_
#define _NAVI_INPUT_H_
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/input/mt.h>
#include <linux/printk.h>
#include "et520.h"
/*****************************************************************
*                                                                *
*                         Configuration                          *
*                                                                *
*****************************************************************/


/*
 * @ ENABLE_SWIPE_UP_DOWN
 *     ENABLE : Listening to swipe-up & swipe-down navigation events.
 *              Configure ENABLE_SWIPE_UP_DOWN properties below.
 *
 *     DISABLE : Ignore swipe-up & swipe-down navigation events.
 *               Don't care properties.
 *
 * @ ENABLE_SWIPE_LEFT_RIGHT
 *     ENABLE : Listening to swipe-left & swipe-right navigation events.
 *              configure ENABLE_SWIPE_LEFT_RIGHT properties below.
 *
 *     DISABLE : Ignore swipe-left & swipe-right navigation events.
 *               Don't care properties.

 */
 enum {
	KEY_RELEASE,
	KEY_PRESS,
	KEY_PRESS_RELEASE
};
#define	DISABLE		0
#define	ENABLE		1
#define ENABLE_SWIPE_UP_DOWN ENABLE
#define ENABLE_SWIPE_LEFT_RIGHT	ENABLE


/*
 * ENABLE_SWIPE_UP_DOWN properties
 *
 * If ENABLE_SWIPE_UP_DOWN set to DISABLE, these should neglected
 *
 *
 * @ KEYEVENT_UP : The key-event should be sent when swipe-up.
 * @ KEYEVENT_UP_ACTION : Action of KEYEVENT_UP.
 *
 * @ KEYEVENT_DOWN : The key-event should be sent when swipe-down.
 * @ KEYEVENT_DOWN_ACTION : Action of KEYEVENT_UP.
 *
 * @ ACTION:
 *   KEY_PRESS : Press key button
 *   KEY_RELEASE : Release key button
 *   KEY_PRESS_RELEASE : Combined action of press-then-release
 */
#define	KEYEVENT_UP				KEY_UP
#define	KEYEVENT_UP_ACTION		KEY_PRESS_RELEASE
#define	KEYEVENT_DOWN			KEY_DOWN
#define	KEYEVENT_DOWN_ACTION	KEY_PRESS_RELEASE


/*
 *ENABLE_SWIPE_LEFT_RIGHT properties.
 *
 * If ENABLE_SWIPE_LEFT_RIGHT set to DISABLE, these should neglected
 *
 *
 * @ KEYEVENT_RIGHT : The key-event should be sent when swipe-right.
 * @ KEYEVENT_RIGHT_ACTION : Action of KEYEVENT_RIGHT.
 *
 * @ KEYEVENT_LEFT : The key-event should be sent when swipe-left.
 * @ KEYEVENT_LEFT_ACTION : Action of KEYEVENT_LEFT.
 *
 * @ ACTION:
 *   KEY_PRESS : Press key button
 *   KEY_RELEASE : Release key button
 *   KEY_PRESS_RELEASE : Combined action of press-then-release
 */
#define	KEYEVENT_RIGHT			KEY_RIGHT
#define	KEYEVENT_RIGHT_ACTION	KEY_PRESS_RELEASE
#define	KEYEVENT_LEFT			KEY_LEFT
#define	KEYEVENT_LEFT_ACTION	KEY_PRESS_RELEASE




/*
 * @ TRANSLATED_COMMAND
 *     ENABLE : TRANSLATED command. Navigation events will be translated to
 *              logical user-events. e.g. click, double-click, long-click
 *              Configure TRANSLATED properties.
 *
 *     DISABLE : STRAIGHT command. Navigation events will be sent one-by-one
 *               directly.
 *               Configure STRAIGHT properties.
 */
#define	TRANSLATED_COMMAND		DISABLE
#define ENABLE_TRANSLATED_SINGLE_CLICK	ENABLE
#define ENABLE_TRANSLATED_DOUBLE_CLICK	ENABLE
#define ENABLE_TRANSLATED_LONG_TOUCH	ENABLE



#if TRANSLATED_COMMAND

//-------------------TRANSLATED properties---------------------

/*
 * @ ENABLE_TRANSLATED_SINGLE_CLICK
 *     ENABLE/DISABLE : enable/disable single-click event.
 *
 * @ ENABLE_TRANSLATED_DOUBLE_CLICK
 *     ENABLE/DISABLE : enable/disable double-click event.
 *
 * @ ENABLE_TRANSLATED_LONG_TOUCH
 *     ENABLE/DISABLE : enable/disable long-touch event.
 */



/*
 * @ LONGTOUCH_INTERVAL : Minimum time finger stay-on that counted to long-touch.
 *     Only concerned while ENABLE_TRANSLATED_LONG_TOUCH set to ENABLE.
 *     In millisecond (ms)
 *
 * @ DOUBLECLICK_INTERVAL : Maximum time between two click that counted to double-click.
 *     Only concerned while ENABLE_TRANSLATED_DOUBLE_CLICK set to ENABLE.
 *     In millisecond (ms)
 *
 * @ KEYEVENT_CLICK : The key-event should be sent when single-click.
 * @ KEYEVENT_CLICK_ACTION : Action of KEYEVENT_CLICK.
 *     Only concerned while ENABLE_TRANSLATED_SINGLE_CLICK set to ENABLE.
 *
 * @ KEYEVENT_DOUBLECLICK : The key-event should be sent when double-click.
 * @ KEYEVENT_DOUBLECLICK_ACTION : Action of KEYEVENT_DOUBLECLICK.
 *     Only concerned while ENABLE_TRANSLATED_DOUBLE_CLICK set to ENABLE.
 *
 * @ KEYEVENT_LONGTOUCH : The key-event should be sent when long-touch.
 * @ KEYEVENT_LONGTOUCH_ACTION : Action of KEYEVENT_LONGTOUCH.
 *     Only concerned while ENABLE_TRANSLATED_LONG_TOUCH set to ENABLE.
 *
 * @ ACTION:
 *   KEY_PRESS : Press key button
 *   KEY_RELEASE : Release key button
 *   KEY_PRESS_RELEASE : Combined action of press-then-release
 */
#define LONGTOUCH_INTERVAL		1000
#define DOUBLECLICK_INTERVAL	500
#define	KEYEVENT_CLICK				KEY_EXIT
#define	KEYEVENT_CLICK_ACTION		KEY_PRESS_RELEASE
#define	KEYEVENT_DOUBLECLICK		KEY_DELETE
#define	KEYEVENT_DOUBLECLICK_ACTION	KEY_PRESS_RELEASE
#define	KEYEVENT_LONGTOUCH			KEY_ENTER
#define	KEYEVENT_LONGTOUCH_ACTION	KEY_PRESS_RELEASE


//---------------End of TRANSLATED properties-----------------


#else	//STRAIGHT COMMAND


//-------------------STRAIGHT properties----------------------

/*
 * @ ENABLE_STRAIGHT_CANCEL
 *     ENABLE/DISABLE : enable/disable cancel event.
 *
 * @ ENABLE_STRAIGHT_ON
 *     ENABLE/DISABLE : enable/disable finger-on event.
 *
 * @ ENABLE_STRAIGHT_OFF
 *     ENABLE/DISABLE : enable/disable finger-off event.
 */
#define ENABLE_STRAIGHT_CANCEL	DISABLE
#define ENABLE_STRAIGHT_ON		ENABLE
#define ENABLE_STRAIGHT_OFF		ENABLE


/*
 * @ KEYEVENT_CANCEL : The key-event should be sent when cancel.
 * @ KEYEVENT_CANCEL_ACTION : Action of KEYEVENT_CANCEL.
 *     Only concerned while ENABLE_STRAIGHT_CANCEL set to ENABLE.
 *
 * @ KEYEVENT_ON : The key-event should be sent when finger-on.
 * @ KEYEVENT_ON_ACTION : Action of KEYEVENT_ON.
 *     Only concerned while ENABLE_STRAIGHT_ON set to ENABLE.
 *
 * @ KEYEVENT_OFF : The key-event should be sent when long-touch.
 * @ KEYEVENT_OFF_ACTION : Action of KEYEVENT_OFF.
 *     Only concerned while ENABLE_STRAIGHT_OFF set to ENABLE.
 *
 * @ ACTION:
 *   KEY_PRESS : Press key button
 *   KEY_RELEASE : Release key button
 *   KEY_PRESS_RELEASE : Combined action of press-then-release
 */
#define	KEYEVENT_CANCEL			KEY_0
#define	KEYEVENT_CANCEL_ACTION	KEY_PRESS_RELEASE
#define	KEYEVENT_ON				KEY_EXIT
#define	KEYEVENT_ON_ACTION		KEY_PRESS
#define	KEYEVENT_OFF			KEY_EXIT
#define	KEYEVENT_OFF_ACTION		KEY_RELEASE
#define	KEYEVENT_CLICK				KEY_EXIT
#define	KEYEVENT_CLICK_ACTION		KEY_PRESS_RELEASE
#define	KEYEVENT_DOUBLECLICK		KEY_DELETE
#define	KEYEVENT_DOUBLECLICK_ACTION	KEY_PRESS_RELEASE
#define	KEYEVENT_LONGTOUCH			KEY_ENTER
#define	KEYEVENT_LONGTOUCH_ACTION	KEY_PRESS_RELEASE

//-----------------End of STRAIGHT properties-------------------


#endif


/****************************************************************
*                                                               *
*                      End of Configuration                     *
*                                                               *
****************************************************************/

void uinput_egis_init(struct etspi_data *fp);
void uinput_egis_destroy(struct etspi_data *fp);
void sysfs_egis_init(struct etspi_data *fp);
void sysfs_egis_destroy(struct etspi_data *fp);
void send_key_event(struct etspi_data *fp, unsigned int code, int value);
void straight_command_converter(char cmd, struct etspi_data *fp);

#endif
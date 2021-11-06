#ifndef _LGE_DS3_H
#define _LGE_DS3_H

#define DS_VID				0x1004
#define DS_PID				0x637a

#define DS_DLOAD_VID			0x0483
#define DS_DLOAD_PID			0xdf11

#define DS2_PRODUCT_STR			"LMV515N"
#define DS3_TIME_PRODUCT_STR	"LMV600N"
#define DS3_CAYMAN_PRODUCT_STR  "LMG905N"

#define IS_DS2_USB(udev) \
	((udev->descriptor.idVendor == DS_VID) && \
	 (udev->descriptor.idProduct == DS_PID) && \
	 (udev->product && \
	  (!strcmp(udev->product, DS2_PRODUCT_STR) || \
	   !strcmp(udev->product, "DS2"))))

#define IS_DS2_DLOAD_USB(udev) \
	((udev->descriptor.idVendor == DS_DLOAD_VID) && \
	 (udev->descriptor.idProduct == DS_DLOAD_PID) && \
	 (udev->product && !strcmp(udev->product, DS2_PRODUCT_STR)))

#define IS_DS3_USB(udev) \
	((udev->descriptor.idVendor == DS_VID) && \
	 (udev->descriptor.idProduct == DS_PID) && \
	 (udev->product && \
	  (!strcmp(udev->product, DS3_TIME_PRODUCT_STR) || \
	   !strcmp(udev->product, DS3_CAYMAN_PRODUCT_STR) || \
	   !strcmp(udev->product, "DS3"))))

#define IS_DS3_DLOAD_USB(udev) \
	((udev->descriptor.idVendor == DS_DLOAD_VID) && \
	 (udev->descriptor.idProduct == DS_DLOAD_PID) && \
	 (udev->product && \
	 (!strcmp(udev->product, DS3_TIME_PRODUCT_STR) || \
	  !strcmp(udev->product, DS3_CAYMAN_PRODUCT_STR))))

#define IS_DS3_ANY_USB(udev) \
	(IS_DS3_USB(udev) || IS_DS3_DLOAD_USB(udev))

#define IS_DS2_ANY_USB(udev) \
	(IS_DS2_USB(udev) || IS_DS2_DLOAD_USB(udev))

enum {
	DS_STATE_DISCONNECTED = 0,
	DS_STATE_HALLIC_CONNECTED,
	DS_STATE_ACC_ID_CONNECTED,
	DS_STATE_USB_CONNECTED,
	DS_STATE_DP_CONNECTED,
	DS_STATE_HPD_ENABLED,
};

#ifdef CONFIG_LGE_DUAL_SCREEN
bool is_ds_connected(void);
int check_ds_connect_state(void);
void set_hallic_status(bool enable);
void set_vbus_recovery(bool set);
#else
static inline bool is_ds_connected(void) { return false; };
static inline int check_ds_connect_state(void) { return false; };
static inline bool void set_hallic_status(bool enable) { return false; };
static inline void set_vbus_recovery(bool set) { return; };
#endif

#endif // _LGE_DS3_H

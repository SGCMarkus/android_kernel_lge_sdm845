#include "navi_input.h"

#ifdef USE_FP_ID_DUAL
static int g_fp_id;
#endif

#define PROPERTY_NAVIGATION_ENABLE_DEFAULT  true

struct navi_struct {
	char cmd;
	struct etspi_data *fp;
	struct work_struct workq;
};

enum navi_event {
	NAVI_EVENT_CANCEL,		// 0
	NAVI_EVENT_ON,			// 1
	NAVI_EVENT_OFF,			// 2
	NAVI_EVENT_SWIPE,		// 3
	NAVI_EVENT_UP,			// 4
	NAVI_EVENT_DOWN,		// 5
	NAVI_EVENT_RIGHT,		// 6
	NAVI_EVENT_LEFT,		// 7
	NAVI_EVENT_DIRECT_CLICK,	// 8
	NAVI_EVENT_DIRECT_LONG_TOUCH,   // 9
	NAVI_EVENT_DIRECT_DOUBLE_CLICK  // 10
};

static struct timer_list long_touch_timer;
struct navi_struct navi_work_queue;
static bool g_KeyEventRaised = true;



/* Set event bits according to what events we would generate */
void init_event_enable(struct etspi_data *fp)
{
	set_bit(EV_KEY, fp->input_dev->evbit);
	set_bit(EV_SYN, fp->input_dev->evbit);
#if TRANSLATED_COMMAND
	set_bit(KEYEVENT_CLICK, fp->input_dev->keybit);
	set_bit(KEYEVENT_DOUBLECLICK, fp->input_dev->keybit);
	set_bit(KEYEVENT_LONGTOUCH, fp->input_dev->keybit);
	set_bit(KEYEVENT_UP, fp->input_dev->keybit);
	set_bit(KEYEVENT_DOWN, fp->input_dev->keybit);
	set_bit(KEYEVENT_RIGHT, fp->input_dev->keybit);
	set_bit(KEYEVENT_LEFT, fp->input_dev->keybit);
#else
	set_bit(KEYEVENT_CANCEL, fp->input_dev->keybit);
	set_bit(KEYEVENT_ON, fp->input_dev->keybit);
	set_bit(KEYEVENT_OFF, fp->input_dev->keybit);
	set_bit(KEYEVENT_UP, fp->input_dev->keybit);
	set_bit(KEYEVENT_DOWN, fp->input_dev->keybit);
	set_bit(KEYEVENT_RIGHT, fp->input_dev->keybit);
	set_bit(KEYEVENT_LEFT, fp->input_dev->keybit);
	set_bit(KEYEVENT_CLICK, fp->input_dev->keybit);
	set_bit(KEYEVENT_DOUBLECLICK, fp->input_dev->keybit);
	set_bit(KEYEVENT_LONGTOUCH, fp->input_dev->keybit);
#endif
}


void send_key_event(struct etspi_data *fp, unsigned int code, int value)
{
	struct etspi_data *obj = fp;

	if (value == KEY_PRESS_RELEASE) {
		input_report_key(obj->input_dev, code, 1);	// 1 is press
		input_sync(obj->input_dev);
		input_report_key(obj->input_dev, code, 0);	// 0 is release
		input_sync(obj->input_dev);
		printk(KERN_INFO "Egistec navigation driver, send key event KEY_PRESS_RELEASE: %d, action: %d\n", code, value);
	} else {
		input_report_key(obj->input_dev, code, value);
		input_sync(obj->input_dev);
	}

	printk(KERN_INFO "Egistec navigation driver, send key event: %d, action: %d\n", code, value);
}


#if ENABLE_TRANSLATED_LONG_TOUCH
static void long_touch_handler(unsigned long arg)
{
	struct etspi_data *fp = (struct etspi_data*)arg;
	if (g_KeyEventRaised == false) {
		g_KeyEventRaised = true;
		/* Long touch event */
		send_key_event(fp, KEYEVENT_LONGTOUCH, KEYEVENT_LONGTOUCH_ACTION);
	}
}
#endif

#if TRANSLATED_COMMAND
static unsigned long g_DoubleClickJiffies = 0;

void translated_command_converter(char cmd, struct etspi_data *fp)
{
	printk(KERN_INFO "Egistec navigation driver, translated cmd: %d\n", cmd);

	switch (cmd) {
	case NAVI_EVENT_CANCEL:
		g_KeyEventRaised = true;
		g_DoubleClickJiffies = 0;
#if ENABLE_TRANSLATED_LONG_TOUCH
		del_timer(&long_touch_timer);
#endif
		break;

	case NAVI_EVENT_ON:
		g_KeyEventRaised = false;
#if ENABLE_TRANSLATED_LONG_TOUCH
		long_touch_timer.data = (unsigned long)fp;
		mod_timer(&long_touch_timer, jiffies + (HZ * LONGTOUCH_INTERVAL / 1000));
#endif
		break;

	case NAVI_EVENT_OFF:
		if (g_KeyEventRaised == false) {
			g_KeyEventRaised = true;
#if ENABLE_TRANSLATED_DOUBLE_CLICK
			if ((jiffies - g_DoubleClickJiffies) < (HZ * DOUBLECLICK_INTERVAL / 1000)) {
				/* Double click event */
				send_key_event(fp, KEYEVENT_DOUBLECLICK, KEYEVENT_DOUBLECLICK_ACTION);
				g_DoubleClickJiffies = 0;
			} else {
#if ENABLE_TRANSLATED_SINGLE_CLICK
				/* Click event */
				send_key_event(fp, KEYEVENT_CLICK, KEYEVENT_CLICK_ACTION);
#endif
				g_DoubleClickJiffies = jiffies;
			}
#else

#if ENABLE_TRANSLATED_SINGLE_CLICK
			/* Click event */
			send_key_event(fp, KEYEVENT_CLICK, KEYEVENT_CLICK_ACTION);
#endif

#endif // ENABLE_DOUBLE_CLICK
		}
#if ENABLE_TRANSLATED_LONG_TOUCH
		del_timer(&long_touch_timer);
#endif
		break;

	case NAVI_EVENT_UP:
#if ENABLE_SWIPE_UP_DOWN
		if (g_KeyEventRaised == false) {
			g_KeyEventRaised = true;
			send_key_event(fp, KEYEVENT_UP, KEYEVENT_UP_ACTION);
		}
#endif
		break;

	case NAVI_EVENT_DOWN:
#if ENABLE_SWIPE_UP_DOWN
		if (g_KeyEventRaised == false) {
			g_KeyEventRaised = true;
			send_key_event(fp, KEYEVENT_DOWN, KEYEVENT_DOWN_ACTION);
		}
#endif
		break;

	case NAVI_EVENT_RIGHT:
#if ENABLE_SWIPE_LEFT_RIGHT
		if (g_KeyEventRaised == false) {
			g_KeyEventRaised = true;
			send_key_event(fp, KEYEVENT_RIGHT, KEYEVENT_RIGHT_ACTION);
		}
#endif
		break;

	case NAVI_EVENT_LEFT:
#if ENABLE_SWIPE_LEFT_RIGHT
		if (g_KeyEventRaised == false) {
			g_KeyEventRaised = true;
			send_key_event(fp, KEYEVENT_LEFT, KEYEVENT_LEFT_ACTION);
		}
#endif
		break;

	default:
		printk(KERN_ERR "Egistec navigation driver, cmd not match\n");
	}
}

#else //straight command (not define TRANSLATED_COMMAND)

void straight_command_converter(char cmd, struct etspi_data *fp)
{
	printk(KERN_DEBUG "Egistec navigation driver, straight cmd: %d\n", cmd);

	switch (cmd) {
	case NAVI_EVENT_CANCEL:
#if ENABLE_STRAIGHT_CANCEL
		send_key_event(fp, KEYEVENT_CANCEL, KEYEVENT_CANCEL_ACTION);
#endif
		break;

	case NAVI_EVENT_ON:
#if ENABLE_STRAIGHT_ON
		send_key_event(fp, KEYEVENT_ON, KEYEVENT_ON_ACTION);
#endif
		break;

	case NAVI_EVENT_OFF:
#if ENABLE_STRAIGHT_OFF
		send_key_event(fp, KEYEVENT_OFF, KEYEVENT_OFF_ACTION);
#endif
		break;

	case NAVI_EVENT_UP:
#if ENABLE_SWIPE_UP_DOWN
		send_key_event(fp, KEYEVENT_UP, KEYEVENT_UP_ACTION);
#endif
		break;

	case NAVI_EVENT_DOWN:
#if ENABLE_SWIPE_UP_DOWN
		send_key_event(fp, KEYEVENT_DOWN, KEYEVENT_DOWN_ACTION);
#endif
		break;

	case NAVI_EVENT_RIGHT:
#if ENABLE_SWIPE_LEFT_RIGHT
		send_key_event(fp, KEYEVENT_RIGHT, KEYEVENT_RIGHT_ACTION);
#endif
		break;

	case NAVI_EVENT_LEFT:
#if ENABLE_SWIPE_LEFT_RIGHT
		send_key_event(fp, KEYEVENT_LEFT, KEYEVENT_LEFT_ACTION);
#endif
		break;

	case NAVI_EVENT_DIRECT_CLICK:
		send_key_event(fp, KEYEVENT_CLICK, KEYEVENT_CLICK_ACTION);
		break;

	case NAVI_EVENT_DIRECT_DOUBLE_CLICK:
		send_key_event(fp, KEYEVENT_DOUBLECLICK, KEYEVENT_DOUBLECLICK_ACTION);
		break;

	case NAVI_EVENT_DIRECT_LONG_TOUCH:
		send_key_event(fp, KEYEVENT_LONGTOUCH, KEYEVENT_LONGTOUCH_ACTION);
		break;

	default:
		printk(KERN_ERR "Egistec navigation driver, cmd not match\n");
	}
}

#endif  //end of TRANSLATED_COMMAND

void navi_operator(struct work_struct *work)
{
    struct navi_struct *command = container_of(work, struct navi_struct, workq);

#if TRANSLATED_COMMAND
	translated_command_converter(command->cmd, command->fp);
#else
	straight_command_converter(command->cmd, command->fp);
#endif
}

static ssize_t navigation_event_func(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct etspi_data *fp = dev_get_drvdata(device);
	struct device *dev = &fp->pd->dev;

	dev_info(dev, "Egistec navigation driver, %s echo :'%d'\n", __func__, *buf);

	if (fp) {
		dev_dbg(dev, "%s spi_show\n", __func__);
		if (fp->pd) {
			dev_dbg(dev, "%s pd show\n", __func__);
		}
	} else
		dev_err(dev, "Egistec navigation driver, fp is NULL\n");

	if (fp->input_dev == NULL)
		dev_err(dev, "Egistec navigation driver, fp->input_dev is NULL\n");

	navi_work_queue.cmd = *buf;
	navi_work_queue.fp = fp;

	if (schedule_work(&(navi_work_queue.workq)) == 0)
		dev_err(dev, "Egistec navigation driver, fp is NULL\n");

	return count;
}
static DEVICE_ATTR(navigation_event, S_IWUSR, NULL, navigation_event_func);

static ssize_t property_navigation_enable_set(struct device *device,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct etspi_data *fp = dev_get_drvdata(device);
	struct device *dev = &fp->pd->dev;

	dev_info(dev, "Egistec navigation driver, %s echo :'%d'\n", __func__, *buf);

	if (!strncmp(buf, "enable", strlen("enable")))
	{
		fp->property_navigation_enable = 1;
	}
	else if (!strncmp(buf, "disable", strlen("disable")))
	{
		fp->property_navigation_enable = 0;
	}
	else
	{
		dev_err(dev, "strcmp not match\n");
	}
	return count;
}

static ssize_t property_navigation_enable_get(struct device *device,
		struct device_attribute *attr, char *buf)
{
	struct etspi_data *fp = dev_get_drvdata(device);
	struct device *dev = &fp->pd->dev;

	dev_info(dev, "Egistec navigation driver, %s echo :'%d'\n", __func__, *buf);

	return scnprintf(buf, PAGE_SIZE,"%s",
			fp->property_navigation_enable ? "enable":"disable");
}
static DEVICE_ATTR(navigation_enable, S_IRUGO | S_IWUSR,
				property_navigation_enable_get, property_navigation_enable_set);

#ifdef USE_FP_ID_DUAL
static ssize_t get_fp_id (
		struct device* device,
		struct device_attribute* attribute,
		char* buffer)
{
	struct etspi_data *fp = dev_get_drvdata(device);
	struct device *dev = &fp->pd->dev;
	(void) attribute;

	dev_info(dev, "fp_id = %d\n", g_fp_id);

	return scnprintf(buffer, PAGE_SIZE, "%i\n", g_fp_id);
}

static ssize_t set_fp_id (
		struct device* device,
		struct device_attribute* attribute,
		const char* buffer,
		size_t count)
{
	struct etspi_data *fp = dev_get_drvdata(device);
	struct device *dev = &fp->pd->dev;
	(void) attribute;

	if (*buffer == '1') {
		g_fp_id = 1;
	} else {
		g_fp_id = 0;
	}

	dev_info(dev, "fp_id = %d\n", g_fp_id);

	return count;
}

static ssize_t get_fp_id_real (
		struct device* device,
		struct device_attribute* attribute,
		char* buffer)
{
	struct etspi_data *fp = dev_get_drvdata(device);
	struct device *dev = &fp->pd->dev;
	int fp_id = get_fp_id_from_gpio();
	(void) attribute;

	dev_info(dev, "fp_id = %d\n", fp_id);

	return scnprintf(buffer, PAGE_SIZE, "%i\n", fp_id);
}

static DEVICE_ATTR(fp_id, S_IRUGO | S_IWUSR, get_fp_id, set_fp_id);
static DEVICE_ATTR(fp_id_real, S_IRUGO, get_fp_id_real, NULL);
#endif // USE_FP_ID_DUAL

/*-------------------------------------------------------------------------*/
/*
 *	Sysfs node creation
 */
static struct attribute *attributes[] = {
	&dev_attr_navigation_event.attr,
	&dev_attr_navigation_enable.attr,
	NULL,
};
static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

#ifdef USE_FP_ID_DUAL
static struct attribute *input_attributes[] = {
	&dev_attr_fp_id.attr,
	&dev_attr_fp_id_real.attr,
	NULL,
};

static const struct attribute_group attribute_group_input = {
	.attrs = input_attributes,
};
#endif // USE_FP_ID_DUAL

/*-------------------------------------------------------------------------*/


void uinput_egis_init(struct etspi_data *fp)
{
	int error = 0;
	printk(KERN_INFO "Egistec navigation driver, input_allocate_device\n");
	fp->input_dev = input_allocate_device();
	if(!fp->input_dev){
		printk(KERN_ERR "Egistec navigation driver, input_allocate_device failed\n");
	}

#if ENABLE_TRANSLATED_LONG_TOUCH
	init_timer(&long_touch_timer);
	long_touch_timer.function = long_touch_handler;
#endif

	fp->input_dev->name = "uinput-egis";
	fp->input_dev->dev.init_name = "lge_fingerprint";

	init_event_enable(fp);
	printk(KERN_INFO "Egistec navigation driver, init_event_enable\n");
	/* Register the input device */
	error = input_register_device(fp->input_dev);
	if (error) {
		printk(KERN_ERR "Egistec navigation driver, input_register_device failed\n");
		input_free_device(fp->input_dev);
		fp->input_dev = NULL;
	}
#ifdef USE_FP_ID_DUAL
	input_set_drvdata(fp->input_dev, fp);

	if (sysfs_create_group(&fp->input_dev->dev.kobj, &attribute_group_input) < 0)
	{
		printk(KERN_ERR "Egistec navigation driver, sysfs_create_group failed\n");
	}
#endif // USE_FP_ID_DUAL
}

void uinput_egis_destroy(struct etspi_data *fp)
{
	struct device *dev = &fp->pd->dev;

	dev_info(dev, "Egistec navigation driver, %s\n", __func__);

//	destroy_workqueue(&(navi_work_queue.workq));
#ifdef USE_FP_ID_DUAL
	sysfs_remove_group(&fp->input_dev->dev.kobj, &attribute_group);
#endif // USE_FP_ID_DUAL
#if ENABLE_TRANSLATED_LONG_TOUCH
	del_timer(&long_touch_timer);
#endif

	if (fp->input_dev != NULL)
		input_free_device(fp->input_dev);
}

void sysfs_egis_init(struct etspi_data *fp)
{
	struct device *dev = &fp->pd->dev;
	int status;

	dev_info(dev, "Egistec navigation driver, egis_input device init\n");
	fp->pd = platform_device_alloc("egis_input", -1);
	if (!fp->pd) {
		dev_err(dev, "Egistec navigation driver, platform_device_alloc fail\n");
		return;
	}

	status = platform_device_add(fp->pd);
	if (status != 0) {
		dev_err(dev, "Egistec navigation driver, platform_device_add fail\n");
		platform_device_put(fp->pd);
		return;
	}

	dev_set_drvdata(&fp->pd->dev, fp);
	status = sysfs_create_group(&fp->pd->dev.kobj, &attribute_group);
	if (status) {
		dev_err(dev, "Egistec navigation driver, could not create sysfs\n");
		platform_device_del(fp->pd);
		platform_device_put(fp->pd);
		return;
	}
}

void sysfs_egis_destroy(struct etspi_data *fp)
{
	struct device *dev = &fp->pd->dev;

	dev_info(dev, "Egistec navigation driver, %s\n", __func__);

	if (fp->pd) {
		platform_device_del(fp->pd);
		platform_device_put(fp->pd);
	}
}

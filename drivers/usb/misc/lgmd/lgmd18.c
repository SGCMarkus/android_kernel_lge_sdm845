#include "lgmd18.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#ifdef CONFIG_ARCH_QCOM
#include <soc/qcom/lge/board_lge.h>
#endif

extern int lgmd18_init(struct lgmd18 *lgmd18);

#define LGMD18_ATTR_R_u32(_name)					\
ssize_t lgmd18_attr_##_name##_show(struct device *dev,			\
	struct device_attribute *attr, char *buf)			\
{									\
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);		\
	struct iio_dev_attr *iio_attr = to_iio_dev_attr(attr);		\
	struct lgmd18 *lgmd18 = iio_priv(indio_dev);			\
	return sprintf(buf, "%d\n",					\
		       lgmd18->adc_chans[iio_attr->address]._name);	\
}									\
EXPORT_SYMBOL(lgmd18_attr_##_name##_show);

#define LGMD18_ATTR_W_u32(_name)					\
ssize_t lgmd18_attr_##_name##_store(struct device *dev,			\
	struct device_attribute *attr, const  char *buf, size_t count)	\
{									\
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);		\
	struct iio_dev_attr *iio_attr = to_iio_dev_attr(attr);		\
	struct lgmd18 *lgmd18 = iio_priv(indio_dev);			\
	u32 val;							\
	int ret;							\
	ret = kstrtou32(buf, 0, &val);					\
	if (ret)							\
		return ret;						\
	lgmd18->adc_chans[iio_attr->address]._name = val;		\
	return count;							\
}									\
EXPORT_SYMBOL(lgmd18_attr_##_name##_store);

#define LGMD18_ATTR_RW_u32(_name)					\
	LGMD18_ATTR_R_u32(_name)					\
	LGMD18_ATTR_W_u32(_name)

LGMD18_ATTR_R_u32(blue_thr_uv);
LGMD18_ATTR_R_u32(green_thr_uv);
LGMD18_ATTR_R_u32(yellow_thr_uv);
LGMD18_ATTR_R_u32(red_thr_uv);
LGMD18_ATTR_R_u32(black_thr_uv);
LGMD18_ATTR_R_u32(thr_margin_uv);
LGMD18_ATTR_RW_u32(timer_ms);

ssize_t lgmd18_attr_bias_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *iio_attr = to_iio_dev_attr(attr);
	struct lgmd18 *lgmd18 = iio_priv(indio_dev);

	switch (lgmd18->adc_chans[iio_attr->address].bias) {
	case PIN_CONFIG_BIAS_PULL_UP:
		return sprintf(buf, "pull-up\n");
	case PIN_CONFIG_BIAS_PULL_DOWN:
		return sprintf(buf, "pull-down\n");
	default:
		break;
	}

	return sprintf(buf, "high-impedance\n");
}
EXPORT_SYMBOL(lgmd18_attr_bias_show);

static enum alarmtimer_restart lgmd18_timer_function(struct alarm *alarm,
						     ktime_t now)
{
	struct lgmd18_adc *adc = container_of(alarm, struct lgmd18_adc, timer);
	struct iio_dev *indio_dev = adc->indio_dev;
	struct lgmd18 *lgmd18 = iio_priv(indio_dev);
	struct iio_chan_spec const *chan = adc->chan_spec;

	dev_dbg(lgmd18->dev, "%s: chan(%d)\n", __func__, chan->channel);

	if (adc->timer_work.func) {
		pm_stay_awake(lgmd18->dev);
		queue_work(system_highpri_wq, &adc->timer_work);
	} else {
		dev_err(lgmd18->dev, "%s: chan(%d) timer_work is NULL!!!\n",
			__func__, chan->channel);
	}

	return ALARMTIMER_NORESTART;
}

#ifndef CONFIG_LGE_USB_MOISTURE_DETECTION_NO_UX
static void lgmd18_probe_md_work(struct work_struct *w)
{
	struct lgmd18 *lgmd18 = container_of(w, struct lgmd18, probe_md_work);
	int val;
	int val2;
	union power_supply_propval propval = {0};
	int rc;
	int i;

	if (lgmd18->probe_md(lgmd18) > 0)
		goto moisture_detected;

	for (i = 0; i < lgmd18->num_channels; i++) {
		if (lgmd18->adc_chans[i].blue_thr_uv == UINT_MAX)
			continue;

		rc = lgmd18->info->read_raw(
				lgmd18->adc_chans[i].indio_dev,
				lgmd18->adc_chans[i].chan_spec,
				&val,
				&val2,
				IIO_CHAN_INFO_PROCESSED);
		if (rc < 0) {
			dev_err(lgmd18->dev, "Couldn't read raw rc=%d\n", rc);
			continue;
		}

		switch (lgmd18->adc_chans[i].bias) {
		case PIN_CONFIG_BIAS_PULL_UP:
			if (val > lgmd18->adc_chans[i].blue_thr_uv) {
				dev_info(lgmd18->dev, "The ADC value is greater than BLUE.\n");
				goto moisture_detected;
			}
			break;
		default:
			break;
		}
	}

	return;

moisture_detected:
	dev_info(lgmd18->dev, "Moisture detected in probe\n");

	propval.intval = 1;
	power_supply_set_property(lgmd18->usb_psy,
				  POWER_SUPPLY_PROP_MOISTURE_DETECTED,
				  &propval);
	power_supply_changed(lgmd18->usb_psy);
}
#endif

static void lgmd18_shutdown(struct platform_device *pdev)
{
	struct lgmd18 *lgmd18 = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (lgmd18->shutdown)
		lgmd18->shutdown(lgmd18);
}

static int lgmd18_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct lgmd18 *lgmd18;
	struct device_node *child;
	int rc;
	int i;

	pr_info("%s\n", __func__);

	if (lge_get_factory_boot() || lge_get_laf_mode()) {
		dev_err(dev, "If a factory cable is connected, LGMD will not be probed.\n");
		return -ENODEV;
	}

	indio_dev = devm_iio_device_alloc(dev, sizeof(*lgmd18));
	if (!indio_dev)
		return -ENOMEM;

	lgmd18 = iio_priv(indio_dev);
	lgmd18->dev = &pdev->dev;
#ifndef CONFIG_LGE_USB_MOISTURE_DETECTION_NO_UX
	INIT_WORK(&lgmd18->probe_md_work, lgmd18_probe_md_work);
#endif

	platform_set_drvdata(pdev, lgmd18);

	lgmd18->usb_psy = power_supply_get_by_name("usb");
	if (!lgmd18->usb_psy) {
		dev_err(dev, "Could not get USB power_supply, deferring probe\n");
		return -EPROBE_DEFER;
	}

	for_each_child_of_node(dev->of_node, child)
		lgmd18->num_channels++;

	lgmd18->adc_chans = devm_kzalloc(dev, sizeof(struct lgmd18_adc) *
					 lgmd18->num_channels, GFP_KERNEL);
	if (!lgmd18->adc_chans) {
		dev_err(dev, "Unable to allocate adc memory\n");
		return -ENOMEM;
	}

	lgmd18->vadc_chip = qpnp_get_vadc(dev, "channel");
	if (IS_ERR(lgmd18->vadc_chip)) {
		dev_err(dev, "Couldn't get vadc\n");
		rc = PTR_ERR(lgmd18->vadc_chip);
		return rc;
	}

	lgmd18->adc_tm_chip = qpnp_get_adc_tm(dev, "channel");
	if (IS_ERR(lgmd18->adc_tm_chip)) {
		dev_err(dev, "Couldn't get adc_tm\n");
		rc = PTR_ERR(lgmd18->adc_tm_chip);
		return rc;
	}

	i = 0;
	for_each_child_of_node(dev->of_node, child) {
		if (of_property_read_bool(child, "bias-pull-up"))
			lgmd18->adc_chans[i].bias = PIN_CONFIG_BIAS_PULL_UP;
		else if (of_property_read_bool(child, "bias-pull-down"))
			lgmd18->adc_chans[i].bias = PIN_CONFIG_BIAS_PULL_DOWN;
		else
			lgmd18->adc_chans[i].bias = PIN_CONFIG_BIAS_HIGH_IMPEDANCE;

		rc = of_property_read_u32(child, "blue-thr-uv",
					  &lgmd18->adc_chans[i].blue_thr_uv);
		if (rc < 0) {
			dev_err(dev, "Couldn't find blue-thr-uv");
			lgmd18->adc_chans[i].blue_thr_uv = UINT_MAX;
		}

		rc = of_property_read_u32(child, "green-thr-uv",
					  &lgmd18->adc_chans[i].green_thr_uv);
		if (rc < 0) {
			dev_err(dev, "Couldn't find green-thr-uv");
			switch (lgmd18->adc_chans[i].bias) {
			case PIN_CONFIG_BIAS_PULL_UP:
				lgmd18->adc_chans[i].green_thr_uv = UINT_MAX;
				break;
			case PIN_CONFIG_BIAS_PULL_DOWN:
			default:
				lgmd18->adc_chans[i].green_thr_uv = 0;
				break;
			}
		}

		rc = of_property_read_u32(child, "red-thr-uv",
					  &lgmd18->adc_chans[i].red_thr_uv);
		if (rc < 0) {
			dev_err(dev, "Couldn't find red-thr-uv");
			return -ENODEV;
		}

		rc = of_property_read_u32(child, "yellow-thr-uv",
					  &lgmd18->adc_chans[i].yellow_thr_uv);
		if (rc < 0) {
			dev_warn(dev, "Couldn't find yellow-thr-uv");
			lgmd18->adc_chans[i].yellow_thr_uv =
				lgmd18->adc_chans[i].red_thr_uv;
		}

#ifdef CONFIG_ARCH_QCOM
		lgmd18->adc_chans[i].param.high_thr =
		lgmd18->adc_chans[i].param.low_thr =
			lgmd18->adc_chans[i].yellow_thr_uv;
#endif

		rc = of_property_read_u32(child, "black-thr-uv",
					  &lgmd18->adc_chans[i].black_thr_uv);
		if (rc < 0) {
			dev_err(dev, "Couldn't find black-thr-uv");
			switch (lgmd18->adc_chans[i].bias) {
			case PIN_CONFIG_BIAS_PULL_UP:
				lgmd18->adc_chans[i].black_thr_uv = 0;
				break;
			case PIN_CONFIG_BIAS_PULL_DOWN:
			default:
				lgmd18->adc_chans[i].black_thr_uv = UINT_MAX;
				break;
			}
		}

		rc = of_property_read_u32(child, "thr-margin-uv",
					  &lgmd18->adc_chans[i].thr_margin_uv);
		if (rc < 0) {
			dev_err(dev, "Couldn't find thr-margin-uv");
			lgmd18->adc_chans[i].thr_margin_uv = 0;
		}

		rc = of_property_read_u32(child, "timer-ms",
					  &lgmd18->adc_chans[i].timer_ms);
		if (rc < 0) {
			dev_warn(dev, "Couldn't find timer-ms");
			return -ENODEV;
		}

#ifdef CONFIG_ARCH_QCOM
		rc = of_property_read_u32(child, "channel-num",
					  &lgmd18->adc_chans[i].param.channel);
		if (rc < 0) {
			dev_err(dev, "Couldn't find channel-num");
			return -ENODEV;
		}
#endif

		i++;
	}

	rc = lgmd18_init(lgmd18);
	if (rc < 0) {
		dev_err(dev, "Couldn't initialize rc=%d\n", rc);
		return rc;
	}

	indio_dev->dev.parent = dev;
	indio_dev->dev.of_node = node;
	indio_dev->name = pdev->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = lgmd18->info;
	indio_dev->channels = lgmd18->channels;
	indio_dev->num_channels = lgmd18->num_channels;

	for (i = 0; i < lgmd18->num_channels; i++) {
		lgmd18->adc_chans[i].indio_dev = indio_dev;
		lgmd18->adc_chans[i].chan_spec = &indio_dev->channels[i];
#ifdef CONFIG_ARCH_QCOM
		lgmd18->adc_chans[i].param.btm_ctx = &lgmd18->adc_chans[i];
#endif
		alarm_init(&lgmd18->adc_chans[i].timer,
			   ALARM_BOOTTIME,
			   lgmd18_timer_function);
	}

#ifndef CONFIG_LGE_USB_MOISTURE_DETECTION_NO_UX
	queue_work(system_highpri_wq, &lgmd18->probe_md_work);
#endif

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id lgmd18_match_table[] = {
	{ .compatible = "lge,lgmd18" },
	{ }
};
MODULE_DEVICE_TABLE(of, lgmd18_match_table);

static struct platform_driver lgmd18_driver = {
	.driver = {
		.name = "lgmd18",
		.of_match_table = lgmd18_match_table,
	},
	.probe = lgmd18_probe,
	.shutdown = lgmd18_shutdown,
};
module_platform_driver(lgmd18_driver);

#include "lgmd18.h"

#include <linux/usb/fusb252.h>

struct lgmd18_judy {
	struct fusb252_desc		fusb252_desc;
	struct fusb252_instance		*fusb252_inst;
};

static const char * const iio_ev_dir_text[] = {
	[IIO_EV_DIR_EITHER] = "either",
	[IIO_EV_DIR_RISING] = "rising",
	[IIO_EV_DIR_FALLING] = "falling",
	[IIO_EV_DIR_NONE] = "none",
};

static int lgmd18_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct lgmd18 *lgmd18 = iio_priv(indio_dev);
	struct lgmd18_judy *judy = lgmd18->private_data;
	struct lgmd18_adc *adc = &lgmd18->adc_chans[chan->channel];
	struct qpnp_vadc_result result;
	unsigned long fusb252_flag;
	int ret;


	if (mask != IIO_CHAN_INFO_PROCESSED) {
		ret = -EINVAL;
		goto out;
	}

	mutex_lock(&indio_dev->mlock);

	switch (chan->channel) {
	case 0:
		if (adc->param.state_request == ADC_TM_LOW_THR_ENABLE)
			qpnp_adc_tm_disable_chan_meas(lgmd18->adc_tm_chip,
						      &adc->param);

		fusb252_flag = fusb252_get_current_flag(judy->fusb252_inst);

		if (fusb252_flag != FUSB252_MODE_SBU_USBID) {
			if (fusb252_flag == FUSB252_FLAG_SBU_DISABLE) {
				fusb252_get(judy->fusb252_inst,
					    FUSB252_FLAG_SBU_MD);
				fusb252_put(judy->fusb252_inst,
					    FUSB252_FLAG_SBU_DISABLE);
			} else if (fusb252_flag != FUSB252_FLAG_SBU_MD_ING) {
				fusb252_get(judy->fusb252_inst,
					    FUSB252_FLAG_SBU_MD_ING);
			}
		}

		if (fusb252_get_ovp_state(judy->fusb252_inst) > 0) {
			switch (lgmd18->adc_chans[0].bias) {
			case PIN_CONFIG_BIAS_PULL_UP:
				result.physical = lgmd18->adc_chans[0].blue_thr_uv;
				break;
			default:
				qpnp_vadc_read(lgmd18->vadc_chip,
					       adc->param.channel,
					       &result);
				break;
			}
		} else {
			qpnp_vadc_read(lgmd18->vadc_chip,
				       adc->param.channel,
				       &result);
		}

		if (fusb252_flag != FUSB252_MODE_SBU_USBID) {
			if (fusb252_flag == FUSB252_FLAG_SBU_DISABLE) {
				fusb252_get(judy->fusb252_inst,
					    FUSB252_FLAG_SBU_DISABLE);
				fusb252_put(judy->fusb252_inst,
					    FUSB252_FLAG_SBU_MD);
			} else if (fusb252_flag != FUSB252_FLAG_SBU_MD_ING) {
				fusb252_put(judy->fusb252_inst,
					    FUSB252_FLAG_SBU_MD_ING);
			}
		}

		if (adc->param.state_request == ADC_TM_LOW_THR_ENABLE)
			qpnp_adc_tm_channel_measure(lgmd18->adc_tm_chip,
						    &adc->param);

		*val = result.physical;
		ret = IIO_VAL_INT;
		break;
	case 1:
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&indio_dev->mlock);

out:
	dev_dbg(lgmd18->dev, "%s: chan(%d), %d\n", __func__,
		chan->channel, (ret < 0) ? ret : *val);

	return ret;
}

static int lgmd18_read_event_config(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	struct lgmd18 *lgmd18 = iio_priv(indio_dev);
	struct lgmd18_adc *adc = &lgmd18->adc_chans[chan->channel];
	ktime_t remain;
	int ret;

	mutex_lock(&indio_dev->mlock);

	switch (chan->channel) {
	case 0:
		switch (dir) {
		case IIO_EV_DIR_EITHER:
		case IIO_EV_DIR_RISING:
			remain = alarm_expires_remaining(&adc->timer);
			ret = ktime_to_ms(remain) > 0 ? 1 : 0;
			break;
		case IIO_EV_DIR_FALLING:
			ret = adc->param.state_request == ADC_TM_LOW_THR_ENABLE;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	case 1:
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&indio_dev->mlock);

	dev_dbg(lgmd18->dev, "%s: chan(%d), dir(%s), %d\n", __func__,
		chan->channel, iio_ev_dir_text[dir], ret);

	return ret;
}

static int lgmd18_write_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     int state)
{
	struct lgmd18 *lgmd18 = iio_priv(indio_dev);
	struct lgmd18_judy *judy = lgmd18->private_data;
	struct lgmd18_adc *adc = &lgmd18->adc_chans[chan->channel];
	int ret = 0;

	mutex_lock(&indio_dev->mlock);

	switch (chan->channel) {
	case 0:
		alarm_cancel(&adc->timer);
		adc->timer.node.expires = ms_to_ktime(0);

		qpnp_adc_tm_disable_chan_meas(lgmd18->adc_tm_chip,
					      &adc->param);
		adc->param.state_request = ADC_TM_HIGH_LOW_THR_DISABLE;

		if (!state) {
			if (dir == IIO_EV_DIR_RISING)
				fusb252_get(judy->fusb252_inst,
					    FUSB252_FLAG_SBU_DISABLE);
			else
				fusb252_put(judy->fusb252_inst,
					    FUSB252_FLAG_SBU_DISABLE);
			break;
		}

		switch (dir) {
		case IIO_EV_DIR_EITHER:
			fusb252_get(judy->fusb252_inst,
				    FUSB252_FLAG_SBU_DISABLE);

			alarm_start_relative(&adc->timer,
					     ms_to_ktime(adc->timer_ms));
			break;
		case IIO_EV_DIR_RISING:
			fusb252_get(judy->fusb252_inst,
				    FUSB252_FLAG_SBU_DISABLE);

			adc->param.state_request = ADC_TM_HIGH_THR_ENABLE;
			alarm_start_relative(&adc->timer,
					     ms_to_ktime(adc->timer_ms));
			break;
		case IIO_EV_DIR_FALLING:
			fusb252_put(judy->fusb252_inst,
				    FUSB252_FLAG_SBU_DISABLE);

			adc->param.state_request = ADC_TM_LOW_THR_ENABLE;
			qpnp_adc_tm_channel_measure(lgmd18->adc_tm_chip,
						    &adc->param);
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	case 1:
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&indio_dev->mlock);

	dev_dbg(lgmd18->dev, "%s: chan(%d), dir(%s), state(%d), %d\n", __func__,
		chan->channel, iio_ev_dir_text[dir], state, ret);

	return ret;
}

static int lgmd18_write_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int val, int val2)
{
	struct lgmd18 *lgmd18 = iio_priv(indio_dev);
	struct lgmd18_adc *adc = &lgmd18->adc_chans[chan->channel];
	int ret = 0;

	mutex_lock(&indio_dev->mlock);

	switch (chan->channel) {
	case 0:
		switch (dir) {
		case IIO_EV_DIR_RISING:
			adc->param.high_thr = val;
			break;
		case IIO_EV_DIR_FALLING:
			adc->param.low_thr = val;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	case 1:
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&indio_dev->mlock);

	dev_dbg(lgmd18->dev, "%s: chan(%d), dir(%s), val(%d), %d\n", __func__,
		chan->channel, iio_ev_dir_text[dir], val, ret);

	return ret;
}

static int lgmd18_read_event_value(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info,
				   int *val, int *val2)
{
	struct lgmd18 *lgmd18 = iio_priv(indio_dev);
	struct lgmd18_adc *adc = &lgmd18->adc_chans[chan->channel];
	int ret;

	mutex_lock(&indio_dev->mlock);

	switch (chan->channel) {
	case 0:
		switch (dir) {
		case IIO_EV_DIR_RISING:
			*val = adc->param.high_thr;
			ret = IIO_VAL_INT;
			break;
		case IIO_EV_DIR_FALLING:
			*val = adc->param.low_thr;
			ret = IIO_VAL_INT;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	case 1:
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&indio_dev->mlock);

	dev_dbg(lgmd18->dev, "%s: chan(%d), dir(%s), val(%d), %d\n", __func__,
		chan->channel, iio_ev_dir_text[dir], *val, ret);

	return IIO_VAL_INT;
}

static void lgmd18_threshold_notification(enum qpnp_tm_state state, void *ctx)
{
	struct lgmd18_adc *adc = ctx;
	struct iio_dev *indio_dev = adc->indio_dev;
	struct lgmd18 *lgmd18 = iio_priv(indio_dev);
	struct iio_chan_spec const *chan = adc->chan_spec;

	dev_dbg(lgmd18->dev, "%s: chan(%d) %s\n", __func__,
		 chan->channel,
		 (state == ADC_TM_HIGH_STATE) ? "rising" : "falling");

	mutex_lock(&indio_dev->mlock);
	qpnp_adc_tm_disable_chan_meas(lgmd18->adc_tm_chip, &adc->param);
	adc->param.state_request = ADC_TM_HIGH_LOW_THR_DISABLE;
	mutex_unlock(&indio_dev->mlock);

	iio_push_event(indio_dev,
		       (state == ADC_TM_HIGH_STATE) ?
		       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
					    chan->channel,
					    IIO_EV_TYPE_THRESH,
					    IIO_EV_DIR_RISING) :
		       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
					    chan->channel,
					    IIO_EV_TYPE_THRESH,
					    IIO_EV_DIR_FALLING),
		       iio_get_time_ns(indio_dev));
}

static void lgmd18_threshold_work(struct work_struct *w)
{
	struct lgmd18_adc *adc =
		container_of(w, struct lgmd18_adc, timer_work);
	struct iio_dev *indio_dev = adc->indio_dev;
	struct lgmd18 *lgmd18 = iio_priv(indio_dev);
	struct iio_chan_spec const *chan = adc->chan_spec;
	enum qpnp_tm_state state;
#ifdef QUIET_MOISTURE_DETECTION
	int val;
	int val2;
	int ret;
#endif

	dev_dbg(lgmd18->dev, "%s: chan(%d)\n", __func__, chan->channel);

#ifdef QUIET_MOISTURE_DETECTION
	ret = lgmd18_read_raw(indio_dev, chan, &val, &val2,
			      IIO_CHAN_INFO_PROCESSED);
	if (ret < 0)
		return;

	if (adc->param.state_request == ADC_TM_HIGH_THR_ENABLE) {
		if (val >= adc->param.high_thr)
			state = ADC_TM_HIGH_STATE;
		else
			goto restart_timer;
	} else {
		if (val >= adc->param.high_thr)
			state = ADC_TM_HIGH_STATE;
		else if (val < adc->param.low_thr)
			state = ADC_TM_LOW_STATE;
		else
			goto restart_timer;
	}

	lgmd18_threshold_notification(state, adc);
	pm_relax(lgmd18->dev);
	return;

restart_timer:
	alarm_start_relative(&adc->timer,
			     ms_to_ktime(adc->timer_ms));
	pm_relax(lgmd18->dev);
#else
	state = ADC_TM_HIGH_STATE;
	lgmd18_threshold_notification(state, adc);
	pm_relax(lgmd18->dev);
	return;
#endif
}

#define LGMD18_VOLT_ATTR_NUM 8
extern ssize_t lgmd18_attr_bias_show(struct device *dev,
		struct device_attribute *attr, char *buf);
extern ssize_t lgmd18_attr_blue_thr_uv_show(struct device *dev,
		struct device_attribute *attr, char *buf);
extern ssize_t lgmd18_attr_green_thr_uv_show(struct device *dev,
		struct device_attribute *attr, char *buf);
extern ssize_t lgmd18_attr_yellow_thr_uv_show(struct device *dev,
		struct device_attribute *attr, char *buf);
extern ssize_t lgmd18_attr_red_thr_uv_show(struct device *dev,
		struct device_attribute *attr, char *buf);
extern ssize_t lgmd18_attr_black_thr_uv_show(struct device *dev,
		struct device_attribute *attr, char *buf);
extern ssize_t lgmd18_attr_thr_margin_uv_show(struct device *dev,
		struct device_attribute *attr, char *buf);
extern ssize_t lgmd18_attr_timer_ms_show(struct device *dev,
		struct device_attribute *attr, char *buf);
extern ssize_t lgmd18_attr_timer_ms_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static LGMD18_VOLT_ATTR_R(0, bias);
static LGMD18_VOLT_ATTR_R(0, blue_thr_uv);
static LGMD18_VOLT_ATTR_R(0, green_thr_uv);
static LGMD18_VOLT_ATTR_R(0, yellow_thr_uv);
static LGMD18_VOLT_ATTR_R(0, red_thr_uv);
static LGMD18_VOLT_ATTR_R(0, black_thr_uv);
static LGMD18_VOLT_ATTR_R(0, thr_margin_uv);
static LGMD18_VOLT_ATTR_RW(0, timer_ms);
static LGMD18_VOLT_ATTR_R(1, bias);
static LGMD18_VOLT_ATTR_R(1, blue_thr_uv);
static LGMD18_VOLT_ATTR_R(1, green_thr_uv);
static LGMD18_VOLT_ATTR_R(1, yellow_thr_uv);
static LGMD18_VOLT_ATTR_R(1, red_thr_uv);
static LGMD18_VOLT_ATTR_R(1, black_thr_uv);
static LGMD18_VOLT_ATTR_R(1, thr_margin_uv);
static LGMD18_VOLT_ATTR_RW(1, timer_ms);

static struct attribute *lgmd18_attrs[] = {
	&iio_dev_attr_0_bias.dev_attr.attr,
	&iio_dev_attr_0_blue_thr_uv.dev_attr.attr,
	&iio_dev_attr_0_green_thr_uv.dev_attr.attr,
	&iio_dev_attr_0_red_thr_uv.dev_attr.attr,
	&iio_dev_attr_0_yellow_thr_uv.dev_attr.attr,
	&iio_dev_attr_0_black_thr_uv.dev_attr.attr,
	&iio_dev_attr_0_thr_margin_uv.dev_attr.attr,
	&iio_dev_attr_0_timer_ms.dev_attr.attr,
	&iio_dev_attr_1_bias.dev_attr.attr,
	&iio_dev_attr_1_blue_thr_uv.dev_attr.attr,
	&iio_dev_attr_1_green_thr_uv.dev_attr.attr,
	&iio_dev_attr_1_red_thr_uv.dev_attr.attr,
	&iio_dev_attr_1_yellow_thr_uv.dev_attr.attr,
	&iio_dev_attr_1_black_thr_uv.dev_attr.attr,
	&iio_dev_attr_1_thr_margin_uv.dev_attr.attr,
	&iio_dev_attr_1_timer_ms.dev_attr.attr,
	NULL,
};

static struct attribute_group lgmd18_attrs_group = {
	.attrs = lgmd18_attrs,
};

static const struct iio_info lgmd18_info = {
	.attrs = &lgmd18_attrs_group,
	.read_raw = lgmd18_read_raw,
	.read_event_config = &lgmd18_read_event_config,
	.write_event_config = &lgmd18_write_event_config,
	.read_event_value = &lgmd18_read_event_value,
	.write_event_value = &lgmd18_write_event_value,
	.driver_module = THIS_MODULE,
};

static const struct iio_event_spec lgmd18_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec lgmd18_channels[] = {
	LGMD18_ADC_CHAN_VOLT(0, lgmd18_events, ARRAY_SIZE(lgmd18_events)),
};

static int lgmd18_probe_md(struct lgmd18 *lgmd18)
{
	struct lgmd18_judy *judy = lgmd18->private_data;
	int ovp;

	ovp = fusb252_get_ovp_state(judy->fusb252_inst);
	if (ovp > 0) {
		dev_info(lgmd18->dev, "OVP detected on FUSB252.\n");
		return true;
	}

	return false;
}

static void lgmd18_shutdown(struct lgmd18 *lgmd18)
{
	int i;

	for (i = 0; i < lgmd18->num_channels; i++) {
		lgmd18_write_event_config(lgmd18->adc_chans[i].indio_dev,
					  lgmd18->adc_chans[i].chan_spec,
					  IIO_EV_TYPE_THRESH,
					  IIO_EV_DIR_NONE,
					  0);
	}
}

int lgmd18_init(struct lgmd18 *lgmd18)
{
	struct device *dev = lgmd18->dev;
	struct lgmd18_judy *judy;
	int i;

	if (lgmd18->num_channels != ARRAY_SIZE(lgmd18_channels)) {
		dev_err(dev, "number of channels is not match\n");
		return -ENODEV;
	}

	judy = devm_kzalloc(lgmd18->dev, sizeof(*judy), GFP_KERNEL);
	if (!judy)
		return -ENOMEM;

	judy->fusb252_desc.flags = FUSB252_FLAG_SBU_DISABLE |
		FUSB252_FLAG_SBU_MD | FUSB252_FLAG_SBU_MD_ING |
		FUSB252_FLAG_EDGE_MD | FUSB252_FLAG_SBU_USBID |
		FUSB252_FLAG_EDGE_MD_ING;
	judy->fusb252_inst = devm_fusb252_instance_register(lgmd18->dev,
				    &judy->fusb252_desc);
	if (!judy->fusb252_inst) {
		dev_dbg(lgmd18->dev, "Could not get FUSB252, deferring probe\n");
		return -EPROBE_DEFER;
	}

	if (lgmd18->num_channels > 1)
		fusb252_get(judy->fusb252_inst, FUSB252_FLAG_EDGE_MD_ING);
	else
		fusb252_get(judy->fusb252_inst, FUSB252_FLAG_SBU_USBID);

	for (i = 0; i < lgmd18->num_channels; i++) {
		INIT_WORK(&lgmd18->adc_chans[i].timer_work,
			  lgmd18_threshold_work);
		lgmd18->adc_chans[i].param.threshold_notification =
			lgmd18_threshold_notification;
	}

	lgmd18_attrs[lgmd18->num_channels * LGMD18_VOLT_ATTR_NUM] = NULL;

	lgmd18->private_data = judy;
	lgmd18->info = &lgmd18_info;
	lgmd18->channels = lgmd18_channels;

	lgmd18->probe_md = lgmd18_probe_md;
	lgmd18->shutdown = lgmd18_shutdown;

	return 0;
}

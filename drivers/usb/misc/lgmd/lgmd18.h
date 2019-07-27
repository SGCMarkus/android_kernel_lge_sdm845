#ifndef __LGMD18_H__
#define __LGMD18_H__

#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/alarmtimer.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>

#ifdef CONFIG_ARCH_QCOM
#include <linux/qpnp/qpnp-adc.h>
#endif

#define LGMD18_ADC_CHAN(_index, _type, _mask, _event, _num_event)	\
{									\
	.type = _type,							\
	.indexed = 1,							\
	.channel = _index,						\
	.info_mask_separate = _mask,					\
	.event_spec = _event,						\
	.num_event_specs = _num_event,					\
}

#define LGMD18_ADC_CHAN_VOLT(_index, _event, _num_event)		\
	LGMD18_ADC_CHAN(_index, IIO_VOLTAGE,				\
		      BIT(IIO_CHAN_INFO_PROCESSED),			\
		      _event, _num_event)

#define LGMD18_VOLT_ATTR_R(_index, _name)				\
	IIO_DEVICE_ATTR_NAMED(_index##_##_name,				\
			      in_voltage##_index##_##_name,		\
			      S_IRUGO,					\
			      lgmd18_attr_##_name##_show,		\
			      NULL,					\
			      _index);

#define LGMD18_VOLT_ATTR_RW(_index, _name)				\
	IIO_DEVICE_ATTR_NAMED(_index##_##_name,				\
			      in_voltage##_index##_##_name,		\
			      S_IRUGO | S_IWUSR,			\
			      lgmd18_attr_##_name##_show,		\
			      lgmd18_attr_##_name##_store,		\
			      _index);

struct lgmd18 {
	struct device			*dev;
	void				*private_data;

	struct power_supply		*usb_psy;
	struct work_struct		probe_md_work;

	int (*probe_md)(struct lgmd18 *);
	void (*shutdown)(struct lgmd18 *);

	/* iio */
	const struct iio_info		*info;
	const struct iio_chan_spec	*channels;
	int				num_channels;

	/* adc */
#ifdef CONFIG_ARCH_QCOM
	struct qpnp_vadc_chip		*vadc_chip;
	struct qpnp_adc_tm_chip		*adc_tm_chip;
#endif
	struct lgmd18_adc		*adc_chans;
};

struct lgmd18_adc {
	struct iio_dev			*indio_dev;
	const struct iio_chan_spec	*chan_spec;

	struct alarm			timer;
	struct work_struct		timer_work;

	enum pin_config_param		bias;
	uint32_t			blue_thr_uv;
	uint32_t			green_thr_uv;
	uint32_t			yellow_thr_uv;
	uint32_t			red_thr_uv;
	uint32_t			black_thr_uv;
	uint32_t			thr_margin_uv;
	uint32_t			timer_ms;

#ifdef CONFIG_ARCH_QCOM
	struct qpnp_adc_tm_btm_param	param;
#endif
};

#endif /* __LGMD18_H__ */

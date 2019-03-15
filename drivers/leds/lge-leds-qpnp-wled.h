#ifndef _LGE_LEDS_QPNP_WLED_H_
#define _LGE_LEDS_QPNP_WLED_H_

#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX ""
static int wled_num_strings = 0;
u8 wled_strings[QPNP_WLED_MAX_STRINGS];
module_param_array(wled_strings, byte, &wled_num_strings, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(wled_strings, "overwrite device tree WLED strings");

static inline void overwrite_wled_strings(struct qpnp_wled *wled)
{
	int i;
	if (wled_num_strings) {
		wled->num_strings = wled_num_strings;
		for (i = 0; i < wled_num_strings; ++i) {
			wled->strings[i] = wled_strings[i];
		}
		pr_info("[Display] %s: overwrite WLED strings, num_strings=%d\n", __func__, wled_num_strings);
	}
}

#endif // _LGE_LEDS_QPNP_WLED_H_

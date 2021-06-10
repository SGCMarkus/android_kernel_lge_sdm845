/*
 * ES9218P Linux Driver
 *
 * The ES9218P is a high-performance 32-bit, 2-channel audio SABRE HiFi D/A
 * converter with headphone amplifier, analog volume control and output switch
 * designed for audiophile-grade portable application such as mobile phones and
 * digital music player, audiophile-grade portable application such as mobile
 * phones and digital music player, consumer applications such as USB DACs and
 * A/V receivers, as well as professional consumer applications such as USB
 * DACs and A/V receivers, as well as professional such as mixer consoles and
 * digital audio workstations.
 *
 * Copyright (c) 2016, ESS Technology International Ltd.
 * Copyright (c) 2020, Diab Neiroukh
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_qos.h>
#include <linux/pm_wakeup.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <trace/events/asoc.h>

#include "es9218p.h"
#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
#include "../../../../include/soc/qcom/lge/board_lge.h"
#endif

#if USE_CONTROL_EXTERNAL_LDO_FOR_DVDD == 1
#include <linux/regulator/consumer.h>
#endif

struct es9218_reg es9218_common_init_register[] = {
#if 0
	/* System Register */
	{ ES9218P_SYSTEM_REG, 0x00 },
	/* Input Selection */
	{ ES9218P_INPUT_SELECT, 0x8C },
	/* Mixing, Serial Data and Automute Configuration */
	{ ES9218P_AUTOMUTE_CONFIG, 0x34 },
	/* Analog Volume Control */
	{ ES9218P_ANALOG_VOL_CTRL, 0x58 },
	/* Automute Time */
	{ ES9218P_AUTOMUTE_TIME, 0x00 },
	/* Automute Level */
	{ ES9218P_AUTOMUTE_LEVEL, 0x68 },
	/* DoP and Volmue Ramp Rate */
	{ ES9218P_DOP_VOL_RAMP_RATE, 0x42 },
	/* Filter Bandwidth and System Mute */
	{ ES9218P_FILTER_BAND_SYSTEM_MUTE, 0x80 },
	/* GPIO{1,2} Configuration */
	{ ES9218P_GPIO1_2_CONFIG, 0xDD },
	/* GPIO1 Clock Debug Setting */
	{ ES9218P_GPIO1_2_CONFIG, 0xD3 },
	/* Master Mode and Sync Configuration */
	{ ES9218P_MASTERMODE_SYNC_CONFIG, 0x02 },
#endif
	/* Overcurrent Protection */
	{ ES9218P_OVERCURRENT_PROTECT, 0x90 },
	/* {ASRC,DPLL} Bandwidth */
	{ ES9218P_DPLL_BANDWIDTH, 0x8A },
	/* THD Compensation Bypass & Mono Mode */
	{ ES9218P_THD_COMP_MONO_MODE, 0x00 },
	/* Soft Start Configuration */
	{ ES9218P_SOFT_START_CONFIG, 0x07 },
#if 0
	/* Volume Control */
	{ ES9218P_VOL1_CTRL, 0x50 },
	/* Volume Control */
	{ ES9218P_VOL2_CTRL, 0x50 },
	/* Master Trim */
	{ ES9218P_MASTERTRIM_4, 0xFF },
	/* Master Trim */
	{ ES9218P_MASTERTRIM_3, 0xFF },
	/* Master Trim */
	{ ES9218P_MASTERTRIM_2, 0xFF },
	/* Master Trim */
	{ ES9218P_MASTERTRIM_1, 0x7F },
#endif
	/* GPIO Input Selection */
	{ ES9218P_GPIO_INPUT_SEL, 0x0F },
#if 0
	/* THD Compensation C2 (Left Channel) */
	{ ES9218P_THD_COMP_C2_2, 0x00 },
	/* THD Compensation C2 (Left Channel) */
	{ ES9218P_THD_COMP_C2_1, 0x00 },
	/* THD Compensation C2 (Left Channel) */
	{ ES9218P_THD_COMP_C3_2, 0x00 },
	/* THD Compensation C2 (Left Channel) */
	{ ES9218P_THD_COMP_C3_1, 0x00 },
	/* Charge Pump Soft Start Delay */
	{ ES9218P_CP_SS_DELAY, 0x62 },
#endif
	/* Charge Pump Soft Start Delay */
	{ ES9218P_GEN_CONFIG, 0xC4 },
#if 0
	/* General Confguration */
	{ ES9218P_AUTO_CLK_GEAR, 0x00 },
#endif
	/* GPIO Inversion & Automatic Clock Gearing */
	{ ES9218P_CP_CLOCK_2, 0x37 },
	/* GPIO Inversion & Automatic Clock Gearing */
	{ ES9218P_CP_CLOCK_1, 0x30 },
#if 0
	/* Amplifier Configuration */
	{ ES9218P_AMP_CONFIG, 0x00 },
	/* Programmable NCO */
	{ ES9218P_NCO_NUM_4, 0x00 },
	/* Programmable NCO */
	{ ES9218P_NCO_NUM_3, 0x00 },
	/* Programmable NCO */
	{ ES9218P_NCO_NUM_2, 0x00 },
	/* Programmable NCO */
	{ ES9218P_NCO_NUM_1, 0x00 },
	/* Programmable FIR RAM Address */
	{ ES9218P_FIR_RAM_ADDR, 0x00 },
	/* Programmable FIR RAM Data */
	{ ES9218P_FIR_RAM_DATA_3, 0x00 },
	/* Programmable FIR RAM Data */
	{ ES9218P_FIR_RAM_DATA_2, 0x00 },
	/* Programmable FIR RAM Data */
	{ ES9218P_FIR_RAM_DATA_1, 0x00 },
	/* Programmable FIR Configuration */
	{ ES9218P_FIR_CONFIG, 0x00 },
	/* Analog Control Override */
	{ ES9218P_ANALOG_CTRL_OVERRIDE, 0x00 },
	/* Digital Override */
	{ ES9218P_REG46_RESERVED, 0x00 },
	/* Reserved */
	{ ES9218P_REG47_RESERVED, 0x00 },
	/* Seperate Channel THD */
	{ ES9218P_REG48_RESERVED, 0x02 },
	/* Automatic Clock Gearing Thresholds */
	{ ES9218P_CLK_GEAR_THRESH_3, 0x62 },
	/* Automatic Clock Gearing Thresholds */
	{ ES9218P_CLK_GEAR_THRESH_2, 0xC0 },
	/* Automatic Clock Gearing Thresholds */
	{ ES9218P_CLK_GEAR_THRESH_1, 0x0D },
	/* THD Compensation C2 (Right Channel) */
	{ ES9218P_REG53_RESERVED, 0x00 },
	/* THD Compensation C2 (Right Channel) */
	{ ES9218P_REG54_RESERVED, 0x00 },
	/* THD Compensation C2 (Right Channel) */
	{ ES9218P_REG55_RESERVED, 0x00 },
	/* THD Compensation C2 (Right Channel) */
	{ ES9218P_REG56_RESERVED, 0x00 },
	/* DAC Analog Trim Control */
	{ ES9218P_REG60_RESERVED, 0x00 },
#endif
};

struct es9218_reg es9218_dop_init_register[] = {
#if 0
	/* Default Register */
	{ ES9218P_SYSTEM_REG, 0x00 },
#endif
	/* 32-bit Serial Input Selection */
	{ ES9218P_INPUT_SELECT, 0x80 },
	/* Enable DoP */
	{ ES9218P_DOP_VOL_RAMP_RATE, 0x4A },
	/* Enable DoP Master Mode */
	{ ES9218P_MASTERMODE_SYNC_CONFIG, 0x82 },
#if 0
	/* Enable DoP Master Mode */
	{ ES9218P_MASTERMODE_SYNC_CONFIG, 0x02 },
#endif
#if ENABLE_DOP_AUTO_MUTE == 1
	{ ES9218P_SOFT_START_CONFIG, 0x07 },
#endif
#if 0
	/* Variable Value from AP */
	{ ES9218P_VOL1_CTRL, 0x00 },
	/* Variable Value from AP */
	{ ES9218P_VOL2_CTRL, 0x00 },
#endif
	/* Disable Automatic Clock Gearing */
	{ ES9218P_AUTO_CLK_GEAR, 0x00 },
};

struct es9218_reg es9218_pcm_init_register[] = {
	/* Default Register */
	{ ES9218P_SYSTEM_REG, 0x00 },
#if ENABLE_DOP_AUTO_MUTE == 1
	{ ES9218P_AUTOMUTE_CONFIG, 0x34 },
	{ ES9218P_AUTOMUTE_TIME, 0x00 },
#endif
	/* Disable DoP */
	{ ES9218P_DOP_VOL_RAMP_RATE, 0x43 },
	/* Slave Mode */
	{ ES9218P_MASTERMODE_SYNC_CONFIG, 0x02 },
#if 0
	{ ES9218P_VOL1_CTRL, 0x00 },
	{ ES9218P_VOL2_CTRL, 0x00 },
#endif
	/* Disable Automatic Clock Gearing */
	{ ES9218P_AUTO_CLK_GEAR, 0x06 },
};

static struct es9218_priv *g_es9218_priv = NULL;

#if ES9218P_SYSFS == 1
/*
 * We only include the analogue supplies here; the digital supplies need to be
 * available well before this driver can be probed.
 */
struct registry_map es9218p_regmap[] = {
	{ "00_SYSTEM_REGISTERS", ES9218P_SYSTEM_REG, 1 },
	{ "01_INPUT_SELECTION", ES9218P_INPUT_SELECT, 1 },
	{ "02_MIXING_&_AUTOMUTE_CONFIGURATION", ES9218P_AUTOMUTE_CONFIG, 1 },
	{ "03_ANALOG_VOLUME_CONTROL", ES9218P_ANALOG_VOL_CTRL, 1 },
	{ "04_AUTOMUTE_TIME", ES9218P_AUTOMUTE_TIME, 1 },
	{ "05_AUTOMUTE_LEVEL", ES9218P_AUTOMUTE_LEVEL, 1 },
	{ "06_DoP_&_VOLUME_RAMP_RATE", ES9218P_DOP_VOL_RAMP_RATE, 1 },
	{ "07_FIILTER_BANDWIDTH_&_SYSTEM_MUTE", ES9218P_FILTER_BAND_SYSTEM_MUTE,
	  1 },
	{ "08_GPIO1-2_CONFIGURATION", ES9218P_GPIO1_2_CONFIG, 1 },
	{ "09_RESERVED_09", ES9218P_REG09_RESERVED, 1 },
	{ "10_MASTER_MODE_&_SYNC_CONFIGURATION", ES9218P_MASTERMODE_SYNC_CONFIG,
	  1 },
	{ "11_OVERCURRENT_PROTECTION", ES9218P_OVERCURRENT_PROTECT, 1 },
	{ "12_ASRC/DPLL_BANDWIDTH", ES9218P_DPLL_BANDWIDTH, 1 },
	{ "13_THD_COMPENSATION_BYPASS", ES9218P_THD_COMP_MONO_MODE, 1 },
	{ "14_SOFT_START_CONFIGURATION", ES9218P_SOFT_START_CONFIG, 1 },
	{ "15_VOLUME_CONTROL_1", ES9218P_VOL1_CTRL, 1 },
	{ "16_VOLUME_CONTROL_2", ES9218P_VOL2_CTRL, 1 },
	{ "17_MASTER_TRIM_3", ES9218P_MASTERTRIM_4, 1 },
	{ "18_MASTER_TRIM_2", ES9218P_MASTERTRIM_3, 1 },
	{ "19_MASTER_TRIM_1", ES9218P_MASTERTRIM_2, 1 },
	{ "20_MASTER_TRIM_0", ES9218P_MASTERTRIM_1, 1 },
	{ "21_GPIO_INPUT_SELECTION", ES9218P_GPIO_INPUT_SEL, 1 },
	{ "22_THD_COMPENSATION_C2_2", ES9218P_THD_COMP_C2_2, 1 },
	{ "23_THD_COMPENSATION_C2_1", ES9218P_THD_COMP_C2_1, 1 },
	{ "24_THD_COMPENSATION_C3_2", ES9218P_THD_COMP_C3_2, 1 },
	{ "25_THD_COMPENSATION_C3_1", ES9218P_THD_COMP_C3_1, 1 },
	{ "26_CHARGE_PUMP_SOFT_START_DELAY", ES9218P_CP_SS_DELAY, 1 },
	{ "27_GENERAL_CONFIGURATION", ES9218P_GEN_CONFIG, 1 },
	{ "28_RESERVED", ES9218P_REG28_RESERVED, 1 },
	{ "29_GIO_INVERSION_&_AUTO_CLOCK_GEAR", ES9218P_AUTO_CLK_GEAR, 1 },
	{ "30_CHARGE_PUMP_CLOCK_2", ES9218P_CP_CLOCK_2, 1 },
	{ "31_CHARGE_PUMP_CLOCK_1", ES9218P_CP_CLOCK_1, 1 },
	{ "32_AMPLIFIER_CONFIGURATION", ES9218P_AMP_CONFIG, 1 },
	{ "33_RESERVED", ES9218P_REG33_RESERVED, 1 },
	{ "34_PROGRAMMABLE_NCO_4", ES9218P_NCO_NUM_4, 1 },
	{ "35_PROGRAMMABLE_NCO_3", ES9218P_NCO_NUM_3, 1 },
	{ "36_PROGRAMMABLE_NCO_2", ES9218P_NCO_NUM_2, 1 },
	{ "37_PROGRAMMABLE_NCO_1", ES9218P_NCO_NUM_1, 1 },
	{ "38_RESERVED_38", ES9218P_REG38_RESERVED, 1 },
	{ "39_RESERVED_39", ES9218P_REG39_RESERVED, 1 },
	{ "40_PROGRAMMABLE_FIR_RAM_ADDRESS", ES9218P_FIR_RAM_ADDR, 1 },
	{ "41_PROGRAMMABLE_FIR_RAM_DATA_3", ES9218P_FIR_RAM_DATA_3, 1 },
	{ "42_PROGRAMMABLE_FIR_RAM_DATA_2", ES9218P_FIR_RAM_DATA_2, 1 },
	{ "43_PROGRAMMABLE_FIR_RAM_DATA_1", ES9218P_FIR_RAM_DATA_1, 1 },
	{ "44_PROGRAMMABLE_FIR_CONFIGURATION", ES9218P_FIR_CONFIG, 1 },
	{ "45_ANALOG_CONTROL_OVERRIDE", ES9218P_ANALOG_CTRL_OVERRIDE, 1 },
	{ "46_DIGITAL_OVERRIDE", ES9218P_REG46_RESERVED, 1 },
	{ "47_RESERVED", ES9218P_REG47_RESERVED, 1 },
	{ "48_SEPERATE_CH_THD", ES9218P_REG48_RESERVED, 1 },
	{ "49_AUTOMATIC_CLOCK_GEARING_THRESHOLDS_3", ES9218P_CLK_GEAR_THRESH_3,
	  1 },
	{ "50_AUTOMATIC_CLOCK_GEARING_THRESHOLDS_2", ES9218P_CLK_GEAR_THRESH_2,
	  1 },
	{ "51_AUTOMATIC_CLOCK_GEARING_THRESHOLDS_1", ES9218P_CLK_GEAR_THRESH_1,
	  1 },
	{ "52_RESERVED", ES9218P_REG52_RESERVED, 1 },
	{ "53_THD_COMPENSATION_C2_2", ES9218P_REG53_RESERVED, 1 },
	{ "54_THD_COMPENSATION_C2_1", ES9218P_REG54_RESERVED, 1 },
	{ "55_THD_COMPENSATION_C3_2", ES9218P_REG55_RESERVED, 1 },
	{ "56_THD_COMPENSATION_C3_1", ES9218P_REG56_RESERVED, 1 },
	{ "57_RESERVED", ES9218P_REG57_RESERVED, 1 },
	{ "58_RESERVED", ES9218P_REG58_RESERVED, 1 },
	{ "59_RESERVED", ES9218P_REG59_RESERVED, 1 },
	{ "60_DAC_ANALOG_TRIM_CONTROL", ES9218P_REG60_RESERVED, 1 },
	{ "64_CHIP_STATUS", ES9218P_CHIPSTATUS, 0 },
	{ "65_GPIO_READBACK", ES9218P_GPIO_READBACK, 0 },
	{ "66_DPLL_NUMBER_4", ES9218P_DPLL_NUM_4, 0 },
	{ "67_DPLL_NUMBER_3", ES9218P_DPLL_NUM_3, 0 },
	{ "68_DPLL_NUMBER_2", ES9218P_DPLL_NUM_2, 0 },
	{ "69_DPLL_NUMBER_1", ES9218P_DPLL_NUM_1, 0 },
	{ "70_RESERVED", ES9218P_REG70_RESERVED, 0 },
	{ "71_RESERVED", ES9218P_REG71_RESERVED, 0 },
	{ "72_INPUT_SELECTION_AND_AUTOMUTE_STATUS",
	  ES9218P_INPUT_SEL_AUTOMUTE_STATUS, 0 },
	{ "73_RAM_COEFFEICIENT_READBACK_3", ES9218P_RAM_COEFF_OUT_3, 0 },
	{ "74_RAM_COEFFEICIENT_READBACK_2", ES9218P_RAM_COEFF_OUT_2, 0 },
	{ "75_RAM_COEFFEICIENT_READBACK_1", ES9218P_RAM_COEFF_OUT_1, 0 },
};
#endif

#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
struct delayed_work *mute_work;
static struct workqueue_struct *mute_workqueue;
#endif

struct pm_qos_request req;

struct sabre_custom_filter sabre_custom_ft[] = {
	{ SABRE_FILTER_FASTROLLOFF,
	  SABRE_FILTER_SYMMETRY_COSINE,

	  { 39435,    329401,	1360560,  3599246,  6601764,  8388607,
	    6547850,  1001610,	-4505541, -5211008, -621422,  4064305,
	    3502332,  -1242784, -3894015, -1166807, 2855579,  2532881,
	    -1339935, -2924582, -80367,	  2636097,  1146596,  -1970283,
	    -1795151, 1163934,	2064146,  -376196,  -2034097, -299771,
	    1794630,  819647,	-1427924, -1172210, 1001806,  1366909,
	    -568081,  -1424737, 163577,	  1371999,  187673,   -1236340,
	    -472107,  1044192,	684373,	  -819180,  -825173,  581393,
	    899460,   -347185,	-915049,  129183,   881489,   63537,
	    -809087,  -225092,	708188,	  352362,   -588665,  -444558,
	    459555,   502831,	-328750,  -529834,  202807,   529247,
	    -86947,   -505444,	-14910,	  463255,   100207,   -407601,
	    -167645,  343169,	216913,	  -274343,  -248603,  205072,
	    264118,   -138622,	-265371,  77610,    254603,   -24054,
	    -234368,  -20767,	207269,	  56265,    -175733,  -82315,
	    142067,   99291,	-108348,  -108035,  76223,    109591,
	    -47020,   -105189,	21756,	  96258,    -966,     -84164,
	    -15138,   70169,	26578,	  -55507,   -33729,   41164,
	    37138,    -27899,	-37436,	  16357,    35525,    -6752,
	    -32266,   -997,	28572,	  7574,	    -25179,   -14901,
	    21439,    27122,	-9639,	  -47260,   -51810,   -31095,
	    -10524,   -1533 },

	  { 0, 0, 0, 0, 6917, 36010, 117457, 294022, 601886, 1056422, 1624172,
	    2218368, 2720371, 3007535, 0, 0 } },

	{ SABRE_FILTER_SLOWROLLOFF,
	  SABRE_FILTER_SYMMETRY_COSINE,

	  { 0,	      190799,	718147,	 1804532, 3481898,  5536299,  7411221,
	    8388607,  7873874,	5714951, 2416023, -1000917, -3378693, -3938372,
	    -2688907, -411446,	1708498, 2659931, 2122733,  557874,   -1084853,
	    -1908449, -1574818, -414103, 821794,  1417924,  1124432,  225950,
	    -678290,  -1054327, -761721, -67611,  563184,   761258,   480642,
	    -38190,   -449978,	-522233, -273035, 91351,    335183,   332443,
	    134036,   -101185,	-229380, -192459, -50752,   84442,    140173,
	    96992,    12276,	-56785,	 -75761,  -39621,   493,      32962,
	    38633,    17568,	-618,	 -164,	  62,	    18,	      -0,
	    0 },

	  { 6927430, 6927430, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },

	{ SABRE_FILTER_FASTROLLOFF,
	  SABRE_FILTER_SYMMETRY_SINE,

	  { -417,     1473,    3176,	 1404,	   -3339,    -2060,    4177,
	    3291,     -5719,   -4716,	 7588,	   6691,     -10108,   -9048,
	    13208,    11986,   -17155,	 -15435,   22004,    19525,    -28016,
	    -24217,   35328,   29596,	 -44225,   -35626,   54922,    42351,
	    -67758,   -49730,  83039,	 57778,	   -101197,  -66445,   122674,
	    75734,    -148069, -85603,	 178075,   96075,    -213635,  -107173,
	    255963,   119032,  -306783,	 -131888,  368551,   146257,   -445016,
	    -163107,  542118,  184404,	 -669965,  -214274,  847063,   262410,
	    -1111089, -355255, 1550405,	 584903,   -2419106, -1431913, 4642716,
	    8388607,  4642716, -1431913, -2419106, 584903,   1550405,  -355255,
	    -1111089, 262410,  847063,	 -214274,  -669965,  184404,   542118,
	    -163107,  -445016, 146257,	 368551,   -131888,  -306783,  119032,
	    255963,   -107173, -213635,	 96075,	   178075,   -85603,   -148069,
	    75734,    122674,  -66445,	 -101197,  57778,    83039,    -49730,
	    -67758,   42351,   54922,	 -35626,   -44225,   29596,    35328,
	    -24217,   -28016,  19525,	 22004,	   -15435,   -17155,   11986,
	    13208,    -9048,   -10108,	 6691,	   7588,     -4716,    -5719,
	    3291,     4177,    -2060,	 -3339,	   1404,     3176,     1473,
	    -417,     0 },

	  { 0, 0, 0, 8997, 47076, 154163, 392557, 819728, 1472724, 2329930,
	    3284770, 4177061, 4814439, 5043562, 0, 0 } },

	{ SABRE_FILTER_FASTROLLOFF,
	  SABRE_FILTER_SYMMETRY_SINE,

	  { -3131,   -11380,   17068,	5059,	  -21148,  -10470,   41391,
	    3177,    -60665,   6542,	88359,	  -29550,  -117174,  64953,
	    148119,  -119646,  -174668, 195542,	  193084,  -298418,  -194946,
	    432455,  171955,   -606753, -110716,  837093,  -7811,    -1163276,
	    220265,  1699543,  -606879, -2904582, 1311875, 8388607,  8388607,
	    1311875, -2904582, -606879, 1699543,  220265,  -1163276, -7811,
	    837093,  -110716,  -606753, 171955,	  432455,  -194946,  -298418,
	    193084,  195542,   -174668, -119646,  148119,  64953,    -117174,
	    -29550,  88359,    6542,	-60665,	  3177,	   41391,    -10470,
	    -21148,  5059,     17068,	-11380,	  -3131,   0,	     0,
	    0,	     0,	       0,	0,	  0,	   0,	     0,
	    0,	     0,	       0,	0,	  0,	   0,	     0,
	    0,	     0,	       0,	0,	  0,	   0,	     0,
	    0,	     0,	       0,	0,	  0,	   0,	     0,
	    0,	     0,	       0,	0,	  0,	   0,	     0,
	    0,	     0,	       0,	0,	  0,	   0,	     0,
	    0,	     0,	       0,	0,	  0,	   0,	     0,
	    0,	     0,	       0,	0,	  0,	   0,	     0,
	    0,	     0 },

	  { 0, 0, 0, 0, 0, 0, 99386, 355200, 880080, 1746102, 2801046, 3902774,
	    4736866, 5017414, 0, 0 } },
};

struct wakeup_source wl_shutdown;
struct wakeup_source wl_sleep;

/* Tuning for the LG V40 ThinQ */
#if defined(CONFIG_MACH_SDM845_JUDY)
static uint8_t advance_harmonic_comp_left[4] = { 0xfe, 0x01, 0x32, 0x00 };
static uint8_t advance_harmonic_comp_right[4] = { 0x9a, 0x01, 0x64, 0x00 };
static uint8_t aux_harmonic_comp_left[4] = { 0x1b, 0x01, 0x0c, 0xfe };
static uint8_t aux_harmonic_comp_right[4] = { 0xd2, 0x00, 0x34, 0xfe };
/* Tuning for the LG G8 ThinQ */
#elif defined(CONFIG_MACH_SM8150_ALPHA)
static uint8_t advance_harmonic_comp_left[4] = { 0x58, 0x02, 0x3c, 0x00 };
static uint8_t advance_harmonic_comp_right[4] = { 0x21, 0x02, 0x64, 0x00 };
static uint8_t aux_harmonic_comp_left[4] = { 0x4a, 0x01, 0xe4, 0xfd };
static uint8_t aux_harmonic_comp_right[4] = { 0x04, 0x01, 0x0c, 0xfe };
/* Tuning for the LG V50 ThinQ */
#elif defined(CONFIG_MACH_SM8150_FLASH)
static uint8_t advance_harmonic_comp_left[4] = { 0x76, 0x02, 0x3c, 0x00 };
static uint8_t advance_harmonic_comp_right[4] = { 0x53, 0x02, 0x32, 0x00 };
static uint8_t aux_harmonic_comp_left[4] = { 0x5e, 0x01, 0xd0, 0xfd };
static uint8_t aux_harmonic_comp_right[4] = { 0x22, 0x01, 0xee, 0xfd };
#else
static uint8_t advance_harmonic_comp_left[4] = { 0x30, 0x02, 0x3c, 0x00 };
static uint8_t advance_harmonic_comp_right[4] = { 0xd6, 0x01, 0x3c, 0x00 };
static uint8_t aux_harmonic_comp_left[4] = { 0x72, 0x01, 0x84, 0xfe };
static uint8_t aux_harmonic_comp_right[4] = { 0x3b, 0x01, 0x98, 0xfe };
#endif

static const uint8_t avc_vol_tbl[] = {
	0x40, /* 0dB */
	0x41, /* -1dB */
	0x42, /* -2dB */
	0x43, /* -3dB */
	0x44, /* -4dB */
	0x45, /* -5dB */
	0x46, /* -6dB */
	0x47, /* -7dB */
	0x48, /* -8dB */
	0x49, /* -9dB */
	0x4A, /* -10dB */
	0x4B, /* -11dB */
	0x4C, /* -12dB */
	0x4D, /* -13dB */
	0x4E, /* -14dB */
	0x4F, /* -15dB */
	0X50, /* -16dB */
	0X51, /* -17dB */
	0X52, /* -18dB */
	0X53, /* -19dB */
	0X54, /* -20dB */
	0X55, /* -21dB */
	0X56, /* -22dB */
	0X57, /* -23dB */
	0X58, /* -24dB */
};

static int call_common_init_registers = 0;

static unsigned int es9218_bps = 16;
static unsigned int es9218_is_amp_on = 0;
#if ES9218P_NCO == 1
static unsigned int es9218_mclk = 49152000;
#endif
static unsigned int es9218_power_state = ESS_PS_CLOSE;
static unsigned int es9218_rate = 48000;
static unsigned int es9218_start = 0;

#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
static int fade_direction = VOLUME_DOWN;
static int fade_count_debug_param = 99;
static int fade_term_debug_param = 99;
#endif

#if ES9218P_SYSFS == 1
static int forced_avc_volume = -1;
static int forced_headset_type = -1;
#endif

static int g_auto_mute_flag = 0;
static int g_avc_volume = 0;
#if ES9218P_DEBUG == 1
/* The time delay for ESS pop-click debugging. */
static int g_debug_delay = 500;
#endif
static int g_dop_flag = 0;
int g_ess_rev = 2;
bool g_ess_rev_check = false;
#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
static int g_fade_count;
#endif
static int g_headset_type = 0;
#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
static int g_left_fade_vol_per_step = 0;
static int g_left_fade_vol = 0;
#endif
static int g_left_volume = 0;
#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
static int g_right_fade_vol_per_step = 0;
static int g_right_fade_vol = 0;
#endif
static int g_right_volume = 0;
static int g_sabre_cf_num = 8;
static int g_volume = 0;

#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
bool lge_ess_fade_inout_init = false;
#endif

static const uint32_t master_trim_tbl[] = {
	0x7FFFFFFF, /* 0dB */
	0x78D6FC9D, /* 0.5dB */
	0x721482BF, /* 1dB */
	0x6BB2D603, /* 1.5dB */
	0x65AC8C2E, /* 2dB */
	0x5FFC888F, /* 2.5dB */
	0x5A9DF7AA, /* 3dB */
	0x558C4B21, /* 3.5dB */
	0x50C335D3, /* 4dB */
	0x4C3EA838, /* 4.5dB */
	0x47FACCEF, /* 5dB */
	0x43F4057E, /* 5.5dB */
	0x4026E73C, /* 6dB */
	0x3C90386F, /* 6.5dB */
	0x392CED8D, /* 7dB */
	0x35FA26A9, /* 7.5dB */
	0x32F52CFE, /* 8dB */
	0x301B70A7, /* 8.5dB */
	0x2D6A866F, /* 9dB */
	0x2AE025C2, /* 9.5dB */
	0x287A26C4, /* 10dB */
	0x26368073, /* 10.5dB */
	0x241346F5, /* 11dB */
	0x220EA9F3, /* 11.5dB */
	0x2026F30F, /* 12dB */
	0x1E5A8471, /* 12.5dB */
	0x1CA7D767, /* 13dB */
	0x1B0D7B1B, /* 13.5dB */
	0x198A1357, /* 14dB */
	0x181C5761, /* 14.5dB */
	0x16C310E3, /* 15dB */
	0x157D1AE1, /* 15.5dB */
	0x144960C5, /* 16dB */
	0x1326DD70, /* 16.5dB */
	0x12149A5F, /* 17dB */
	0x1111AEDA, /* 17.5dB */
	0x101D3F2D, /* 18dB */
	0xF367BED, /* 18.5dB */
	0xE5CA14C, /* 19dB */
	0xD8EF66D, /* 19.5dB */
	0xCCCCCCC, /* 20dB */
	0xC157FA9, /* 20.5dB */
	0xB687379, /* 21dB */
	0xAC51566, /* 21.5dB */
	0xA2ADAD1, /* 22dB */
	0x99940DB, /* 22.5dB */
	0x90FCBF7, /* 23dB */
	0x88E0783, /* 23.5dB */
	0x8138561, /* 24dB */
	0x79FDD9F, /* 24.5dB */
	0x732AE17, /* 25dB */
	0x6CB9A26, /* 25.5dB */
	0x66A4A52, /* 26dB */
	0x60E6C0B, /* 26.5dB */
	0x5B7B15A, /* 27dB */
	0x565D0AA, /* 27.5dB */
	0x518847F, /* 28dB */
	0x4CF8B43, /* 28.5dB */
	0x48AA70B, /* 29dB */
	0x4499D60, /* 29.5dB */
	0x40C3713, /* 30dB */
	0x3D2400B, /* 30.5dB */
	0x39B8718, /* 31dB */
	0x367DDCB, /* 31.5dB */
	0x337184E, /* 32dB */
	0x3090D3E, /* 32.5dB */
	0x2DD958A, /* 33dB */
	0x2B48C4F, /* 33.5dB */
	0x28DCEBB, /* 34dB */
	0x2693BF0, /* 34.5dB */
	0x246B4E3, /* 35dB */
	0x2261C49, /* 35.5dB */
	0x207567A, /* 36dB */
	0x1EA4958, /* 36.5dB */
	0x1CEDC3C, /* 37dB */
	0x1B4F7E2, /* 37.5dB */
	0x19C8651, /* 38dB */
	0x18572CA, /* 38.5dB */
	0x16FA9BA, /* 39dB */
	0x15B18A4, /* 39.5dB */
	0x147AE14, /* 40dB */
};

static uint8_t normal_harmonic_comp_left[4] = { 0x78, 0x00, 0x9a, 0xfc };
static uint8_t normal_harmonic_comp_right[4] = { 0x1e, 0x00, 0x12, 0xfd };

static const char *power_state[] = {
	"CLOSE", "OPEN", "BYPASS", "HIFI", "IDLE", "ACTIVE",
};

static int prev_dop_flag = 0;

#if ES9218P_SYSFS == 1
#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
static ssize_t get_fade_term_param(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	unsigned val = 1;

	sprintf(buf, "%s : current fade term %d   ", __func__,
		lge_ess_get_fade_term());
	pr_info("%s() : fade_term_define = %d, val %d", __func__,
		lge_ess_get_fade_term(), val);

	return val;
}

static ssize_t set_fade_term_param(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int value;
	unsigned val = count;

	sscanf(buf, "%d", &value);
	fade_term_debug_param = value;

	pr_info("%s() : new mute term = %d", __func__, lge_ess_get_fade_term());

	return val;
}

static ssize_t get_fade_mute_param(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	unsigned val = 1;

	sprintf(buf, "%s : current fade count %d   ", __func__,
		lge_ess_get_fade_count());
	pr_info("%s() : fade_count_define = %d, val %d", __func__,
		lge_ess_get_fade_count(), val);

	return val;
}

static ssize_t set_fade_mute_param(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int value;
	unsigned val = count;

	sscanf(buf, "%d", &value);
	fade_count_debug_param = value;

	pr_info("%s() : new mute count = %d", __func__,
		lge_ess_get_fade_count());

	return val;
}

static DEVICE_ATTR(fade_mute_count, S_IWUSR | S_IRUGO, get_fade_mute_param,
		   set_fade_mute_param);
static DEVICE_ATTR(fade_mute_term, S_IWUSR | S_IRUGO, get_fade_term_param,
		   set_fade_term_param);
#endif

static ssize_t es9218_registers_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	unsigned i, n, reg_count;
	uint8_t read_buf;

	reg_count = sizeof(es9218p_regmap) / sizeof(es9218p_regmap[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		read_buf = es9218_read_reg(g_es9218_priv->i2c_client,
					   es9218p_regmap[i].reg);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%-40s <#%02d>= 0x%02X\n",
			       es9218p_regmap[i].name, es9218p_regmap[i].reg,
			       read_buf);
	}

	return n;
}
static ssize_t es9218_registers_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned i, reg_count, value;
	int error = 0;
	char name[45];

	if (count >= 45) {
		pr_err("%s:input too long\n", __func__);
		return -1;
	}

	if (sscanf(buf, "%40s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -1;
	}

	pr_info("%s: %s %0xx", __func__, name, value);

	reg_count = sizeof(es9218p_regmap) / sizeof(es9218p_regmap[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, es9218p_regmap[i].name)) {
			if (es9218p_regmap[i].writeable) {
				error = es9218_write_reg(
					g_es9218_priv->i2c_client,
					es9218p_regmap[i].reg, value);

				if (error) {
					pr_err("%s:Failed to write %s\n",
					       __func__, name);

					return -1;
				}
			} else {
				pr_err("%s:Register %s is not writeable\n",
				       __func__, name);

				return -1;
			}

			return count;
		}
	}

	pr_err("%s:no such register %s\n", __func__, name);

	return -1;
}
static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO, es9218_registers_show,
		   es9218_registers_store);

static ssize_t get_forced_headset_type(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", g_headset_type);
}
static ssize_t set_forced_headset_type(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int input_val;

	sscanf(buf, "%d", &input_val);
	es9218p_sabre_hifi2lpb();
	g_volume = 0;

	g_headset_type = input_val + 1;
	forced_headset_type = input_val + 1;

	es9218p_sabre_bypass2hifi();

	return count;
}
static DEVICE_ATTR(headset_type, S_IWUSR | S_IRUGO, get_forced_headset_type,
		   set_forced_headset_type);

static ssize_t get_forced_avc_volume(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", g_avc_volume);
}
static ssize_t set_forced_avc_volume(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int input_vol;
	sscanf(buf, "%d", &input_vol);

	if (input_vol >= sizeof(avc_vol_tbl) / sizeof(avc_vol_tbl[0])) {
		pr_err("%s() : Invalid vol = %d return \n", __func__,
		       input_vol);

		return 0;
	}

	g_avc_volume = input_vol;
	forced_avc_volume = input_vol;

	es9218_set_avc_volume(g_es9218_priv->i2c_client, g_avc_volume);

	return count;
}
static DEVICE_ATTR(avc_volume, S_IWUSR | S_IRUGO, get_forced_avc_volume,
		   set_forced_avc_volume);

static struct attribute *es9218_attrs[] = {
#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
	&dev_attr_fade_mute_count.attr, &dev_attr_fade_mute_term.attr,
#endif
	&dev_attr_registers.attr,	&dev_attr_headset_type.attr,
	&dev_attr_avc_volume.attr,	NULL,
};

static const struct attribute_group es9218_attr_group = {
	.attrs = es9218_attrs,
};
#endif /* ES9218P_SYSFS */

#if ES9218P_NCO == 1
/**
 * es9218p_sabre_set_nco_num() - Enable NCO mode by sampling frequency.
 * @iFSR: The sampling frequency of the audio stream.
 * @iMCLK : The Xin/n value. Xin will be 50MHz and 'n' can be '1/2/4/8'.
 * @iMODE : Enable or disable NCO.
 *
 * Return: Describe the return value of function_name.
 * * 0             - Success
 * * -1            - Faliure
 */
static int es9218p_sabre_set_nco_num(int iFSR, int iMCLK, int iMODE)
{
	uint32_t nco_num;
	int ret = 0;
	int i;

	if (es9218_power_state < ESS_PS_HIFI) {
		pr_err("%s() : invalid state = %s\n", __func__,
		       power_state[es9218_power_state]);

		return -1;
	}

	pr_info("%s() : state = %s\n", __func__,
		power_state[es9218_power_state]);

	nco_num = (iFSR * 0x100000000) / iMCLK;
	pr_info("%s nco_num:%x\n", __func__, nco_num);

	for (i = 0; i < 4; i++) {
		if (iMODE)
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       es9218p_regmap[i + 34].reg,
					       (nco_num & 0xff));
		else
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       es9218p_regmap[i + 34].reg,
					       0x00);

		if (ret < 0) {
			pr_err("%s:Failed to write %s\n", __func__,
			       es9218p_regmap[i + 34].name);
			return -1;
		}

		nco_num = (nco_num >> 8);
	}

	return ret;
}
#endif /* ES9218P_NCO */

/*
 * The ES9812P has multiple power states. These are controlled by the variables
 * in the es9218_data structure:
 *
 * reset_gpio      -> This represents the HIFI_RESET_N signal.
 * power_gpio      -> This represents the HIFI_LDO_SW signal.
 * hph_switch_gpio -> This represents the HIFI_MODE2 signal.
 *
 * The states are set as follows:
 *
 * reset_gpio=H && hph_switch_gpio=L -> HiFi State
 * reset_gpio=L && hph_switch_gpio=H -> Low Power Bypass (LPB) State
 * reset_gpio=L && hph_switch_gpio=L -> Standby State
 * reset_gpio=H && hph_switch_gpio=H -> LoFi State
 */

static void es9218_power_gpio_H(void)
{
	gpio_set_value(g_es9218_priv->es9218_data->power_gpio, 1);

	pr_info("%s(): pa_gpio_level = %d\n", __func__,
		__gpio_get_value(g_es9218_priv->es9218_data->power_gpio));
}

static void es9218_power_gpio_L(void)
{
	gpio_set_value(g_es9218_priv->es9218_data->power_gpio, 0);

	pr_info("%s(): pa_gpio_level = %d\n", __func__,
		__gpio_get_value(g_es9218_priv->es9218_data->power_gpio));
}

static void es9218_reset_gpio_H(void)
{
#if USE_CONTROL_EXTERNAL_LDO_FOR_DVDD == 1
	int ret = regulator_enable(g_es9218_priv->es9218_data->vreg_dvdd);

	pr_info("%s(): turn on an external LDO connected to DVDD.[rc=%d]\n",
		__func__, ret);

	msleep(1);
#endif

	gpio_set_value(g_es9218_priv->es9218_data->reset_gpio, 1);

	pr_info("%s(): pa_gpio_level = %d\n", __func__,
		__gpio_get_value(g_es9218_priv->es9218_data->reset_gpio));
}

static void es9218_reset_gpio_L(void)
{
#if USE_CONTROL_EXTERNAL_LDO_FOR_DVDD == 1
	int ret;
#endif

	gpio_set_value(g_es9218_priv->es9218_data->reset_gpio, 0);

	pr_info("%s(): pa_gpio_level = %d\n", __func__,
		__gpio_get_value(g_es9218_priv->es9218_data->reset_gpio));

#if USE_CONTROL_EXTERNAL_LDO_FOR_DVDD == 1
	msleep(1);

	ret = regulator_disable(g_es9218_priv->es9218_data->vreg_dvdd);
	pr_info("%s(): turn off an external LDO connected to DVDD.[rc=%d]\n",
		__func__, ret);
#endif
}

static void es9218_hph_switch_gpio_H(void)
{
	gpio_set_value(g_es9218_priv->es9218_data->hph_switch, 1);
	pr_info("%s(): hph_switch = %d\n", __func__,
		__gpio_get_value(g_es9218_priv->es9218_data->hph_switch));
}

static void es9218_hph_switch_gpio_L(void)
{
	gpio_set_value(g_es9218_priv->es9218_data->hph_switch, 0);
	pr_info("%s(): hph_switch = %d\n", __func__,
		__gpio_get_value(g_es9218_priv->es9218_data->hph_switch));
}

static int es9218_master_trim(struct i2c_client *client, int vol)
{
	int ret = 0;
	uint32_t value;

	if (vol >= sizeof(master_trim_tbl) / sizeof(master_trim_tbl[0])) {
		pr_err("%s() : Invalid vol = %d return \n", __func__, vol);

		return 0;
	}

	value = master_trim_tbl[vol];
	pr_info("%s(): MasterTrim = %08X \n", __func__, value);

	if (es9218_power_state == ESS_PS_IDLE) {
		pr_err("%s() : Invalid vol = %d return \n", __func__, vol);

		return 0;
	}

	ret |= es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_MASTERTRIM_4,
				value & 0xFF);

	ret |= es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_MASTERTRIM_3,
				(value & 0xFF00) >> 8);

	ret |= es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_MASTERTRIM_2,
				(value & 0xFF0000) >> 16);

	ret |= es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_MASTERTRIM_1,
				(value & 0xFF000000) >> 24);

	return ret;
}

static int es9218_set_avc_volume(struct i2c_client *client, int vol)
{
	int ret = 0;
	uint8_t value;

	if (vol >= sizeof(avc_vol_tbl) / sizeof(avc_vol_tbl[0])) {
		pr_err("%s() : Invalid vol = %d return \n", __func__, vol);

		return 0;
	}

	value = avc_vol_tbl[vol];

	pr_info("%s(): AVC Volume = %X \n", __func__, value);

	ret |= es9218_write_reg(g_es9218_priv->i2c_client,
				ES9218P_ANALOG_VOL_CTRL, value);

	return ret;
}

static int es9218_set_thd(struct i2c_client *client, int headset)
{
	int ret = 0;

	switch (headset) {
	/* Normal HiFi Mode (<=50RZ) */
	case 1:
		/* -16dB reduction for THD Compensation 2. */
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_THD_COMP_C2_2,
				       normal_harmonic_comp_left[0]);
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_THD_COMP_C2_1,
				       normal_harmonic_comp_left[1]);

		/* -16dB reduction for THD Compensation 3. */
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_THD_COMP_C3_2,
				       normal_harmonic_comp_left[2]);
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_THD_COMP_C3_1,
				       normal_harmonic_comp_left[3]);

		/* -16dB reduction for THD Compensation 2. */
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_REG53_RESERVED,
				       normal_harmonic_comp_right[0]);
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_REG54_RESERVED,
				       normal_harmonic_comp_right[1]);

		/* -16dB reduction for THD Compensation 3. */
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_REG55_RESERVED,
				       normal_harmonic_comp_right[2]);
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_REG56_RESERVED,
				       normal_harmonic_comp_right[3]);
		break;

	/* High Impedance HiFi Mode (50RZ-600RZ) */
	case 2:
		/*  -1dB reduction for THD Compensation 2. */
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_THD_COMP_C2_2,
				       advance_harmonic_comp_left[0]);
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_THD_COMP_C2_1,
				       advance_harmonic_comp_left[1]);

		/*  -1dB reduction for THD Compensation 3. */
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_THD_COMP_C3_2,
				       advance_harmonic_comp_left[2]);
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_THD_COMP_C3_1,
				       advance_harmonic_comp_left[3]);

		/* -16dB reduction for THD Compensation 2. */
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_REG53_RESERVED,
				       advance_harmonic_comp_right[0]);
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_REG54_RESERVED,
				       advance_harmonic_comp_right[1]);

		/* -16dB reduction for THD Compensation 3. */
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_REG55_RESERVED,
				       advance_harmonic_comp_right[2]);
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_REG56_RESERVED,
				       advance_harmonic_comp_right[3]);
		break;

	/* AUX HiFi Mode (>600RZ) */
	case 3:
		/* -7dB reduction for THD Compensation 2. */
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_THD_COMP_C2_2,
				       aux_harmonic_comp_left[0]);
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_THD_COMP_C2_1,
				       aux_harmonic_comp_left[1]);

		/* -7dB reduction for THD Compensation 3. */
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_THD_COMP_C3_2,
				       aux_harmonic_comp_left[2]);
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_THD_COMP_C3_1,
				       aux_harmonic_comp_left[3]);

		/* -16dB reduction for THD Compensation 2. */
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_REG53_RESERVED,
				       aux_harmonic_comp_right[0]);
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_REG54_RESERVED,
				       aux_harmonic_comp_right[1]);

		/* -16dB reduction for THD Compensation 3. */
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_REG55_RESERVED,
				       aux_harmonic_comp_right[2]);
		ret = es9218_write_reg(g_es9218_priv->i2c_client,
				       ES9218P_REG56_RESERVED,
				       aux_harmonic_comp_right[3]);
		break;

	default:
		pr_err("%s() : Invalid headset = %d \n", __func__, headset);
		break;
	}

	pr_info("%s(): Headset Type = %d \n", __func__, headset);

	return ret;
}

static int es9218p_sabre_amp_start(struct i2c_client *client, int headset)
{
	int ret = 0;

#ifdef CONFIG_MACH_SDM845_JUDY
	/* GPIO2 must already be HIGH as part of standby2lpb. */
	if (!g_dop_flag)
		es9218_hph_switch_gpio_L();
#endif

	switch (headset) {
	/* Normal HiFi Mode (<=50RZ) */
	case 1:
		pr_notice("%s() : 1 valid headset = %d changing to hifi1.\n",
			  __func__, g_headset_type);
		es9218p_sabre_lpb2hifi(1);
		break;

	/* High Impedance HiFi Mode (50RZ-600RZ) */
	case 2:
		pr_notice("%s() : 2 valid headset = %d changing to hifi2.\n",
			  __func__, g_headset_type);
		es9218p_sabre_lpb2hifi(2);
		break;

	/* AUX HiFi Mode (>600RZ) */
	case 3:
		pr_notice("%s() : 3 valid headset = %d changing to hifi1.\n",
			  __func__, g_headset_type);
		es9218p_sabre_lpb2hifi(1);
		break;

	default:
		pr_err("%s() : Unknown headset = %d \n", __func__, headset);
		ret = 1;
		break;
	}

#ifdef CONFIG_MACH_SDM845_JUDY
	if (!g_dop_flag)
		es9218_hph_switch_gpio_H();
#endif

	return ret;
}

static int es9218p_sabre_amp_stop(struct i2c_client *client, int headset)
{
	int ret = 0;

	switch (headset) {
	/* Normal (<=50RZ) */
	case 1:
		pr_notice("%s() : 1 valid headset = %d changing to lpb.\n",
			  __func__, g_headset_type);
		es9218p_sabre_hifi2lofi2lpb();
		break;

	/* High Impedance (50RZ-600RZ) */
	case 2:
		pr_notice("%s() : 2 valid headset = %d changing to lpb.\n",
			  __func__, g_headset_type);
		es9218p_sabre_hifi2lofi2lpb();
		break;

	/* AUX (>600RZ) */
	case 3:
		pr_notice("%s() : 3 valid headset = %d changing to lpb.\n",
			  __func__, g_headset_type);
		es9218p_sabre_hifi2lofi2lpb();
		break;

	default:
		pr_err("%s() : Invalid headset = %d \n", __func__,
		       g_headset_type);
		ret = 1;
		break;
	}

	return ret;
}

/**
 * es9218_sabre_cfg_custom_filter() - Program the filter coefficients.
 */
static int
es9218_sabre_cfg_custom_filter(struct sabre_custom_filter *sabre_filter)
{
	int rc, i, *coeff;
	int count_stage1;
	uint8_t rv, reg;

	reg = es9218_read_reg(g_es9218_priv->i2c_client,
			      ES9218P_FILTER_BAND_SYSTEM_MUTE);
	pr_info("%s(): g_sabre_cf_num = %d, ES9218P_FILTER_BAND_SYSTEM_MUTE:%x \n",
		__func__, g_sabre_cf_num, reg);
	reg &= ~0xE0;

	if (g_sabre_cf_num > 3) {
		if (es9218_rate == 352800 || es9218_rate == 384000)
			rc = es9218_write_reg(g_es9218_priv->i2c_client,
					      ES9218P_FIR_CONFIG, 0x01);
		else
			rc = es9218_write_reg(g_es9218_priv->i2c_client,
					      ES9218P_FIR_CONFIG, 0x00);

		switch (g_sabre_cf_num) {
		/* Linear Phase Fast Roll-off Filter */
		case 4:
			rv |= 0x00;
			rc = es9218_write_reg(g_es9218_priv->i2c_client,
					      ES9218P_FILTER_BAND_SYSTEM_MUTE,
					      rv);
			break;

		/* Linear Phase Slow Roll-off Filter */
		case 5:
			rv |= 0x20;
			rc = es9218_write_reg(g_es9218_priv->i2c_client,
					      ES9218P_FILTER_BAND_SYSTEM_MUTE,
					      rv);
			break;

		/* Minimum Phase Fast Roll-off Filter */
		case 6:
			rv |= 0x40;
			rc = es9218_write_reg(g_es9218_priv->i2c_client,
					      ES9218P_FILTER_BAND_SYSTEM_MUTE,
					      rv);
			break;

		/* Minimum Phase Slow Roll-off Filter */
		case 7:
			rv |= 0x60;
			rc = es9218_write_reg(g_es9218_priv->i2c_client,
					      ES9218P_FILTER_BAND_SYSTEM_MUTE,
					      rv);
			break;

		/* Apodizing Fast Roll-off Filter Type 1 and 2 */
		case 8:
		case 9:
			/*
			 * Switch to the Minimum Phase Fast Roll-off Filter for the correct
			 * responce of the specified sample rate unless decoding MQA.
			 */
			rv |= 0x40;
			rc = es9218_write_reg(g_es9218_priv->i2c_client,
					      ES9218P_FILTER_BAND_SYSTEM_MUTE,
					      rv);
			break;

		/* Hybrid Fast Roll-off Filter */
		case 10:
			rv |= 0xC0;
			rc = es9218_write_reg(g_es9218_priv->i2c_client,
					      ES9218P_FILTER_BAND_SYSTEM_MUTE,
					      rv);
			break;

		/* Brick Wall Filter */
		case 11:
			rv |= 0xE0;
			rc = es9218_write_reg(g_es9218_priv->i2c_client,
					      ES9218P_FILTER_BAND_SYSTEM_MUTE,
					      rv);
			break;

		default:
			pr_info("%s(): default = %d \n", __func__,
				g_sabre_cf_num);
			break;
		}

		return rc;
	}

	count_stage1 = sizeof(sabre_filter->stage1_coeff) /
		       sizeof(sabre_filter->stage1_coeff[0]);

	pr_info("%s: count_stage1 : %d", __func__, count_stage1);

	rv = (sabre_filter->symmetry << 2) |
	     0x02; /* Set the write-enable bit. */
	rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_FIR_CONFIG,
			      rv);
	if (rc < 0) {
		pr_err("%s: rc = %d return ", __func__, rc);
		return rc;
	}

	rv = es9218_read_reg(g_es9218_priv->i2c_client,
			     ES9218P_FIR_RAM_ADDR - 1);

	/* Program the stage 1 filter coefficients. */
	coeff = sabre_filter->stage1_coeff;
	for (i = 0; i < count_stage1; i++) {
		uint8_t value[4];
		value[0] = i;
		value[1] = (*coeff & 0xff);
		value[2] = ((*coeff >> 8) & 0xff);
		value[3] = ((*coeff >> 16) & 0xff);
		i2c_smbus_write_block_data(g_es9218_priv->i2c_client,
					   ES9218P_FIR_RAM_ADDR - 1, 4, value);
		coeff++;
	}

	/* Program the stage 2 filter coefficients. */
	coeff = sabre_filter->stage2_coeff;
	for (i = 0; i < 16; i++) {
		uint8_t value[4];
		value[0] = 128 + i;
		value[1] = (*coeff & 0xff);
		value[2] = ((*coeff >> 8) & 0xff);
		value[3] = ((*coeff >> 16) & 0xff);
		i2c_smbus_write_block_data(g_es9218_priv->i2c_client,
					   ES9218P_FIR_RAM_ADDR - 1, 4, value);
		coeff++;
	}

	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_FIR_RAM_ADDR - 1,
			 rv);

	/* Select the custom filter roll-off shape. */
	rv = (sabre_filter->shape << 5);
	rv |= 0x80;

	rc = es9218_write_reg(g_es9218_priv->i2c_client,
			      ES9218P_FILTER_BAND_SYSTEM_MUTE, rv);
	if (rc < 0) {
		pr_err("%s: rc = %d return ", __func__, rc);

		return rc;
	}

	rv = (sabre_filter->symmetry << 2); /* Unset the write-enable bit. */
	rv |= 0x1; /* Use the custom oversampling filter. */
	rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_FIR_CONFIG,
			      rv);
	if (rc < 0) {
		pr_err("%s: rc = %d return ", __func__, rc);

		return rc;
	}

	return 0;
}

static int es9218p_sabre_lpb2hifi(int mode)
{
	/*
	 * Declare the register starting point so that we can use incremental
	 * OR and AND+1C instead of hex literals. For example:
	 *
	 * x |= y would set bits in x which are 1 in y.
	 * x &= ~y would clear bits in x which are 1 in y.
	 */
	uint8_t register_45_value = 0;
	uint8_t register_46_value = 0;
	uint8_t register_47_value = 0;
	uint8_t register_48_value = 0;

	int value = 0;

	pr_info("%s(): entry: state = %s\n", __func__,
		power_state[es9218_power_state]);

	/*
	 * HPAHiQ implies EN_SEPARATE_THD_COMP = 1 and STATE3_CTRL_SEL = 11, for
	 * the minimum state-machine delay time.
	 */
#if USE_HPAHiQ == 1
	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG48_RESERVED,
			 register_48_value = 0x0F);
#else
	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG48_RESERVED,
			 register_48_value = 0x07);
#endif
#if ES9218P_DEBUG == 1
	pr_info("%s(): R48 = %X \n", __func__, register_48_value);
	mdelay(g_debug_delay);
#endif

	/*
	 * Enable overrides since the amp input shunt and output shunt are both
	 * engaged.
	 */
	if (g_es9218_priv->es9218_data->use_internal_ldo) {
		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_REG46_RESERVED,
				 register_46_value |= 0x80);
	} else {
		register_46_value |= 0x80;

		/* Set SEL1V to 1 and use the external LDO. */
		register_46_value |= 0x04;

		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_REG46_RESERVED, register_46_value);
	}
#if ES9218P_DEBUG == 1
	pr_info("%s(): R46 = %X \n", __func__, register_46_value);
	mdelay(g_debug_delay);
#endif

	switch (mode) {
	case 1:
		/* Set AMP_PDB_SS to 0. */
		es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_AMP_CONFIG,
				 0x02);
#if ES9218P_DEBUG == 1
		pr_info("%s(): R32 = %X \n", __func__, 0x02);
		mdelay(g_debug_delay);
#endif
		break;
	case 2:
		/*
		 * Set AMP_PDB_SS to 0.
		 *
		 * Note: This block is different for the HiFi2 Mode.
		 */
		es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_AMP_CONFIG,
				 0x03);
#if ES9218P_DEBUG == 1
		pr_info("%s(): R32 = %X \n", __func__, 0x03);
		mdelay(g_debug_delay);
#endif
		break;
	}

	/* Set ATC to the minimum . */
	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_ANALOG_VOL_CTRL,
			 0x18);
#if ES9218P_DEBUG == 1
	pr_info("%s(): R03 = %X \n", __func__, 0x18);
	mdelay(g_debug_delay);
#endif

	/* Preset low voltage chargepump for the weak CPL mode. */
	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG47_RESERVED,
			 register_47_value |= 0x08);
#if ES9218P_DEBUG == 1
	pr_info("%s(): R47 = %X \n", __func__, register_47_value);
	mdelay(g_debug_delay);
#endif

	/* Preset high voltage chargepump for the weak CPH mode. */
	es9218_write_reg(g_es9218_priv->i2c_client,
			 ES9218P_ANALOG_CTRL_OVERRIDE,
			 register_45_value |= 0x08);
#if ES9218P_DEBUG == 1
	pr_info("%s(): R45 = %X \n", __func__, register_45_value);
	mdelay(g_debug_delay);
#endif

	/* Enable override control of AUX switch. */
	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG47_RESERVED,
			 register_47_value |= 0x60);
#if ES9218P_DEBUG == 1
	pr_info("%s(): R47 = %X \n", __func__, register_47_value);
	mdelay(g_debug_delay);
#endif

	/* Set high voltage chargepump for the strong CPH mode. */
	es9218_write_reg(g_es9218_priv->i2c_client,
			 ES9218P_ANALOG_CTRL_OVERRIDE,
			 register_45_value |= 0x14);
#if ES9218P_DEBUG == 1
	pr_info("%s(): R45 = %X \n", __func__, register_45_value);
	mdelay(g_debug_delay);
#endif

	/* Set high voltage chargepump for the strong CPL mode. */
	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG47_RESERVED,
			 register_47_value |= 0x10);
#if ES9218P_DEBUG == 1
	pr_info("%s(): R47 = %X \n", __func__, register_47_value);
	mdelay(g_debug_delay);
#endif

	/*
	 * This is *required* to allow the Vref (APDB) voltage to settle before
	 * enabling the AVDD_DAC regulator (AREG_PDB).
	 */
	mdelay(5);

	/* Enable internal AVCC_DAC regulator. */
	es9218_write_reg(g_es9218_priv->i2c_client,
			 ES9218P_ANALOG_CTRL_OVERRIDE,
			 register_45_value |= 0x60);
#if ES9218P_DEBUG == 1
	pr_info("%s(): R45 = %X \n", __func__, register_45_value);
	mdelay(g_debug_delay);
#endif

	/* Disengage the amplifier input shunt. */
	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG46_RESERVED,
			 register_46_value |= 0x01);
#if ES9218P_DEBUG == 1
	pr_info("%s(): R46 = %X \n", __func__, register_46_value);
	mdelay(g_debug_delay);
#endif

	/* Enable the amplifier output stage. */
	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG48_RESERVED,
			 register_48_value |= 0x40);
#if ES9218P_DEBUG == 1
	pr_info("%s(): R48 = %X \n", __func__, register_48_value);
	mdelay(g_debug_delay);
#endif

	if (mode == 2) {
		/*
		 * Engage the input shunt.
		 *
		 * Note: This block is exclusive to the HiFi2 Mode.
		 */
		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_REG46_RESERVED,
				 register_46_value &= ~0x01);
#if ES9218P_DEBUG == 1
		pr_info("%s(): R46 = %X \n", __func__, register_46_value);
		mdelay(g_debug_delay);
#endif

		/*
		 * Set the amplifier power switch.
		 *
		 * Note: This block is exclusive to the HiFi2 Mode.
		 */
		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_REG47_RESERVED,
				 register_47_value |= 0x05);
#if ES9218P_DEBUG == 1
		pr_info("%s(): R46 = %X \n", __func__, register_47_value);
		mdelay(g_debug_delay);
#endif

		/*
		 * Disengage the input shunt.
		 *
		 * Note: This block is exclusive to the HiFi2 Mode.
		 */
		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_REG46_RESERVED,
				 register_46_value |= 0x01);
#if ES9218P_DEBUG == 1
		pr_info("%s(): R46 = %X \n", __func__, register_46_value);
		mdelay(g_debug_delay);
#endif

		/*
		 * Change the power switch to the strong mode.
		 *
		 * Note: This block is exclusive to the HiFi2 Mode.
		 */
		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_REG47_RESERVED,
				 register_47_value |= 0x02);
#if ES9218P_DEBUG == 1
		pr_info("%s(): R46 = %X \n", __func__, register_47_value);
		mdelay(g_debug_delay);
#endif
	}

	value = avc_vol_tbl[g_avc_volume];
	pr_info("%s(): AVC Volume = %X \n", __func__, value);

	/*
	 * Set ATC to the original level.
	 *
	 * Note: Register 3 is also programmed in bypass2hifi, so beware of
	 * conflicts.
	 */
	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_ANALOG_VOL_CTRL,
			 value);
#if ES9218P_DEBUG == 1
	pr_info("%s(): R03 = %X \n", __func__, value);
	mdelay(g_debug_delay);
#endif

	/*
	 * Disable overrides.
	 *
	 * Note: Register 32 will take over control and hold SABRE in HiFi1
	 * Mode, while the amp input shunt and output shunt are both disengaged
	 * in HiFi2 Mode.
	 */
	if (g_es9218_priv->es9218_data->use_internal_ldo) {
		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_REG46_RESERVED,
				 register_46_value = 0x03);
	} else {
		register_46_value = 0x03;
		/* Set the SEL1V bit, and use the external LDO. */
		register_46_value |= 0x04;
		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_REG46_RESERVED, register_46_value);
	}
#if ES9218P_DEBUG == 1
	pr_info("%s(): R46 = %X \n", __func__, register_46_value);
	mdelay(g_debug_delay);
#endif

	/*
	 * SABRE is now in HiFi{1,2} Mode.
	 *
	 * Note: Override bits are not cleared but they are disabled.
	 */

	return 0;
}

/**
 * es9218p_sabre_standby2lpb() - Switch from standby to LPB Mode.
 *
 * This is the ESS recommended sequence for transitioning from Standby to Low
 * Power Bypass (LPB).
 */
static int es9218p_sabre_standby2lpb(void)
{
	pr_info("%s(): entry: state = %s\n", __func__,
		power_state[es9218_power_state]);

	/* Set GPIO2 to HIGH to switch to LPB Mode. */
	es9218_hph_switch_gpio_H();

	return 0;
}

static int es9218p_sabre_lpb2standby(void)
{
	/* Set GPIO2 to LOW to switch to Standby. */
	es9218_hph_switch_gpio_L();

	/*
	 * Allow time for chargepumps to completely discharge before power
	 * off.
	 */
	mdelay(3);

	return 0;
}

static int es9218p_sabre_hifi2lofi2lpb(void)
{
#if WORKAROUND_FOR_LOFI_POP == 1
	uint8_t rv = es9218_read_reg(g_es9218_priv->i2c_client,
				     ES9218P_FILTER_BAND_SYSTEM_MUTE);
#endif

	pr_info("%s()\n", __func__);

#if WORKAROUND_FOR_LOFI_POP == 1
	if (rv & 1)
		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_FILTER_BAND_SYSTEM_MUTE, rv |= 0x01);
#endif

	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_AMP_CONFIG, 0x00);

	/* SABRE is now in LoFi Mode. */

	if (g_dop_flag) {
		pr_debug(
			"%s: add 300ms delay before reset gpio set for dop playback case.\n",
			__func__);
		msleep(300);
	}

	es9218_reset_gpio_L();

	/* SABRE is now in Low Power Bypass Mode. */
	return 0;
}

static int es9218p_sabre_bypass2hifi(void)
{
	uint8_t reg;
	pr_info("%s() : enter. state = %s\n", __func__,
		power_state[es9218_power_state]);

	if (es9218_power_state != ESS_PS_BYPASS) {
		pr_err("%s() : invalid state = %s\n", __func__,
		       power_state[es9218_power_state]);
		return 0;
	}

#if ES9218P_DEBUG == 1
	es9218_reset_gpio_H();
	mdelay(1);

	/*
	 * Set RESETB to twice to improve the automatic power-on sequence and
	 * prevent I2C faliure.
	 */
	es9218_reset_gpio_L();
	mdelay(1);
	es9218_reset_gpio_H();
	mdelay(2);

	/*
	 * Send an I2C soft reset cimmand to improve the automatic power-on
	 * sequence and correctly read the default values of registers.
	 */
	i2c_smbus_write_byte_data(g_es9218_priv->i2c_client, ES9218P_SYSTEM_REG,
				  0x01);
	mdelay(1);
#else
	es9218_reset_gpio_H();
	mdelay(2);
#endif

	/* SABRE is now in LoFi Mode. */

	if (call_common_init_registers == 1) {
		call_common_init_registers = 0;

		es9218p_initialize_registers(ESS_MODE_INIT);
		pr_info("%s(): call es9218_common_init_register.\n", __func__);
	}

	if (g_dop_flag == 0) {
		pr_info("%s(): PCM Format Reg Initial in es9218p_sabre_bypass2hifi() \n",
			__func__);

		/* Initialise registers for PCM playback. */
		es9218p_initialize_registers(ESS_MODE_PCM);

		/* Set the bit-width to an ESS-advised value. */
		es9218_set_bit_width(es9218_bps, ESS_MODE_PCM);
	} else if (g_dop_flag > 0) {
		pr_info("%s(): DOP Format Reg Initial in es9218p_sabre_bypass2hifi() \n",
			__func__);

#if ENABLE_DOP_SOFT_MUTE == 1
		reg = es9218_read_reg(g_es9218_priv->i2c_client,
				      ES9218P_FILTER_BAND_SYSTEM_MUTE);
		reg |= 0x01;
		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_FILTER_BAND_SYSTEM_MUTE, reg);
		mdelay(2);
#endif

		/* Initialise registers for DoP playback. */
		es9218p_initialize_registers(ESS_MODE_DoP);

		/* Set the bit-width to an ESS-advised value. */
		es9218_set_bit_width(g_dop_flag, ESS_MODE_DoP);
	}

#if ES9218P_SYSFS == 1
	if (forced_headset_type != -1 && forced_headset_type != g_headset_type)
		g_headset_type = forced_headset_type;
#endif

	es9218_set_thd(g_es9218_priv->i2c_client, g_headset_type);
#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
	es9218_sabre_cfg_custom_filter(&sabre_custom_ft[g_sabre_cf_num]);
#endif

	/* Set left and right channel digital volume levels. */
	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_VOL1_CTRL,
			 g_left_volume);
	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_VOL2_CTRL,
			 g_right_volume);

	pr_info("%s() : g_left_volume = %d, g_right_volume = %d \n", __func__,
		g_left_volume, g_right_volume);

	/* Set master trim level. */
	es9218_master_trim(g_es9218_priv->i2c_client, g_volume);

#if ES9218P_SYSFS == 1
	if (forced_avc_volume != -1 && forced_avc_volume != g_avc_volume)
		g_avc_volume = forced_avc_volume;
#endif

	/* Set Analog Volume Control prior to starting the amp. */
	es9218_set_avc_volume(g_es9218_priv->i2c_client, g_avc_volume);
	/* Switch to HiFi Mode. */
	es9218p_sabre_amp_start(g_es9218_priv->i2c_client, g_headset_type);

	es9218_power_state = ESS_PS_HIFI;

	pr_info("%s() : exit. state = %s\n", __func__,
		power_state[es9218_power_state]);

	return 0;
}

static int es9218p_sabre_hifi2lpb(void)
{
	if (es9218_power_state < ESS_PS_HIFI) {
		pr_err("%s() : invalid state = %s\n", __func__,
		       power_state[es9218_power_state]);

		return 0;
	}
	pr_info("%s() : state = %s\n", __func__,
		power_state[es9218_power_state]);

	es9218p_sabre_amp_stop(g_es9218_priv->i2c_client, g_headset_type);

	es9218_power_state = ESS_PS_BYPASS;

	return 0;
}

/**
 * es9218_sabre_audio_idle() - Control idle behaviour during HiFi Mode.
 *
 * This occurs when playback is stopped or paused.
 */
static int es9218_sabre_audio_idle(void)
{
	if (es9218_power_state != ESS_PS_HIFI) {
		pr_err("%s() : invalid state = %s\n", __func__,
		       power_state[es9218_power_state]);

		return 0;
	}
	pr_info("%s() : state = %s\n", __func__,
		power_state[es9218_power_state]);

#if 0
	es9218_write_reg(g_es9218_priv->i2c_client, ESS9218_02, 0x34);
#endif

	es9218_power_state = ESS_PS_IDLE;

	return 0;
}

static int es9218_sabre_audio_active(void)
{
	if (es9218_power_state != ESS_PS_IDLE) {
		pr_err("%s() : invalid state = %s\n", __func__,
		       power_state[es9218_power_state]);

		return 0;
	}
	pr_info("%s() : state = %s\n", __func__,
		power_state[es9218_power_state]);

	es9218_power_state = ESS_PS_HIFI;

	return 0;
}

/**
 * __es9218_sabre_headphone_on() - Power on and switch to Bypass Mode when
 *                                 headphones are plugged in.
 *
 * SABRE is still powered down but the switch is on and the SW is positioned
 * at AUX or QC_CODEC. Thus, the I2C link will not function as HiFi_RESET_N
 * is still 0.
 */
static int __es9218_sabre_headphone_on(void)
{
	pr_info("%s(): entry: state = %s\n", __func__,
		power_state[es9218_power_state]);

	/* If SABRE has been powered off, power it back on. */
	if (es9218_power_state == ESS_PS_CLOSE) {
		/* Set GPIO2 to LOW for a quiet power on. */
		es9218_hph_switch_gpio_L();
		/* Set RESETb to LOW for a quiet power on. */
		es9218_reset_gpio_L();
		/* Power on GPIO2. */
		es9218_power_gpio_H();

		/* SABRE is now in Standby Mode. */

		es9218p_sabre_standby2lpb();

		es9218_power_state = ESS_PS_BYPASS;

		return 0;
	} else if (es9218_power_state == ESS_PS_BYPASS && es9218_is_amp_on) {
		/*
		 * If SABRE is already powered on and waiting in LPB Mode, transition
		 * from LPB to HiFi depending on the load detected.
		 */
		pr_info("%s() : state = %s , is_amp_on = %d \n", __func__,
			power_state[es9218_power_state], es9218_is_amp_on);

		/*
		 * We call the common init registers for the side case that
		 * es9218_startup() has not been called. This is because both HiFi Mode
		 * and MP3 offloading both spend time in Standby Mode (1s and 10s,
		 * respectively) and then invoke es9218_startup().
		 */
		call_common_init_registers = 1;

		cancel_delayed_work_sync(&g_es9218_priv->sleep_work);

		if (pm_qos_request_active(&req)) {
			pr_info("%s(): pm qos active state. so, remove pm qos request",
				__func__);
			pm_qos_remove_request(&req);
		}

		/* Guarantee enough time to check headphone impedance before we switch to HiFi Mode. */
		schedule_delayed_work(&g_es9218_priv->hifi_in_standby_work,
				      msecs_to_jiffies(250));
		pr_info("%s(): end calling es9218p_sabre_bypass2hifi() after 250ms \n",
			__func__);

		return 1;
	} else {
		pr_err("%s() : state = %s , skip enabling EDO.\n", __func__,
		       power_state[es9218_power_state]);

		return 0;
	}

	pr_info("%s(): end \n", __func__);

	return 0;
}

/**
 * __es9218_sabre_headphone_off() - Power down when the headphone is plugged out.
 *
 * This state is the same as system power up state.
 */
static int __es9218_sabre_headphone_off(void)
{
	if (es9218_power_state == ESS_PS_CLOSE) {
		pr_err("%s() : invalid state = %s\n", __func__,
		       power_state[es9218_power_state]);

		return 0;
	}

	cancel_delayed_work_sync(&g_es9218_priv->hifi_in_standby_work);

	/*
	 * If the power state indicates that the chip is in HiFi Mode, move to LPB
	 * Mode.
	 */
	if (es9218_power_state != ESS_PS_BYPASS ||
	    es9218_power_state != ESS_PS_IDLE) {
		es9218p_sabre_hifi2lpb();
	}
	pr_info("%s() : state = %d\n", __func__, es9218_power_state);

	es9218p_sabre_lpb2standby();

	/* Power off when the ESS ship shuts down. */
	if (g_es9218_priv->es9218_data->always_power_on == false)
		es9218_power_gpio_L();

	/* Set the reset pin to Low if the reset gpio is set to high. */
	if (__gpio_get_value(g_es9218_priv->es9218_data->reset_gpio) == 1)
		es9218_reset_gpio_L();

	es9218_power_state = ESS_PS_CLOSE;

	return 0;
}

/**
 * es9218_sabre_headphone_on() - Power on and switch to Bypass Mode when
 *                               headphones are plugged in.
 *
 * SABRE is still powered down but the switch is on and the SW is positioned
 * at AUX or QC_CODEC. Thus, the I2C link will not function as HiFi_RESET_N
 * is still 0.
 */
int es9218_sabre_headphone_on(void)
{
	pr_info("%s() Called !! \n", __func__);

	mutex_lock(&g_es9218_priv->power_lock);
	__es9218_sabre_headphone_on();
	mutex_unlock(&g_es9218_priv->power_lock);

	return 0;
}
EXPORT_SYMBOL(es9218_sabre_headphone_on);

/**
 * es9218_sabre_headphone_off() - Power down when the headphone is plugged out.
 *
 * This state is the same as system power up state.
 */
int es9218_sabre_headphone_off(void)
{
	pr_info("%s() Called !! \n", __func__);

	mutex_lock(&g_es9218_priv->power_lock);
	__es9218_sabre_headphone_off();
	mutex_unlock(&g_es9218_priv->power_lock);

	return 0;
}
EXPORT_SYMBOL(es9218_sabre_headphone_off);

static int es9218_set_volume_rate(unsigned int sample_rate,
				  unsigned int ess_mode)
{
	int ret = -1;

	/*
	 * As per the sample rate, set the volume rate. We also need to set the
	 * soft time start.
	 */
	if (ess_mode == ESS_MODE_PCM) {
		switch (sample_rate) {
		case 44100:
		case 48000:
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_DOP_VOL_RAMP_RATE, 0x44);
			break;

		default:
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_DOP_VOL_RAMP_RATE, 0x43);
			break;
		}
	}

	pr_info("%s() exit - setting volume rate for %d Hz is %s\n", __func__,
		sample_rate, !ret ? "done" : "failed");

	return ret;
}
static int es9218p_set_sample_rate(unsigned int bit_width,
				   unsigned int sample_rate,
				   unsigned int ess_mode)
{
	int ret = -1;
	uint8_t reg = 0;

	if (ess_mode == ESS_MODE_PCM) {
		reg = es9218_read_reg(g_es9218_priv->i2c_client,
				      ES9218P_FILTER_BAND_SYSTEM_MUTE);

		if ((g_sabre_cf_num == 4 || g_sabre_cf_num == 5) &&
		    (reg & 0x80)) {
			pr_info("%s set custom filter when g_sabre_cf_num is %d\n",
				__func__, g_sabre_cf_num);
			es9218_sabre_cfg_custom_filter(
				&sabre_custom_ft[g_sabre_cf_num]);
		}

		switch (sample_rate) {
		case 352800:
		case 384000:
			reg = es9218_read_reg(g_es9218_priv->i2c_client,
					      ES9218P_FILTER_BAND_SYSTEM_MUTE);

			if (g_sabre_cf_num == 8 || g_sabre_cf_num == 9) {
				reg &= ~0xE0;
				reg |= 0x48;
			} else {
				reg |= 0x8;
			}

			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_FILTER_BAND_SYSTEM_MUTE,
					       reg);
			pr_info("%s register_07:%x, sample_rate:%d\n", __func__,
				reg, sample_rate);

			reg = es9218_read_reg(g_es9218_priv->i2c_client,
					      ES9218P_FIR_CONFIG);
			reg |= 0x01;
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_FIR_CONFIG, reg);
			pr_info("%s register_44:%x\n", __func__, reg);
			break;

		default:
			reg = es9218_read_reg(g_es9218_priv->i2c_client,
					      ES9218P_FILTER_BAND_SYSTEM_MUTE);

			if (g_sabre_cf_num == 8 || g_sabre_cf_num == 9) {
				reg &= ~0xE8;
				reg |= 0x40;
			} else {
				reg &= ~0x8;
			}

			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_FILTER_BAND_SYSTEM_MUTE,
					       reg);
			pr_info("%s register_07:%x, sample_rate:%d\n", __func__,
				reg, sample_rate);

			reg = es9218_read_reg(g_es9218_priv->i2c_client,
					      ES9218P_FIR_CONFIG);
			reg &= ~0x01;
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_FIR_CONFIG, reg);
			pr_info("%s register_44:%x\n", __func__, reg);
			break;
		}
	}

	return ret;
}

static int es9218_set_bit_width(unsigned int bit_width, unsigned int ess_mode)
{
	int ret = -1;
	uint8_t i2c_len_reg = 0;
	uint8_t in_cfg_reg = 0;

	if (ess_mode == ESS_MODE_PCM) {
		switch (bit_width) {
		case 16:
			i2c_len_reg = 0x0;
			in_cfg_reg |= i2c_len_reg;
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_INPUT_SELECT,
					       in_cfg_reg);
			break;

		case 24:
		case 32:
		default:
			i2c_len_reg = 0x80;
			in_cfg_reg |= i2c_len_reg;
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_INPUT_SELECT,
					       in_cfg_reg);
			break;
		}
	} else if (ess_mode == ESS_MODE_DoP) {
		switch (bit_width) {
		case 64:
#if ENABLE_DOP_AUTO_MUTE == 1
			/*
			 * Set the automute timings to 170ms, and 34ms
			 * respectively.
			 */
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_AUTOMUTE_TIME, 0x05);
			/* Adjust the volume rate for automute. */
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_DOP_VOL_RAMP_RATE, 0x4E);
			/* Enable automute. */
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_AUTOMUTE_CONFIG, 0xF4);
#endif
			/* Set Registry 0 to the default HW value. */
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_SYSTEM_REG, 0x04);
			break;

		case 128:
		default:
#if ENABLE_DOP_AUTO_MUTE == 1
			/*
			 * Set the automute timings to 170ms, and 34ms
			 * respectively.
			 */
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_AUTOMUTE_TIME, 0x05);
			/* Adjust the volume rate for automute. */
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_DOP_VOL_RAMP_RATE, 0x4E);
			/* Enable automute. */
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_AUTOMUTE_CONFIG, 0xF4);
#endif
			/* Set Registry 0 to the default HW value. */
			ret = es9218_write_reg(g_es9218_priv->i2c_client,
					       ES9218P_SYSTEM_REG, 0x00);
			break;
		}
	}

	pr_info("%s() exit - setting %d bits width is %s\n", __func__,
		bit_width, !ret ? "done" : "failed");

	return ret;
}

static void es9218p_initialize_registers(unsigned int ess_mode)
{
	int i;

	if (ess_mode == ESS_MODE_PCM) {
		for (i = 0; i < sizeof(es9218_pcm_init_register) /
					sizeof(es9218_pcm_init_register[0]);
		     i++) {
			es9218_write_reg(g_es9218_priv->i2c_client,
					 es9218_pcm_init_register[i].num,
					 es9218_pcm_init_register[i].value);
		}
	} else if (ess_mode == ESS_MODE_DoP) {
		for (i = 0; i < sizeof(es9218_dop_init_register) /
					sizeof(es9218_dop_init_register[0]);
		     i++) {
			es9218_write_reg(g_es9218_priv->i2c_client,
					 es9218_dop_init_register[i].num,
					 es9218_dop_init_register[i].value);
		}
	} else if (ess_mode == ESS_MODE_INIT) {
		for (i = 0; i < sizeof(es9218_common_init_register) /
					sizeof(es9218_common_init_register[0]);
		     i++) {
			es9218_write_reg(g_es9218_priv->i2c_client,
					 es9218_common_init_register[i].num,
					 es9218_common_init_register[i].value);
		}
	}
}

static void es9218_sabre_hifi_in_standby_work(struct work_struct *work)
{
	mutex_lock(&g_es9218_priv->power_lock);

	pr_info("%s() enter - go to hifi mode from standy mode status:%s, bps : %d , rate : %d\n",
		__func__, power_state[es9218_power_state], es9218_bps,
		es9218_rate);

	es9218p_sabre_bypass2hifi();

	if (g_dop_flag == 0) { // PCM
		/* Set the bit-width to an ESS-advised value for PCM playback. */
		es9218_set_bit_width(es9218_bps, ESS_MODE_PCM);
		es9218p_set_sample_rate(es9218_bps, es9218_rate, ESS_MODE_PCM);
		/* Set the volume rate to an ESS-advised value. */
		es9218_set_volume_rate(es9218_rate, ESS_MODE_PCM);
	} else if (g_dop_flag > 0) {
		/* Set the bit-width to an ESS-advised value for DoP playback. */
		es9218_set_bit_width(g_dop_flag, ESS_MODE_DoP);

		/*
		 * DoP has teardown mode in AudioFWK, so shutdown() is already invoked.
		 * Therefore, we forced switch to Idle Mode when for DoP playback.
		 */
		es9218_sabre_audio_idle();
		pr_info("%s() force change a mode status:%s\n", __func__,
			power_state[es9218_power_state]);
	}

	pr_info("%s() exit - go to hifi mode from standy mode status:%s\n",
		__func__, power_state[es9218_power_state]);

	mutex_unlock(&g_es9218_priv->power_lock);

	return;
}

static void es9218_sabre_sleep_work(struct work_struct *work)
{
	__pm_wakeup_event(&wl_sleep, jiffies_to_msecs(2000));

	mutex_lock(&g_es9218_priv->power_lock);

	if (es9218_power_state == ESS_PS_IDLE) {
		pr_info("%s(): sleep_work state is %s running \n", __func__,
			power_state[es9218_power_state]);

		es9218p_sabre_hifi2lpb();
	} else {
		pr_info("%s(): sleep_work state is %s skip operation \n",
			__func__, power_state[es9218_power_state]);
	}

	es9218_is_amp_on = 0;

	if (pm_qos_request_active(&req)) {
		pr_info("%s(): pm qos active state. so, remove pm qos request",
			__func__);
		pm_qos_remove_request(&req);
	}

	mutex_unlock(&g_es9218_priv->power_lock);

	return;
}

static int es9218_power_state_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s(): power state = %d\n", __func__, es9218_power_state);

	ucontrol->value.enumerated.item[0] = es9218_power_state;
	pr_debug("%s(): ucontrol = %d\n", __func__,
		 ucontrol->value.enumerated.item[0]);

	return 0;
}

static int es9218_power_state_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;

	pr_debug("%s():ucontrol = %d, power state=%d\n", __func__,
		 ucontrol->value.enumerated.item[0], es9218_power_state);

	if (es9218_power_state == ucontrol->value.enumerated.item[0]) {
		pr_info("%s():no power state change\n", __func__);
	}

	switch (ucontrol->value.enumerated.item[0]) {
	/* Open */
	case 0:
		__es9218_sabre_headphone_on();
		break;
	/* Close */
	case 1:
		__es9218_sabre_headphone_off();
		break;
	/* Bypass */
	case 2:
		es9218p_sabre_hifi2lpb();
		break;
	/* HiFi */
	case 3:
		es9218p_sabre_bypass2hifi();
		break;
	/* Idle */
	case 4:
		es9218_sabre_audio_idle();
		break;
	/* Active */
	case 5:
		es9218_sabre_audio_active();
		break;
	/* High Power */
	case 6:
		es9218_reset_gpio_H();
		break;
	/* Low Power */
	case 7:
		es9218_reset_gpio_L();
		break;
	/* HPH High */
	case 8:
		es9218_hph_switch_gpio_H();
		break;
	/* HPH Low */
	case 9:
		es9218_hph_switch_gpio_L();
		break;
	default:
		break;
	}

	return ret;
}

static int
es9218_normal_harmonic_comp_put_left(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	int i;

	for (i = 0; i < 4; i++) {
		normal_harmonic_comp_left[i] =
			(uint8_t)ucontrol->value.integer.value[i];
	}

	return 0;
}
static int
es9218_advance_harmonic_comp_put_left(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int i;

	for (i = 0; i < 4; i++) {
		advance_harmonic_comp_left[i] =
			(uint8_t)ucontrol->value.integer.value[i];
	}

	return 0;
}
static int
es9218_aux_harmonic_comp_put_left(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	int i;

	for (i = 0; i < 4; i++) {
		aux_harmonic_comp_left[i] =
			(uint8_t)ucontrol->value.integer.value[i];
	}

	return 0;
}
static int
es9218_normal_harmonic_comp_put_right(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int i;

	for (i = 0; i < 4; i++) {
		normal_harmonic_comp_right[i] =
			(uint8_t)ucontrol->value.integer.value[i];
	}

	return 0;
}
static int
es9218_advance_harmonic_comp_put_right(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int i;

	for (i = 0; i < 4; i++) {
		advance_harmonic_comp_right[i] =
			(uint8_t)ucontrol->value.integer.value[i];
	}

	return 0;
}
static int
es9218_aux_harmonic_comp_put_right(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int i;

	for (i = 0; i < 4; i++) {
		aux_harmonic_comp_right[i] =
			(uint8_t)ucontrol->value.integer.value[i];
	}

	return 0;
}

static int es9218_headset_type_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_headset_type;

	pr_info("%s(): type = %d \n", __func__, g_headset_type);

	return 0;
}

static int es9218_headset_type_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int value = (int)ucontrol->value.integer.value[0];

	if (value != 0) {
#if ES9218P_SYSFS == 1
		if (forced_headset_type != -1 &&
		    forced_headset_type != g_headset_type)
			g_headset_type = forced_headset_type;
		else
#else
		if (1)
#endif
			g_headset_type = value;
		pr_info("%s(): type = %d, state = %s\n ", __func__, value,
			power_state[es9218_power_state]);
	} else {
		/*
		 * The value of the mixer control "Es9018 HEADSET TYPE" is defined by
		 * the following modes:
		 *
		 * 0 -> No headphone or device has been plugged in.
		 * 1 -> A "normal" (<=50RZ) headphone has been plugged in.
		 * 2 -> A "high impedance" (50RZ-600RZ) headphone has been plugged in.
		 * 3 -> An "AUX" device (>600RZ) has been plugged in.
		 */
		pr_err("%s() : invalid headset type = %d, state = %s\n",
		       __func__, value, power_state[es9218_power_state]);

		return 0;
	}

	if (es9218_power_state < ESS_PS_HIFI) {
		pr_debug("%s() : invalid state = %s\n", __func__,
			 power_state[es9218_power_state]);

		return 0;
	}

	es9218_set_thd(g_es9218_priv->i2c_client, g_headset_type);

	return 0;
}

static int es9218_auto_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_auto_mute_flag;
	pr_debug("%s(): type = %d \n", __func__, g_auto_mute_flag);

	return 0;
}

static int es9218_auto_mute_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;

	g_auto_mute_flag = (int)ucontrol->value.integer.value[0];

	pr_debug("%s(): g_auto_mute_flag = %d \n", __func__, g_auto_mute_flag);

	if (es9218_power_state < ESS_PS_HIFI) {
		pr_err("%s() : return = %s\n", __func__,
		       power_state[es9218_power_state]);

		return ret;
	}

	if (g_auto_mute_flag) {
		pr_debug("%s(): Disable g_auto_mute_flag = %d \n", __func__,
			 g_auto_mute_flag);
#if 0
		es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_AUTOMUTE_CONFIG,0x34);
#endif
	} else {
		/* Set the Bits Per Second. */
		switch (es9218_bps) {
		case 24:
		case 32:
			switch (es9218_rate) {
			case 96000:
			case 192000:
				/* Enable automute. */
				pr_debug(
					"%s(): Enable g_auto_mute_flag = %d \n",
					__func__, g_auto_mute_flag);
#if 0
				es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_AUTOMUTE_CONFIG,0xF4);
#endif
				break;
			}
		}
	}

	return ret;
}

#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
static int
lge_ess_digital_filter_setting_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_sabre_cf_num;
	pr_info("%s(): ucontrol = %d\n", __func__, g_sabre_cf_num);

	return 0;
}

static int
lge_ess_digital_filter_setting_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	g_sabre_cf_num = (int)ucontrol->value.integer.value[0];
	pr_info("%s():filter num= %d\n", __func__, g_sabre_cf_num);

	return 0;
}

static int lge_ess_get_fade_count(void)
{
	if (fade_count_debug_param == 99)
		return FADE_INOUT_COUNT;
	else
		return fade_count_debug_param;
}

static int lge_ess_get_fade_term(void)
{
	if (fade_term_debug_param == 99)
		return FADE_INOUT_TERM;
	else
		return fade_term_debug_param;
}

static void mute_work_function(struct work_struct *work)
{
	int result = cancel_delayed_work(mute_work);
	pr_info("%s(): called volume direction %d, g_fade_count %d, cancle_delayed_work result %d , g_left_fade_vol %d, g_right_fade_vol %d ",
		__func__, fade_direction, g_fade_count, result, g_left_fade_vol,
		g_right_fade_vol);

	if (fade_direction == VOLUME_DOWN) {
		if (g_fade_count != 0) {
			g_fade_count--;
			pr_debug(
				"%s(): case 1 called volume direction %d, g_fade_count %d\n",
				__func__, fade_direction, g_fade_count);

			g_left_fade_vol =
				g_left_fade_vol + g_left_fade_vol_per_step;
			g_right_fade_vol =
				g_right_fade_vol + g_right_fade_vol_per_step;

			if (g_left_fade_vol > 0xff)
				g_left_fade_vol = 0xff;
			if (g_right_fade_vol > 0xff)
				g_right_fade_vol = 0xff;

			es9218_write_reg(g_es9218_priv->i2c_client,
					 ES9218P_VOL1_CTRL, g_left_fade_vol);
			es9218_write_reg(g_es9218_priv->i2c_client,
					 ES9218P_VOL2_CTRL, g_right_fade_vol);
		} else {
			pr_debug("%s(): case 2 called volume g_fade_count %d\n",
				 __func__, g_fade_count);

			if (g_sabre_cf_num == SHORT_FILTER)
				g_volume = 0;
			else if (g_sabre_cf_num == SHARP_FILTER)
				g_volume = 4;
			else
				g_volume = 2;

			es9218_master_trim(g_es9218_priv->i2c_client, g_volume);
			fade_direction = VOLUME_UP;
		}

		queue_delayed_work(mute_workqueue, mute_work,
				   msecs_to_jiffies(lge_ess_get_fade_term()));
	} else {
		if (g_fade_count < (lge_ess_get_fade_count())) {
			g_fade_count++;
			pr_debug(
				"%s(): case 3 called volume g_fade_count %d g_left_fade_vol %d\n",
				__func__, g_fade_count, g_left_fade_vol);

			g_left_fade_vol =
				g_left_fade_vol - g_left_fade_vol_per_step;
			g_right_fade_vol =
				g_right_fade_vol - g_right_fade_vol_per_step;

			if (g_left_fade_vol < g_left_volume)
				g_left_fade_vol = g_left_volume;
			if (g_right_fade_vol < g_right_volume)
				g_right_fade_vol = g_right_volume;

			es9218_write_reg(g_es9218_priv->i2c_client,
					 ES9218P_VOL1_CTRL, g_left_fade_vol);
			es9218_write_reg(g_es9218_priv->i2c_client,
					 ES9218P_VOL2_CTRL, g_right_fade_vol);

			queue_delayed_work(
				mute_workqueue, mute_work,
				msecs_to_jiffies(lge_ess_get_fade_term()));
		} else {
			es9218_write_reg(g_es9218_priv->i2c_client,
					 ES9218P_VOL1_CTRL, g_left_volume);
			es9218_write_reg(g_es9218_priv->i2c_client,
					 ES9218P_VOL2_CTRL, g_right_volume);

			pr_debug(
				"%s(): case 4 called volume direction %d, g_fade_count %d\n",
				__func__, fade_direction, g_fade_count);
		}
	}

	return;
}

static int lge_ess_fade_inout_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	if (es9218_power_state < ESS_PS_HIFI) {
		pr_err("%s() : invalid state = %s\n", __func__,
		       power_state[es9218_power_state]);

		return 0;
	}
	if (lge_ess_fade_inout_init != true) {
		lge_ess_fade_inout_init = true;
		pr_info("%s(): fade in out work queue initialize. \n",
			__func__);

		mute_workqueue = create_workqueue("mute_workqueue");
		mute_work =
			devm_kzalloc(&g_es9218_priv->i2c_client->dev,
				     sizeof(struct delayed_work), GFP_KERNEL);

		if (!mute_work) {
			lge_ess_fade_inout_init = false;
			pr_err("%s() : devm_kzalloc failed!!\n", __func__);

			return 0;
		}

		INIT_DELAYED_WORK(mute_work, mute_work_function);
	}

	if (work_pending(&mute_work->work)) {
		pr_info("%s(): previous fade in/out is not yet finished. \n",
			__func__);

		g_fade_count = lge_ess_get_fade_count();
		fade_direction = VOLUME_DOWN;
	} else {
		g_fade_count = lge_ess_get_fade_count();
		g_left_fade_vol = g_left_volume;
		g_right_fade_vol = g_right_volume;
		g_left_fade_vol_per_step =
			(0xff - g_left_volume) / g_fade_count;
		g_right_fade_vol_per_step =
			(0xff - g_right_volume) / g_fade_count;
		fade_direction = VOLUME_DOWN;

		pr_info("%s(): start fade in/out g_left_volume %d, g_right_volume %d \n",
			__func__, g_left_volume, g_right_volume);
	}

	queue_delayed_work(mute_workqueue, mute_work,
			   msecs_to_jiffies(lge_ess_get_fade_term()));

	return 0;
}

static int lge_ess_fade_inout_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}
#endif

static int es9218_avc_volume_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_avc_volume;
	pr_debug("%s(): AVC Volume= -%d db\n", __func__, g_avc_volume);

	return 0;
}

static int es9218_avc_volume_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	int vol = 25;

	/* The range of g_avc_volume is from 0 to 24. */
	vol = (int)ucontrol->value.integer.value[0];

	if (vol >= sizeof(avc_vol_tbl) / sizeof(avc_vol_tbl[0])) {
		pr_err("%s() : Invalid vol = %d return \n", __func__, vol);
		return 0;
	}

	g_avc_volume = vol;

	pr_debug("%s(): AVC Volume= -%d db  state = %s\n", __func__,
		 g_avc_volume, power_state[es9218_power_state]);

	if (es9218_power_state < ESS_PS_HIFI) {
		pr_debug("%s() : invalid state = %s\n", __func__,
			 power_state[es9218_power_state]);

		return 0;
	}

#if ES9218P_SYSFS == 1
	if (forced_avc_volume != -1 && forced_avc_volume != g_avc_volume)
		g_avc_volume = forced_avc_volume;
#endif
	es9218_set_avc_volume(g_es9218_priv->i2c_client, g_avc_volume);

	return ret;
}

static int es9218_master_volume_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_volume;
	pr_debug("%s(): Master Volume= -%d db\n", __func__, g_volume / 2);

	return 0;
}

static int es9218_master_volume_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;

	g_volume = (int)ucontrol->value.integer.value[0];
	pr_debug("%s(): Master Volume= -%d db\n", __func__, g_volume / 2);

	if (es9218_power_state < ESS_PS_HIFI) {
		pr_err("%s() : invalid state = %s\n", __func__,
		       power_state[es9218_power_state]);

		return ret;
	}

	es9218_master_trim(g_es9218_priv->i2c_client, g_volume);

	return ret;
}

static int es9218_left_volume_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_left_volume;
	pr_debug("%s(): Left Volume= -%d db\n", __func__, g_left_volume / 2);

	return 0;
}

static int es9218_left_volume_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;

	g_left_volume = (int)ucontrol->value.integer.value[0];
	pr_debug("%s(): Left Volume= -%d db\n", __func__, g_left_volume / 2);

	if (es9218_power_state < ESS_PS_HIFI) {
		pr_err("%s() : invalid state = %s\n", __func__,
		       power_state[es9218_power_state]);

		return ret;
	}

	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_VOL1_CTRL,
			 g_left_volume);

	return ret;
}

static int es9218_right_volume_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_right_volume;
	pr_debug("%s(): Right Volume= -%d db\n", __func__, g_right_volume / 2);

	return 0;
}

static int es9218_right_volume_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;

	g_right_volume = (int)ucontrol->value.integer.value[0];
	pr_debug("%s(): Right Volume= -%d db\n", __func__, g_right_volume / 2);

	if (es9218_power_state < ESS_PS_HIFI) {
		pr_err("%s() : invalid state = %s\n", __func__,
		       power_state[es9218_power_state]);

		return ret;
	}

	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_VOL2_CTRL,
			 g_right_volume);

	return ret;
}

static int es9218_filter_enum_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_sabre_cf_num;
	pr_debug("%s(): ucontrol = %d\n", __func__, g_sabre_cf_num);

	return 0;
}

static int es9218_filter_enum_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	uint8_t reg;
#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
	int new_filter = 0;

	new_filter = (int)ucontrol->value.integer.value[0];
	pr_info("%s(): g_sabre_cf_num %d, new filter num = %d \n", __func__,
		g_sabre_cf_num, new_filter);

	if (es9218_power_state < ESS_PS_HIFI) {
		pr_info("%s() : invalid state = %s\n", __func__,
			power_state[es9218_power_state]);

		return ret;
	}

#endif
	g_sabre_cf_num = (int)ucontrol->value.integer.value[0];
	pr_debug("%s():filter num= %d\n", __func__, g_sabre_cf_num);
	reg = es9218_read_reg(g_es9218_priv->i2c_client,
			      ES9218P_FILTER_BAND_SYSTEM_MUTE);
	reg |= 0x01;
	es9218_write_reg(g_es9218_priv->i2c_client,
			 ES9218P_FILTER_BAND_SYSTEM_MUTE, reg);
	mdelay(2);
	es9218_sabre_cfg_custom_filter(&sabre_custom_ft[g_sabre_cf_num]);
	reg = es9218_read_reg(g_es9218_priv->i2c_client,
			      ES9218P_FILTER_BAND_SYSTEM_MUTE);
	reg &= ~0x01;
	es9218_write_reg(g_es9218_priv->i2c_client,
			 ES9218P_FILTER_BAND_SYSTEM_MUTE, reg);
	mdelay(2);

	return ret;
}

static int es9218_dop_get(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_dop_flag;
	pr_debug("%s() %d\n", __func__, g_dop_flag);

	return 0;
}

static int mode_changed = 0;
static int es9218_dop_put(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	/* g_dop_flag checks the playback format as shown below:
	 *
	 * 0   -> PCM
	 * 64  -> DoP64 (otherwise known as DSD64)
	 * 128 -> DoP128 (otherwise known as DSD128)
	*/
	prev_dop_flag = g_dop_flag;

	g_dop_flag = (int)ucontrol->value.integer.value[0];
	pr_debug("%s() dop_enable:%d, state:%d\n", __func__, g_dop_flag,
		 es9218_power_state);

	if (!(g_dop_flag == 0 || g_dop_flag == 64 || g_dop_flag == 128)) {
		pr_err("%s() dop_enable error:%d. invalid arg.\n", __func__,
		       g_dop_flag);
	}

	if (prev_dop_flag != g_dop_flag)
		mode_changed = 1;

	return 0;
}

static int es9218_chip_state_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	int ret;
	uint8_t readChipStatus;
	unsigned char chipId = 0x00;
	int readCnt;

	pr_debug("%s(): enter, es9218_power_state=%s.\n", __func__,
		 power_state[es9218_power_state]);

	mutex_lock(&g_es9218_priv->power_lock);

	es9218_power_gpio_H();
	mdelay(1);
	es9218_reset_gpio_H();
	mdelay(1);

	/* The ESS revision check is done once during boot. */
	if (!g_ess_rev_check) {
		for (readCnt = 0; readCnt < 3; readCnt++) {
			readChipStatus = es9218_read_reg(
				g_es9218_priv->i2c_client, ES9218P_CHIPSTATUS);
			chipId = readChipStatus & 0xF0;

			pr_err("%s: chipId:0x%x readCnt : %d \n", __func__,
			       chipId, readCnt);

			if (chipId == 0xd0) {
				pr_err("%s: ESS revsion = ESS_9218p\n",
				       __func__);
				g_ess_rev_check = true;
				break;
			} else if (chipId == 0xe0) {
				pr_err("%s: ESS revsion = ESS_9228\n",
				       __func__);
				g_ess_rev_check = true;
				break;
			}
		}
	}

	ret = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_SYSTEM_REG);
	if (ret < 0) {
		pr_err("%s : i2_read fail : %d\n", __func__, ret);
		ucontrol->value.enumerated.item[0] = 0;
	} else {
		pr_notice("%s : i2_read success : %d\n", __func__, ret);
		ucontrol->value.enumerated.item[0] = 1;
	}

	if (es9218_power_state < ESS_PS_HIFI) {
		es9218_reset_gpio_L();

		/* Power off when the ESS chip shuts down. */
		if (g_es9218_priv->es9218_data->always_power_on == false) {
			if (es9218_power_state == ESS_PS_BYPASS) {
				pr_info("%s(): headphones is already inserted from booting up. call es9218p_sabre_standby2lpb() to re-enter to lpb. \n",
					__func__);
				es9218p_sabre_standby2lpb();

				/* SABRE is now in LPB Mode. */

				pr_info("%s(): re-enter to lpb by calling es9218p_sabre_standby2lpb(). \n",
					__func__);
			} else {
				es9218_power_gpio_L();
			}
		}
	}

	mutex_unlock(&g_es9218_priv->power_lock);

	pr_debug("%s(): leave, es9218_power_state=%s.\n", __func__,
		 power_state[es9218_power_state]);
	return 0;
}

static int es9218_chip_state_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;

#if 0
	ret = ucontrol->value.enumerated.item[0];
#endif
	pr_debug("%s():ret = %d\n", __func__, ret);

	return ret;
}

static int chargerlogo_chipstate_get(void)
{
	int ret;
	uint8_t readChipStatus;
	unsigned char chipId = 0x00;
	int readCnt;

	pr_debug("%s(): enter, es9218_power_state=%s.\n", __func__,
		 power_state[es9218_power_state]);

	mutex_lock(&g_es9218_priv->power_lock);

	es9218_power_gpio_H();
	mdelay(1);
	es9218_reset_gpio_H();
	mdelay(1);

	/* The ESS revision check is done once during boot. */
	if (!g_ess_rev_check) {
		for (readCnt = 0; readCnt < 3; readCnt++) {
			readChipStatus = es9218_read_reg(
				g_es9218_priv->i2c_client, ES9218P_CHIPSTATUS);
			chipId = readChipStatus & 0xF0;

			pr_info("%s: chipId:0x%x readCnt : %d \n", __func__,
				chipId, readCnt);

			if (chipId == 0xd0) {
				pr_info("%s: ESS revsion = ESS_9218p\n",
					__func__);
				g_ess_rev_check = true;
				break;
			} else if (chipId == 0xe0) {
				pr_info("%s: ESS revsion = ESS_9228\n",
					__func__);
				g_ess_rev_check = true;
				break;
			}
		}
	}

	ret = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_SYSTEM_REG);
	if (ret < 0) {
		pr_err("%s : i2_read fail : %d\n", __func__, ret);
	} else {
		pr_notice("%s : i2_read success : %d\n", __func__, ret);
	}

	if (es9218_power_state < ESS_PS_HIFI) {
		es9218_reset_gpio_L();

		/* Power off when the ESS chip shuts down. */
		if (g_es9218_priv->es9218_data->always_power_on == false) {
			if (es9218_power_state == ESS_PS_BYPASS) {
				pr_info("%s(): headphones is already inserted from booting up. call es9218p_sabre_standby2lpb() to re-enter to lpb. \n",
					__func__);
				es9218p_sabre_standby2lpb();

				/* SABRE is now in LPB Mode. */

				pr_info("%s(): re-enter to lpb by calling es9218p_sabre_standby2lpb(). \n",
					__func__);
			} else {
				es9218_power_gpio_L();
			}
		}
	}

	mutex_unlock(&g_es9218_priv->power_lock);

	pr_debug("%s(): leave, es9218_power_state=%s.\n", __func__,
		 power_state[es9218_power_state]);

	return 0;
}

static int es9218_sabre_wcdon2bypass_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	pr_err("%s(): power state = %d\n", __func__, es9218_power_state);

	ucontrol->value.enumerated.item[0] = es9218_power_state;
	pr_err("%s(): ucontrol = %d\n", __func__,
	       ucontrol->value.enumerated.item[0]);

	return 0;
}

static int es9218_sabre_wcdon2bypass_put(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;

	mutex_lock(&g_es9218_priv->power_lock);

	ret = (int)ucontrol->value.integer.value[0];

	pr_err("%s(): entry wcd on : %d \n ", __func__, ret);

	if (ret == 0) {
		if (es9218_start) {
			if (__es9218_sabre_headphone_on() == 0)
				es9218p_sabre_bypass2hifi();
			es9218_is_amp_on = 1;
			pr_info("%s() : state = %s : WCD On State ByPass -> HiFi !!\n",
				__func__, power_state[es9218_power_state]);
		} else {
			pr_info("%s() : state = %s : don't change\n", __func__,
				power_state[es9218_power_state]);
		}
	} else {
#if 0
		if (es9218_power_state > ESS_PS_BYPASS &&
			es9218_power_state == ESS_PS_IDLE) {
#else
		if (es9218_power_state > ESS_PS_BYPASS) {
#endif
		pr_info("%s() : state = %s : WCD On State HiFi -> ByPass !!\n",
			__func__, power_state[es9218_power_state]);
		cancel_delayed_work_sync(&g_es9218_priv->sleep_work);
		if (pm_qos_request_active(&req)) {
			pr_info("%s(): pm qos active state. so, remove pm qos request",
				__func__);
			pm_qos_remove_request(&req);
		}
		es9218p_sabre_hifi2lpb();
	}
	else
	{
		pr_info("%s() : Invalid state = %s !!\n", __func__,
			power_state[es9218_power_state]);
	}

	es9218_is_amp_on = 0;
}
pr_debug("%s(): exit\n", __func__);

mutex_unlock(&g_es9218_priv->power_lock);

return 0;
}

static int es9218_clk_divider_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	int err_check = -1;
	uint8_t reg_val;

	if (es9218_power_state < ESS_PS_HIFI) {
		pr_err("%s() : invalid state = %s\n", __func__,
		       power_state[es9218_power_state]);

		return 0;
	}

	err_check = es9218_read_reg(g_es9218_priv->i2c_client,
				    ES9218P_MASTERMODE_SYNC_CONFIG);
	if (err_check >= 0) {
		reg_val = err_check;
	} else {
		return -1;
	}

	reg_val = reg_val >> 5;
	ucontrol->value.integer.value[0] = reg_val;

	pr_debug("%s: i2s_length = 0x%x\n", __func__, reg_val);

	return 0;
}

static int es9218_clk_divider_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	int err_check = -1;
	uint8_t reg_val;

	if (es9218_power_state < ESS_PS_HIFI) {
		pr_err("%s() : invalid state = %s\n", __func__,
		       power_state[es9218_power_state]);

		return 0;
	}

	pr_debug("%s: ucontrol->value.integer.value[0]  = %ld\n", __func__,
		 ucontrol->value.integer.value[0]);

	err_check = es9218_read_reg(g_es9218_priv->i2c_client,
				    ES9218P_MASTERMODE_SYNC_CONFIG);
	if (err_check >= 0) {
		reg_val = err_check;
	} else {
		return -1;
	}

	reg_val &= ~(I2S_CLK_DIVID_MASK);
	reg_val |= ucontrol->value.integer.value[0] << 5;

	es9218_write_reg(g_es9218_priv->i2c_client,
			 ES9218P_MASTERMODE_SYNC_CONFIG, reg_val);
	return 0;
}

static const char *const es9218_power_state_texts[] = {
	"Close",  "Open",      "Bypass",   "Hifi",    "Idle",
	"Active", "ResetHigh", "ResetLow", "HphHigh", "HphLow"
};

static const char *const es9218_clk_divider_texts[] = { "DIV4", "DIV8", "DIV16",
							"DIV16" };

static const struct soc_enum es9218_power_state_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(es9218_power_state_texts),
			es9218_power_state_texts);

static const struct soc_enum es9218_clk_divider_enum = SOC_ENUM_SINGLE_EXT(
	ARRAY_SIZE(es9218_clk_divider_texts), es9218_clk_divider_texts);

static struct snd_kcontrol_new es9218_digital_ext_snd_controls[] = {
	SOC_SINGLE_EXT("Es9018 Left Volume", SND_SOC_NOPM, 0, 256, 0,
		       es9218_left_volume_get, es9218_left_volume_put),
	SOC_SINGLE_EXT("Es9018 Right Volume", SND_SOC_NOPM, 0, 256, 0,
		       es9218_right_volume_get, es9218_right_volume_put),
	SOC_SINGLE_EXT("Es9018 Master Volume", SND_SOC_NOPM, 0, 100, 0,
		       es9218_master_volume_get, es9218_master_volume_put),
	SOC_SINGLE_EXT(
		"HIFI Custom Filter", SND_SOC_NOPM, 0, 12,
		0, /* 0-3 are custom filters while 4-11 are built-in filters. */
		es9218_filter_enum_get, es9218_filter_enum_put),
#if 0
	SOC_SINGLE_EXT("HIFI THD Value", SND_SOC_NOPM, 0, 0xFFFFFF, 0,
				es9018_set_filter_enum,
				es9218_filter_enum_put),
#endif
#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
	SOC_SINGLE_EXT("LGE ESS FADE INOUT", SND_SOC_NOPM, 0, 12, 0,
		       lge_ess_fade_inout_get, lge_ess_fade_inout_put),
	SOC_SINGLE_EXT("LGE ESS DIGITAL FILTER SETTING", SND_SOC_NOPM, 0, 12, 0,
		       lge_ess_digital_filter_setting_get,
		       lge_ess_digital_filter_setting_put),
#endif
	SOC_ENUM_EXT("Es9018 State", es9218_power_state_enum,
		     es9218_power_state_get, es9218_power_state_put),
	SOC_ENUM_EXT("Es9018 CLK Divider", es9218_clk_divider_enum,
		     es9218_clk_divider_get, es9218_clk_divider_put),
	SOC_SINGLE_EXT("Es9018 Chip State", SND_SOC_NOPM, 0, 1, 0,
		       es9218_chip_state_get, es9218_chip_state_put),
	SOC_SINGLE_EXT("Es9018 AVC Volume", SND_SOC_NOPM, 0, 25, 0,
		       es9218_avc_volume_get, es9218_avc_volume_put),
	SOC_SINGLE_EXT("Es9018 HEADSET TYPE", SND_SOC_NOPM, 0, 4, 0,
		       es9218_headset_type_get, es9218_headset_type_put),
	SOC_SINGLE_EXT("Es9018 Dop", SND_SOC_NOPM, 0, 128, 0, es9218_dop_get,
		       es9218_dop_put),
	SOC_SINGLE_EXT("Es9218 AUTO_MUTE", SND_SOC_NOPM, 0, 1, 0,
		       es9218_auto_mute_get, es9218_auto_mute_put),
	SOC_SINGLE_EXT("Es9218 Bypass", SND_SOC_NOPM, 0, 1, 0,
		       es9218_sabre_wcdon2bypass_get,
		       es9218_sabre_wcdon2bypass_put),
	SOC_SINGLE_MULTI_EXT("Es9218 NORMAL_HARMONIC LEFT", SND_SOC_NOPM, 0,
			     256, 0, 4, NULL,
			     es9218_normal_harmonic_comp_put_left),
	SOC_SINGLE_MULTI_EXT("Es9218 ADVANCE_HARMONIC LEFT", SND_SOC_NOPM, 0,
			     256, 0, 4, NULL,
			     es9218_advance_harmonic_comp_put_left),
	SOC_SINGLE_MULTI_EXT("ES9218 AUX_HARMONIC LEFT", SND_SOC_NOPM, 0, 256,
			     0, 4, NULL, es9218_aux_harmonic_comp_put_left),
	SOC_SINGLE_MULTI_EXT("Es9218 NORMAL_HARMONIC RIGHT", SND_SOC_NOPM, 0,
			     256, 0, 4, NULL,
			     es9218_normal_harmonic_comp_put_right),
	SOC_SINGLE_MULTI_EXT("Es9218 ADVANCE_HARMONIC RIGHT", SND_SOC_NOPM, 0,
			     256, 0, 4, NULL,
			     es9218_advance_harmonic_comp_put_right),
	SOC_SINGLE_MULTI_EXT("Es9218 AUX_HARMONIC RIGHT", SND_SOC_NOPM, 0, 256,
			     0, 4, NULL, es9218_aux_harmonic_comp_put_right),
};

static int es9218_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		pr_err("%s: err %d\n", __func__, ret);

	return ret;
}

static int es9218_write_reg(struct i2c_client *client, int reg, uint8_t value)
{
	int ret, i;

#if 0
	pr_notice("%s(): %03d=0x%x\n", __func__, reg, value);
#endif

	for (i = 0; i < 3; i++) {
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (ret < 0) {
			pr_err("%s: err %d,and try again\n", __func__, ret);
			mdelay(50);
		} else {
			break;
		}
	}

	if (ret < 0)
		pr_err("%s: err %d\n", __func__, ret);

	return ret;
}
static int es9218_populate_get_pdata(struct device *dev,
				     struct es9218_data *pdata)
{
#if USE_CONTROL_EXTERNAL_LDO_FOR_DVDD == 1
	uint32_t vol_suply[2];
	int ret;

#endif
	pdata->reset_gpio =
		of_get_named_gpio(dev->of_node, "dac,reset-gpio", 0);

	if (pdata->reset_gpio < 0) {
		pr_err("Looking up %s property in node %s failed %d\n",
		       "dac,reset-gpio", dev->of_node->full_name,
		       pdata->reset_gpio);
		goto err;
	}

	pr_info("%s: reset gpio %d", __func__, pdata->reset_gpio);

	pdata->hph_switch = of_get_named_gpio(dev->of_node, "dac,hph-sw", 0);
	if (pdata->hph_switch < 0) {
		pr_err("Looking up %s property in node %s failed %d\n",
		       "dac,hph-sw", dev->of_node->full_name,
		       pdata->hph_switch);
		goto err;
	}

	pr_info("%s: hph switch %d", __func__, pdata->hph_switch);

#if DEDICATED_I2C == 1
	pdata->i2c_scl_gpio =
		of_get_named_gpio(dev->of_node, "dac,i2c-scl-gpio", 0);
	if (pdata->i2c_scl_gpio < 0) {
		pr_err("Looking up %s property in node %s failed %d\n",
		       "dac,i2c-scl-gpio", dev->of_node->full_name,
		       pdata->i2c_scl_gpio);
		goto err;
	}

	dev_dbg(dev, "%s: i2c_scl_gpio %d", __func__, pdata->i2c_scl_gpio);

	pdata->i2c_sda_gpio =
		of_get_named_gpio(dev->of_node, "dac,i2c-sda-gpio", 0);
	if (pdata->i2c_sda_gpio < 0) {
		pr_err("Looking up %s property in node %s failed %d\n",
		       "dac,i2c-sda-gpio", dev->of_node->full_name,
		       pdata->i2c_sda_gpio);
		goto err;
	}
	pr_info("%s: i2c_sda_gpio %d", __func__, pdata->i2c_sda_gpio);
#endif

	pdata->power_gpio =
		of_get_named_gpio(dev->of_node, "dac,power-gpio", 0);
	if (pdata->power_gpio < 0) {
		pr_err("Looking up %s property in node %s failed %d\n",
		       "dac,power-gpio", dev->of_node->full_name,
		       pdata->power_gpio);
		goto err;
	}

	pr_info("%s: power gpio %d\n", __func__, pdata->power_gpio);

#if 0
	pdata->ear_dbg = of_get_named_gpio(dev->of_node, "dac,ear-dbg", 0);
	if (pdata->ear_dbg < 0) {
		pr_err("Looking up %s property in node %s failed %d\n", "dac,ear-dbg",
			dev->of_node->full_name, pdata->ear_dbg);
		goto err;
	}
	pr_info("%s: ear_dbg gpio %d\n", __func__, pdata->ear_dbg);
#endif

#if USE_CONTROL_EXTERNAL_LDO_FOR_DVDD == 1
	pdata->vreg_dvdd = regulator_get(dev, "dac,dvdd");
	if (IS_ERR(pdata->vreg_dvdd)) {
		/*
		 * If the analog voltage regulator (VA) is not ready yet, return
		 * -EPROBE_DEFER to kernel so that probe will be called at a later
		 * point in time.
		 */
		if (PTR_ERR(pdata->vreg_dvdd) == -EPROBE_DEFER) {
			pr_err("In %s, vreg_dvdd probe defer\n", __func__);
			devm_kfree(dev, pdata);

			return PTR_ERR(pdata->vreg_dvdd);
		}
	}

	pr_info("%s: DVDD ldo GET\n", __func__);

	ret = of_property_read_uint32_t_array(
		dev->of_node, "dac,va-supply-voltage", vol_suply, 2);
	if (ret < 0) {
		pr_err("%s Invalid property name\n", __func__);
		regulator_put(pdata->vreg_dvdd);
		devm_kfree(dev, pdata);
		return -EINVAL;
	} else {
		pdata->low_vol_level = vol_suply[0];
		pdata->high_vol_level = vol_suply[1];
		pr_info("%s: MIN uV=%d, MAX uV=%d. \n", __func__,
			pdata->low_vol_level, pdata->high_vol_level);
	}
#endif

	/*
	 * There are two conditions to use dynamic power management:
	 *
	 * 1) Two LDOs are dedicated for each 1.8V, 3.3V line (to handle LDOs
	 *    on/off).
	 * 2) The DVDD line must be connected to GND (to avoid current leakage
	 *    during sleep).
	 */
	if (of_property_read_bool(dev->of_node,
				  "dac,disable-always-power-on")) {
		pdata->always_power_on = false;
	} else {
		pdata->always_power_on = true;
	}

	pr_info("%s: always-power-on is [%s]\n", __func__,
		pdata->always_power_on ? "enabled" : "disabled");

	/*
	 * If the DVDD pin is connected to GND, the internal LDO must be used.
	 *
	 * However, if DVDD pin is connected to the 1.2V line, we have to choose
	 * between the internal and external LDO by measuring the current during
	 * chip activity and sleep, then finding out which has better power
	 * consumption (while looking out for current leakages during sleep).
	 */
	if (of_property_read_bool(dev->of_node, "dac,use-internal-ldo")) {
		pdata->use_internal_ldo = true;
	} else {
		pdata->use_internal_ldo = false;
	}

	pr_info("%s: use-internal-ldo is [%s]\n", __func__,
		pdata->use_internal_ldo ? "enabled" : "disabled");

	return 0;

err:
	devm_kfree(dev, pdata);
	return -1;
}

static unsigned int es9218_codec_read(struct snd_soc_codec *codec,
				      unsigned int reg)
{
#if 0
	struct es9218_priv *priv = codec->control_data;
#endif

	return 0;
}

static int es9218_codec_write(struct snd_soc_codec *codec, unsigned int reg,
			      unsigned int value)
{
#if 0
	struct es9218_priv *priv = codec->control_data;
#endif

	return 0;
}

static int es9218_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	int ret = 0;
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

#if 0
	dev_dbg(codec->dev, "%s(codec, level = 0x%04x): entry\n", __func__, level);
#endif

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		break;

	case SND_SOC_BIAS_STANDBY:
		break;

	case SND_SOC_BIAS_OFF:
		break;
	}

#if 0
	codec->dapm.bias_level = level;
#else
	dapm->bias_level = level;
#endif

/* This #if directive is intentionally seperate from the one above. */
#if 0
	dev_dbg(codec->dev, "%s(): exit\n", __func__);
#endif

	return ret;
}

static int es9218_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *codec_dai)
{
#if 0
	struct snd_soc_codec *codec = codec_dai->codec;
	struct es9218_priv *priv = codec->control_data;
#endif

	int ret = -1;
	es9218_bps = params_width(params);
	es9218_rate = params_rate(params);

	pr_info("%s(): entry , bps : %d , rate : %d\n", __func__, es9218_bps,
		es9218_rate);

	/* PCM Playback */
	if (g_dop_flag == 0) {
		pr_info("%s(): PCM Format Running \n", __func__);

		/* Set the bit-width to an ESS-advised value. */
		ret = es9218_set_bit_width(es9218_bps, ESS_MODE_PCM);
		ret = es9218p_set_sample_rate(es9218_bps, es9218_rate,
					      ESS_MODE_PCM);
		/* Set the volume rate to an ESS-advised value. */
		ret = es9218_set_volume_rate(es9218_rate, ESS_MODE_PCM);

#if ENABLE_DOP_AUTO_MUTE == 1
		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_AUTOMUTE_TIME, 0x00);
		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_AUTOMUTE_CONFIG, 0x34);
#endif
		/* DoP Playback */
	} else if (g_dop_flag > 0) { // DoP
		pr_info("%s(): DOP Format Running \n", __func__);

		/* Set the bit-width to an ESS-advised value. */
		ret = es9218_set_bit_width(g_dop_flag, ESS_MODE_DoP);
	}

	/* We've finished changing mode here, so reset mode_changed. */
	mode_changed = 0;

	pr_info("%s(): exit, ret=%d\n", __func__, ret);

	return ret;
}

static int es9218_mute(struct snd_soc_dai *dai, int mute)
{
#if 0
	struct snd_soc_codec *codec = dai->codec;
	struct es9218_priv *priv = codec->control_data;
#endif
	uint8_t reg;
#if ENABLE_DOP_SOFT_MUTE == 1

	pr_info("%s(): entry, mute_state = %d , power_state = %s\n", __func__,
		mute, power_state[es9218_power_state]);

	if (es9218_power_state < ESS_PS_HIFI) {
		pr_info("%s() : return = %s\n", __func__,
			power_state[es9218_power_state]);
		return 0;
	}

	if (mute) {
		reg = es9218_read_reg(g_es9218_priv->i2c_client,
				      ES9218P_FILTER_BAND_SYSTEM_MUTE);
		reg |= 0x01;
		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_FILTER_BAND_SYSTEM_MUTE, reg);
	} else {
		if (g_dop_flag) {
			pr_info("%s(): dop add delay\n", __func__);
			mdelay(10);
		}

		reg = es9218_read_reg(g_es9218_priv->i2c_client,
				      ES9218P_FILTER_BAND_SYSTEM_MUTE);
		reg &= ~0x01;
		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_FILTER_BAND_SYSTEM_MUTE, reg);
	}

	mdelay(5);
#endif

	return 0;
}

static int es9218_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
				 unsigned int freq, int dir)
{
#if 0
	struct snd_soc_codec *codec = codec_dai->codec;
	struct es9218_priv *priv = codec->control_data;
	pr_info("%s(): entry\n", __func__);
#endif

	return 0;
}

static int es9218_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
#if 0
	struct snd_soc_codec *codec = codec_dai->codec;
	struct es9218_priv *priv = codec->control_data;
	pr_info("%s(): entry\n", __func__);
#endif

	return 0;
}

static void es9218_switchmode(void)
{
	uint8_t reg;

	pr_info("%s() : However, changed a format from/to PCM/DoP(%d -> %d), so need to re-initialize registers !!\n",
		__func__, prev_dop_flag, g_dop_flag);

	if (g_dop_flag == 0) {
		/* We've switched to PCM, reinitialise the registers. */
		es9218p_initialize_registers(ESS_MODE_PCM);
	} else if (g_dop_flag > 0) {
#if ENABLE_DOP_SOFT_MUTE == 1
		reg = es9218_read_reg(g_es9218_priv->i2c_client,
				      ES9218P_FILTER_BAND_SYSTEM_MUTE);
		reg |= 0x01;
		es9218_write_reg(g_es9218_priv->i2c_client,
				 ES9218P_FILTER_BAND_SYSTEM_MUTE, reg);
		mdelay(2);

#endif
		/* We've switched to DoP, reinitialise the registers. */
		es9218p_initialize_registers(ESS_MODE_DoP);
	}
}

static int es9218_startup(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
#if 0
	struct snd_soc_codec *codec = codec_dai->codec;

#endif
	pr_info("%s(): entry\n", __func__);

	mutex_lock(&g_es9218_priv->power_lock);

	call_common_init_registers = 1;

	cancel_delayed_work_sync(&g_es9218_priv->sleep_work);
	if (pm_qos_request_active(&req)) {
		pr_info("%s(): pm qos active state. so, remove pm qos request",
			__func__);
		pm_qos_remove_request(&req);
	}
	if (es9218_power_state == ESS_PS_IDLE) {
		pr_info("%s() : state = %s : Audio Active !!\n", __func__,
			power_state[es9218_power_state]);
		/* Check if we've switched between DoP64 and DoP128. */
		if ((prev_dop_flag != g_dop_flag) &&
		    (prev_dop_flag * g_dop_flag > 0)) {
			pr_info("%s() : And, changed DoP's bit width from/to DoP64/DoP128(%d -> %d), so don't need to re-initialize registers !!\n",
				__func__, prev_dop_flag, g_dop_flag);
			/* Check if we've switched between PCM and DoP. */
		} else if ((prev_dop_flag != g_dop_flag) &&
			   (prev_dop_flag * g_dop_flag == 0)) {
			es9218_switchmode();
			/* Check if we've switched between PCM and DoP using mode_changed. */
		} else if (mode_changed) {
			es9218_switchmode();
		}

		es9218_sabre_audio_active();
	} else {
		pr_info("%s() : state = %s : goto HIFI !!\n", __func__,
			power_state[es9218_power_state]);
		if (__es9218_sabre_headphone_on() == 0)
			es9218p_sabre_bypass2hifi();
	}
	es9218_is_amp_on = 1;
	es9218_start = 1;

	pr_info("%s(): exit\n", __func__);

	mutex_unlock(&g_es9218_priv->power_lock);

	return 0;
}

static void es9218_shutdown(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	mutex_lock(&g_es9218_priv->power_lock);

	dev_info(codec->dev, "%s(): entry\n", __func__);

	es9218_sabre_audio_idle();

	req.type = PM_QOS_REQ_AFFINE_CORES;
	req.irq = -1;
	cpumask_copy(&req.cpus_affine, cpu_present_mask);

#if ES9218P_DEBUG == 1
	__pm_wakeup_event(&wl_shutdown, jiffies_to_msecs(10));
	schedule_delayed_work(&g_es9218_priv->sleep_work, msecs_to_jiffies(10));
#else
	__pm_wakeup_event(&wl_shutdown, jiffies_to_msecs(5000));

	if (!pm_qos_request_active(&req)) {
		pr_info("%s(): pm qos nonactive state. so, pm_qos_add_request",
			__func__);
		pm_qos_add_request(&req, PM_QOS_CPU_DMA_LATENCY, 0);
	}

	schedule_delayed_work(&g_es9218_priv->sleep_work,
			      msecs_to_jiffies(2000));
#endif

	es9218_start = 0;
	mutex_unlock(&g_es9218_priv->power_lock);
}

static int es9218_hw_free(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	dev_info(codec->dev, "%s(): entry\n", __func__);

	return 0;
}

static const struct snd_soc_dai_ops es9218_dai_ops = {
	.hw_params		= es9218_pcm_hw_params,
	.digital_mute		= es9218_mute,
	.set_fmt		= es9218_set_dai_fmt,
	.set_sysclk		= es9218_set_dai_sysclk,
	.startup		= es9218_startup,
	.shutdown		= es9218_shutdown,
	.hw_free		= es9218_hw_free,
};

static struct snd_soc_dai_driver es9218_dai[] = {
	{
		.name	= "es9218-hifi",
		.playback = {
			.stream_name	= "Playback",
			.channels_min	= 2,
			.channels_max	= 2,
			.rates		= ES9218_RATES,
			.formats	= ES9218_FORMATS,
		},
		.capture = {
			.stream_name	= "Capture",
			.channels_min	= 2,
			.channels_max	= 2,
			.rates		= ES9218_RATES,
			.formats	= ES9218_FORMATS,
		},
		.ops	= &es9218_dai_ops,
	},
};

static int es9218_codec_probe(struct snd_soc_codec *codec)
{
	struct es9218_priv *priv = snd_soc_codec_get_drvdata(codec);

	pr_notice("%s(): entry\n", __func__);

	if (priv)
		priv->codec = codec;
	else
		pr_err("%s(): fail !!!!!!!!!!\n", __func__);

	codec->control_data = snd_soc_codec_get_drvdata(codec);

	wakeup_source_init(&wl_sleep, "sleep_lock");
	wakeup_source_init(&wl_shutdown, "shutdown_lock");
	es9218_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	pr_notice("%s(): exit \n", __func__);

	return 0;
}

static int es9218_codec_remove(struct snd_soc_codec *codec)
{
	es9218_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_es9218 = {
	.probe		= es9218_codec_probe,
	.remove		= es9218_codec_remove,
	.read		= es9218_codec_read,
	.write		= es9218_codec_write,
	.component_driver = {
		.controls	= es9218_digital_ext_snd_controls,
		.num_controls	= ARRAY_SIZE(es9218_digital_ext_snd_controls),
	},
};

static int es9218_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct es9218_priv *priv;
	struct es9218_data *pdata;
	int ret = 0;

	pr_info("%s: enter.\n", __func__);

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s: no support for i2c read/write byte data\n",
		       __func__);

		return -EIO;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(struct es9218_data),
				     GFP_KERNEL);
		if (!pdata) {
			pr_err("Failed to allocate memory\n");

			return -ENOMEM;
		}

		ret = es9218_populate_get_pdata(&client->dev, pdata);
		if (ret) {
			pr_err("Parsing DT failed(%d)", ret);

			return ret;
		}
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		pr_err("%s: no platform data\n", __func__);

		return -EINVAL;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct es9218_priv),
			    GFP_KERNEL);
	if (priv == NULL) {
		return -ENOMEM;
	}

	priv->i2c_client = client;
	priv->es9218_data = pdata;
	i2c_set_clientdata(client, priv);

	g_es9218_priv = priv;
	INIT_DELAYED_WORK(&g_es9218_priv->hifi_in_standby_work,
			  es9218_sabre_hifi_in_standby_work);
	INIT_DELAYED_WORK(&g_es9218_priv->sleep_work, es9218_sabre_sleep_work);

	mutex_init(&g_es9218_priv->power_lock);
#if 0
	ret = gpio_request_one(pdata->ear_dbg, GPIOF_IN,"gpio_earjack_debugger");
	if (ret < 0) {
		pr_err("%s(): debugger gpio request failed\n", __func__);
		goto ear_dbg_gpio_request_error;
	}
	pr_info("%s: ear dbg. gpio num : %d, value : %d!\n",
		__func__, pdata->ear_dbg, gpio_get_value(pdata->ear_dbg));
#endif

	ret = gpio_request(pdata->power_gpio, "ess_power");
	if (ret < 0) {
		pr_err("%s(): ess power request failed\n", __func__);
		goto power_gpio_request_error;
	}

	ret = gpio_direction_output(pdata->power_gpio, 1);
	if (ret < 0) {
		pr_err("%s: ess power set failed\n", __func__);
		goto power_gpio_request_error;
	}

	gpio_set_value(pdata->power_gpio, 0);

	ret = gpio_request(pdata->hph_switch, "ess_switch");
	if (ret < 0) {
		pr_err("%s(): ess switch request failed\n", __func__);
		goto switch_gpio_request_error;
	}

	ret = gpio_direction_output(pdata->hph_switch, 1);
	if (ret < 0) {
		pr_err("%s: ess switch set failed\n", __func__);
		goto switch_gpio_request_error;
	}

	gpio_set_value(pdata->hph_switch, 0);

	ret = gpio_request(pdata->reset_gpio, "ess_reset");
	if (ret < 0) {
		pr_err("%s(): ess reset request failed\n", __func__);
		goto reset_gpio_request_error;
	}

	ret = gpio_direction_output(pdata->reset_gpio, 1);
	if (ret < 0) {
		pr_err("%s: ess reset set failed\n", __func__);
		goto reset_gpio_request_error;
	}

	gpio_set_value(pdata->reset_gpio, 0);

	ret = snd_soc_register_codec(&client->dev, &soc_codec_dev_es9218,
				     es9218_dai, ARRAY_SIZE(es9218_dai));

#if ES9218P_SYSFS == 1
	ret = sysfs_create_group(&client->dev.kobj, &es9218_attr_group);
#endif
	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		pr_info("%s chargerlogo mode call chargerlogo_chipstate_get function\n",
			__func__);
		chargerlogo_chipstate_get();
	}

	pr_info("%s: snd_soc_register_codec ret = %d\n", __func__, ret);

	return ret;

switch_gpio_request_error:
	gpio_free(pdata->hph_switch);
reset_gpio_request_error:
	gpio_free(pdata->reset_gpio);
power_gpio_request_error:
	gpio_free(pdata->power_gpio);
#if 0
ear_dbg_gpio_request_error:
	gpio_free(pdata->ear_dbg);
#endif
	return ret;
}

static int es9218_remove(struct i2c_client *client)
{
#if USE_CONTROL_EXTERNAL_LDO_FOR_DVDD == 1
	struct es9218_data *pdata;

	pdata = (struct es9218_data *)i2c_get_clientdata(client);
	if (pdata->vreg_dvdd != NULL)
		regulator_put(pdata->vreg_dvdd);
#endif
	snd_soc_unregister_codec(&client->dev);
	mutex_destroy(&g_es9218_priv->power_lock);

	return 0;
}

static struct of_device_id es9218_match_table[] = {
	{
		.compatible = "dac,es9218-codec",
	},
	{}
};

static const struct i2c_device_id es9218_id[] = {
	{ "es9218-codec", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, es9218_id);

static struct i2c_driver es9218_i2c_driver = {
	.driver	= {
		.name		= "es9218-codec",
		.owner		= THIS_MODULE,
		.of_match_table	= es9218_match_table,
	},
	.probe		= es9218_probe,
	.remove		= es9218_remove,
	.id_table	= es9218_id,
};

static int __init es9218_init(void)
{
	pr_notice("%s()\n", __func__);

	return i2c_add_driver(&es9218_i2c_driver);
}

static void __exit es9218_exit(void)
{
	i2c_del_driver(&es9218_i2c_driver);
}

module_init(es9218_init);
module_exit(es9218_exit);

MODULE_DESCRIPTION("ASoC ES9218 driver");
MODULE_AUTHOR("ESS-LINshaodong");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:es9218-codec");

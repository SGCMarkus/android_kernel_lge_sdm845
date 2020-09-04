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

#ifndef __ES9218P_H__
#define __ES9218P_H__

#define AMP_MODE_MASK			0x07
#define AMP_OC_EN			0x04
#define AREG_OUT_MASK			0x30
#define AVC_EN_MASK			0x20
#define CLK_GEAR_MASK			0x0C
#define DOP_EN_MASK			0x08
#define ES9218P_I2C_LEN_MASK		0xC0
#define ES9218P_SOFT_START_MASK		0x80
#define I2S_BIT_FORMAT_MASK		0xC0
#define I2S_CLK_DIVID_MASK		0x60
#define PRESET_FILTER_MASK		0xE0

#define ES9218_FORMATS                                                         \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE |                   \
	 SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE |                 \
	 SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE |                   \
	 SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S24_3BE |                 \
	 SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE)

#define ES9218_RATES                                                           \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 |   \
	 SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |  \
	 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_88200 |  \
	 SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000 |                       \
	 SNDRV_PCM_RATE_352800 | SNDRV_PCM_RATE_384000)

/* Enable if the ES9218P is connected over a dedicated I2C link. */
#define DEDICATED_I2C				0
/* Enable to mute after inactivity when playing back DSD{64,128}. */
#define ENABLE_DOP_AUTO_MUTE			1
/*
 * Enable to mute when switching between PCM and DoP (DSD{64,128}) to avoid
 * popping.
 */
#define ENABLE_DOP_SOFT_MUTE			1
/*
 * Enable to to introduce debug messages and time delays that are useful for
 * debugging.
 */
#define ES9218P_DEBUG				0
/* Enable the programmable numerically controlled oscillator. */
#define ES9218P_NCO				0
/*
 * Enable to allow overriding automatic mode selection (which depends on
 * headphone impedance) and expose more tunables to userland.
 */
#define ES9218P_SYSFS				1
/* Enable to control an external LDO exposed by the PMIC. */
#define USE_CONTROL_EXTERNAL_LDO_FOR_DVDD	0
/*
 * Enable to use the high quality mode for headphone amplifier, this increases
 * the THD by ~2dB and power consumption by ~2mA.
 */
#define USE_HPAHiQ				1
/*
 * Enable to mute the amplifier prior to switching the amplifier mode to avoid
 * popping on broken hardware.
 */
#define WORKAROUND_FOR_LOFI_POP			0

enum es9218_activity_mode {
	ESS_PS_CLOSE,
	ESS_PS_OPEN,
	ESS_PS_BYPASS,
	ESS_PS_HIFI,
	ESS_PS_IDLE,
	ESS_PS_ACTIVE,
};

enum es9218_playback_mode {
	/* Register Initialisation Mode */
	ESS_MODE_INIT,
	/* PCM Mode */
	ESS_MODE_PCM,
	/* DoP Mode */
	ESS_MODE_DoP,
};

enum es9218p_regs {
	ES9218P_SYSTEM_REG,
	ES9218P_INPUT_SELECT,
	ES9218P_AUTOMUTE_CONFIG,
	ES9218P_ANALOG_VOL_CTRL,
	ES9218P_AUTOMUTE_TIME,
	ES9218P_AUTOMUTE_LEVEL,
	ES9218P_DOP_VOL_RAMP_RATE,
	ES9218P_FILTER_BAND_SYSTEM_MUTE,
	ES9218P_GPIO1_2_CONFIG,
	ES9218P_REG09_RESERVED,
	ES9218P_MASTERMODE_SYNC_CONFIG,
	ES9218P_OVERCURRENT_PROTECT,
	ES9218P_DPLL_BANDWIDTH,
	ES9218P_THD_COMP_MONO_MODE,
	ES9218P_SOFT_START_CONFIG,
	ES9218P_VOL1_CTRL,
	ES9218P_VOL2_CTRL,
	ES9218P_MASTERTRIM_4,
	ES9218P_MASTERTRIM_3,
	ES9218P_MASTERTRIM_2,
	ES9218P_MASTERTRIM_1,
	ES9218P_GPIO_INPUT_SEL,
	ES9218P_THD_COMP_C2_2,
	ES9218P_THD_COMP_C2_1,
	ES9218P_THD_COMP_C3_2,
	ES9218P_THD_COMP_C3_1,
	ES9218P_CP_SS_DELAY,
	ES9218P_GEN_CONFIG,
	ES9218P_REG28_RESERVED,
	ES9218P_AUTO_CLK_GEAR,
	ES9218P_CP_CLOCK_2,
	ES9218P_CP_CLOCK_1,
	ES9218P_AMP_CONFIG,
	ES9218P_REG33_RESERVED,
	ES9218P_NCO_NUM_4,
	ES9218P_NCO_NUM_3,
	ES9218P_NCO_NUM_2,
	ES9218P_NCO_NUM_1,
	ES9218P_REG38_RESERVED,
	ES9218P_REG39_RESERVED,
	ES9218P_FIR_RAM_ADDR,
	ES9218P_FIR_RAM_DATA_3,
	ES9218P_FIR_RAM_DATA_2,
	ES9218P_FIR_RAM_DATA_1,
	ES9218P_FIR_CONFIG,
	ES9218P_ANALOG_CTRL_OVERRIDE,
	ES9218P_REG46_RESERVED,
	ES9218P_REG47_RESERVED,
	ES9218P_REG48_RESERVED,
	ES9218P_CLK_GEAR_THRESH_3,
	ES9218P_CLK_GEAR_THRESH_2,
	ES9218P_CLK_GEAR_THRESH_1,
	ES9218P_REG52_RESERVED,
	ES9218P_REG53_RESERVED,
	ES9218P_REG54_RESERVED,
	ES9218P_REG55_RESERVED,
	ES9218P_REG56_RESERVED,
	ES9218P_REG57_RESERVED,
	ES9218P_REG58_RESERVED,
	ES9218P_REG59_RESERVED,
	ES9218P_REG60_RESERVED,
	ES9218P_REG61_RESERVED,
	ES9218P_REG62_RESERVED,
	ES9218P_REG63_RESERVED,
	ES9218P_CHIPSTATUS,
	ES9218P_GPIO_READBACK,
	ES9218P_DPLL_NUM_4,
	ES9218P_DPLL_NUM_3,
	ES9218P_DPLL_NUM_2,
	ES9218P_DPLL_NUM_1,
	ES9218P_REG70_RESERVED,
	ES9218P_REG71_RESERVED,
	ES9218P_INPUT_SEL_AUTOMUTE_STATUS,
	ES9218P_RAM_COEFF_OUT_3,
	ES9218P_RAM_COEFF_OUT_2,
	ES9218P_RAM_COEFF_OUT_1,
	ES9218P_REG76_RESERVED,
	ES9218P_REG77_RESERVED,
	ES9218P_REG78_RESERVED,
	ES9218P_REG79_RESERVED,
	ES9218P_REG80_RESERVED,
};

enum sabre_filter_shape {
	SABRE_FILTER_FASTROLLOFF,
	SABRE_FILTER_SLOWROLLOFF,
	SABRE_FILTER_MINIMUM,
	SABRE_FILTER_MINSLOW,
	SABRE_FILTER_APODFAST1,
	SABRE_FILTER_HYBRIDFAST = 6,
	SABRE_FILTER_BRICKWALL,
};

enum sabre_filter_symmetry {
	SABRE_FILTER_SYMMETRY_SINE,
	SABRE_FILTER_SYMMETRY_COSINE,
};

#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
enum lge_ess_filter {
	VOLUME_DOWN,
	VOLUME_UP,
	SHARP_FILTER = 4,
	SLOW_FILTER,
	FADE_INOUT_COUNT = 7,
	SHORT_FILTER = 9,
	FADE_INOUT_TERM	= 40,

};
#endif

struct es9218_data {
	int reset_gpio;
	int power_gpio;
	int hph_switch;
	int ear_dbg;
	int hw_rev;
#if DEDICATED_I2C == 1
	int i2c_scl_gpio;
	int i2c_sda_gpio;
	int i2c_addr;
#endif
#if USE_CONTROL_EXTERNAL_LDO_FOR_DVDD == 1
	struct regulator *vreg_dvdd;
	u32 low_vol_level;
	u32 high_vol_level;
#endif
	bool always_power_on;
	bool use_internal_ldo;
};

struct es9218_priv {
	struct snd_soc_codec *codec;
	struct i2c_client *i2c_client;
	struct es9218_data *es9218_data;
	struct delayed_work hifi_in_standby_work;
	struct delayed_work sleep_work;
	struct mutex power_lock;
} es9218_priv;

struct es9218_reg {
	unsigned char num;
	unsigned char value;
};

#if ES9218P_SYSFS == 1
struct registry_map {
	const char *name;
	uint8_t reg;
	int writeable;
};
#endif

struct sabre_custom_filter {
	/* The shape of the stage 1 filter. */
	enum sabre_filter_shape shape;
	/* The symmetry of the stage 2 filter. */
	enum sabre_filter_symmetry symmetry;
	/* The coefficients of the stage 1 filter. */
	int stage1_coeff[128];
	/* The coefficients of the stage 2 filter. */
	int stage2_coeff[16];
};

static int es9218_read_reg(struct i2c_client *client, int reg);
static int es9218_write_reg(struct i2c_client *client, int reg, uint8_t value);

static int es9218_set_avc_volume(struct i2c_client *client, int vol);
static int es9218_set_bit_width(unsigned int bit_width, unsigned int ess_mode);
static int es9218_set_volume_rate(unsigned int sample_rate,
				  unsigned int ess_mode);

static int es9218p_sabre_bypass2hifi(void);
static int es9218p_sabre_hifi2lofi2lpb(void);
static int es9218p_sabre_hifi2lpb(void);
static int es9218p_sabre_lpb2hifi(int mode);
static int es9218p_sabre_lpb2standby(void);
static int es9218p_sabre_standby2lpb(void);

static int es9218p_sabre_amp_start(struct i2c_client *client, int headset);
static int es9218p_sabre_amp_stop(struct i2c_client *client, int headset);

static void es9218p_initialize_registers(unsigned int ess_mode);

#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
static int lge_ess_get_fade_count(void);
static int lge_ess_get_fade_term(void);
#endif

#endif /* __ES9218P_H__ */

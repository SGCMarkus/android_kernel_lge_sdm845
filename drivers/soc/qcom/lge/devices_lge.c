#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <soc/qcom/lge/board_lge.h>

#ifdef CONFIG_MACH_LGE
/*sync with android/bootable/bootloader/edk2/QcomModulePkg/Include/Library/ShutdownServices.h */
enum {
	NORMAL_MODE         = 0x0,
	RECOVERY_MODE       = 0x1,
	FASTBOOT_MODE       = 0x2,
	ALARM_BOOT          = 0x3,
	DM_VERITY_LOGGING   = 0x4,
	DM_VERITY_ENFORCING = 0x5,
	DM_VERITY_KEYSCLEAR = 0x6,
	NORMAL              = 0x20,
	WALLPAPER_FAIL      = 0x21,
	FOTA                = 0x22,
	FOTA_LCD_OFF        = 0x23,
	FOTA_OUT_LCD_OFF    = 0x24,
	LCD_OFF             = 0x25,
	CHARGE_RESET        = 0x26,
	LAF_DLOAD_MODE      = 0x27,
	LAF_RESTART_MODE    = 0x28,
	LAF_ONRS            = 0x29,
	XBOOT_AAT_WRITE     = 0x30,
	SHIP_MODE           = 0x31,
	OPID_MISMATCHED     = 0x32,
#ifdef CONFIG_LGE_USB_GADGET
	LAF_DLOAD_BOOT      = 0x40,
	LAF_DLOAD_MTP       = 0x41,
	LAF_DLOAD_TETHER    = 0x42,
	LAF_DLOAD_FACTORY   = 0x43,
#endif
	EMERGENCY_DLOAD     = 0xFF,
} RebootReasonType;

static int lge_boot_reason = -1;

static int __init lge_check_bootreason(char *reason)
{
	int ret = 0;

	/* handle corner case of kstrtoint */
	if (!strcmp(reason, "0xffffffff")) {
		lge_boot_reason = 0xffffffff;
		return 1;
	}

	ret = kstrtoint(reason, 16, &lge_boot_reason);
	if (!ret)
		pr_info("LGE BOOT REASON: 0x%x\n", lge_boot_reason);
	else
		pr_info("LGE BOOT REASON: Couldn't get bootreason - %d\n", ret);

	return 1;
}
__setup("androidboot.product.lge.bootreasoncode=", lge_check_bootreason);

int lge_get_bootreason(void)
{
	return lge_boot_reason;
}

bool lge_check_recoveryboot(void)
{
	if(lge_boot_reason == RECOVERY_MODE)
	{
		pr_info("LGE BOOT MODE is RECOVERY!!\n");
		return true;
	}
	else
	{
		 pr_info("LGE BOOT MODE is not RECOVERY!!\n");
		return false;
	}
}
#endif


#ifdef CONFIG_LGE_USB_FACTORY
/* get boot mode information from cmdline.
 * If any boot mode is not specified,
 * boot mode is normal type.
 */

#ifdef CONFIG_LGE_PM
extern void unified_bootmode_android(char* arg);
extern void unified_bootmode_cable(char* arg);
#endif

static cable_boot_type boot_cable_type = NONE_INIT_CABLE;

static int __init boot_cable_setup(char *boot_cable)
{
        if (!strcmp(boot_cable, "LT_56K"))
                boot_cable_type = LT_CABLE_56K;
        else if (!strcmp(boot_cable, "LT_130K"))
                boot_cable_type = LT_CABLE_130K;
        else if (!strcmp(boot_cable, "400MA"))
                boot_cable_type = USB_CABLE_400MA;
        else if (!strcmp(boot_cable, "DTC_500MA"))
                boot_cable_type = USB_CABLE_DTC_500MA;
        else if (!strcmp(boot_cable, "Abnormal_400MA"))
                boot_cable_type = ABNORMAL_USB_CABLE_400MA;
        else if (!strcmp(boot_cable, "LT_910K"))
                boot_cable_type = LT_CABLE_910K;
        else if (!strcmp(boot_cable, "NO_INIT"))
                boot_cable_type = NONE_INIT_CABLE;
        else
                boot_cable_type = NONE_INIT_CABLE;

#ifdef CONFIG_LGE_PM
	unified_bootmode_cable(boot_cable);
#endif
        pr_info("Boot cable : %s %d\n", boot_cable, boot_cable_type);

        return 1;
}

__setup("androidboot.vendor.lge.hw.cable=", boot_cable_setup);

cable_boot_type lge_get_boot_cable(void)
{
	return boot_cable_type;
}

static lge_boot_mode_t lge_boot_mode = LGE_BOOT_MODE_NORMAL;
int __init lge_boot_mode_init(char *s)
{
	if (!strcmp(s, "charger"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGER;
	else if (!strcmp(s, "chargerlogo"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGERLOGO;
	else if (!strcmp(s, "qem_56k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_56K;
	else if (!strcmp(s, "qem_130k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_130K;
	else if (!strcmp(s, "qem_910k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_910K;
	else if (!strcmp(s, "pif_56k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_56K;
	else if (!strcmp(s, "pif_130k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_130K;
	else if (!strcmp(s, "pif_910k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_910K;
	/* LGE_UPDATE_S for MINIOS2.0 */
	else if (!strcmp(s, "miniOS"))
		lge_boot_mode = LGE_BOOT_MODE_MINIOS;
	pr_info("ANDROID BOOT MODE : %d %s\n", lge_boot_mode, s);
	/* LGE_UPDATE_E for MINIOS2.0 */

#ifdef CONFIG_LGE_PM
	unified_bootmode_android(s);
#endif
	return 1;
}
__setup("androidboot.mode=", lge_boot_mode_init);

lge_boot_mode_t lge_get_boot_mode(void)
{
	return lge_boot_mode;
}

bool lge_get_factory_boot(void)
{
	/*   if boot mode is factory,
	 *   cable must be factory cable.
	 */
	switch (lge_boot_mode) {
	case LGE_BOOT_MODE_QEM_56K:
	case LGE_BOOT_MODE_QEM_130K:
	case LGE_BOOT_MODE_QEM_910K:
	case LGE_BOOT_MODE_PIF_56K:
	case LGE_BOOT_MODE_PIF_130K:
	case LGE_BOOT_MODE_PIF_910K:
	case LGE_BOOT_MODE_MINIOS:
		return true;

	default:
		break;
	}

	return false;
}
EXPORT_SYMBOL(lge_get_factory_boot);

static int lge_mfts_mode = 0;

static int __init lge_check_mfts_mode(char *s)
{
	int ret = 0;

	ret = kstrtoint(s, 10, &lge_mfts_mode);
	if(!ret)
		pr_info("LGE MFTS MODE: %d\n", lge_mfts_mode);
	else
		pr_info("LGE MFTS MODE: faile to get mfts mode %d\n", lge_mfts_mode);

	return 1;
}
__setup("mfts.mode=", lge_check_mfts_mode);

int lge_get_mfts_mode(void)
{
	return lge_mfts_mode;
}
EXPORT_SYMBOL(lge_get_mfts_mode);

lge_factory_cable_t lge_get_factory_cable(void)
{
	/* if boot mode is factory, cable must be factory cable. */
	switch (lge_boot_mode) {
	case LGE_BOOT_MODE_QEM_56K:
	case LGE_BOOT_MODE_PIF_56K:
		return LGE_FACTORY_CABLE_56K;

	case LGE_BOOT_MODE_QEM_130K:
	case LGE_BOOT_MODE_PIF_130K:
		return LGE_FACTORY_CABLE_130K;

	case LGE_BOOT_MODE_QEM_910K:
	case LGE_BOOT_MODE_PIF_910K:
		return LGE_FACTORY_CABLE_910K;

	default:
		break;
	}

	return LGE_FACTORY_CABLE_NONE;
}

#ifdef CONFIG_LGE_QFPROM_INTERFACE
static struct platform_device qfprom_device = {
	.name = "lge-qfprom",
	.id = -1,
};

static int __init lge_add_qfprom_devices(void)
{
	return platform_device_register(&qfprom_device);
}

arch_initcall(lge_add_qfprom_devices);
#endif

static int lge_bd_rev = HW_REV_1_0;

char *lge_rev_str[] = {
	"evb1", "evb2", "evb3", "rev_0", "rev_01",
	"rev_02", "rev_03", "rev_a", "rev_b", "rev_c", "rev_d",
	"rev_e", "rev_10", "rev_11", "rev_12",
	"rev_13", "reserved"
};

char *lge_get_board_revision(void)
{
	return lge_rev_str[lge_bd_rev];
}

enum hw_rev_no lge_get_board_rev_no(void)
{
	return lge_bd_rev;
}

static int __init board_revno_setup(char *rev_info)
{
	int i;

	for (i = 0; i < HW_REV_MAX; i++) {
		if (!strncmp(rev_info, lge_rev_str[i], 6)) {
			lge_bd_rev = i;
			break;
		}
	}
	pr_info("[LGE-HW-REV] %s\n", lge_rev_str[lge_bd_rev]);

	return 1;
}
__setup("androidboot.vendor.lge.hw.revision=", board_revno_setup);


static int lge_bd_subrev = HW_SUB_REV_0;

char *lge_subrev_str[] = {
	"subrev_0", "subrev_1", "subrev_2", "subrev_3", "subrev_4",
	"subrev_5", "subrev_6", "subrev_7", "subrev_8", "subrev_9",
};

char *lge_get_board_subrevision(void)
{
	return lge_subrev_str[lge_bd_subrev];
}

enum hw_subrev_no lge_get_board_subrev_no(void)
{
	return lge_bd_subrev;
}

static int __init board_subrevno_setup(char *subrev_info)
{
	int i;

	for (i = 0; i < HW_SUB_REV_MAX; i++) {
		if (!strncmp(subrev_info, lge_subrev_str[i], 8)) {
			lge_bd_subrev = i;
			break;
		}
	}
	pr_info("[LGE-HW-REV] %s\n", lge_subrev_str[lge_bd_subrev]);

	return 1;
}
__setup("androidboot.vendor.lge.hw.subrev=", board_subrevno_setup);


/*
   for download complete using LAF image
   return value : 1 --> right after laf complete & reset
 */
static bool android_dlcomplete = 0;

int __init lge_android_dlcomplete(char *s)
{
	if(strncmp(s,"1",1) == 0)
		android_dlcomplete = true;
	else
		android_dlcomplete = false;
	printk("androidboot.dlcomplete = %d\n", android_dlcomplete);

	return 1;
}
__setup("androidboot.dlcomplete=", lge_android_dlcomplete);

bool lge_get_android_dlcomplete(void)
{
	return android_dlcomplete;
}

static lge_laf_mode_t lge_laf_mode = LGE_LAF_MODE_NORMAL;

int __init lge_laf_mode_init(char *s)
{
	if (strcmp(s, "") && strcmp(s, "MID"))
		lge_laf_mode = LGE_LAF_MODE_LAF;

	return 1;
}
__setup("androidboot.vendor.lge.laf=", lge_laf_mode_init);

lge_laf_mode_t lge_get_laf_mode(void)
{
	return lge_laf_mode;
}
#endif

static int lge_hydra_name = HYDRA_PRIME;
int __init hydra_name_init(char *s)
{
	if (!strcmp(s, "Alpha"))
		lge_hydra_name = HYDRA_ALPHA;
	else if (!strcmp(s, "Prime"))
		lge_hydra_name = HYDRA_PRIME;
	else if (!strcmp(s, "Plus"))
		lge_hydra_name = HYDRA_PLUS;
	else if (!strcmp(s, "Plus1"))
		lge_hydra_name = HYDRA_PLUS1;
	else if (!strcmp(s, "Plus2"))
		lge_hydra_name = HYDRA_PLUS2;
	else if (!strcmp(s, "Signature"))
		lge_hydra_name = HYDRA_SIGNATURE;
	else
		lge_hydra_name = HYDRA_NONE;

	pr_info("androidboot.vendor.lge.hydra: %d\n",lge_hydra_name);

	return 1;
}

__setup("androidboot.vendor.lge.hydra=", hydra_name_init);

enum lge_hydra_name lge_get_hydra_name(void)
{
	return lge_hydra_name;
}
EXPORT_SYMBOL(lge_get_hydra_name);

#ifdef CONFIG_LGE_ONE_BINARY_SKU
/* For LAOP SKU Carrier Support */
static enum lge_sku_carrier_type lge_sku_carrier = HW_SKU_MAX;
int __init lge_sku_carrier_init(char *s)
{
	if (!strcmp(s, "KR_ALL"))
		lge_sku_carrier = HW_SKU_KR;
	else if (!strcmp(s, "LGU"))
		lge_sku_carrier = HW_SKU_KR_LGU;
	else if (!strcmp(s, "SKT"))
		lge_sku_carrier = HW_SKU_KR_SKT;
	else if (!strcmp(s, "KT"))
		lge_sku_carrier = HW_SKU_KR_KT;
	else if (!strcmp(s, "NA_GSM"))
		lge_sku_carrier = HW_SKU_NA_GSM;
	else if (!strcmp(s, "NA_ATT"))
		lge_sku_carrier = HW_SKU_NA_GSM_ATT;
	else if (!strcmp(s, "NA_TMUS"))
		lge_sku_carrier = HW_SKU_NA_GSM_TMUS;
	else if (!strcmp(s, "NA_CDMA"))
		lge_sku_carrier = HW_SKU_NA_CDMA;
	else if (!strcmp(s, "NA_VZW"))
		lge_sku_carrier = HW_SKU_NA_CDMA_VZW;
	else if (!strcmp(s, "NA_SPR"))
		lge_sku_carrier = HW_SKU_NA_CDMA_SPR;
	else if (!strcmp(s, "NA_ALL"))
		lge_sku_carrier = HW_SKU_NA;
	else if (!strcmp(s, "GLOBAL"))
		lge_sku_carrier = HW_SKU_GLOBAL;
	else if (!strcmp(s, "AISA"))
		lge_sku_carrier = HW_SKU_GLOBAL_ASIA;
	else if (!strcmp(s, "MEA"))
		lge_sku_carrier = HW_SKU_GLOBAL_MEA;
	else if (!strcmp(s, "EU"))
		lge_sku_carrier = HW_SKU_GLOBAL_EU;
	else if (!strcmp(s, "SCA"))
		lge_sku_carrier = HW_SKU_GLOBAL_SCA;
	else
		lge_sku_carrier = HW_SKU_MAX;

	pr_info("LGE One Binary Sku carrier : %d %s\n", lge_sku_carrier, s);

	return 1;
}
__setup("androidboot.vendor.lge.sku_carrier=", lge_sku_carrier_init);

char *lge_sku_str[] ={
	"KR_ALL", "LGU", "SKT", "KT", "KR_RESERVED", "NA_GSM", "NA_ATT", "NA_TMUS",
	"NA_RESERVED1", "NA_RESERVED2", "NA_CDMA", "NA_VZW", "NA_SPR", "NA_RESERVED3",
	"NA_ALL", "GLOBAL", "ASIA", "MEA", "EU", "SCA", "SKU_ERR"
};

enum lge_sku_carrier_type lge_get_sku_carrier(void)
{
	return lge_sku_carrier;
}

char *lge_get_sku_carrier_str(void)
{
	return lge_sku_str[lge_sku_carrier];
}
EXPORT_SYMBOL(lge_get_sku_carrier);


/* For LAOP NTCode Operator Support */
static enum lge_laop_operator_type lge_ntcode_op = OP_INVALID;
int __init lge_ntcode_op_init(char *s)
{
	if (!strcmp(s, "OPEN_KR"))
		lge_ntcode_op = OP_OPEN_KR;
	else if (!strcmp(s, "SKT"))
		lge_ntcode_op = OP_SKT_KR;
	else if (!strcmp(s, "KT"))
		lge_ntcode_op = OP_KT_KR;
	else if (!strcmp(s, "LGU"))
		lge_ntcode_op = OP_LGU_KR;
	else if (!strcmp(s, "ATT"))
		lge_ntcode_op = OP_ATT_US;
	else if (!strcmp(s, "TMUS"))
		lge_ntcode_op = OP_TMO_US;
	else if (!strcmp(s, "MPCS"))
		lge_ntcode_op = OP_MPCS_US;
	else if (!strcmp(s, "USC"))
		lge_ntcode_op = OP_USC_US;
	else if (!strcmp(s, "CCA_LRA_ACG"))
		lge_ntcode_op = OP_CCA_LRA_ACG_US;
	else if (!strcmp(s, "TRF"))
		lge_ntcode_op = OP_TRF_US;
	else if (!strcmp(s, "AMAZON"))
		lge_ntcode_op = OP_AMZ_US;
	else if (!strcmp(s, "GOOGLEFI"))
		lge_ntcode_op = OP_GFI_US;
	else if (!strcmp(s, "VZW_POSTPAID"))
		lge_ntcode_op = OP_VZW_POSTPAID;
	else if (!strcmp(s, "VZW_PREPAID"))
		lge_ntcode_op = OP_VZW_PREPAID;
	else if (!strcmp(s, "COMCAST_US"))
		lge_ntcode_op = OP_COMCAST_US;
	else if (!strcmp(s, "OPEN_US"))
		lge_ntcode_op = OP_OPEN_US;
	else if (!strcmp(s, "OPEN_CA"))
		lge_ntcode_op = OP_OPEN_CA;
	else
		lge_ntcode_op = OP_GLOBAL;

	pr_info("LGE One Binary NTCode Operator : %d %s\n", lge_ntcode_op, s);

	return 1;
}

__setup("lge.ntcode_op=", lge_ntcode_op_init);

enum lge_laop_operator_type lge_get_laop_operator(void)
{
	return lge_ntcode_op;
}
EXPORT_SYMBOL(lge_get_laop_operator);

#endif

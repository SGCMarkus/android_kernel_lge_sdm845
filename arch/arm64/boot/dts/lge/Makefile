ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
# Per Platform
ifeq ($(CONFIG_MACH_SDM845_JUDY),y)
DTS_PROJECT := sdm845-judy
endif

# Per Device
ifeq ($(CONFIG_MACH_SDM845_JUDY_KR),y)
DTS_TARGET := sdm845-judy_kr
endif

ifeq ($(CONFIG_MACH_SDM845_JUDY_LAO_COM),y)
DTS_TARGET := sdm845-judy_lao_com
endif

ifeq ($(CONFIG_MACH_SDM845_JUDY_LAO_LDU),y)
DTS_TARGET := sdm845-judy_lao_ldu
endif

ifeq ($(CONFIG_MACH_SDM845_JUDY_VZW),y)
DTS_TARGET := sdm845-judy_vzw
endif

ifeq ($(CONFIG_MACH_SDM845_JUDY_SPR),y)
DTS_TARGET := sdm845-judy_spr
endif

ifeq ($(CONFIG_MACH_SDM845_JUDY_DCM_JP),y)
DTS_TARGET := sdm845-judy_dcm_jp
endif

# Per Device - LCD NOTCH
ifeq ($(CONFIG_MACH_SDM845_JUDYLN_KR),y)
DTS_TARGET := sdm845-judyln_kr
endif

ifeq ($(CONFIG_MACH_SDM845_JUDYLN_LAO_COM),y)
DTS_TARGET := sdm845-judyln_lao_com
endif

ifeq ($(CONFIG_MACH_SDM845_JUDYLN_LAO_LDU),y)
DTS_TARGET := sdm845-judyln_lao_ldu
endif

ifeq ($(CONFIG_MACH_SDM845_JUDYLN_VZW),y)
DTS_TARGET := sdm845-judyln_vzw
endif

ifeq ($(CONFIG_MACH_SDM845_JUDYLN_SPR),y)
DTS_TARGET := sdm845-judyln_spr
endif

ifeq ($(CONFIG_MACH_SDM845_JUDYLN_DCM_JP),y)
DTS_TARGET := sdm845-judyln_dcm_jp
endif

# Per Device - POLED NOTCH
ifeq ($(CONFIG_MACH_SDM845_JUDYPN_KR),y)
DTS_TARGET := sdm845-judypn_kr
endif

ifeq ($(CONFIG_MACH_SDM845_JUDYPN_LAO_COM),y)
DTS_TARGET := sdm845-judypn_lao_com
endif

ifeq ($(CONFIG_MACH_SDM845_JUDYPN_VZW),y)
DTS_TARGET := sdm845-judypn_vzw
endif

ifeq ($(CONFIG_MACH_SDM845_JUDYPN_SPR),y)
DTS_TARGET := sdm845-judypn_spr
endif

# Per Device - POLED
ifeq ($(CONFIG_MACH_SDM845_JUDYP_KR),y)
DTS_TARGET := sdm845-judyp_kr
endif

ifeq ($(CONFIG_MACH_SDM845_JUDYP_LAO_COM),y)
DTS_TARGET := sdm845-judyp_lao_com
endif

ifeq ($(CONFIG_MACH_SDM845_JUDYP_VZW),y)
DTS_TARGET := sdm845-judyp_vzw
endif

ifeq ($(CONFIG_MACH_SDM845_JUDYP_SPR),y)
DTS_TARGET := sdm845-judyp_spr
endif

ifeq ($(CONFIG_MACH_SDM845_CAYMANSLM),y)
DTS_TARGET := sdm845-caymanslm
endif

ifeq ($(CONFIG_MACH_SDM845_STYLE3LM_DCM_JP),y)
DTS_TARGET := sdm845-style3lm_dcm_jp
endif

ALL_DTS := $(notdir $(wildcard echo $(srctree)/$(src)/$(DTS_PROJECT)/$(DTS_TARGET)/*.dts))
dtbo-y += $(patsubst %.dts,%.dtbo,$(ALL_DTS))

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtbo,$(DTB_NAMES))
else
DTB_LIST := $(dtbo-y)
endif

always  := $(dtbo-y)

$(obj)/%.dtbo: $(src)/$(DTS_PROJECT)/$(DTS_TARGET)/%.dts FORCE
	$(call if_changed_dep,dtc)
	$(call if_changed,dtbo_verify)

clean-files := *.dtb
endif

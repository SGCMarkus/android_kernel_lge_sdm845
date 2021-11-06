ifeq ($(CONFIG_SND_SOC_TFA9878), y)
# NXP audio amplifier - TFA9878

#TFA_DEBUG                =1
TFADSP_DEBUG             =1
TFA_VERSION              =tfa9878
TFADSP_32BITS            =1
TFADSP_DSP_MSG_PACKET_STRATEGY=1
TFADSP_DSP_BUFFER_POOL=1
TFA_EXCEPTION_AT_TRANSITION=1
TFA_USE_DEVICE_SPECIFIC_CONTROL=1
TFA_PROFILE_ON_DEVICE    =1
TFA_VOID_APIV_IN_FILE    =1
TFA_SET_EXT_INTERNALLY   =1
TFA_RAMPDOWN_BEFORE_MUTE =1
#TFA_MUTE_DURING_SWITCHING_PROFILE=1
TFA_RECONFIG_WITHOUT_RESET =1
TFA_SRC_DIR              =/asoc/codecs/$(TFA_VERSION)
TFA98XX_FLAGS           := -DUSE_TFA9878 -DQPLATFORM

# cc flags
#ccflags-y               := -DDEBUG
ccflags-y               += -I$(TFA_SRC_DIR)/inc
ccflags-y               += -Werror
ifdef TFA_DEBUG
ccflags-y               += -DTFA_DEBUG
endif
ifdef TFADSP_32BITS
ccflags-y               += -DTFADSP_32BITS
endif
ifdef TFADSP_DSP_MSG_PACKET_STRATEGY
ccflags-y               += -DTFADSP_DSP_MSG_PACKET_STRATEGY
endif
ifdef TFADSP_DSP_BUFFER_POOL
ccflags-y               += -DTFADSP_DSP_BUFFER_POOL
endif
ifdef TFA_EXCEPTION_AT_TRANSITION
ccflags-y               += -DTFA_EXCEPTION_AT_TRANSITION
endif
ifdef TFA_USE_DEVICE_SPECIFIC_CONTROL
ccflags-y               += -DTFA_USE_DEVICE_SPECIFIC_CONTROL
endif
ifdef TFA_PROFILE_ON_DEVICE
ccflags-y               += -DTFA_PROFILE_ON_DEVICE
endif
ifdef TFADSP_DEBUG
ccflags-y               += -DTFADSP_DEBUG
endif
ifdef TFA_VOID_APIV_IN_FILE
ccflags-y               += -DTFA_VOID_APIV_IN_FILE
endif
ifdef TFA_SET_EXT_INTERNALLY
ccflags-y               += -DTFA_SET_EXT_INTERNALLY
endif
ifdef TFA_RAMPDOWN_BEFORE_MUTE
ccflags-y               += -DTFA_RAMPDOWN_BEFORE_MUTE
endif
ifdef TFA_MUTE_DURING_SWITCHING_PROFILE
ccflags-y               += -DTFA_MUTE_DURING_SWITCHING_PROFILE
endif
ifdef TFA_RECONFIG_WITHOUT_RESET
ccflags-y               += -DTFA_RECONFIG_WITHOUT_RESET
endif

# objs
snd-soc-tfa98xx-objs    := tfa98xx.o
snd-soc-tfa98xx-objs    += tfa_container.o
snd-soc-tfa98xx-objs    += tfa_dsp.o
snd-soc-tfa98xx-objs    += tfa_init.o
ifdef TFA_DEBUG
snd-soc-tfa98xx-objs    += tfa_debug.o
endif

# CFLAGS
CFLAGS_tfa98xx.o       += $(TFA98XX_FLAGS)
CFLAGS_tfa_container.o += $(TFA98XX_FLAGS)
CFLAGS_tfa_dsp.o       += $(TFA98XX_FLAGS)
CFLAGS_tfa_init.o  += $(TFA98XX_FLAGS)
ifdef TFA_DEBUG
CFLAGS_tfa_debug.o     += $(TFA98XX_FLAGS)
endif

# built-in driver
obj-y	+= snd-soc-tfa98xx.o
endif
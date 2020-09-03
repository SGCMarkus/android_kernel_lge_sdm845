#
# Makefile for the fingerprint drivers.
#

# Each configuration option enables a list of files.
ccflags-y += -Idrivers/misc/

ifeq ($(CONFIG_FINGERPRINT_GOODIX_GF3208),y)
obj-$(CONFIG_FINGERPRINT_GOODIX_GF3208) += gf_spi.o platform.o netlink.o gf_common.o
endif

ifeq ($(CONFIG_INPUT_FINGERPRINT_FPC1020),y)
obj-$(CONFIG_INPUT_FINGERPRINT_FPC1020)		+= fpc1020_platform_tee.o
endif

ifeq ($(CONFIG_EGISTEC_FINGERPRINT),y)
obj-$(CONFIG_EGISTEC_FINGERPRINT) += et520.o navi_input.o
endif


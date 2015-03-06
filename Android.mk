MY_LOCAL_PATH := $(call my-dir)

ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk312x)
include $(MY_LOCAL_PATH)/tinyalsa_hal/Android.mk
else
 ifeq ($(strip $(TARGET_BOARD_PLATFORM_PRODUCT)), box)
 include $(MY_LOCAL_PATH)/tinyalsa_hal/Android.mk
 else
 include $(MY_LOCAL_PATH)/legacy_hal/Android.mk
 endif
endif


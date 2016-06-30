LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

# Setup bdroid local make variables for handling configuration
ifneq ($(BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR),)
  bdroid_C_INCLUDES := $(BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR)
  bdroid_CFLAGS += -DHAS_BDROID_BUILDCFG
else
  bdroid_C_INCLUDES :=
  bdroid_CFLAGS += -DHAS_NO_BDROID_BUILDCFG
endif

BDROID_DIR:= system/bt

LOCAL_CFLAGS += $(bdroid_CFLAGS)

LOCAL_SRC_FILES := \
    fm_hci.c

LOCAL_SHARED_LIBRARIES := \
         libdl \
         libcutils

LOCAL_CFLAGS := -Wno-unused-parameter

LOCAL_CFLAGS += -std=c99

LOCAL_C_INCLUDES += \
        $(BDROID_DIR)/hci/include \
        $(BDROID_DIR)/stack/include \
        $(BDROID_DIR)/osi/include \
        $(LOCAL_PATH)/../helium \
        $(LOCAL_PATH)/fm_hci

LOCAL_MODULE := libfm-hci
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES

include $(BUILD_SHARED_LIBRARY)

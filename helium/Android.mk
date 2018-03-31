ifneq ($(QCPATH),)
#ifeq ($(BOARD_HAVE_QCOM_FM),true)
#ifneq (,$(filter $(QCOM_BOARD_PLATFORMS),$(TARGET_BOARD_PLATFORM)))

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
        radio_helium_hal.c \
        radio_helium_hal_cmds.c

LOCAL_SHARED_LIBRARIES := \
         libfm-hci \
         libdl \
         liblog \
         libnativehelper \
         libcutils

FM_HCI_DIR:= $(LOCAL_PATH)/..

LOCAL_C_INCLUDES += $(FM_HCI_DIR)/fm_hci

LOCAL_MODULE := fm_helium
LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

#endif # is-vendor-board-platform
#endif # BOARD_HAVE_QCOM_FM
endif



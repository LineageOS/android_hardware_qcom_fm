ifneq ($(QCPATH),)
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
    fm_hci.cpp

LOCAL_SHARED_LIBRARIES := \
         libdl \
         libcutils \
         libbase \
         libhidlbase \
         libhidltransport \
         libhwbinder \
         liblog \
         libutils \
         vendor.qti.hardware.fm@1.0

LOCAL_CFLAGS := -Wno-unused-parameter

LOCAL_C_INCLUDES += \
        $(LOCAL_PATH)/../helium \
        $(LOCAL_PATH)/fm_hci

LOCAL_MODULE := libfm-hci
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES

include $(BUILD_SHARED_LIBRARY)
endif

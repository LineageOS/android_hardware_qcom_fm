ifeq ($(BOARD_HAVE_QCOM_FM),true)
LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
android_hardware_fm.cpp \
ConfFileParser.cpp \
ConfigFmThs.cpp \
FmIoctlsInterface.cpp \
FmPerformanceParams.cpp

LOCAL_LDLIBS += -ldl
LOCAL_SHARED_LIBRARIES := \
        libandroid_runtime \
        libnativehelper \
        liblog \
        libcutils

LOCAL_MODULE := libqcomfm_jni
LOCAL_MODULE_TAGS := optional

ifeq ($(TARGET_FM_LEGACY_PATCHLOADER),true)
    LOCAL_CFLAGS += -DFM_LEGACY_PATCHLOADER
endif

ifeq ($(TARGET_QCOM_NO_FM_FIRMWARE),true)
    LOCAL_CFLAGS += -DQCOM_NO_FM_FIRMWARE
endif

include $(BUILD_SHARED_LIBRARY)

endif # BOARD_HAVE_QCOM_FM

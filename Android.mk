ifeq ($(BOARD_HAVE_QCOM_FM),true)

LOCAL_PATH:= $(call my-dir)
LOCAL_DIR_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := optional

LOCAL_SRC_FILES := $(call all-java-files-under, qcom/fmradio)
LOCAL_JNI_SHARED_LIBRARIES := libqcomfm_jni

LOCAL_SHARED_LIBRARIES := liblog
LOCAL_MODULE:= qcom.fmradio

include $(BUILD_JAVA_LIBRARY)

ifeq ($(BOARD_HAS_QCA_FM_SOC), "cherokee")
LOCAL_CFLAGS += -DFM_SOC_TYPE_CHEROKEE
endif

include $(LOCAL_PATH)/jni/Android.mk
LOCAL_PATH := $(LOCAL_DIR_PATH)
include $(LOCAL_PATH)/fmapp2/Android.mk

LOCAL_PATH := $(LOCAL_DIR_PATH)
include $(LOCAL_PATH)/fm_hci/Android.mk

LOCAL_PATH := $(LOCAL_DIR_PATH)
include $(LOCAL_PATH)/helium/Android.mk

endif # BOARD_HAVE_QCOM_FM

SAMPLE_PATH := $(call my-dir)/../../src
<<<<<<< HEAD
=======
CURL_PATH := $(call my-dir)/../../../../curl-7.40.0/build-android
>>>>>>> a95b352369ae682b85e5c4344b6066e308e7a909

# external-deps
GAMEPLAY_DEPS := $(call my-dir)/../../../external-deps/lib/android/$(TARGET_ARCH_ABI)

# libgameplay
LOCAL_PATH := $(call my-dir)/../../../gameplay/android/libs/$(TARGET_ARCH_ABI)
include $(CLEAR_VARS)
LOCAL_MODULE    := libgameplay
LOCAL_SRC_FILES := libgameplay.so
include $(PREBUILT_SHARED_LIBRARY)

# libgameplay-deps
LOCAL_PATH := $(GAMEPLAY_DEPS)
include $(CLEAR_VARS)
LOCAL_MODULE    := libgameplay-deps
LOCAL_SRC_FILES := libgameplay-deps.a
include $(PREBUILT_STATIC_LIBRARY)

<<<<<<< HEAD
=======
# libcurl
LOCAL_PATH := $(CURL_PATH)/lib
include $(CLEAR_VARS)
LOCAL_MODULE	:= libcurl
LOCAL_SRC_FILES	:= libcurl.a
include $(PREBUILT_STATIC_LIBRARY)

>>>>>>> a95b352369ae682b85e5c4344b6066e308e7a909
# t4tapp
LOCAL_PATH := $(SAMPLE_PATH)
include $(CLEAR_VARS)
LOCAL_MODULE    := t4tapp
<<<<<<< HEAD
LOCAL_SRC_FILES := ../../gameplay/src/gameplay-main-android.cpp T4TApp.cpp

=======
LOCAL_SRC_FILES := ../../gameplay/src/gameplay-main-android.cpp T4TApp.cpp Models.cpp Grid.cpp MyNode.cpp Mode.cpp NavigateMode.cpp HullMode.cpp Project.cpp Satellite.cpp Rocket.cpp Buggy.cpp Robot.cpp LandingPod.cpp CEV.cpp Launcher.cpp
>>>>>>> a95b352369ae682b85e5c4344b6066e308e7a909

LOCAL_CPPFLAGS += -std=c++11 -frtti -Wno-switch-enum -Wno-switch
LOCAL_ARM_MODE := arm
LOCAL_LDLIBS    := -llog -landroid -lEGL -lGLESv2 -lOpenSLES
<<<<<<< HEAD
LOCAL_CFLAGS    := -D__ANDROID__ -I"../../external-deps/include" -I"../../gameplay/src"
LOCAL_STATIC_LIBRARIES := android_native_app_glue libgameplay-deps
=======
LOCAL_CFLAGS    := -D__ANDROID__ -I"../../external-deps/include" -I"../../gameplay/src" -I"../../../curl-7.40.0/build-android/include"
LOCAL_STATIC_LIBRARIES := android_native_app_glue libgameplay-deps libcurl
>>>>>>> a95b352369ae682b85e5c4344b6066e308e7a909
LOCAL_SHARED_LIBRARIES := gameplay
include $(BUILD_SHARED_LIBRARY)

$(call import-module,android/native_app_glue)

#pragma once

#include <android/log.h>

namespace hf {
extern int gLogLevel;
}

#define HF_TAG "HandFusion"
#define HF_LOGE(...) __android_log_print(ANDROID_LOG_ERROR, HF_TAG, __VA_ARGS__)
#define HF_LOGI(...) \
    do { if (hf::gLogLevel >= 1) __android_log_print(ANDROID_LOG_INFO, HF_TAG, __VA_ARGS__); } while (0)
#define HF_LOGD(...) \
    do { if (hf::gLogLevel >= 2) __android_log_print(ANDROID_LOG_DEBUG, HF_TAG, __VA_ARGS__); } while (0)

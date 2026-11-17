// Zygisk entry point: runs the hand fusion inside PICO Connect's streaming process only.
#include <cstring>

#include <zygisk/zygisk.hpp>

#include "log.h"

namespace hf {
void startInProcessHooks();
}

namespace {

constexpr const char* kTargetProcess = "com.picovr.picostreamassistant:deviceservice";

class HandFusionModule : public zygisk::ModuleBase {
public:
    void onLoad(zygisk::Api* api, JNIEnv* env) override {
        api_ = api;
        env_ = env;
    }

    void preAppSpecialize(zygisk::AppSpecializeArgs* args) override {
        target_ = false;
        if (args && args->nice_name) {
            const char* name = env_->GetStringUTFChars(args->nice_name, nullptr);
            if (name) {
                target_ = strcmp(name, kTargetProcess) == 0;
                env_->ReleaseStringUTFChars(args->nice_name, name);
            }
        }
        // Leave every other process untouched.
        if (!target_) api_->setOption(zygisk::Option::DLCLOSE_MODULE_LIBRARY);
    }

    void postAppSpecialize(const zygisk::AppSpecializeArgs*) override {
        if (target_) hf::startInProcessHooks();
    }

    void preServerSpecialize(zygisk::ServerSpecializeArgs*) override {
        api_->setOption(zygisk::Option::DLCLOSE_MODULE_LIBRARY);
    }

private:
    zygisk::Api* api_ = nullptr;
    JNIEnv* env_ = nullptr;
    bool target_ = false;
};

}  // namespace

REGISTER_ZYGISK_MODULE(HandFusionModule)

// One Euro filter (Casiez et al., CHI 2012) for positions and orientations.
// Strong smoothing at low speed (kills tremor), little lag at high speed.
#pragma once

#include <cmath>

#include "xr_math.h"

namespace hf {

inline float lowpassAlpha(float cutoffHz, float dt) {
    const float tau = 1.f / (6.2831853f * cutoffHz);
    return 1.f / (1.f + tau / dt);
}

struct OneEuroParams {
    float minCutoff = 1.f;  // Hz; <= 0 disables the filter
    float beta = 0.f;       // cutoff increase per unit of speed
    float dCutoff = 1.f;    // Hz, for the speed estimate
};

class OneEuroVec3 {
public:
    void reset() { init_ = false; }
    XrVector3f filter(const XrVector3f& x, float dt, const OneEuroParams& p) {
        if (p.minCutoff <= 0.f) {
            init_ = false;
            return x;
        }
        if (!init_) {
            init_ = true;
            x_ = x;
            dx_ = 0.f;
            return x;
        }
        const float speed = length(sub(x, x_)) / dt;
        dx_ += lowpassAlpha(p.dCutoff, dt) * (speed - dx_);
        const float a = lowpassAlpha(p.minCutoff + p.beta * dx_, dt);
        x_ = lerp(x_, x, a);
        return x_;
    }

private:
    bool init_ = false;
    XrVector3f x_{};
    float dx_ = 0.f;
};

class OneEuroQuat {
public:
    void reset() { init_ = false; }
    XrQuaternionf filter(const XrQuaternionf& q, float dt, const OneEuroParams& p) {
        if (p.minCutoff <= 0.f) {
            init_ = false;
            return q;
        }
        if (!init_) {
            init_ = true;
            q_ = q;
            dx_ = 0.f;
            return q;
        }
        const float speed = qangle(q_, q) / dt;  // rad/s
        dx_ += lowpassAlpha(p.dCutoff, dt) * (speed - dx_);
        const float a = lowpassAlpha(p.minCutoff + p.beta * dx_, dt);
        q_ = qslerp(q_, q, a);
        return q_;
    }

private:
    bool init_ = false;
    XrQuaternionf q_{0.f, 0.f, 0.f, 1.f};
    float dx_ = 0.f;
};

class OneEuroPose {
public:
    void reset() {
        p_.reset();
        q_.reset();
    }
    XrPosef filter(const XrPosef& x, float dt, const OneEuroParams& posParams, const OneEuroParams& rotParams) {
        XrPosef r;
        r.position = p_.filter(x.position, dt, posParams);
        r.orientation = q_.filter(x.orientation, dt, rotParams);
        return r;
    }

private:
    OneEuroVec3 p_;
    OneEuroQuat q_;
};

}  // namespace hf

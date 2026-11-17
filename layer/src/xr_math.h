// Small pose math helpers on top of OpenXR types.
#pragma once

#include <cmath>
#include <openxr/openxr.h>

namespace hf {

inline XrVector3f v3(float x, float y, float z) { return {x, y, z}; }
inline XrVector3f add(const XrVector3f& a, const XrVector3f& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline XrVector3f sub(const XrVector3f& a, const XrVector3f& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline XrVector3f scale(const XrVector3f& a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(const XrVector3f& a, const XrVector3f& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float length(const XrVector3f& a) { return std::sqrt(dot(a, a)); }
inline XrVector3f lerp(const XrVector3f& a, const XrVector3f& b, float t) { return add(a, scale(sub(b, a), t)); }

inline XrQuaternionf qidentity() { return {0.f, 0.f, 0.f, 1.f}; }

inline XrQuaternionf qnormalize(const XrQuaternionf& q) {
    float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (n < 1e-8f) return qidentity();
    return {q.x / n, q.y / n, q.z / n, q.w / n};
}

inline XrQuaternionf qconj(const XrQuaternionf& q) { return {-q.x, -q.y, -q.z, q.w}; }

inline XrQuaternionf qmul(const XrQuaternionf& a, const XrQuaternionf& b) {
    return {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

inline XrVector3f qrotate(const XrQuaternionf& q, const XrVector3f& v) {
    // v' = v + 2w(u x v) + 2u x (u x v)
    XrVector3f u{q.x, q.y, q.z};
    XrVector3f t{2.f * (u.y * v.z - u.z * v.y), 2.f * (u.z * v.x - u.x * v.z), 2.f * (u.x * v.y - u.y * v.x)};
    XrVector3f c{u.y * t.z - u.z * t.y, u.z * t.x - u.x * t.z, u.x * t.y - u.y * t.x};
    return {v.x + q.w * t.x + c.x, v.y + q.w * t.y + c.y, v.z + q.w * t.z + c.z};
}

inline float qdot(const XrQuaternionf& a, const XrQuaternionf& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

// Angle in radians between two orientations.
inline float qangle(const XrQuaternionf& a, const XrQuaternionf& b) {
    float d = std::fabs(qdot(a, b));
    if (d > 1.f) d = 1.f;
    return 2.f * std::acos(d);
}

inline XrQuaternionf qslerp(const XrQuaternionf& a, XrQuaternionf b, float t) {
    float d = qdot(a, b);
    if (d < 0.f) {
        b = {-b.x, -b.y, -b.z, -b.w};
        d = -d;
    }
    if (d > 0.9995f) {
        // Nearly identical: normalized lerp is accurate and stable.
        return qnormalize({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t,
                           a.w + (b.w - a.w) * t});
    }
    float theta = std::acos(d);
    float s = std::sin(theta);
    float wa = std::sin((1.f - t) * theta) / s;
    float wb = std::sin(t * theta) / s;
    return {a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb, a.w * wa + b.w * wb};
}

// Pose composition: (a * b) maps b's local frame through a.
inline XrPosef pmul(const XrPosef& a, const XrPosef& b) {
    XrPosef r;
    r.orientation = qnormalize(qmul(a.orientation, b.orientation));
    r.position = add(a.position, qrotate(a.orientation, b.position));
    return r;
}

inline XrPosef pinv(const XrPosef& p) {
    XrPosef r;
    r.orientation = qconj(p.orientation);
    r.position = qrotate(r.orientation, scale(p.position, -1.f));
    return r;
}

inline XrPosef pblend(const XrPosef& a, const XrPosef& b, float t) {
    XrPosef r;
    r.orientation = qslerp(a.orientation, b.orientation, t);
    r.position = lerp(a.position, b.position, t);
    return r;
}

inline XrPosef pidentity() {
    XrPosef p;
    p.orientation = qidentity();
    p.position = {0.f, 0.f, 0.f};
    return p;
}

}  // namespace hf

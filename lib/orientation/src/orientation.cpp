#include "orientation.hpp"

#include <cmath>
#include <numbers>
namespace {
constexpr float RAD_TO_DEG = 180.0f / std::numbers::pi_v<float>; // ≈ 57.2958
} // namespace

namespace orientation {
ComplementaryFilter::ComplementaryFilter(float time_constant_s, float accel_tolerance_g)
    : time_constant_s_(time_constant_s), accel_tolerance_g_(accel_tolerance_g) {}

void ComplementaryFilter::reset() {
    angles_ = Angles{};
    initialized_ = false;
}

Angles ComplementaryFilter::angles() const {
    return angles_;
}
Angles ComplementaryFilter::update(const Vec3& accel, const Vec3& gyro_dps, float dt_s) {
    if (dt_s <= 0.0f)
        return angles_;
    Angles accel_ang = tilt_from_accel(accel);
    float accel_length = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    bool accel_trust =
        accel_length > 1.0f - accel_tolerance_g_ && accel_length < 1.0f + accel_tolerance_g_;

    if (!initialized_) {
        angles_ = accel_ang;
        initialized_ = true;
        return angles_;
    }

    angles_.roll_deg += gyro_dps.x * dt_s;
    angles_.pitch_deg += gyro_dps.y * dt_s;

    float alpha = time_constant_s_ / (time_constant_s_ + dt_s);
    if (accel_trust) {
        angles_.roll_deg = alpha * angles_.roll_deg + (1.0f - alpha) * accel_ang.roll_deg;
        angles_.pitch_deg = alpha * angles_.pitch_deg + (1.0f - alpha) * accel_ang.pitch_deg;
    }

    return angles_;
}
Angles tilt_from_accel(const Vec3& accel_g) {
    const float roll_rad = atan2f(accel_g.y, accel_g.z);
    const float pitch_rad =
        atan2f(-accel_g.x, sqrtf(accel_g.y * accel_g.y + accel_g.z * accel_g.z));
    return Angles{roll_rad * RAD_TO_DEG, pitch_rad * RAD_TO_DEG};
}
} // namespace orientation

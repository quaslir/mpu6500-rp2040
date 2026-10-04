#pragma once
#include "math/vec3.hpp"
namespace orientation {
struct Angles {
    float roll_deg{};
    float pitch_deg{};
};
inline constexpr float DEFAULT_TIME_CONSTANT_S = 0.5f;   // how slowly accel corrects gyro drift
inline constexpr float DEFAULT_ACCEL_TOLERANCE_G = 0.2f; // accel trusted only if |a| is 1 +- this
class ComplementaryFilter {
public:
    ComplementaryFilter(float time_constant_s = 0.2f, float accel_tolerance_g = 0.5f);
    [[nodiscard]] Angles update(const math::Vec3& accel, const math::Vec3& gyro_dps, float dt_s);
    [[nodiscard]] Angles angles() const;
    void reset();

private:
    Angles angles_{};
    bool initialized_{};
    float time_constant_s_;
    float accel_tolerance_g_;
};
Angles tilt_from_accel(const math::Vec3& accel_g);
} // namespace orientation

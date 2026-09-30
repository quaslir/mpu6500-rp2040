#pragma once

#include "vec3.hpp"
#include <cstdint>
namespace mpu6500::calibration {

enum class Sensor { Accel, Gyro };
struct MeasureOptions {
    uint16_t samples;
    uint16_t warmup_samples;
    float max_spread;
};

inline constexpr MeasureOptions DEFAULT_GYRO_OPTIONS = {
    .samples = 200, .warmup_samples = 10, .max_spread = 2.0f};
inline constexpr MeasureOptions DEFAULT_ACCEL_OPTIONS = {
    .samples = 200, .warmup_samples = 10, .max_spread = 0.05f};
inline constexpr Vec3 GRAVITY_Z_UP{0.0f, 0.0f, 1.0f};

inline constexpr float MIN_GRAVITY_G{0.8f};
inline constexpr float MAX_GRAVITY_G{1.2f};

} // namespace mpu6500::calibration

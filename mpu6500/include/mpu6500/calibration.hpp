#pragma once

#include "bus/status.hpp"
#include "math/vec3.hpp"
#include <cstdint>

namespace mpu6500 {
class Mpu6500;
} // namespace mpu6500

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
inline constexpr math::Vec3 GRAVITY_Z_UP{0.0f, 0.0f, 1.0f};

inline constexpr float MIN_GRAVITY_G{0.8f};
inline constexpr float MAX_GRAVITY_G{1.2f};

[[nodiscard]] bus::Status
measure_mean(const Mpu6500& imu, Sensor sensor, math::Vec3& mean, const MeasureOptions& options);
[[nodiscard]] bus::Status measure_gyro_offset(const Mpu6500& imu,
                                         math::Vec3& offset,
                                         const MeasureOptions& options = DEFAULT_GYRO_OPTIONS);
[[nodiscard]] bus::Status calibrate_gyro(Mpu6500& imu,
                                    const MeasureOptions& options = DEFAULT_GYRO_OPTIONS);

[[nodiscard]] bus::Status measure_accel_offset(const Mpu6500& imu,
                                          math::Vec3& offset,
                                          const math::Vec3& expected_gravity_g = GRAVITY_Z_UP,
                                          const MeasureOptions& options = DEFAULT_ACCEL_OPTIONS);
[[nodiscard]] bus::Status calibrate_accel(Mpu6500& imu,
                                     const math::Vec3& expected_gravity_g = GRAVITY_Z_UP,
                                     const MeasureOptions& options = DEFAULT_ACCEL_OPTIONS);
} // namespace mpu6500::calibration

#pragma once

#include <cstdint>
namespace mpu6500::calibration {
inline constexpr uint8_t DEFAULT_GYRO_SAMPLES{200};
inline constexpr float DEFAULT_GYRO_MAX_SPREAD_DPS{2.0f};
inline constexpr uint8_t DEFAULT_GYRO_WARMUP_SAMPLES{10};
} // namespace mpu6500::calibration

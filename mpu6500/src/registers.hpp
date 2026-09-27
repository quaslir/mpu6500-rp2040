#pragma once

#include <cstdint>
namespace mpu6500 {
inline constexpr uint8_t WHOAMI{0x75};
inline constexpr uint8_t PWR_MGMT_1{0x6B};
inline constexpr uint8_t SIGNAL_PATH_RESET{0x68};
inline constexpr uint8_t USER_CTRL{0x6A};
inline constexpr uint8_t GYRO_CONFIG{0x1B};
inline constexpr uint8_t ACCEL_CONFIG{0x1C};
} // namespace mpu6500

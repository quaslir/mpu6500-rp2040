#pragma once

#include <cstdint>

namespace mpu6500::bits {

namespace pwr_mgmt_1 {
inline constexpr uint8_t DEVICE_RESET{0x80};
inline constexpr uint8_t SLEEP{0x40};
inline constexpr uint8_t CLKSEL_MASK{0x07};
inline constexpr uint8_t CLKSEL_AUTO{0x01};

inline constexpr uint8_t RESET{DEVICE_RESET};
inline constexpr uint8_t NORMAL{CLKSEL_AUTO};
} // namespace pwr_mgmt_1

namespace signal_path_reset {
inline constexpr uint8_t GYRO{0x04};
inline constexpr uint8_t ACCEL{0x02};
inline constexpr uint8_t TEMP{0x01};
inline constexpr uint8_t ALL{GYRO | ACCEL | TEMP};
} // namespace signal_path_reset

namespace user_ctrl {
inline constexpr uint8_t I2C_IF_DIS{0x10};
} // namespace user_ctrl

namespace fs_sel {
inline constexpr uint8_t SHIFT{3};
inline constexpr uint8_t MASK{0x18};
} // namespace fs_sel

namespace config {
inline constexpr uint8_t DLPF_CFG_MASK{0x07};      // bits 2:0, gyro/temp filter
inline constexpr uint8_t EXT_SYNC_SET_MASK{0x38};  // bits 5:3, FSYNC (unused, keep 0)
inline constexpr uint8_t FIFO_MODE{0x40};          // bit 6 (unused, keep 0)
} // namespace config

namespace accel_config2 {
inline constexpr uint8_t A_DLPF_CFG_MASK{0x07};    // bits 2:0, accel filter
inline constexpr uint8_t ACCEL_FCHOICE_B{0x08};    // bit 3, must be 0 or the filter is bypassed
} // namespace accel_config2


} // namespace mpu6500::bits

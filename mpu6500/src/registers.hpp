#pragma once

#include <cstdint>

namespace mpu6500 {

inline constexpr uint8_t GYRO_CONFIG{0x1B};
inline constexpr uint8_t ACCEL_CONFIG{0x1C};
inline constexpr uint8_t ACCEL_XOUT_H{0x3B};
inline constexpr uint8_t SIGNAL_PATH_RESET{0x68};
inline constexpr uint8_t USER_CTRL{0x6A};
inline constexpr uint8_t PWR_MGMT_1{0x6B};
inline constexpr uint8_t WHOAMI{0x75};

inline constexpr uint8_t EXPECTED_DEVICE_ID{0x70};

inline constexpr uint8_t PWR_MGMT_1_DEVICE_RESET{0x80};
inline constexpr uint8_t PWR_MGMT_1_SLEEP{0x40};
inline constexpr uint8_t PWR_MGMT_1_CLKSEL_MASK{0x07};
inline constexpr uint8_t PWR_MGMT_1_CLKSEL_AUTO{0x01};
inline constexpr uint8_t PWR_MGMT_1_RESET{PWR_MGMT_1_DEVICE_RESET};
inline constexpr uint8_t PWR_MGMT_1_NORMAL{PWR_MGMT_1_CLKSEL_AUTO};

inline constexpr uint8_t SIGNAL_PATH_RESET_GYRO{0x04};
inline constexpr uint8_t SIGNAL_PATH_RESET_ACCEL{0x02};
inline constexpr uint8_t SIGNAL_PATH_RESET_TEMP{0x01};
inline constexpr uint8_t SIGNAL_PATH_RESET_ALL{SIGNAL_PATH_RESET_GYRO | SIGNAL_PATH_RESET_ACCEL |
                                               SIGNAL_PATH_RESET_TEMP};

inline constexpr uint8_t USER_CTRL_I2C_IF_DIS{0x10};

inline constexpr uint8_t FS_SEL_SHIFT{3};
inline constexpr uint8_t FS_SEL_MASK{0x18};

inline constexpr uint32_t INIT_WAIT_INTERVAL_MS{100};

} // namespace mpu6500

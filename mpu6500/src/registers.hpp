#pragma once

#include <cstdint>

namespace mpu6500::reg {

inline constexpr uint8_t GYRO_CONFIG{0x1B};
inline constexpr uint8_t ACCEL_CONFIG{0x1C};

inline constexpr uint8_t ACCEL_XOUT_H{0x3B};
inline constexpr uint8_t ACCEL_XOUT_L{0x3C};
inline constexpr uint8_t ACCEL_YOUT_H{0x3D};
inline constexpr uint8_t ACCEL_YOUT_L{0x3E};
inline constexpr uint8_t ACCEL_ZOUT_H{0x3F};
inline constexpr uint8_t ACCEL_ZOUT_L{0x40};
inline constexpr uint8_t TEMP_OUT_H{0x41};
inline constexpr uint8_t TEMP_OUT_L{0x42};
inline constexpr uint8_t GYRO_XOUT_H{0x43};
inline constexpr uint8_t GYRO_XOUT_L{0x44};
inline constexpr uint8_t GYRO_YOUT_H{0x45};
inline constexpr uint8_t GYRO_YOUT_L{0x46};
inline constexpr uint8_t GYRO_ZOUT_H{0x47};
inline constexpr uint8_t GYRO_ZOUT_L{0x48};

inline constexpr uint8_t SIGNAL_PATH_RESET{0x68};
inline constexpr uint8_t USER_CTRL{0x6A};
inline constexpr uint8_t PWR_MGMT_1{0x6B};
inline constexpr uint8_t PWR_MGMT_2{0x6C};
inline constexpr uint8_t WHO_AM_I{0x75};

// Filters
inline constexpr uint8_t CONFIG{0x1A};        // Gyro and temperature filters
inline constexpr uint8_t ACCEL_CONFIG2{0x1D}; // Accel filter
inline constexpr uint8_t SMPLRT_DIV{0x19};

inline constexpr uint8_t XG_OFFSET_H{0x13};
inline constexpr uint8_t XG_OFFSET_L{0x14};
inline constexpr uint8_t YG_OFFSET_H{0x15};
inline constexpr uint8_t YG_OFFSET_L{0x16};
inline constexpr uint8_t ZG_OFFSET_H{0x17};
inline constexpr uint8_t ZG_OFFSET_L{0x18};

inline constexpr uint8_t LP_ACCEL_ODR{0x1E};

// FIFO
inline constexpr uint8_t FIFO_EN{0x23};
inline constexpr uint8_t INT_STATUS{0x3A};
inline constexpr uint8_t FIFO_COUNT_H{0x72};
inline constexpr uint8_t FIFO_COUNT_L{0x73};
inline constexpr uint8_t FIFO_R_W{0x74};
} // namespace mpu6500::reg

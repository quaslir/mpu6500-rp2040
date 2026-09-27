#pragma once

#include <cstdint>

namespace mpu6500 {

inline constexpr uint8_t GYRO_CONFIG{0x1B};
inline constexpr uint8_t ACCEL_CONFIG{0x1C};
inline constexpr uint8_t SIGNAL_PATH_RESET{0x68};
inline constexpr uint8_t USER_CTRL{0x6A};
inline constexpr uint8_t PWR_MGMT_1{0x6B};
inline constexpr uint8_t WHOAMI{0x75};
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

inline constexpr uint8_t SAMPLE_BUFFER_SIZE{14};
inline constexpr uint8_t ACC_X_START{0};
inline constexpr uint8_t ACC_Y_START{2};
inline constexpr uint8_t ACC_Z_START{4};
inline constexpr uint8_t TEMP_START{6};
inline constexpr uint8_t GYR_X_START{8};
inline constexpr uint8_t GYR_Y_START{10};
inline constexpr uint8_t GYR_Z_START{12};

inline constexpr float ACCEL_SCALE_G2{16384.0f};
inline constexpr float ACCEL_SCALE_G4{8192.0f};
inline constexpr float ACCEL_SCALE_G8{4096.0f};
inline constexpr float ACCEL_SCALE_G16{2048.0f};

inline constexpr float GYRO_SCALE_DPS250{131.0f};
inline constexpr float GYRO_SCALE_DPS500{65.5f};
inline constexpr float GYRO_SCALE_DPS1000{32.8f};
inline constexpr float GYRO_SCALE_DPS2000{16.4f};

inline constexpr float TEMP_SENSITIVITY{333.87f};
inline constexpr float TEMP_OFFSET_C{21.0f};
} // namespace mpu6500

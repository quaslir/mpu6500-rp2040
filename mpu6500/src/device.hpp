#pragma once

#include <cstdint>

namespace mpu6500::device {

inline constexpr uint8_t EXPECTED_ID{0x70};
inline constexpr uint32_t RESET_WAIT_MS{100};
inline constexpr uint32_t INTERNAL_SAMPLE_RATE_HZ{1000};

inline constexpr uint32_t GYRO_RATE_NO_DLPF_HZ{8000};
inline constexpr uint32_t GYRO_RATE_BYPASS_HZ{32000};
inline constexpr uint32_t ACCEL_RATE_BYPASS_HZ{4000};
inline constexpr uint32_t MIN_DIVIDED_RATE_HZ{4};
inline constexpr uint16_t FIFO_SIZE_BYTES{512}; // MPU 6500 fifo buffer size
} // namespace mpu6500::device

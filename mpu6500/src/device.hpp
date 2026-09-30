#pragma once

#include <cstdint>

namespace mpu6500::device {

inline constexpr uint8_t EXPECTED_ID{0x70};
inline constexpr uint32_t RESET_WAIT_MS{100};
inline constexpr uint32_t INTERNAL_SAMPLE_RATE_HZ{1000};
} // namespace mpu6500::device

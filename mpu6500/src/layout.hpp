#pragma once

#include <cstddef>

namespace mpu6500::layout {

inline constexpr std::size_t BURST_SIZE{14};
inline constexpr std::size_t VEC3_SIZE{6};
inline constexpr std::size_t TEMP_SIZE{2};

inline constexpr std::size_t ACCEL_OFFSET{0};
inline constexpr std::size_t TEMP_OFFSET{6};
inline constexpr std::size_t GYRO_OFFSET{8};

} // namespace mpu6500::layout

#pragma once

#include <cstddef>

namespace mpu6500::layout {

inline constexpr std::size_t BURST_SIZE{14};
inline constexpr std::size_t VEC3_SIZE{6};
inline constexpr std::size_t TEMP_SIZE{2};

inline constexpr std::size_t ACCEL_OFFSET{0};
inline constexpr std::size_t TEMP_OFFSET{6};
inline constexpr std::size_t GYRO_OFFSET{8};

inline constexpr std::size_t GYRO_HW_OFFSET_SIZE{6}; // XG_OFFSET_H .. ZG_OFFSET_L
inline constexpr std::size_t HW_OFFSET_X{0};         // positions inside the 6-byte block
inline constexpr std::size_t HW_OFFSET_Y{2};
inline constexpr std::size_t HW_OFFSET_Z{4};
} // namespace mpu6500::layout

#pragma once
#include "codec/layout.hpp"
#include "math/vec3.hpp"
#include "mpu6500/config.hpp"
#include <cstdint>
#include <span>
namespace detail {
int16_t to_int16(std::span<const uint8_t> data, uint8_t start_pos);
float accel_range_to_scale(mpu6500::config::AccelRange range);
float gyro_range_to_scale(mpu6500::config::GyroRange range);
math::Vec3 decode_vec3(std::span<const uint8_t, mpu6500::layout::VEC3_SIZE> sample_buffer, float scale);
float decode_temperature(std::span<const uint8_t, mpu6500::layout::TEMP_SIZE> sample_buffer);
math::RawVec3 bytes_to_raw_vec3(std::span<const uint8_t, mpu6500::layout::VEC3_SIZE> bytes);
math::Vec3 raw_vec3_to_vec3(const math::RawVec3& raw, float scale);
} // namespace detail

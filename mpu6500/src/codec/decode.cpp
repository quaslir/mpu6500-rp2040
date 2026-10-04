#include "codec/decode.hpp"

#include "codec/layout.hpp"
#include "math/vec3.hpp"
#include "mpu6500/config.hpp"
#include "codec/scales.hpp"
#include <cstdint>
#include <span>
namespace detail {
int16_t to_int16(std::span<const uint8_t> data, uint8_t start_pos) {
    return static_cast<int16_t>((data[start_pos] << 8) | data[start_pos + 1]);
}

float accel_range_to_scale(mpu6500::config::AccelRange range) {
    switch (range) {
        case mpu6500::config::AccelRange::G2:
            return mpu6500::scale::ACCEL_G2;
        case mpu6500::config::AccelRange::G4:
            return mpu6500::scale::ACCEL_G4;
        case mpu6500::config::AccelRange::G8:
            return mpu6500::scale::ACCEL_G8;
        case mpu6500::config::AccelRange::G16:
            return mpu6500::scale::ACCEL_G16;
    }

    return mpu6500::scale::ACCEL_G2;
}

float gyro_range_to_scale(mpu6500::config::GyroRange range) {
    switch (range) {
        case mpu6500::config::GyroRange::Dps250:
            return mpu6500::scale::GYRO_DPS250;
        case mpu6500::config::GyroRange::Dps500:
            return mpu6500::scale::GYRO_DPS500;
        case mpu6500::config::GyroRange::Dps1000:
            return mpu6500::scale::GYRO_DPS1000;
        case mpu6500::config::GyroRange::Dps2000:
            return mpu6500::scale::GYRO_DPS2000;
    }
    return mpu6500::scale::GYRO_DPS250;
}
math::Vec3 decode_vec3(std::span<const uint8_t, mpu6500::layout::VEC3_SIZE> sample_buffer, float scale) {
    const math::RawVec3 raw = bytes_to_raw_vec3(sample_buffer);
    return raw_vec3_to_vec3(raw, scale);
}
float decode_temperature(std::span<const uint8_t, mpu6500::layout::TEMP_SIZE> sample_buffer) {
    const int16_t temp = to_int16(sample_buffer, 0);

    return temp / mpu6500::scale::TEMP_SENSITIVITY + mpu6500::scale::TEMP_REFERENCE_C;
}
math::RawVec3 bytes_to_raw_vec3(std::span<const uint8_t, mpu6500::layout::VEC3_SIZE> bytes) {
    math::RawVec3 to_return{};
    to_return.x = to_int16(bytes, 0);
    to_return.y = to_int16(bytes, 2);
    to_return.z = to_int16(bytes, 4);

    return to_return;
}
math::Vec3 raw_vec3_to_vec3(const math::RawVec3& raw, float scale) {
    math::Vec3 corrected{};
    corrected.x = raw.x / scale;
    corrected.y = raw.y / scale;
    corrected.z = raw.z / scale;

    return corrected;
}
} // namespace detail

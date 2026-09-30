#include "decode.hpp"
#include "layout.hpp"
#include "mpu6500/mpu6500.hpp"
namespace mpu6500 {
Status Mpu6500::read_all_raw(Sample& sample) const {
    std::array<uint8_t, layout::BURST_SIZE> sample_buffer{};
    const Status read_all_result = regs_.read_burst(sample_buffer);
    if (read_all_result != Status::OK)
        return read_all_result;
    // fill acceleration
    const float acc_scale = ::detail::accel_range_to_scale(accel_range_);

    sample.accel_g = ::detail::decode_vec3(
        std::span{sample_buffer}.subspan<layout::ACCEL_OFFSET, layout::VEC3_SIZE>(), acc_scale);
    // fill temperature

    sample.temperature_c = ::detail::decode_temperature(
        std::span{sample_buffer}.subspan<layout::TEMP_OFFSET, layout::TEMP_SIZE>());
    // fill gyro

    const float gyro_scale = ::detail::gyro_range_to_scale(gyro_range_);

    sample.gyro_dps = ::detail::decode_vec3(
        std::span{sample_buffer}.subspan<layout::GYRO_OFFSET, layout::VEC3_SIZE>(), gyro_scale);

    return Status::OK;
}

Status Mpu6500::read_accel_raw(Vec3& sample) const {
    std::array<uint8_t, layout::VEC3_SIZE> sample_buffer{};
    const Status read_accel_result = regs_.read_accel_bytes(sample_buffer);
    if (read_accel_result != Status::OK)
        return read_accel_result;

    const float acc_scale = ::detail::accel_range_to_scale(accel_range_);

    sample = ::detail::decode_vec3(sample_buffer, acc_scale);

    return Status::OK;
}
Status Mpu6500::read_gyro_raw(Vec3& sample) const {
    std::array<uint8_t, layout::VEC3_SIZE> sample_buffer{};
    const Status read_gyro_result = regs_.read_gyro_bytes(sample_buffer);
    if (read_gyro_result != Status::OK)
        return read_gyro_result;

    const float gyro_scale = ::detail::gyro_range_to_scale(gyro_range_);

    sample = ::detail::decode_vec3(sample_buffer, gyro_scale);

    return Status::OK;
}

Status Mpu6500::read_all(Sample& sample) const {
    const Status read_status = read_all_raw(sample);
    if (read_status != Status::OK)
        return read_status;
    sample.accel_g -= accel_offset_;
    sample.gyro_dps -= gyro_offset_;

    return Status::OK;
}
Status Mpu6500::read_accel(Vec3& sample) const {
    const Status read_status = read_accel_raw(sample);
    if (read_status != Status::OK)
        return read_status;
    sample -= accel_offset_;

    return Status::OK;
}
Status Mpu6500::read_gyro(Vec3& sample) const {
    const Status read_status = read_gyro_raw(sample);
    if (read_status != Status::OK)
        return read_status;
    sample -= gyro_offset_;
    return Status::OK;
}

Status Mpu6500::read_temp(float& sample) const {
    std::array<uint8_t, layout::TEMP_SIZE> sample_buffer{};
    const Status read_temp_result = regs_.read_temp_bytes(sample_buffer);
    if (read_temp_result != Status::OK)
        return read_temp_result;

    sample = ::detail::decode_temperature(sample_buffer);
    return Status::OK;
}
} // namespace mpu6500

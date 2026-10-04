#include "bus/status.hpp"
#include "codec/decode.hpp"
#include "codec/layout.hpp"
#include "mpu6500/mpu6500.hpp"
namespace mpu6500 {
Status Mpu6500::read_all_uncorrected(Sample& sample) const {
    std::array<uint8_t, layout::BURST_SIZE> sample_buffer{};
    MPU_RETURN_IF_ERROR(regs_.read_burst(sample_buffer));
    // fill acceleration
    const float acc_scale = ::detail::accel_range_to_scale(config_.measurement.accel.range);

    sample.accel_g = ::detail::decode_vec3(
        std::span{sample_buffer}.subspan<layout::ACCEL_OFFSET, layout::VEC3_SIZE>(), acc_scale);
    // fill temperature

    sample.temperature_c = ::detail::decode_temperature(
        std::span{sample_buffer}.subspan<layout::TEMP_OFFSET, layout::TEMP_SIZE>());
    // fill gyro

    const float gyro_scale = ::detail::gyro_range_to_scale(config_.measurement.gyro.range);

    sample.gyro_dps = ::detail::decode_vec3(
        std::span{sample_buffer}.subspan<layout::GYRO_OFFSET, layout::VEC3_SIZE>(), gyro_scale);

    return Status::OK;
}

Status Mpu6500::read_accel_uncorrected(Vec3& sample) const {
    std::array<uint8_t, layout::VEC3_SIZE> sample_buffer{};
    MPU_RETURN_IF_ERROR(regs_.read_accel_bytes(sample_buffer));

    const float acc_scale = ::detail::accel_range_to_scale(config_.measurement.accel.range);

    sample = ::detail::decode_vec3(sample_buffer, acc_scale);

    return Status::OK;
}
Status Mpu6500::read_gyro_uncorrected(Vec3& sample) const {
    std::array<uint8_t, layout::VEC3_SIZE> sample_buffer{};
    MPU_RETURN_IF_ERROR(regs_.read_gyro_bytes(sample_buffer));

    const float gyro_scale = ::detail::gyro_range_to_scale(config_.measurement.gyro.range);

    sample = ::detail::decode_vec3(sample_buffer, gyro_scale);

    return Status::OK;
}

Status Mpu6500::read_all(Sample& sample) const {
    MPU_RETURN_IF_ERROR(read_all_uncorrected(sample));
    sample.accel_g -= config_.calibration.accel_offset_g;
    sample.gyro_dps -= config_.calibration.gyro_offset_dps;

    return Status::OK;
}
Status Mpu6500::read_accel(Vec3& sample) const {
    MPU_RETURN_IF_ERROR(read_accel_uncorrected(sample));
    sample -= config_.calibration.accel_offset_g;

    return Status::OK;
}
Status Mpu6500::read_gyro(Vec3& sample) const {
    MPU_RETURN_IF_ERROR(read_gyro_uncorrected(sample));
    sample -= config_.calibration.gyro_offset_dps;
    return Status::OK;
}

Status Mpu6500::read_temp(float& sample) const {
    std::array<uint8_t, layout::TEMP_SIZE> sample_buffer{};
    MPU_RETURN_IF_ERROR(regs_.read_temp_bytes(sample_buffer));

    sample = ::detail::decode_temperature(sample_buffer);
    return Status::OK;
}
} // namespace mpu6500

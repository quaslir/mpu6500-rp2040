#include "bus/status.hpp"
#include "mpu6500/mpu6500.hpp"

namespace mpu6500 {
Status Mpu6500::set_accel_range(config::AccelRange range) {
    MPU_RETURN_IF_ERROR(regs_.write_accel_range(range));
    config_.measurement.accel.range = range;
    return Status::OK;
}

Status Mpu6500::set_gyro_range(config::GyroRange range) {
    MPU_RETURN_IF_ERROR(regs_.write_gyro_range(range));
    config_.measurement.gyro.range = range;
    return Status::OK;
}
Status Mpu6500::set_gyro_filter(config::GyroFilter filter) {
    MPU_RETURN_IF_ERROR(regs_.write_gyro_filter(filter));
    config_.measurement.gyro.filter = filter;
    return Status::OK;
}
Status Mpu6500::set_accel_filter(config::AccelFilter filter) {
    MPU_RETURN_IF_ERROR(regs_.write_accel_filter(filter));
    config_.measurement.accel.filter = filter;
    return Status::OK;
}

Status Mpu6500::set_sample_rate_divider(uint8_t divider) {
    MPU_RETURN_IF_ERROR(regs_.write_sample_rate_divider(divider));
    config_.measurement.sample_divider = divider;
    return Status::OK;
}

Status Mpu6500::set_gyro_hw_offset(const RawVec3& offset) {
    MPU_RETURN_IF_ERROR(regs_.write_gyro_hw_offset(offset));

    config_.calibration.gyro_hw_offset = offset;

    return Status::OK;
}
void Mpu6500::set_accel_offset(const Vec3& offset) {
    config_.calibration.accel_offset_g = offset;
}
void Mpu6500::set_gyro_offset(const Vec3& offset) {
    config_.calibration.gyro_offset_dps = offset;
}

void Mpu6500::clear_offsets() {
    config_.calibration.accel_offset_g = Vec3{};
    config_.calibration.gyro_offset_dps = Vec3{};
}
} // namespace mpu6500

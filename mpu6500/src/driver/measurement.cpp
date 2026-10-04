#include "bus/status.hpp"
#include "mpu6500/config.hpp"
#include "mpu6500/mpu6500.hpp"

namespace mpu6500 {
bus::Status Mpu6500::set_accel_range(config::AccelRange range) {
    MPU_RETURN_IF_ERROR(regs_.write_accel_range(range));
    config_.measurement.accel.range = range;
    return bus::Status::OK;
}

bus::Status Mpu6500::set_gyro_range(config::GyroRange range) {
    MPU_RETURN_IF_ERROR(regs_.write_gyro_range(range));
    config_.measurement.gyro.range = range;
    return bus::Status::OK;
}
bus::Status Mpu6500::set_gyro_filter(config::GyroFilter filter) {
    MPU_RETURN_IF_ERROR(regs_.write_gyro_filter(filter));
    config_.measurement.gyro.filter = filter;
    return bus::Status::OK;
}
bus::Status Mpu6500::set_accel_filter(config::AccelFilter filter) {
    config_.measurement.accel.filter = filter;
    if (config_.power.mode == config::PowerMode::LowPowerAccel)
        return bus::Status::OK;
    return regs_.write_accel_filter(filter);
}

bus::Status Mpu6500::set_sample_rate_divider(uint8_t divider) {
    MPU_RETURN_IF_ERROR(regs_.write_sample_rate_divider(divider));
    config_.measurement.sample_divider = divider;
    return bus::Status::OK;
}

bus::Status Mpu6500::set_gyro_hw_offset(const math::RawVec3& offset) {
    MPU_RETURN_IF_ERROR(regs_.write_gyro_hw_offset(offset));

    config_.calibration.gyro_hw_offset = offset;

    return bus::Status::OK;
}
void Mpu6500::set_accel_offset(const math::Vec3& offset) {
    config_.calibration.accel_offset_g = offset;
}
void Mpu6500::set_gyro_offset(const math::Vec3& offset) {
    config_.calibration.gyro_offset_dps = offset;
}

void Mpu6500::clear_offsets() {
    config_.calibration.accel_offset_g = math::Vec3{};
    config_.calibration.gyro_offset_dps = math::Vec3{};
}
} // namespace mpu6500

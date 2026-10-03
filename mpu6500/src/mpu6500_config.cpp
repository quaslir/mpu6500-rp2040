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

} // namespace mpu6500

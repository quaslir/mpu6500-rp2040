#include "bus/status.hpp"
#include "mpu6500/mpu6500.hpp"

namespace mpu6500 {
Status Mpu6500::set_accel_range(config::AccelRange range) {
    MPU_RETURN_IF_ERROR(regs_.write_accel_range(range));
    accel_.range = range;
    return Status::OK;
}

Status Mpu6500::set_gyro_range(config::GyroRange range) {
    MPU_RETURN_IF_ERROR(regs_.write_gyro_range(range));
    gyro_.range = range;
    return Status::OK;
}
Status Mpu6500::set_gyro_filter(config::GyroFilter filter) {
    MPU_RETURN_IF_ERROR(regs_.write_gyro_filter(filter));
    gyro_.filter = filter;
    return Status::OK;
}
Status Mpu6500::set_accel_filter(config::AccelFilter filter) {
    MPU_RETURN_IF_ERROR(regs_.write_accel_filter(filter));
    accel_.filter = filter;
    return Status::OK;
}

Status Mpu6500::set_sample_rate_divider(uint8_t divider) {
    MPU_RETURN_IF_ERROR(regs_.write_sample_rate_divider(divider));
    sample_divider_ = divider;
    return Status::OK;
}

config::AccelRange Mpu6500::accel_range() const {
    return accel_.range;
}

config::GyroRange Mpu6500::gyro_range() const {
    return gyro_.range;
}

config::GyroFilter Mpu6500::gyro_filter() const {
    return gyro_.filter;
}
config::AccelFilter Mpu6500::accel_filter() const {
    return accel_.filter;
}
uint8_t Mpu6500::sample_divider() const {
    return sample_divider_;
}
} // namespace mpu6500

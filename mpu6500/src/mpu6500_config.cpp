#include "mpu6500/mpu6500.hpp"

namespace mpu6500 {
Status Mpu6500::set_accel_range(config::AccelRange range) {
    const Status set_accel_range_result = regs_.write_accel_range(range);
    if (set_accel_range_result != Status::OK)
        return set_accel_range_result;
    accel_range_ = range;
    return Status::OK;
}

Status Mpu6500::set_gyro_range(config::GyroRange range) {
    const Status set_gyro_range_result = regs_.write_gyro_range(range);
    if (set_gyro_range_result != Status::OK)
        return set_gyro_range_result;
    gyro_range_ = range;
    return Status::OK;
}
Status Mpu6500::set_gyro_filter(config::GyroFilter filter) {

    const Status set_gyro_filter_result = regs_.write_gyro_filter(filter);
    if (set_gyro_filter_result != Status::OK)
        return set_gyro_filter_result;
    gyro_filter_ = filter;
    return Status::OK;
}
Status Mpu6500::set_accel_filter(config::AccelFilter filter) {
    const Status set_accel_filter_result = regs_.write_accel_filter(filter);
    if (set_accel_filter_result != Status::OK) {
        return set_accel_filter_result;
    }
    accel_filter_ = filter;
    return Status::OK;
}

Status Mpu6500::set_sample_rate_divider(uint8_t divider) {
    Status set_sample_divider_result = regs_.write_sample_rate_divider(divider);
    if (set_sample_divider_result != Status::OK) {
        return set_sample_divider_result;
    }
    sample_divider_ = divider;
    return Status::OK;
}

config::AccelRange Mpu6500::accel_range() const {
    return accel_range_;
}

config::GyroRange Mpu6500::gyro_range() const {
    return gyro_range_;
}

config::GyroFilter Mpu6500::gyro_filter() const {
    return gyro_filter_;
}
config::AccelFilter Mpu6500::accel_filter() const {
    return accel_filter_;
}
uint8_t Mpu6500::sample_divider() const {
    return sample_divider_;
}
} // namespace mpu6500

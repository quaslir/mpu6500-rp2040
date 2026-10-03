#include "bus/status.hpp"
#include "mpu6500/config.hpp"
#include "mpu6500/mpu6500.hpp"

namespace mpu6500 {
Status Mpu6500::set_sleep(bool enabled) {
    MPU_RETURN_IF_ERROR(regs_.write_sleep(enabled));
    config_.power.sleeping = enabled;
    return Status::OK;
}
Status Mpu6500::set_gyro_standby(bool enabled) {
    MPU_RETURN_IF_ERROR(regs_.write_gyro_standby(enabled));
    config_.power.gyro_standby = enabled;
    return Status::OK;
}
Status Mpu6500::set_temperature_enabled(bool enabled) {
    MPU_RETURN_IF_ERROR(regs_.write_temperature_enabled(enabled));

    config_.power.temperature_enabled = enabled;
    return Status::OK;
}
Status Mpu6500::set_clock_source(config::ClockSource source) {
    MPU_RETURN_IF_ERROR(regs_.write_clock_source(source));
    config_.power.clock_source = source;
    return Status::OK;
}
Status Mpu6500::set_enabled_axes(const config::EnabledAxes& enabled) {
    MPU_RETURN_IF_ERROR(regs_.write_enabled_axes(enabled));
    config_.power.enabled_axes = enabled;
    return Status::OK;
}

Status Mpu6500::enter_low_power_accel(config::LowPowerAccelRate rate) {
    if (low_mode_.active)
        return Status::ERROR;
    low_mode_.backup.enabled_axes = config_.power.enabled_axes;
    low_mode_.backup.temperature_enabled = config_.power.temperature_enabled;
    low_mode_.backup.accel_filter = config_.measurement.accel.filter;
    config::EnabledAxes gyro_disabled_axes = config_.power.enabled_axes;
    gyro_disabled_axes.gyro_x = false;
    gyro_disabled_axes.gyro_y = false;
    gyro_disabled_axes.gyro_z = false;
    MPU_RETURN_IF_ERROR(set_enabled_axes(gyro_disabled_axes));
    MPU_RETURN_IF_ERROR(set_accel_filter(config::AccelFilter::Bypass1130Hz));
    MPU_RETURN_IF_ERROR(set_temperature_enabled(false));
    MPU_RETURN_IF_ERROR(regs_.write_lp_accel_rate(rate));
    MPU_RETURN_IF_ERROR(set_sleep(false));
    MPU_RETURN_IF_ERROR(regs_.write_cycle(true));
    low_mode_.active = true;
    low_mode_.rate = rate;
    return Status::OK;
}

Status Mpu6500::exit_low_power_accel() {
    if (!low_mode_.active)
        return Status::ERROR;
    MPU_RETURN_IF_ERROR(regs_.write_cycle(false));
    MPU_RETURN_IF_ERROR(set_accel_filter(low_mode_.backup.accel_filter));
    MPU_RETURN_IF_ERROR(set_temperature_enabled(low_mode_.backup.temperature_enabled));
    MPU_RETURN_IF_ERROR(set_enabled_axes(low_mode_.backup.enabled_axes));

    low_mode_.active = false;

    return Status::OK;
}

bool Mpu6500::is_low_power() const {
    return low_mode_.active;
}
config::LowPowerAccelRate Mpu6500::low_power_rate() const {
    return low_mode_.rate;
}
} // namespace mpu6500

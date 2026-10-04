#include "bus/status.hpp"
#include "mpu6500/config.hpp"
#include "mpu6500/mpu6500.hpp"

namespace mpu6500 {
Status Mpu6500::set_gyro_standby(bool enabled) {
    config_.power.gyro_standby = enabled;
    return apply_power();
}
Status Mpu6500::set_temperature_enabled(bool enabled) {
    config_.power.temperature_enabled = enabled;

    return apply_power();
}
Status Mpu6500::set_clock_source(config::ClockSource source) {
    MPU_RETURN_IF_ERROR(regs_.write_clock_source(source));
    config_.power.clock_source = source;
    return Status::OK;
}
Status Mpu6500::set_enabled_axes(const config::EnabledAxes& enabled) {
    config_.power.enabled_axes = enabled;
    return apply_power();
}

Status Mpu6500::set_power_mode(config::PowerMode power_mode) {
    config_.power.mode = power_mode;
    return apply_power();
}
Status Mpu6500::set_low_power_rate(config::LowPowerAccelRate rate) {
    config_.power.low_power_rate = rate;
    return apply_power();
}

Status Mpu6500::apply_power() {
    const bool low_power_accel_mode = config_.power.mode == config::PowerMode::LowPowerAccel;
    config::EnabledAxes low_power_accel_mode_axes = config_.power.enabled_axes;
    low_power_accel_mode_axes.gyro_x = false;
    low_power_accel_mode_axes.gyro_y = false;
    low_power_accel_mode_axes.gyro_z = false;
    MPU_RETURN_IF_ERROR(regs_.write_cycle(false));
    MPU_RETURN_IF_ERROR(
        regs_.write_gyro_standby(low_power_accel_mode ? false : config_.power.gyro_standby));
    MPU_RETURN_IF_ERROR(regs_.write_temperature_enabled(
        low_power_accel_mode ? false : config_.power.temperature_enabled));
    MPU_RETURN_IF_ERROR(regs_.write_enabled_axes(
        low_power_accel_mode ? low_power_accel_mode_axes : config_.power.enabled_axes));
    MPU_RETURN_IF_ERROR(regs_.write_accel_filter(low_power_accel_mode
                                                     ? config::AccelFilter::Bypass1130Hz
                                                     : config_.measurement.accel.filter));
    if (low_power_accel_mode) {
        MPU_RETURN_IF_ERROR(regs_.write_lp_accel_rate(config_.power.low_power_rate));
    }
    MPU_RETURN_IF_ERROR(regs_.write_sleep(config_.power.mode == config::PowerMode::Sleep));

    if (low_power_accel_mode) {
        MPU_RETURN_IF_ERROR(regs_.write_cycle(true));
    }

    return Status::OK;
}
} // namespace mpu6500

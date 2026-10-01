#include "mpu6500/config.hpp"
#include "mpu6500/mpu6500.hpp"

namespace mpu6500 {
Status Mpu6500::set_sleep(bool enabled) {
    const Status set_sleep_result = regs_.write_sleep(enabled);

    if (set_sleep_result != Status::OK)
        return set_sleep_result;

    sleeping_ = enabled;
    return Status::OK;
}
Status Mpu6500::set_gyro_standby(bool enabled) {
    const Status set_gyro_standby_result = regs_.write_gyro_standby(enabled);

    if (set_gyro_standby_result != Status::OK)
        return set_gyro_standby_result;

    gyro_standby_ = enabled;
    return Status::OK;
}
Status Mpu6500::set_temperature_enabled(bool enabled) {
    const Status set_temperature_result = regs_.write_temperature_enabled(enabled);

    if (set_temperature_result != Status::OK)
        return set_temperature_result;

    temperature_enabled_ = enabled;
    return Status::OK;
}
Status Mpu6500::set_clock_source(config::ClockSource source) {
    const Status set_clock_result = regs_.write_clock_source(source);

    if (set_clock_result != Status::OK)
        return set_clock_result;

    clock_source_ = source;
    return Status::OK;
}
Status Mpu6500::set_enabled_axes(const config::EnabledAxes& enabled) {

    const Status set_axes_result = regs_.write_enabled_axes(enabled);

    if (set_axes_result != Status::OK)
        return set_axes_result;

    enabled_axes_ = enabled;
    return Status::OK;
}

bool Mpu6500::is_sleeping() const {
    return sleeping_;
}
bool Mpu6500::gyro_standby() const {
    return gyro_standby_;
}
bool Mpu6500::temperature_enabled() const {
    return temperature_enabled_;
}
config::ClockSource Mpu6500::clock_source() const {
    return clock_source_;
}
config::EnabledAxes Mpu6500::enabled_axes() const {
    return enabled_axes_;
}

Status Mpu6500::enter_low_power_accel(config::LowPowerAccelRate rate) {
    if (low_mode_)
        return Status::ERROR;
    low_power_backup_.enabled_axes = enabled_axes_;
    low_power_backup_.temperature_enabled = temperature_enabled_;
    low_power_backup_.accel_filter = accel_filter_;
    config::EnabledAxes gyro_disabled_axes = enabled_axes_;
    gyro_disabled_axes.gyro_x = false;
    gyro_disabled_axes.gyro_y = false;
    gyro_disabled_axes.gyro_z = false;
    const Status set_enabled_axes_result = set_enabled_axes(gyro_disabled_axes);
    if (set_enabled_axes_result != Status::OK)
        return set_enabled_axes_result;
    const Status set_accel_filter_result = set_accel_filter(config::AccelFilter::Bypass1130Hz);
    if (set_accel_filter_result != Status::OK)
        return set_accel_filter_result;

    const Status set_temp_enabled_result = set_temperature_enabled(false);
    if (set_temp_enabled_result != Status::OK)
        return set_temp_enabled_result;

    const Status set_lp_accel_rate_result = regs_.write_lp_accel_rate(rate);
    if (set_lp_accel_rate_result != Status::OK)
        return set_lp_accel_rate_result;

    const Status set_sleep_result = set_sleep(false);
    if (set_sleep_result != Status::OK)
        return set_sleep_result;

    const Status write_cycle_result = regs_.write_cycle(true);
    if (write_cycle_result != Status::OK)
        return write_cycle_result;
    low_mode_ = true;
    low_power_rate_ = rate;
    return Status::OK;
}

Status Mpu6500::exit_low_power_accel() {
    if (!low_mode_)
        return Status::ERROR;

    const Status write_cycle_result = regs_.write_cycle(false);
    if (write_cycle_result != Status::OK)
        return write_cycle_result;
    const Status set_accel_filter_result = set_accel_filter(low_power_backup_.accel_filter);
    if (set_accel_filter_result != Status::OK)
        return set_accel_filter_result;

    const Status set_temperature_status =
        set_temperature_enabled(low_power_backup_.temperature_enabled);
    if (set_temperature_status != Status::OK)
        return set_temperature_status;

    const Status set_enabled_axes_result = set_enabled_axes(low_power_backup_.enabled_axes);
    if (set_enabled_axes_result != Status::OK)
        return set_enabled_axes_result;

    low_mode_ = false;

    return Status::OK;
}

bool Mpu6500::is_low_power() const {
    return low_mode_;
}
config::LowPowerAccelRate Mpu6500::low_power_rate() const {
    return low_power_rate_;
}
} // namespace mpu6500

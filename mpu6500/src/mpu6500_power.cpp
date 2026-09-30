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
} // namespace mpu6500

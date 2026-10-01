#include "mpu6500/mpu6500.hpp"

#include "bus/bus.hpp"
#include "bus/status.hpp"
#include "device.hpp"
#include "mpu6500/config.hpp"
#include "registers.hpp"
#include <cmath>
#include <cstdint>

namespace {
float low_power_rate_to_hz(mpu6500::config::LowPowerAccelRate rate) {
    using mpu6500::config::LowPowerAccelRate;
    switch (rate) {
        case LowPowerAccelRate::Hz0_24:
            return 0.24f;
        case LowPowerAccelRate::Hz0_49:
            return 0.49f;
        case LowPowerAccelRate::Hz0_98:
            return 0.98f;
        case LowPowerAccelRate::Hz1_95:
            return 1.95f;
        case LowPowerAccelRate::Hz3_91:
            return 3.91f;
        case LowPowerAccelRate::Hz7_81:
            return 7.81f;
        case LowPowerAccelRate::Hz15_63:
            return 15.63f;
        case LowPowerAccelRate::Hz31_25:
            return 31.25f;
        case LowPowerAccelRate::Hz62_5:
            return 62.5f;
        case LowPowerAccelRate::Hz125:
            return 125.0f;
        case LowPowerAccelRate::Hz250:
            return 250.0f;
        case LowPowerAccelRate::Hz500:
            return 500.0f;
    }
    return 0.24f;
}
} // namespace
namespace mpu6500 {
Mpu6500::Mpu6500(bus::Bus& bus, WaitFunction wait, const config::Config& config)
    : regs_(bus), wait_(wait), use_i2c_(config.use_i2c),
      accel_range_(config.starting_accelerometer_range),
      gyro_range_(config.starting_gyroscope_range), accel_filter_(config.starting_accel_filter),
      gyro_filter_(config.starting_gyro_filter), sample_divider_(config.starting_sample_divider),
      accel_offset_(config.starting_accel_offset), gyro_offset_(config.starting_gyro_offset),
      sleeping_(false), clock_source_(config.starting_clock_source),
      enabled_axes_(config.starting_enabled_axes),
      temperature_enabled_(config.starting_temperature_enabled),
      gyro_standby_(config.starting_gyro_standby), gyro_hw_offset_(config.starting_gyro_hw_offset),
      low_mode_(false), low_power_rate_(config::LowPowerAccelRate::Hz0_24), low_power_backup_({}) {}

Status Mpu6500::who_am_i(uint8_t& id) {
    return regs_.read_who_am_i(id);
}

Status Mpu6500::init() {

    uint8_t id{};
    const Status who_am_i_result = who_am_i(id);
    if (who_am_i_result != Status::OK)
        return who_am_i_result;
    if (id != device::EXPECTED_ID)
        return Status::ERROR;
    const Status reset_result = regs_.device_reset();
    if (reset_result != Status::OK)
        return reset_result;
    wait_(device::RESET_WAIT_MS);

    if (low_mode_) {
        accel_filter_ = low_power_backup_.accel_filter;
        temperature_enabled_ = low_power_backup_.temperature_enabled;
        enabled_axes_ = low_power_backup_.enabled_axes;

        low_mode_ = false;
    }

    const Status signal_path_reset_result = regs_.signal_path_reset();
    if (signal_path_reset_result != Status::OK)
        return signal_path_reset_result;
    wait_(device::RESET_WAIT_MS);

    if (!use_i2c_) { // disable I2C if config was stated that SPI is used. If user uses I2C, NACK
                     // will be a result of following writing.
        const Status user_ctrl_result = regs_.disable_i2c_interface();
        if (user_ctrl_result != Status::OK)
            return user_ctrl_result;
    }

    const Status normal_mode_result = regs_.write_power_normal();
    if (normal_mode_result != Status::OK)
        return normal_mode_result;

    const Status set_sleeping_result = set_sleep(sleeping_);
    if (set_sleeping_result != Status::OK)
        return set_sleeping_result;

    const Status set_clock_source_result = set_clock_source(clock_source_);
    if (set_clock_source_result != Status::OK)
        return set_clock_source_result;

    const Status set_temperature_enabled_result = set_temperature_enabled(temperature_enabled_);
    if (set_temperature_enabled_result != Status::OK)
        return set_temperature_enabled_result;

    const Status set_gyro_standby_result = set_gyro_standby(gyro_standby_);
    if (set_gyro_standby_result != Status::OK)
        return set_gyro_standby_result;

    const Status set_enabled_axes_result = set_enabled_axes(enabled_axes_);
    if (set_enabled_axes_result != Status::OK)
        return set_enabled_axes_result;

    const Status set_accel_range_result = set_accel_range(accel_range_);
    if (set_accel_range_result != Status::OK)
        return set_accel_range_result;

    const Status set_gyro_range_result = set_gyro_range(gyro_range_);
    if (set_gyro_range_result != Status::OK) {
        return set_gyro_range_result;
    }

    const Status set_accel_filter_result = set_accel_filter(accel_filter_);
    if (set_accel_filter_result != Status::OK)
        return set_accel_filter_result;

    const Status set_gyro_filter_result = set_gyro_filter(gyro_filter_);
    if (set_gyro_filter_result != Status::OK)
        return set_gyro_filter_result;

    const Status set_sample_divider_result = set_sample_rate_divider(sample_divider_);
    if (set_sample_divider_result != Status::OK)
        return set_sample_divider_result;

    const Status set_gyro_hw_offset_result = set_gyro_hw_offset(gyro_hw_offset_);
    if (set_gyro_hw_offset_result != Status::OK)
        return set_gyro_hw_offset_result;

    return Status::OK;
}

Status Mpu6500::reset_signal_paths(bool gyro, bool accel, bool temp) {
    return regs_.write_signal_path_reset(gyro, accel, temp);
}
Status Mpu6500::reset_sensor_registers() {
    return regs_.write_sensor_reset();
}

float Mpu6500::gyro_sample_rate_hz() const {
    if (low_mode_)
        return 0.0f;
    switch (gyro_filter_) {
        case config::GyroFilter::Bypass3600Hz:
        case config::GyroFilter::Bypass8800Hz:
            return device::GYRO_RATE_BYPASS_HZ;
        case config::GyroFilter::Hz250:
        case config::GyroFilter::Hz3600:
            return device::GYRO_RATE_NO_DLPF_HZ;
        default:
            return static_cast<float>(device::INTERNAL_SAMPLE_RATE_HZ) /
                   static_cast<float>((1 + sample_divider_));
    }
}
float Mpu6500::accel_sample_rate_hz() const {
    if (low_mode_) {
        return low_power_rate_to_hz(low_power_rate_);
    } else {
        switch (accel_filter_) {
            case config::AccelFilter::Bypass1130Hz:
                return device::ACCEL_RATE_BYPASS_HZ;
            default:
                return static_cast<float>(device::INTERNAL_SAMPLE_RATE_HZ) /
                       static_cast<float>((1 + sample_divider_));
        }
    }
}
bool Mpu6500::divider_effective() const {
    switch (gyro_filter_) {
        case config::GyroFilter::Hz250:
        case config::GyroFilter::Hz3600:
        case config::GyroFilter::Bypass3600Hz:
        case config::GyroFilter::Bypass8800Hz:
            return false;
        default:
            return true;
    }
}

Status Mpu6500::set_sample_rate_hz(uint16_t hz) {
    if (!divider_effective())
        return Status::ERROR;
    if (hz < device::MIN_DIVIDED_RATE_HZ || hz > device::INTERNAL_SAMPLE_RATE_HZ)
        return Status::ERROR;
    uint8_t divider = static_cast<uint8_t>(
        std::lround(static_cast<float>(device::INTERNAL_SAMPLE_RATE_HZ) / static_cast<float>(hz)) -
        1);
    return set_sample_rate_divider(divider);
}
} // namespace mpu6500

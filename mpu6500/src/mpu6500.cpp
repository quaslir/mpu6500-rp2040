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
    : regs_(bus), wait_(wait), use_i2c_(config.use_i2c), accel_(config.starting_accel_params),
      gyro_(config.starting_gyro_params), sample_divider_(config.starting_sample_divider),
      sleeping_(false), clock_source_(config.starting_clock_source),
      enabled_axes_(config.starting_enabled_axes),
      temperature_enabled_(config.starting_temperature_enabled), low_mode_(config::LowPowerMode{}) {
}

Status Mpu6500::who_am_i(uint8_t& id) {
    return regs_.read_who_am_i(id);
}

Status Mpu6500::init() {

    uint8_t id{};
    MPU_RETURN_IF_ERROR(who_am_i(id));
    if (id != device::EXPECTED_ID)
        return Status::ERROR;
    MPU_RETURN_IF_ERROR(regs_.device_reset());
    wait_(device::RESET_WAIT_MS);

    if (low_mode_.active) {
        accel_.filter = low_mode_.backup.accel_filter;
        temperature_enabled_ = low_mode_.backup.temperature_enabled;
        enabled_axes_ = low_mode_.backup.enabled_axes;

        low_mode_.active = false;
    }
    MPU_RETURN_IF_ERROR(regs_.signal_path_reset());
    wait_(device::RESET_WAIT_MS);

    if (!use_i2c_) { // disable I2C if config was stated that SPI is used. If user uses I2C, NACK
                     // will be a result of following writing.

        MPU_RETURN_IF_ERROR(regs_.disable_i2c_interface());
    }
    MPU_RETURN_IF_ERROR(regs_.write_power_normal());

    MPU_RETURN_IF_ERROR(set_sleep(sleeping_));

    MPU_RETURN_IF_ERROR(set_clock_source(clock_source_));

    MPU_RETURN_IF_ERROR(set_temperature_enabled(temperature_enabled_));

    MPU_RETURN_IF_ERROR(set_gyro_standby(gyro_.standby));

    MPU_RETURN_IF_ERROR(set_enabled_axes(enabled_axes_));

    MPU_RETURN_IF_ERROR(set_accel_range(accel_.range));

    MPU_RETURN_IF_ERROR(set_gyro_range(gyro_.range));

    MPU_RETURN_IF_ERROR(set_accel_filter(accel_.filter));

    MPU_RETURN_IF_ERROR(set_gyro_filter(gyro_.filter));

    MPU_RETURN_IF_ERROR(set_sample_rate_divider(sample_divider_));

    MPU_RETURN_IF_ERROR(set_gyro_hw_offset(gyro_.hw_offset));
    return Status::OK;
}

Status Mpu6500::reset_signal_paths(bool gyro, bool accel, bool temp) {
    return regs_.write_signal_path_reset(gyro, accel, temp);
}
Status Mpu6500::reset_sensor_registers() {
    return regs_.write_sensor_reset();
}

float Mpu6500::gyro_sample_rate_hz() const {
    if (low_mode_.active)
        return 0.0f;
    switch (gyro_.filter) {
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
    if (low_mode_.active) {
        return low_power_rate_to_hz(low_mode_.rate);
    } else {
        switch (accel_.filter) {
            case config::AccelFilter::Bypass1130Hz:
                return device::ACCEL_RATE_BYPASS_HZ;
            default:
                return static_cast<float>(device::INTERNAL_SAMPLE_RATE_HZ) /
                       static_cast<float>((1 + sample_divider_));
        }
    }
}
bool Mpu6500::divider_effective() const {
    switch (gyro_.filter) {
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

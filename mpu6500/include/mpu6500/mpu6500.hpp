#pragma once

#include "bus/bus.hpp"
#include "bus/status.hpp"
#include "mpu6500/calibration.hpp"
#include "mpu6500/config.hpp"
#include "mpu6500/detail/mpu6500_registers.hpp"
#include "mpu6500/sample.hpp"
#include "mpu6500/vec3.hpp"
#include <cstdint>
namespace mpu6500 {

using WaitFunction = void (*)(uint32_t delay_ms);
class Mpu6500 {
public:
    Mpu6500(bus::Bus& bus, WaitFunction, const config::Config& config);
    Mpu6500(const Mpu6500&) = delete;
    Mpu6500& operator=(const Mpu6500&) = delete;

    [[nodiscard]] Status init();
    [[nodiscard]] Status who_am_i(uint8_t& id);
    [[nodiscard]] Status set_accel_range(config::AccelRange range);
    [[nodiscard]] Status set_gyro_range(config::GyroRange range);
    [[nodiscard]] config::AccelRange accel_range() const;
    [[nodiscard]] config::GyroRange gyro_range() const;
    // Measurement
    [[nodiscard]] Status read_all(Sample& sample) const;

    [[nodiscard]] Status read_accel(Vec3& sample) const;
    [[nodiscard]] Status read_gyro(Vec3& sample) const;
    [[nodiscard]] Status read_temp(float& sample) const;

    [[nodiscard]] Status read_all_raw(Sample& sample) const;
    [[nodiscard]] Status read_accel_raw(Vec3& sample) const;
    [[nodiscard]] Status read_gyro_raw(Vec3& sample) const;

    // Filters
    [[nodiscard]] Status set_gyro_filter(config::GyroFilter filter);
    [[nodiscard]] Status set_accel_filter(config::AccelFilter filter);
    [[nodiscard]] Status set_sample_rate_divider(uint8_t divider);

    [[nodiscard]] config::GyroFilter gyro_filter() const;
    [[nodiscard]] config::AccelFilter accel_filter() const;
    [[nodiscard]] uint8_t sample_divider() const;

    // Offset
    void set_accel_offset(const Vec3& offset);
    void set_gyro_offset(const Vec3& offset);

    [[nodiscard]] const Vec3& accel_offset() const;
    [[nodiscard]] const Vec3& gyro_offset() const;
    void clear_offsets();

    // Calibration
    [[nodiscard]] Status measure_mean(calibration::Sensor sensor,
                                      Vec3& mean,
                                      const calibration::MeasureOptions& options) const;
    [[nodiscard]] Status measure_gyro_offset(
        Vec3& offset,
        const calibration::MeasureOptions& options = calibration::DEFAULT_GYRO_OPTIONS) const;
    [[nodiscard]] Status
    calibrate_gyro(const calibration::MeasureOptions& options = calibration::DEFAULT_GYRO_OPTIONS);

    [[nodiscard]] Status measure_accel_offset(
        Vec3& offset,
        const Vec3& expected_gravity_g = calibration::GRAVITY_Z_UP,
        const calibration::MeasureOptions& options = calibration::DEFAULT_ACCEL_OPTIONS) const;
    [[nodiscard]] Status calibrate_accel(
        const Vec3& expected_gravity_g = calibration::GRAVITY_Z_UP,
        const calibration::MeasureOptions& options = calibration::DEFAULT_ACCEL_OPTIONS);
    [[nodiscard]] Status set_sleep(bool enabled);
    [[nodiscard]] Status set_gyro_standby(bool enabled);
    [[nodiscard]] Status set_temperature_enabled(bool enabled);
    [[nodiscard]] Status set_clock_source(config::ClockSource source);
    [[nodiscard]] Status set_enabled_axes(const config::EnabledAxes& enabled);

    [[nodiscard]] bool is_sleeping() const;
    [[nodiscard]] bool gyro_standby() const;
    [[nodiscard]] bool temperature_enabled() const;
    [[nodiscard]] config::ClockSource clock_source() const;
    [[nodiscard]] config::EnabledAxes enabled_axes() const;

    [[nodiscard]] Status reset_signal_paths(bool gyro = true, bool accel = true, bool temp = true);
    [[nodiscard]] Status reset_sensor_registers();

    [[nodiscard]] float gyro_sample_rate_hz() const;
    [[nodiscard]] float accel_sample_rate_hz() const;
    [[nodiscard]] bool divider_effective() const;
    [[nodiscard]] Status set_sample_rate_hz(uint16_t sample_rate);

    [[nodiscard]] Status set_gyro_hw_offset(const RawVec3& offset);
    [[nodiscard]] RawVec3 gyro_hw_offset() const;

private:
    using ReadVec3Fn = Status (Mpu6500::*)(Vec3&) const;
    [[nodiscard]] Status measure_mean_impl(ReadVec3Fn func,
                                           Vec3& mean,
                                           const calibration::MeasureOptions& options,
                                           uint32_t period_ms) const;
    detail::Mpu6500Regs regs_;
    WaitFunction wait_;
    bool use_i2c_;
    config::AccelRange accel_range_;
    config::GyroRange gyro_range_;
    config::AccelFilter accel_filter_;
    config::GyroFilter gyro_filter_;
    uint8_t sample_divider_;
    Vec3 accel_offset_;
    Vec3 gyro_offset_;
    bool sleeping_;
    config::ClockSource clock_source_;
    config::EnabledAxes enabled_axes_;
    bool temperature_enabled_;
    bool gyro_standby_;
    RawVec3 gyro_hw_offset_;
};
} // namespace mpu6500

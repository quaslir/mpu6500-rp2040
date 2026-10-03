#pragma once

#include "bus/bus.hpp"
#include "bus/status.hpp"
#include "math/vec3.hpp"
#include "mpu6500/calibration.hpp"
#include "mpu6500/config.hpp"
#include "mpu6500/detail/mpu6500_registers.hpp"
#include "mpu6500/sample.hpp"
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

    [[nodiscard]] const config::Config& config() const;

    [[nodiscard]] Status set_accel_range(config::AccelRange range);
    [[nodiscard]] Status set_gyro_range(config::GyroRange range);
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

    // Offset
    void set_accel_offset(const Vec3& offset);
    void set_gyro_offset(const Vec3& offset);

    void clear_offsets();

    // Calibration

    [[nodiscard]] Status set_sleep(bool enabled);
    [[nodiscard]] Status set_gyro_standby(bool enabled);
    [[nodiscard]] Status set_temperature_enabled(bool enabled);
    [[nodiscard]] Status set_clock_source(config::ClockSource source);
    [[nodiscard]] Status set_enabled_axes(const config::EnabledAxes& enabled);

    [[nodiscard]] Status reset_signal_paths(bool gyro = true, bool accel = true, bool temp = true);
    [[nodiscard]] Status reset_sensor_registers();

    [[nodiscard]] float gyro_sample_rate_hz() const;
    [[nodiscard]] float accel_sample_rate_hz() const;
    [[nodiscard]] bool divider_effective() const;
    [[nodiscard]] Status set_sample_rate_hz(uint16_t sample_rate);

    [[nodiscard]] Status set_gyro_hw_offset(const RawVec3& offset);
    [[nodiscard]] Status enter_low_power_accel(config::LowPowerAccelRate rate);
    [[nodiscard]] Status exit_low_power_accel();
    [[nodiscard]] bool is_low_power() const;
    [[nodiscard]] config::LowPowerAccelRate low_power_rate() const;
    [[nodiscard]] WaitFunction wait() const;

private:
    [[nodiscard]] Status apply_config();
    detail::Mpu6500Regs regs_;
    WaitFunction wait_;
    config::Config config_;
    config::LowPowerMode low_mode_;
};
} // namespace mpu6500

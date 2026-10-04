#pragma once

#include "bus/bus.hpp"
#include "bus/status.hpp"
#include "math/vec3.hpp"
#include "mpu6500/calibration.hpp"
#include "mpu6500/config.hpp"
#include "mpu6500/detail/mpu6500_registers.hpp"
#include "mpu6500/sample.hpp"
#include <cstdint>
#include <pico/types.h>
namespace mpu6500 {

using WaitFunction = void (*)(uint32_t delay_ms);
class Mpu6500 {
public:
    Mpu6500(bus::Bus& bus, WaitFunction wait, const config::Config& config);
    Mpu6500(const Mpu6500&) = delete;
    Mpu6500& operator=(const Mpu6500&) = delete;

    [[nodiscard]] bus::Status init();
    [[nodiscard]] bus::Status who_am_i(uint8_t& id);

    [[nodiscard]] const config::Config& config() const;

    [[nodiscard]] bus::Status set_accel_range(config::AccelRange range);
    [[nodiscard]] bus::Status set_gyro_range(config::GyroRange range);
    // Measurement
    [[nodiscard]] bus::Status read_all(Sample& sample) const;

    [[nodiscard]] bus::Status read_accel(Vec3& sample) const;
    [[nodiscard]] bus::Status read_gyro(Vec3& sample) const;
    [[nodiscard]] bus::Status read_temp(float& sample) const;

    [[nodiscard]] bus::Status read_all_uncorrected(Sample& sample) const;
    [[nodiscard]] bus::Status read_accel_uncorrected(Vec3& sample) const;
    [[nodiscard]] bus::Status read_gyro_uncorrected(Vec3& sample) const;

    // Filters
    [[nodiscard]] bus::Status set_gyro_filter(config::GyroFilter filter);
    [[nodiscard]] bus::Status set_accel_filter(config::AccelFilter filter);
    [[nodiscard]] bus::Status set_sample_rate_divider(uint8_t divider);

    // Offset
    void set_accel_offset(const Vec3& offset);
    void set_gyro_offset(const Vec3& offset);

    void clear_offsets();

    // Calibration

    [[nodiscard]] bus::Status set_power_mode(config::PowerMode power_mode);
    [[nodiscard]] bus::Status set_low_power_rate(config::LowPowerAccelRate rate);
    [[nodiscard]] bus::Status set_gyro_standby(bool enabled);
    [[nodiscard]] bus::Status set_temperature_enabled(bool enabled);
    [[nodiscard]] bus::Status set_clock_source(config::ClockSource source);
    [[nodiscard]] bus::Status set_enabled_axes(const config::EnabledAxes& enabled);

    [[nodiscard]] bus::Status reset_signal_paths(bool gyro = true, bool accel = true, bool temp = true);
    [[nodiscard]] bus::Status reset_sensor_registers();

    [[nodiscard]] float gyro_sample_rate_hz() const;
    [[nodiscard]] float accel_sample_rate_hz() const;
    [[nodiscard]] bool divider_effective() const;
    [[nodiscard]] bus::Status set_sample_rate_hz(uint16_t sample_rate);

    [[nodiscard]] bus::Status set_gyro_hw_offset(const RawVec3& offset);
    [[nodiscard]] WaitFunction wait() const;

    // FIFO

    [[nodiscard]] bus::Status fifo_reset();
    [[nodiscard]] bus::Status fifo_frame_count(uint16_t& count) const;
    [[nodiscard]] bus::Status read_fifo(std::span<Sample> samples, config::FifoReadResult& fifo_result);

    // INTERRUPTS
    [[nodiscard]] bus::Status take_interrupt_flags(InterruptFlags& flags);


    // Wake on motion

    [[nodiscard]] bus::Status set_wake_on_motion(bool enabled);
    [[nodiscard]] bus::Status set_wom_threshold(uint16_t threshold);
private:
    [[nodiscard]] bus::Status apply_config();
    [[nodiscard]] bus::Status poll_int_status();
    [[nodiscard]] bus::Status apply_power();
    detail::Mpu6500Regs regs_;
    WaitFunction wait_;
    config::Config config_;

    uint8_t pending_int_flags_;
};
} // namespace mpu6500

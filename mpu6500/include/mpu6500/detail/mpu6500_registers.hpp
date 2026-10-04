#pragma once

#include "bus/bus.hpp"
#include "bus/status.hpp"
#include "math/vec3.hpp"
#include "mpu6500/config.hpp"
#include <cstdint>
#include <span>
namespace mpu6500::detail {
class Mpu6500Regs {
public:
    explicit Mpu6500Regs(bus::Bus& bus);
    Mpu6500Regs(const Mpu6500Regs&) = delete;
    Mpu6500Regs& operator=(const Mpu6500Regs&) = delete;

    [[nodiscard]] bus::Status read_who_am_i(uint8_t& id) const;
    [[nodiscard]] bus::Status device_reset();
    [[nodiscard]] bus::Status signal_path_reset();
    [[nodiscard]] bus::Status disable_i2c_interface();
    [[nodiscard]] bus::Status write_power_normal();

    [[nodiscard]] bus::Status write_accel_range(config::AccelRange range);
    [[nodiscard]] bus::Status write_gyro_range(config::GyroRange range);
    [[nodiscard]] bus::Status write_accel_filter(config::AccelFilter filter);
    [[nodiscard]] bus::Status write_gyro_filter(config::GyroFilter filter);
    [[nodiscard]] bus::Status write_sample_rate_divider(uint8_t divider);

    [[nodiscard]] bus::Status write_sleep(bool enabled);
    [[nodiscard]] bus::Status write_gyro_standby(bool enabled);
    [[nodiscard]] bus::Status write_temperature_enabled(bool enabled);
    [[nodiscard]] bus::Status write_clock_source(config::ClockSource source);
    [[nodiscard]] bus::Status write_enabled_axes(const config::EnabledAxes& enabled);

    [[nodiscard]] bus::Status read_burst(std::span<uint8_t, 14> buffer) const;
    [[nodiscard]] bus::Status read_accel_bytes(std::span<uint8_t, 6> buffer) const;
    [[nodiscard]] bus::Status read_gyro_bytes(std::span<uint8_t, 6> buffer) const;
    [[nodiscard]] bus::Status read_temp_bytes(std::span<uint8_t, 2> buffer) const;

    [[nodiscard]] bus::Status write_signal_path_reset(bool gyro, bool accel, bool temp);
    [[nodiscard]] bus::Status write_sensor_reset();
    [[nodiscard]] bus::Status write_gyro_hw_offset(const math::RawVec3& offset);

    [[nodiscard]] bus::Status write_lp_accel_rate(config::LowPowerAccelRate rate_hz);
    [[nodiscard]] bus::Status write_cycle(bool enabled);

    // FIFO
    [[nodiscard]] bus::Status write_fifo_sources(const config::FifoSources& sources);
    [[nodiscard]] bus::Status write_fifo_mode(const config::FifoMode& mode);
    [[nodiscard]] bus::Status write_fifo_enabled(bool enabled);
    [[nodiscard]] bus::Status fifo_reset();
    [[nodiscard]] bus::Status read_fifo_count(uint16_t& count) const;
    [[nodiscard]] bus::Status read_fifo_bytes(std::span<uint8_t> buffer) const;
    [[nodiscard]] bus::Status read_int_status(uint8_t& status) const;
    // INTERRUPTS
    [[nodiscard]] bus::Status write_int_pin_config(const config::Interrupts& interrupts);
    [[nodiscard]] bus::Status write_int_sources(const config::InterruptSources& sources);

    // Wake on motion
    [[nodiscard]] bus::Status write_wom_threshold(uint16_t threshold);
    [[nodiscard]] bus::Status write_accel_intel(bool enabled);

private:
    [[nodiscard]] bus::Status update_bits(uint8_t reg, uint8_t mask, uint8_t data);
    [[nodiscard]] bus::Status write_int16(uint8_t high_reg, uint8_t low_reg, int16_t data);
    bus::Bus& bus_;
};
} // namespace mpu6500::detail

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

    [[nodiscard]] Status read_who_am_i(uint8_t& id) const;
    [[nodiscard]] Status device_reset();
    [[nodiscard]] Status signal_path_reset();
    [[nodiscard]] Status disable_i2c_interface();
    [[nodiscard]] Status write_power_normal();

    [[nodiscard]] Status write_accel_range(config::AccelRange range);
    [[nodiscard]] Status write_gyro_range(config::GyroRange range);
    [[nodiscard]] Status write_accel_filter(config::AccelFilter filter);
    [[nodiscard]] Status write_gyro_filter(config::GyroFilter filter);
    [[nodiscard]] Status write_sample_rate_divider(uint8_t divider);

    [[nodiscard]] Status write_sleep(bool enabled);
    [[nodiscard]] Status write_gyro_standby(bool enabled);
    [[nodiscard]] Status write_temperature_enabled(bool enabled);
    [[nodiscard]] Status write_clock_source(config::ClockSource source);
    [[nodiscard]] Status write_enabled_axes(const config::EnabledAxes& enabled);

    [[nodiscard]] Status read_burst(std::span<uint8_t, 14> buffer) const;
    [[nodiscard]] Status read_accel_bytes(std::span<uint8_t, 6> buffer) const;
    [[nodiscard]] Status read_gyro_bytes(std::span<uint8_t, 6> buffer) const;
    [[nodiscard]] Status read_temp_bytes(std::span<uint8_t, 2> buffer) const;

    [[nodiscard]] Status write_signal_path_reset(bool gyro, bool accel, bool temp);
    [[nodiscard]] Status write_sensor_reset();
    [[nodiscard]] Status write_gyro_hw_offset(const RawVec3& offset);

    [[nodiscard]] Status write_lp_accel_rate(config::LowPowerAccelRate rate_hz);
    [[nodiscard]] Status write_cycle(bool enabled);

    // FIFO
    [[nodiscard]] Status write_fifo_sources(const config::FifoSources& sources);
    [[nodiscard]] Status write_fifo_mode(const config::FifoMode& mode);
    [[nodiscard]] Status write_fifo_enabled(bool enabled);
    [[nodiscard]] Status fifo_reset();
    [[nodiscard]] Status read_fifo_count(uint16_t& count) const;
    [[nodiscard]] Status read_fifo_bytes(std::span<uint8_t> buffer) const;
    [[nodiscard]] Status read_int_status(uint8_t& status) const;
    // INTERRUPTS
    [[nodiscard]] Status write_int_pin_config(const config::Interrupts& interrupts);
    [[nodiscard]] Status write_int_sources(const config::InterruptSources& sources);

private:
    [[nodiscard]] Status update_bits(uint8_t reg, uint8_t mask, uint8_t data);
    [[nodiscard]] Status write_int16(uint8_t high_reg, uint8_t low_reg, int16_t data);
    bus::Bus& bus_;
};
} // namespace mpu6500::detail

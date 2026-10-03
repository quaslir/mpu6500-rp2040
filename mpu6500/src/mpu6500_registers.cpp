#include "mpu6500/detail/mpu6500_registers.hpp"

#include "bits.hpp"
#include "bus/bus.hpp"
#include "bus/status.hpp"
#include "mpu6500/config.hpp"
#include "registers.hpp"
#include <array>
#include <cstdint>
#include <span>
namespace mpu6500::detail {

Mpu6500Regs::Mpu6500Regs(bus::Bus& bus) : bus_(bus) {}

Status Mpu6500Regs::update_bits(uint8_t reg, uint8_t mask, uint8_t data) {
    uint8_t current{};
    MPU_RETURN_IF_ERROR(bus_.read_regs(reg, std::span<uint8_t>(&current, 1)));
    const uint8_t updated = static_cast<uint8_t>((current & ~mask) | (data & mask));
    return bus_.write_reg(reg, updated);
}

Status Mpu6500Regs::read_who_am_i(uint8_t& id) const {
    return bus_.read_regs(reg::WHO_AM_I, std::span<uint8_t>(&id, 1));
}
Status Mpu6500Regs::device_reset() {
    return bus_.write_reg(reg::PWR_MGMT_1, bits::pwr_mgmt_1::DEVICE_RESET);
}
Status Mpu6500Regs::signal_path_reset() {
    return bus_.write_reg(reg::SIGNAL_PATH_RESET, bits::signal_path_reset::ALL);
}
Status Mpu6500Regs::disable_i2c_interface() {
    return update_bits(reg::USER_CTRL, bits::user_ctrl::I2C_IF_DIS, bits::user_ctrl::I2C_IF_DIS);
}
Status Mpu6500Regs::write_power_normal() {
    return bus_.write_reg(reg::PWR_MGMT_1, bits::pwr_mgmt_1::NORMAL);
}

Status Mpu6500Regs::write_accel_range(config::AccelRange range) {
    return update_bits(
        reg::ACCEL_CONFIG, bits::fs_sel::MASK, static_cast<uint8_t>(range) << bits::fs_sel::SHIFT);
}
Status Mpu6500Regs::write_gyro_range(config::GyroRange range) {
    return update_bits(
        reg::GYRO_CONFIG, bits::fs_sel::MASK, static_cast<uint8_t>(range) << bits::fs_sel::SHIFT);
}
Status Mpu6500Regs::write_accel_filter(config::AccelFilter filter) {
    return update_bits(reg::ACCEL_CONFIG2,
                       bits::accel_config2::A_DLPF_CFG_MASK | bits::accel_config2::ACCEL_FCHOICE_B,
                       static_cast<uint8_t>(filter));
}
Status Mpu6500Regs::write_gyro_filter(config::GyroFilter filter) {
    switch (filter) {
        case config::GyroFilter::Bypass3600Hz:
        case config::GyroFilter::Bypass8800Hz: {
            const uint8_t data = filter == config::GyroFilter::Bypass3600Hz
                                     ? bits::gyro_config::FCHOICE_B_BYPASS_3600HZ
                                     : bits::gyro_config::FCHOICE_B_BYPASS_8800HZ;
            MPU_RETURN_IF_ERROR(
                update_bits(reg::GYRO_CONFIG, bits::gyro_config::FCHOICE_B_MASK, data));
            break;
        }
        default:
            MPU_RETURN_IF_ERROR(update_bits(reg::GYRO_CONFIG,
                                            bits::gyro_config::FCHOICE_B_MASK,
                                            bits::gyro_config::FCHOICE_B_USE_DLPF));
            MPU_RETURN_IF_ERROR(update_bits(
                reg::CONFIG, bits::config::DLPF_CFG_MASK, static_cast<uint8_t>(filter)));
            break;
    }

    return Status::OK;
}
Status Mpu6500Regs::write_sample_rate_divider(uint8_t divider) {
    return bus_.write_reg(reg::SMPLRT_DIV, divider);
}

Status Mpu6500Regs::write_sleep(bool enabled) {
    return update_bits(
        reg::PWR_MGMT_1, bits::pwr_mgmt_1::SLEEP, enabled ? bits::pwr_mgmt_1::SLEEP : 0);
}
Status Mpu6500Regs::write_gyro_standby(bool enabled) {
    return update_bits(reg::PWR_MGMT_1,
                       bits::pwr_mgmt_1::GYRO_STANDBY,
                       enabled ? bits::pwr_mgmt_1::GYRO_STANDBY : 0);
}
Status Mpu6500Regs::write_temperature_enabled(bool enabled) {
    return update_bits(
        reg::PWR_MGMT_1, bits::pwr_mgmt_1::TEMP_DIS, enabled ? 0 : bits::pwr_mgmt_1::TEMP_DIS);
}
Status Mpu6500Regs::write_clock_source(config::ClockSource source) {
    return update_bits(
        reg::PWR_MGMT_1, bits::pwr_mgmt_1::CLKSEL_MASK, static_cast<uint8_t>(source));
}
Status Mpu6500Regs::write_enabled_axes(const config::EnabledAxes& enabled) {
    uint8_t disabled_bits{};
    if (!enabled.accel_x)
        disabled_bits |= bits::pwr_mgmt_2::DIS_XA;
    if (!enabled.accel_y)
        disabled_bits |= bits::pwr_mgmt_2::DIS_YA;
    if (!enabled.accel_z)
        disabled_bits |= bits::pwr_mgmt_2::DIS_ZA;
    if (!enabled.gyro_x)
        disabled_bits |= bits::pwr_mgmt_2::DIS_XG;
    if (!enabled.gyro_y)
        disabled_bits |= bits::pwr_mgmt_2::DIS_YG;
    if (!enabled.gyro_z)
        disabled_bits |= bits::pwr_mgmt_2::DIS_ZG;

    return update_bits(reg::PWR_MGMT_2, bits::pwr_mgmt_2::DIS_ALL_MASK, disabled_bits);
}

Status Mpu6500Regs::read_burst(std::span<uint8_t, 14> buffer) const {
    return bus_.read_regs(reg::ACCEL_XOUT_H, buffer);
}
Status Mpu6500Regs::read_accel_bytes(std::span<uint8_t, 6> buffer) const {
    return bus_.read_regs(reg::ACCEL_XOUT_H, buffer);
}
Status Mpu6500Regs::read_gyro_bytes(std::span<uint8_t, 6> buffer) const {
    return bus_.read_regs(reg::GYRO_XOUT_H, buffer);
}
Status Mpu6500Regs::read_temp_bytes(std::span<uint8_t, 2> buffer) const {
    return bus_.read_regs(reg::TEMP_OUT_H, buffer);
}

Status Mpu6500Regs::write_signal_path_reset(bool gyro, bool accel, bool temp) {
    uint8_t byte = static_cast<uint8_t>((gyro ? bits::signal_path_reset::GYRO : 0) |
                                        (accel ? bits::signal_path_reset::ACCEL : 0) |
                                        (temp ? bits::signal_path_reset::TEMP : 0));
    return bus_.write_reg(reg::SIGNAL_PATH_RESET, byte);
}
Status Mpu6500Regs::write_sensor_reset() {
    return update_bits(
        reg::USER_CTRL, bits::user_ctrl::SIG_COND_RST, bits::user_ctrl::SIG_COND_RST);
}

Status Mpu6500Regs::write_gyro_hw_offset(const RawVec3& offset) {
    MPU_RETURN_IF_ERROR(write_int16(reg::XG_OFFSET_H, reg::XG_OFFSET_L, offset.x));
    MPU_RETURN_IF_ERROR(write_int16(reg::YG_OFFSET_H, reg::YG_OFFSET_L, offset.y));

    return write_int16(reg::ZG_OFFSET_H, reg::ZG_OFFSET_L, offset.z);
}

Status Mpu6500Regs::write_int16(uint8_t high_reg, uint8_t low_reg, int16_t data) {
    const uint16_t converted = static_cast<uint16_t>(data);
    MPU_RETURN_IF_ERROR(bus_.write_reg(high_reg, static_cast<uint8_t>(converted >> 8)));
    return bus_.write_reg(low_reg, static_cast<uint8_t>(converted & 0xff));
}

Status Mpu6500Regs::write_lp_accel_rate(config::LowPowerAccelRate rate_hz) {
    return update_bits(
        reg::LP_ACCEL_ODR, bits::lp_accel_odr::LPOSC_CLKSEL_MASK, static_cast<uint8_t>(rate_hz));
}
Status Mpu6500Regs::write_cycle(bool enabled) {
    return update_bits(
        reg::PWR_MGMT_1, bits::pwr_mgmt_1::CYCLE, enabled ? bits::pwr_mgmt_1::CYCLE : 0);
}
Status Mpu6500Regs::write_fifo_sources(const config::FifoSources& sources) {
    uint8_t byte = (sources.accel ? bits::fifo_en::ACCEL : 0) |
                   (sources.temperature ? bits::fifo_en::TEMP : 0) |
                   (sources.gyro ? bits::fifo_en::GYRO_ALL : 0);
    return update_bits(reg::FIFO_EN, bits::fifo_en::MASK, byte);
}
Status Mpu6500Regs::write_fifo_mode(const config::FifoMode& mode) {
    return update_bits(reg::CONFIG,
                       bits::config::FIFO_MODE,
                       mode == config::FifoMode::StopWhenFull ? bits::config::FIFO_MODE : 0);
}
Status Mpu6500Regs::write_fifo_enabled(bool enabled) {
    return update_bits(
        reg::USER_CTRL, bits::user_ctrl::FIFO_EN, enabled ? bits::user_ctrl::FIFO_EN : 0);
}
Status Mpu6500Regs::fifo_reset() {
    return update_bits(reg::USER_CTRL, bits::user_ctrl::FIFO_RST, bits::user_ctrl::FIFO_RST);
}
Status Mpu6500Regs::read_fifo_count(uint16_t& count) const {
    std::array<uint8_t, 2> buffer{};
    MPU_RETURN_IF_ERROR(bus_.read_regs(reg::FIFO_COUNT_H, buffer));
    count = static_cast<uint16_t>(((buffer[0] & bits::fifo_count::HIGH_MASK) << 8));
    count |= buffer[1];
    return Status::OK;
}
Status Mpu6500Regs::read_fifo_bytes(std::span<uint8_t> buffer) const {
    return bus_.read_regs(reg::FIFO_R_W, buffer);
}
Status Mpu6500Regs::read_int_status(uint8_t& status) const {
    return bus_.read_regs(reg::INT_STATUS, std::span<uint8_t>(&status, 1));
}
} // namespace mpu6500::detail

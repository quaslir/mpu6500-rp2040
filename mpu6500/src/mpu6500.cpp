#include "mpu6500/mpu6500.hpp"

#include "bits.hpp"
#include "bus/bus.hpp"
#include "bus/status.hpp"
#include "config.hpp"
#include "device.hpp"
#include "layout.hpp"
#include "registers.hpp"
#include "scales.hpp"
#include "vec3.hpp"
#include "sample.hpp"
#include <array>
#include <cstdint>
#include <span>
namespace {
int16_t to_int16(std::span<uint8_t> data, uint8_t start_pos) {
    return static_cast<int16_t>((data[start_pos] << 8) | data[start_pos + 1]);
}

float accel_range_to_scale(mpu6500::config::AccelRange range) {
    switch (range) {
        case mpu6500::config::AccelRange::G2:
            return mpu6500::scale::ACCEL_G2;
        case mpu6500::config::AccelRange::G4:
            return mpu6500::scale::ACCEL_G4;
        case mpu6500::config::AccelRange::G8:
            return mpu6500::scale::ACCEL_G8;
        case mpu6500::config::AccelRange::G16:
            return mpu6500::scale::ACCEL_G16;
    }

    return mpu6500::scale::ACCEL_G2;
}

float gyro_range_to_scale(mpu6500::config::GyroRange range) {
    switch (range) {
        case mpu6500::config::GyroRange::Dps250:
            return mpu6500::scale::GYRO_DPS250;
        case mpu6500::config::GyroRange::Dps500:
            return mpu6500::scale::GYRO_DPS500;
        case mpu6500::config::GyroRange::Dps1000:
            return mpu6500::scale::GYRO_DPS1000;
        case mpu6500::config::GyroRange::Dps2000:
            return mpu6500::scale::GYRO_DPS2000;
    }
    return mpu6500::scale::GYRO_DPS250;
}
Vec3 decode_vec3(std::span<uint8_t, 6> sample_buffer, float scale) {
    Vec3 vec3;
    const int16_t acc_x = to_int16(sample_buffer, 0);
    const int16_t acc_y = to_int16(sample_buffer, 2);
    const int16_t acc_z = to_int16(sample_buffer, 4);

    vec3.x = acc_x / scale;
    vec3.y = acc_y / scale;
    vec3.z = acc_z / scale;

    return vec3;
}
float decode_temperature(std::span<uint8_t, mpu6500::layout::TEMP_SIZE> sample_buffer) {
    const int16_t temp = to_int16(sample_buffer, 0);

    return temp / mpu6500::scale::TEMP_SENSITIVITY + mpu6500::scale::TEMP_REFERENCE_C;
}
} // namespace

namespace mpu6500 {
Mpu6500::Mpu6500(bus::Bus& bus, WaitFunction wait, const config::Config& config)
    : bus_(bus), wait_(wait), use_i2c_(config.use_i2c),
      accel_range_(config.starting_accelerometer_range),
      gyro_range_(config.starting_gyroscope_range), accel_filter_(config.starting_accel_filter),
      gyro_filter_(config.starting_gyro_filter), sample_divider_(config.starting_sample_divider) {}

Status Mpu6500::who_am_i(uint8_t& id) {
    return bus_.read_regs(reg::WHO_AM_I, std::span<uint8_t>(&id, 1));
}

Status Mpu6500::init() {
    uint8_t id{};
    const Status who_am_i_result = who_am_i(id);
    if (who_am_i_result != Status::OK)
        return who_am_i_result;
    if (id != device::EXPECTED_ID)
        return Status::ERROR;

    const Status reset_result = bus_.write_reg(reg::PWR_MGMT_1, bits::pwr_mgmt_1::RESET);
    if (reset_result != Status::OK)
        return reset_result;
    wait_(device::RESET_WAIT_MS);

    const Status signal_path_reset_result =
        bus_.write_reg(reg::SIGNAL_PATH_RESET, bits::signal_path_reset::ALL);
    if (signal_path_reset_result != Status::OK)
        return signal_path_reset_result;
    wait_(device::RESET_WAIT_MS);

    if (!use_i2c_) { // disable I2C if config was stated that SPI is used. If user uses I2C, NACK
                     // will be a result of following writing.
        const Status user_ctrl_result = bus_.write_reg(reg::USER_CTRL, bits::user_ctrl::I2C_IF_DIS);
        if (user_ctrl_result != Status::OK)
            return user_ctrl_result;
    }

    const Status normal_mode_result = bus_.write_reg(reg::PWR_MGMT_1, bits::pwr_mgmt_1::NORMAL);
    if (normal_mode_result != Status::OK)
        return normal_mode_result;

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

    return Status::OK;
}

config::AccelRange Mpu6500::accel_range() const {
    return accel_range_;
}

config::GyroRange Mpu6500::gyro_range() const {
    return gyro_range_;
}

Status Mpu6500::set_accel_range(config::AccelRange range) {
    const Status set_accel_range_result =
        bus_.write_reg(reg::ACCEL_CONFIG, static_cast<uint8_t>(range) << bits::fs_sel::SHIFT);
    if (set_accel_range_result != Status::OK)
        return set_accel_range_result;
    accel_range_ = range;
    return Status::OK;
}

Status Mpu6500::set_gyro_range(config::GyroRange range) {
    const Status set_gyro_range_result =
        bus_.write_reg(reg::GYRO_CONFIG, static_cast<uint8_t>(range) << bits::fs_sel::SHIFT);
    if (set_gyro_range_result != Status::OK)
        return set_gyro_range_result;
    gyro_range_ = range;
    return Status::OK;
}

Status Mpu6500::read_all(Sample& sample) const {
    std::array<uint8_t, layout::BURST_SIZE> sample_buffer{};
    const Status read_all_result = bus_.read_regs(reg::ACCEL_XOUT_H, sample_buffer);
    if (read_all_result != Status::OK)
        return read_all_result;
    // fill acceleration
    const float acc_scale = accel_range_to_scale(accel_range_);

    sample.accel_g = decode_vec3(
        std::span{sample_buffer}.subspan<layout::ACCEL_OFFSET, layout::VEC3_SIZE>(), acc_scale);
    // fill temperature

    sample.temperature_c = decode_temperature(
        std::span{sample_buffer}.subspan<layout::TEMP_OFFSET, layout::TEMP_SIZE>());
    // fill gyro

    const float gyro_scale = gyro_range_to_scale(gyro_range_);

    sample.gyro_dps = decode_vec3(
        std::span{sample_buffer}.subspan<layout::GYRO_OFFSET, layout::VEC3_SIZE>(), gyro_scale);

    return Status::OK;
}

Status Mpu6500::read_accel(Vec3& sample) const {
    std::array<uint8_t, layout::VEC3_SIZE> sample_buffer{};
    const Status read_accel_result = bus_.read_regs(reg::ACCEL_XOUT_H, sample_buffer);
    if (read_accel_result != Status::OK)
        return read_accel_result;

    const float acc_scale = accel_range_to_scale(accel_range_);

    sample = decode_vec3(sample_buffer, acc_scale);

    return Status::OK;
}
Status Mpu6500::read_gyro(Vec3& sample) const {
    std::array<uint8_t, layout::VEC3_SIZE> sample_buffer{};
    const Status read_gyro_result = bus_.read_regs(reg::GYRO_XOUT_H, sample_buffer);
    if (read_gyro_result != Status::OK)
        return read_gyro_result;

    const float gyro_scale = gyro_range_to_scale(gyro_range_);

    sample = decode_vec3(sample_buffer, gyro_scale);

    return Status::OK;
}
Status Mpu6500::read_temp(float& sample) const {
    std::array<uint8_t, layout::TEMP_SIZE> sample_buffer{};
    const Status read_temp_result = bus_.read_regs(reg::TEMP_OUT_H, sample_buffer);
    if (read_temp_result != Status::OK)
        return read_temp_result;

    sample = decode_temperature(sample_buffer);
    return Status::OK;
}

Status Mpu6500::set_gyro_filter(config::GyroFilter filter) {
    Status set_gyro_filter_result = bus_.write_reg(reg::CONFIG, static_cast<uint8_t>(filter));
    if (set_gyro_filter_result != Status::OK)
        return set_gyro_filter_result;
    gyro_filter_ = filter;
    return Status::OK;
}
Status Mpu6500::set_accel_filter(config::AccelFilter filter) {
    Status set_accel_filter_result =
        bus_.write_reg(reg::ACCEL_CONFIG2, static_cast<uint8_t>(filter));
    if (set_accel_filter_result != Status::OK) {
        return set_accel_filter_result;
    }
    accel_filter_ = filter;
    return Status::OK;
}
Status Mpu6500::set_sample_rate_divider(uint8_t divider) {
    Status set_sample_divider_result = bus_.write_reg(reg::SMPLRT_DIV, divider);
    if (set_sample_divider_result != Status::OK) {
        return set_sample_divider_result;
    }
    sample_divider_ = divider;
    return Status::OK;
}

config::GyroFilter Mpu6500::gyro_filter() const {
    return gyro_filter_;
}
config::AccelFilter Mpu6500::accel_filter() const {
    return accel_filter_;
}
uint8_t Mpu6500::sample_divider() const {
    return sample_divider_;
}
} // namespace mpu6500

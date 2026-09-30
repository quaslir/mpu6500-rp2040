#include "mpu6500/mpu6500.hpp"

#include "bits.hpp"
#include "bus/bus.hpp"
#include "bus/status.hpp"
#include "calibration.hpp"
#include "config.hpp"
#include "device.hpp"
#include "layout.hpp"
#include "registers.hpp"
#include "sample.hpp"
#include "scales.hpp"
#include "vec3.hpp"
#include <array>
#include <cmath>
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
      gyro_filter_(config.starting_gyro_filter), sample_divider_(config.starting_sample_divider),
      accel_offset_(config.starting_accel_offset), gyro_offset_(config.starting_gyro_offset),
      sleeping_(false), clock_source_(config.starting_clock_source),
      enabled_axes_(config.starting_enabled_axes),
      temperature_enabled_(config.starting_temperature_enabled),
      gyro_standby_(config.starting_gyro_standby) {}

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

    return Status::OK;
}

config::AccelRange Mpu6500::accel_range() const {
    return accel_range_;
}

config::GyroRange Mpu6500::gyro_range() const {
    return gyro_range_;
}

Status Mpu6500::set_accel_range(config::AccelRange range) {
    const Status set_accel_range_result = update_bits(
        reg::ACCEL_CONFIG, bits::fs_sel::MASK, static_cast<uint8_t>(range) << bits::fs_sel::SHIFT);
    if (set_accel_range_result != Status::OK)
        return set_accel_range_result;
    accel_range_ = range;
    return Status::OK;
}

Status Mpu6500::set_gyro_range(config::GyroRange range) {
    const Status set_gyro_range_result = update_bits(
        reg::GYRO_CONFIG, bits::fs_sel::MASK, static_cast<uint8_t>(range) << bits::fs_sel::SHIFT);
    if (set_gyro_range_result != Status::OK)
        return set_gyro_range_result;
    gyro_range_ = range;
    return Status::OK;
}

Status Mpu6500::read_all_raw(Sample& sample) const {
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

Status Mpu6500::read_accel_raw(Vec3& sample) const {
    std::array<uint8_t, layout::VEC3_SIZE> sample_buffer{};
    const Status read_accel_result = bus_.read_regs(reg::ACCEL_XOUT_H, sample_buffer);
    if (read_accel_result != Status::OK)
        return read_accel_result;

    const float acc_scale = accel_range_to_scale(accel_range_);

    sample = decode_vec3(sample_buffer, acc_scale);

    return Status::OK;
}
Status Mpu6500::read_gyro_raw(Vec3& sample) const {
    std::array<uint8_t, layout::VEC3_SIZE> sample_buffer{};
    const Status read_gyro_result = bus_.read_regs(reg::GYRO_XOUT_H, sample_buffer);
    if (read_gyro_result != Status::OK)
        return read_gyro_result;

    const float gyro_scale = gyro_range_to_scale(gyro_range_);

    sample = decode_vec3(sample_buffer, gyro_scale);

    return Status::OK;
}

Status Mpu6500::read_all(Sample& sample) const {
    const Status read_status = read_all_raw(sample);
    if (read_status != Status::OK)
        return read_status;
    sample.accel_g -= accel_offset_;
    sample.gyro_dps -= gyro_offset_;

    return Status::OK;
}
Status Mpu6500::read_accel(Vec3& sample) const {
    const Status read_status = read_accel_raw(sample);
    if (read_status != Status::OK)
        return read_status;
    sample -= accel_offset_;

    return Status::OK;
}
Status Mpu6500::read_gyro(Vec3& sample) const {
    const Status read_status = read_gyro_raw(sample);
    if (read_status != Status::OK)
        return read_status;
    sample -= gyro_offset_;
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

Status Mpu6500::update_bits(uint8_t reg, uint8_t mask, uint8_t data) {
    uint8_t current{};
    const Status read_current_result = bus_.read_regs(reg, std::span<uint8_t>(&current, 1));
    if (read_current_result != Status::OK) {
        return read_current_result;
    }
    const uint8_t updated = static_cast<uint8_t>((current & ~mask) | (data & mask));
    return bus_.write_reg(reg, updated);
}

Status Mpu6500::set_gyro_filter(config::GyroFilter filter) {

    switch (filter) {
        case config::GyroFilter::Bypass3600Hz:
        case config::GyroFilter::Bypass8800Hz: {
            const uint8_t data = filter == config::GyroFilter::Bypass3600Hz
                                     ? bits::gyro_config::FCHOICE_B_BYPASS_3600HZ
                                     : bits::gyro_config::FCHOICE_B_BYPASS_8800HZ;
            const Status set_gyro_bypass_result =
                update_bits(reg::GYRO_CONFIG, bits::gyro_config::FCHOICE_B_MASK, data);
            if (set_gyro_bypass_result != Status::OK)
                return set_gyro_bypass_result;
            break;
        }
        default: {
            const Status set_gyro_bypass_result =
                update_bits(reg::GYRO_CONFIG,
                            bits::gyro_config::FCHOICE_B_MASK,
                            bits::gyro_config::FCHOICE_B_USE_DLPF);
            if (set_gyro_bypass_result != Status::OK)
                return set_gyro_bypass_result;
            const Status set_gyro_filter_result =
                update_bits(reg::CONFIG, bits::config::DLPF_CFG_MASK, static_cast<uint8_t>(filter));
            if (set_gyro_filter_result != Status::OK) {
                return set_gyro_filter_result;
            }
            break;
        }
    }
    gyro_filter_ = filter;
    return Status::OK;
}
Status Mpu6500::set_accel_filter(config::AccelFilter filter) {
    const Status set_accel_filter_result =
        update_bits(reg::ACCEL_CONFIG2,
                    bits::accel_config2::A_DLPF_CFG_MASK | bits::accel_config2::ACCEL_FCHOICE_B,
                    static_cast<uint8_t>(filter));
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

void Mpu6500::set_accel_offset(const Vec3& offset) {
    accel_offset_ = offset;
}
void Mpu6500::set_gyro_offset(const Vec3& offset) {
    gyro_offset_ = offset;
}

const Vec3& Mpu6500::accel_offset() const {
    return accel_offset_;
}
const Vec3& Mpu6500::gyro_offset() const {
    return gyro_offset_;
}

void Mpu6500::clear_offsets() {
    accel_offset_ = Vec3{};
    gyro_offset_ = Vec3{};
}

Status Mpu6500::measure_mean(calibration::Sensor sensor,
                             Vec3& mean,
                             const calibration::MeasureOptions& options) const {
    ReadVec3Fn func;
    switch (sensor) {
        case calibration::Sensor::Gyro:
            func = &Mpu6500::read_gyro_raw;
            break;
        case calibration::Sensor::Accel:
            func = &Mpu6500::read_accel_raw;
            break;
        default:
            return Status::ERROR;
    }

    return measure_mean_impl(func, mean, options);
}

Status Mpu6500::measure_mean_impl(ReadVec3Fn func,
                                  Vec3& mean,
                                  const calibration::MeasureOptions& options) const {
    if (options.samples == 0)
        return Status::ERROR;
    const uint32_t period = sample_divider_ + 1;
    auto warm_up = [func, this, period](uint16_t samples) -> Status {
        Vec3 sample{};
        for (uint16_t i = 0; i < samples; i++) {
            const Status read_result = (this->*func)(sample);
            if (read_result != Status::OK)
                return read_result;
            wait_(period);
        }

        return Status::OK;
    };

    const Status warm_up_status = warm_up(options.warmup_samples);
    if (warm_up_status != Status::OK)
        return warm_up_status;
    Vec3 sample{}, sum{};
    const Status read_result_first = (this->*func)(sample);
    if (read_result_first != Status::OK)
        return read_result_first;
    Vec3 min_value = sample;
    Vec3 max_value = sample;
    wait_(period);
    for (uint16_t i = 0; i < options.samples; i++) {
        const Status read_result = (this->*func)(sample);
        if (read_result != Status::OK)
            return read_result;
        sum += sample;
        min_value = component_min(min_value, sample);
        max_value = component_max(max_value, sample);
        wait_(period);
    }

    const Vec3 diff = max_value - min_value;
    if (diff.x > options.max_spread || diff.y > options.max_spread || diff.z > options.max_spread)
        return Status::ERROR;

    mean.x = sum.x / static_cast<float>(options.samples);
    mean.y = sum.y / static_cast<float>(options.samples);
    mean.z = sum.z / static_cast<float>(options.samples);

    return Status::OK;
}

Status Mpu6500::measure_gyro_offset(Vec3& offset,
                                    const calibration::MeasureOptions& options) const {
    return measure_mean(calibration::Sensor::Gyro, offset, options);
}
Status Mpu6500::calibrate_gyro(const calibration::MeasureOptions& options) {
    Vec3 offset{};
    const Status measure_gyro_result = measure_gyro_offset(offset, options);
    if (measure_gyro_result != Status::OK)
        return measure_gyro_result;

    set_gyro_offset(offset);
    return Status::OK;
}

Status Mpu6500::measure_accel_offset(Vec3& offset,
                                     const Vec3& expected_gravity_g,
                                     const calibration::MeasureOptions& options) const {
    Vec3 mean{};
    const Status measure_mean_result = measure_mean(calibration::Sensor::Accel, mean, options);
    if (measure_mean_result != Status::OK) {
        return measure_mean_result;
    }

    const float magnitude = std::sqrt(mean.x * mean.x + mean.y * mean.y + mean.z * mean.z);
    if (magnitude < calibration::MIN_GRAVITY_G || magnitude > calibration::MAX_GRAVITY_G) {
        return Status::ERROR;
    }

    offset = mean - expected_gravity_g;

    return Status::OK;
}
Status Mpu6500::calibrate_accel(const Vec3& expected_gravity_g,
                                const calibration::MeasureOptions& options) {
    Vec3 offset{};
    const Status measure_accel_result = measure_accel_offset(offset, expected_gravity_g, options);
    if (measure_accel_result != Status::OK)
        return measure_accel_result;

    set_accel_offset(offset);
    return Status::OK;
}

Status Mpu6500::set_sleep(bool enabled) {
    const Status set_sleep_result = update_bits(
        reg::PWR_MGMT_1, bits::pwr_mgmt_1::SLEEP, enabled ? bits::pwr_mgmt_1::SLEEP : 0);

    if (set_sleep_result != Status::OK)
        return set_sleep_result;

    sleeping_ = enabled;
    return Status::OK;
}
Status Mpu6500::set_gyro_standby(bool enabled) {
    const Status set_gyro_standby_result =
        update_bits(reg::PWR_MGMT_1,
                    bits::pwr_mgmt_1::GYRO_STANDBY,
                    enabled ? bits::pwr_mgmt_1::GYRO_STANDBY : 0);

    if (set_gyro_standby_result != Status::OK)
        return set_gyro_standby_result;

    gyro_standby_ = enabled;
    return Status::OK;
}
Status Mpu6500::set_temperature_enabled(bool enabled) {
    const Status set_temperature_result = update_bits(
        reg::PWR_MGMT_1, bits::pwr_mgmt_1::TEMP_DIS, enabled ? 0 : bits::pwr_mgmt_1::TEMP_DIS);

    if (set_temperature_result != Status::OK)
        return set_temperature_result;

    temperature_enabled_ = enabled;
    return Status::OK;
}
Status Mpu6500::set_clock_source(config::ClockSource source) {
    const Status set_clock_result =
        update_bits(reg::PWR_MGMT_1, bits::pwr_mgmt_1::CLKSEL_MASK, static_cast<uint8_t>(source));

    if (set_clock_result != Status::OK)
        return set_clock_result;

    clock_source_ = source;
    return Status::OK;
}
Status Mpu6500::set_enabled_axes(const config::EnabledAxes& enabled) {
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

    const Status set_axes_result =
        update_bits(reg::PWR_MGMT_2, bits::pwr_mgmt_2::DIS_ALL_MASK, disabled_bits);

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

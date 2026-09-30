#pragma once

#include "bus/bus.hpp"
#include "bus/status.hpp"
#include "config.hpp"
#include "sample.hpp"
#include "vec3.hpp"
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
    [[nodiscard]] Status measure_gyro_offset(uint16_t samples, Vec3& offset, float max_spread_dps = 2.0f) const;
    [[nodiscard]] Status calibrate_gyro(uint16_t samples);
private:
    bus::Bus& bus_;
    WaitFunction wait_;
    bool use_i2c_;
    config::AccelRange accel_range_;
    config::GyroRange gyro_range_;
    config::AccelFilter accel_filter_;
    config::GyroFilter gyro_filter_;
    uint8_t sample_divider_;
    Vec3 accel_offset_;
    Vec3 gyro_offset_;
};
} // namespace mpu6500

#pragma once

#include "bus/bus.hpp"
#include "bus/status.hpp"
#include <cstdint>
namespace mpu6500 {

enum class AccelRange : uint8_t { G2 = 0, G4 = 1, G8 = 2, G16 = 3 };

enum class GyroRange : uint8_t { Dps250 = 0, Dps500 = 1, Dps1000 = 2, Dps2000 = 3 };

enum class GyroFilter : uint8_t {
    Hz250 = 0,
    Hz184 = 1,
    Hz92 = 2,
    Hz41 = 3,
    Hz20 = 4,
    Hz10 = 5,
    Hz5 = 6
};

enum class AccelFilter : uint8_t {
    Hz460 = 0,
    Hz184 = 1,
    Hz92 = 2,
    Hz41 = 3,
    Hz20 = 4,
    Hz10 = 5,
    Hz5 = 6
};

struct Config {
    bool use_i2c = true;
    AccelRange starting_accelerometer_range = AccelRange::G2;
    GyroRange starting_gyroscope_range = GyroRange::Dps250;
    AccelFilter starting_accel_filter = AccelFilter::Hz460;
    GyroFilter starting_gyro_filter = GyroFilter::Hz250;
    uint8_t starting_sample_divider = 0;
};

struct Vec3 {
    float x{}, y{}, z{};
};

struct Sample {
    Vec3 accel_g{};
    Vec3 gyro_dps{};
    float temperature_c{};
};

using WaitFunction = void (*)(uint32_t delay_ms);
class Mpu6500 {
public:
    Mpu6500(bus::Bus& bus, WaitFunction, const Config& config);
    Mpu6500(const Mpu6500&) = delete;
    Mpu6500& operator=(const Mpu6500&) = delete;

    [[nodiscard]] Status init();
    [[nodiscard]] Status who_am_i(uint8_t& id);
    [[nodiscard]] Status set_accel_range(AccelRange range);
    [[nodiscard]] Status set_gyro_range(GyroRange range);
    [[nodiscard]] AccelRange accel_range() const;
    [[nodiscard]] GyroRange gyro_range() const;
    [[nodiscard]] Status read_all(Sample& sample) const;
    // Measurement
    [[nodiscard]] Status read_accel(Vec3& sample) const;
    [[nodiscard]] Status read_gyro(Vec3& sample) const;
    [[nodiscard]] Status read_temp(float& sample) const;

    // Filters
    [[nodiscard]] Status set_gyro_filter(GyroFilter filter);
    [[nodiscard]] Status set_accel_filter(AccelFilter filter);
    [[nodiscard]] Status set_sample_rate_divider(uint8_t divider);

    [[nodiscard]] GyroFilter gyro_filter() const;
    [[nodiscard]] AccelFilter accel_filter() const;
    [[nodiscard]] uint8_t sample_divider() const;

private:
    bus::Bus& bus_;
    WaitFunction wait_;
    bool use_i2c_;
    AccelRange accel_range_;
    GyroRange gyro_range_;
    AccelFilter accel_filter_;
    GyroFilter gyro_filter_;
    uint8_t sample_divider_;
};
} // namespace mpu6500

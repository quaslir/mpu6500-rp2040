#pragma once
#include "vec3.hpp"
#include <cstdint>
namespace mpu6500::config {
enum class AccelRange : uint8_t { G2 = 0, G4 = 1, G8 = 2, G16 = 3 };

enum class GyroRange : uint8_t { Dps250 = 0, Dps500 = 1, Dps1000 = 2, Dps2000 = 3 };

enum class GyroFilter : uint8_t {
    Hz250 = 0,
    Hz184 = 1,
    Hz92 = 2,
    Hz41 = 3,
    Hz20 = 4,
    Hz10 = 5,
    Hz5 = 6,
    Hz3600 = 7,
    Bypass3600Hz = 8,
    Bypass8800Hz = 9

};

enum class AccelFilter : uint8_t {
    Hz460 = 0,
    Hz184 = 1,
    Hz92 = 2,
    Hz41 = 3,
    Hz20 = 4,
    Hz10 = 5,
    Hz5 = 6,
    Bypass1130Hz = 8
};

enum class ClockSource : uint8_t { Internal20MHz = 0, Auto = 1, Stopped = 7 };
struct EnabledAxes {
    bool accel_x = true;
    bool accel_y = true;
    bool accel_z = true;
    bool gyro_x = true;
    bool gyro_y = true;
    bool gyro_z = true;
};

struct Config {
    bool use_i2c = true;
    AccelRange starting_accelerometer_range = AccelRange::G2;
    GyroRange starting_gyroscope_range = GyroRange::Dps250;
    AccelFilter starting_accel_filter = AccelFilter::Hz460;
    GyroFilter starting_gyro_filter = GyroFilter::Hz250;
    uint8_t starting_sample_divider = 0;
    Vec3 starting_accel_offset{};
    Vec3 starting_gyro_offset{};
    ClockSource starting_clock_source{};
    EnabledAxes starting_enabled_axes{};
    bool starting_temperature_enabled{};
    bool starting_gyro_standby{};
};

} // namespace mpu6500::config

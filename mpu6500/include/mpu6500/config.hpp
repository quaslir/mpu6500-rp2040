#pragma once
#include "math/vec3.hpp"
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

enum class LowPowerAccelRate : uint8_t {
    Hz0_24 = 0,
    Hz0_49 = 1,
    Hz0_98 = 2,
    Hz1_95 = 3,
    Hz3_91 = 4,
    Hz7_81 = 5,
    Hz15_63 = 6,
    Hz31_25 = 7,
    Hz62_5 = 8,
    Hz125 = 9,
    Hz250 = 10,
    Hz500 = 11
};

struct EnabledAxes {
    bool accel_x = true;
    bool accel_y = true;
    bool accel_z = true;
    bool gyro_x = true;
    bool gyro_y = true;
    bool gyro_z = true;
};

struct LowPowerBackup {
    config::AccelFilter accel_filter = config::AccelFilter::Hz460;
    bool temperature_enabled{};
    config::EnabledAxes enabled_axes{};
};

struct LowPowerMode {
    bool active{};
    config::LowPowerAccelRate rate{LowPowerAccelRate::Hz0_24};
    LowPowerBackup backup{};
};

struct Accel {
    AccelRange range = AccelRange::G2;
    AccelFilter filter = AccelFilter::Hz460;
    Vec3 offset{};
};

struct Gyro {
    bool standby{};
    RawVec3 hw_offset{};
    Vec3 offset{};
    GyroRange range{GyroRange::Dps250};
    GyroFilter filter{GyroFilter::Hz250};
};

struct Config {
    bool use_i2c = true;
    Accel starting_accel_params{};
    Gyro starting_gyro_params{};
    uint8_t starting_sample_divider = 0;
    ClockSource starting_clock_source = ClockSource::Auto;
    EnabledAxes starting_enabled_axes{};
    bool starting_temperature_enabled{};
};

} // namespace mpu6500::config

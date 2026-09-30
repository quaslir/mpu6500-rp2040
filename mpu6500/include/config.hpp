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
    Vec3 starting_accel_offset{};
    Vec3 starting_gyro_offset{};
};

} // namespace mpu6500::config

#pragma once
#include "math/vec3.hpp"
#include <cstddef>
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
enum class PowerMode : uint8_t { Normal = 0, Sleep = 1, LowPowerAccel = 2 };

struct EnabledAxes {
    bool accel_x = true;
    bool accel_y = true;
    bool accel_z = true;
    bool gyro_x = true;
    bool gyro_y = true;
    bool gyro_z = true;
};

struct Accel {
    AccelRange range{AccelRange::G2};
    AccelFilter filter{AccelFilter::Hz460};
};

struct Gyro {
    GyroRange range{GyroRange::Dps250};
    GyroFilter filter{GyroFilter::Hz250};
};

struct Measurement {
    Accel accel{};
    Gyro gyro{};
    uint8_t sample_divider{};
};

struct Power {
    PowerMode mode{PowerMode::Normal};
    LowPowerAccelRate low_power_rate{LowPowerAccelRate::Hz0_24};
    bool gyro_standby{};
    bool temperature_enabled{};
    ClockSource clock_source{ClockSource::Auto};
    EnabledAxes enabled_axes{};
};

struct Calibration {
    math::Vec3 accel_offset_g{};
    math::Vec3 gyro_offset_dps{};
    math::RawVec3 gyro_hw_offset{};
};
enum class FifoMode : uint8_t { Overwrite = 0, StopWhenFull = 1 };
struct FifoSources {
    bool accel{true};
    bool temperature{false};
    bool gyro{true};
};

struct Fifo {
    bool enabled{false};
    FifoSources sources;
    FifoMode mode{FifoMode::Overwrite};
};

enum class IntLevel : uint8_t { ActiveHigh = 0, ActiveLow = 1 };

enum class IntDrive : uint8_t { PushPull = 0, OpenDrain = 1 };

enum class IntMode : uint8_t { Pulse = 0, Latched = 1 };

struct InterruptSources {
    bool raw_data_ready{};
    bool fifo_overflow{};
    bool wake_on_motion{};
};

struct Interrupts {
    InterruptSources sources{};
    IntLevel level{IntLevel::ActiveHigh};
    IntDrive drive{IntDrive::PushPull};
    IntMode mode{IntMode::Pulse};
};

struct WakeOnMotion {
    bool enabled{false};
    uint16_t threshold_mg{100};
};

struct Config {
    bool use_i2c{true};
    Measurement measurement{};
    Power power{};
    Calibration calibration{};
    Fifo fifo{};
    Interrupts interrupts{};
    WakeOnMotion wake_on_motion{};
};

} // namespace mpu6500::config

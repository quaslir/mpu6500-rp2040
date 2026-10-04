#include "bus_pico/i2c_bus.hpp"
#include "example_utils.hpp"
#include "i2c_config.hpp"
#include "mpu6500/calibration.hpp"
#include "mpu6500/mpu6500.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <pico/stdlib.h>

namespace {

constexpr uint32_t STARTUP_DELAY_MS = 3000; // time to open the serial terminal
constexpr uint32_t I2C_TIMEOUT_US = 30000;
constexpr uint8_t SAMPLE_DIVIDER = 9; // 1000 / (1 + 9) = 100 Hz
constexpr uint32_t SAMPLE_PERIOD_MS = SAMPLE_DIVIDER + 1;

constexpr int CALIBRATION_ATTEMPTS = 3;
constexpr int COUNTDOWN_S = 3;

constexpr uint16_t CHECK_SAMPLES = 100; // samples used for the before/after comparison
constexpr uint32_t LIVE_PRINT_PERIOD_MS = 200;

constexpr float RAD_TO_DEG = 57.29578f;

float magnitude(const math::Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// Average of `samples` accel readings, with or without the stored offset.
bus::Status
measure_accel_mean(const mpu6500::Mpu6500& imu, uint16_t samples, bool raw, math::Vec3& mean) {
    math::Vec3 sum{};
    for (uint16_t i = 0; i < samples; ++i) {
        math::Vec3 accel{};
        const bus::Status status = raw ? imu.read_accel_uncorrected(accel) : imu.read_accel(accel);
        if (status != bus::Status::OK)
            return status;
        sum += accel;
        sleep_ms(SAMPLE_PERIOD_MS);
    }
    mean.x = sum.x / samples;
    mean.y = sum.y / samples;
    mean.z = sum.z / samples;
    return bus::Status::OK;
}

// Tilt angles from gravity only (valid while the board is not accelerating).
void tilt_deg(const math::Vec3& a, float& roll, float& pitch) {
    roll = std::atan2(a.y, a.z) * RAD_TO_DEG;
    pitch = std::atan2(-a.x, std::sqrt(a.y * a.y + a.z * a.z)) * RAD_TO_DEG;
}

void print_vec(const char* label, const math::Vec3& v) {
    std::printf("  %-24s x=%+7.3f  y=%+7.3f  z=%+7.3f g   |a|=%.3f g\n",
                label,
                v.x,
                v.y,
                v.z,
                magnitude(v));
}

void countdown() {
    for (int s = COUNTDOWN_S; s > 0; --s) {
        std::printf("  starting in %d...\n", s);
        sleep_ms(1000);
    }
}

bus::Status run_calibration(mpu6500::Mpu6500& imu) {
    const auto& options = mpu6500::calibration::DEFAULT_ACCEL_OPTIONS;
    const uint32_t duration_ms = (options.warmup_samples + options.samples) * SAMPLE_PERIOD_MS;
    bus::Status status = bus::Status::ERROR;

    for (int attempt = 1; attempt <= CALIBRATION_ATTEMPTS; ++attempt) {
        std::printf("  calibrating (attempt %d/%d, ~%lu ms)...\n",
                    attempt,
                    CALIBRATION_ATTEMPTS,
                    static_cast<unsigned long>(duration_ms));

        status =
            mpu6500::calibration::calibrate_accel(imu, mpu6500::calibration::GRAVITY_Z_UP, options);
        if (status == bus::Status::OK)
            return status;

        std::printf("  failed: %s (board moved, not lying Z-up, or bus error), retrying in 1 s\n",
                    example::status_text(status));
        sleep_ms(1000);
    }
    return status;
}

} // namespace

int main() {
    stdio_init_all();
    sleep_ms(STARTUP_DELAY_MS);

    std::printf("\n=== MPU6500 accelerometer calibration ===\n\n");

    // --- setup ---------------------------------------------------------------
    i2c_config::init_test_i2c();
    bus::pico::I2CBus bus{i2c0, i2c_config::DEVICE_ADDR, I2C_TIMEOUT_US};

    mpu6500::config::Config config{};
    config.measurement.gyro.filter = mpu6500::config::GyroFilter::Hz41;
    config.measurement.accel.filter = mpu6500::config::AccelFilter::Hz41;
    config.measurement.sample_divider = SAMPLE_DIVIDER;

    mpu6500::Mpu6500 imu(bus, sleep_ms, config);

    uint8_t id{};
    const bus::Status id_status = imu.who_am_i(id);
    std::printf("WHO_AM_I: 0x%02x (%s)\n", id, example::status_text(id_status));

    const bus::Status init_status = imu.init();
    example::print_status("init", init_status);
    if (init_status != bus::Status::OK)
        example::halt("init failed", init_status);

    std::printf("Filter 41 Hz, sample rate %u Hz\n\n", 1000u / (SAMPLE_DIVIDER + 1u));
    std::printf("Place the board FLAT on a LEVEL surface, chip facing up (Z up),\n");
    std::printf("and DO NOT TOUCH IT until told.\n");
    countdown();

    // --- 1. before -----------------------------------------------------------
    std::printf("\n[1] Before calibration (average of %u samples):\n", CHECK_SAMPLES);
    math::Vec3 before{};
    const bus::Status before_status = measure_accel_mean(imu, CHECK_SAMPLES, false, before);
    if (before_status != bus::Status::OK)
        example::halt("reading accel failed", before_status);
    print_vec("read_accel", before);
    std::printf("  -> ideal would be x=0 y=0 z=+1 and |a|=1\n");

    // --- 2. calibration ------------------------------------------------------
    std::printf("\n[2] Calibration (expected gravity: Z up):\n");
    const bus::Status calibration_status = run_calibration(imu);
    if (calibration_status != bus::Status::OK)
        example::halt("accel calibration failed", calibration_status);

    const math::Vec3 offset = imu.config().calibration.accel_offset_g;
    std::printf("  measured offset          x=%+7.3f  y=%+7.3f  z=%+7.3f g\n",
                offset.x,
                offset.y,
                offset.z);

    // --- 3. after ------------------------------------------------------------
    std::printf("\n[3] After calibration (average of %u samples):\n", CHECK_SAMPLES);
    math::Vec3 after{};
    math::Vec3 after_raw{};
    const bus::Status after_status = measure_accel_mean(imu, CHECK_SAMPLES, false, after);
    const bus::Status after_raw_status = measure_accel_mean(imu, CHECK_SAMPLES, true, after_raw);
    if (after_status != bus::Status::OK || after_raw_status != bus::Status::OK)
        example::halt("reading accel failed", bus::Status::ERROR);
    print_vec("read_accel (corrected)", after);
    print_vec("read_accel_raw", after_raw);
    std::printf("  -> corrected should be about (0, 0, +1), raw unchanged\n");

    std::printf("\nTo reuse this calibration without measuring again, put this in your Config:\n");
    std::printf("  config.calibration.accel_offset_g = {%.4ff, %.4ff, %.4ff};\n",
                offset.x,
                offset.y,
                offset.z);
    std::printf("Note: the tilt of the surface is included in this offset.\n");

    // --- 4. live -------------------------------------------------------------
    std::printf("\n[4] Live data: you can move the board now.\n");
    std::printf("    Tilt it slowly: roll/pitch should follow, and read 0 deg when put back.\n");
    std::printf("    Turn it upside down: corrected z should be about -1 g.\n\n");
    std::printf("  %-24s | %-24s | %s\n", "corrected [g]", "raw [g]", "roll / pitch [deg]");

    for (;;) {
        math::Vec3 accel{};
        math::Vec3 accel_raw{};
        const bus::Status status = imu.read_accel(accel);
        const bus::Status raw_status = imu.read_accel_uncorrected(accel_raw);

        if (status != bus::Status::OK || raw_status != bus::Status::OK) {
            std::printf("  read failed: %s / %s\n",
                        example::status_text(status),
                        example::status_text(raw_status));
        } else {
            float roll = 0.0f;
            float pitch = 0.0f;
            tilt_deg(accel, roll, pitch);
            std::printf("  %+6.3f %+6.3f %+6.3f    | %+6.3f %+6.3f %+6.3f    | %+7.1f %+7.1f\n",
                        accel.x,
                        accel.y,
                        accel.z,
                        accel_raw.x,
                        accel_raw.y,
                        accel_raw.z,
                        roll,
                        pitch);
        }
        sleep_ms(LIVE_PRINT_PERIOD_MS);
    }
}

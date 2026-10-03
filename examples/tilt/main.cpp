// Tilt demo: roll and pitch from the accelerometer alone vs. the complementary filter.
// Lay the board flat, let it calibrate, then tilt and shake it and compare the columns.

#include "bus_pico/i2c_bus.hpp"
#include "example_utils.hpp"
#include "i2c_config.hpp"
#include "mpu6500/calibration.hpp"
#include "mpu6500/mpu6500.hpp"
#include "orientation.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <pico/stdlib.h>

namespace {

namespace cfg = mpu6500::config;
namespace cal = mpu6500::calibration;

constexpr uint32_t STARTUP_DELAY_MS = 3000;
constexpr uint32_t I2C_TIMEOUT_US = 30000;

constexpr uint8_t SAMPLE_DIVIDER = 9;      // 1000 / (1 + 9) = 100 Hz
constexpr uint32_t LOOP_PERIOD_US = 10000; // read every 10 ms, matches 100 Hz
constexpr int PRINT_EVERY = 10;            // print every 10th loop -> 10 lines per second

constexpr int CALIBRATION_ATTEMPTS = 3;
constexpr int COUNTDOWN_S = 3;

constexpr float DEG_TO_RAD = 0.01745329f;
constexpr float RAD_TO_DEG = 57.29578f;

// One angle: how far the board is tilted away from flat, in any direction.
// 0 deg = flat, 90 deg = standing on its edge, 180 deg = upside down.
float total_tilt_deg(const orientation::Angles& a) {
    float c = std::cos(a.roll_deg * DEG_TO_RAD) * std::cos(a.pitch_deg * DEG_TO_RAD);
    if (c > 1.0f)
        c = 1.0f;
    if (c < -1.0f)
        c = -1.0f;
    return std::acos(c) * RAD_TO_DEG;
}

float magnitude(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

void countdown() {
    for (int s = COUNTDOWN_S; s > 0; --s) {
        std::printf("  starting in %d...\n", s);
        sleep_ms(1000);
    }
}

Status calibrate_gyro_with_retries(mpu6500::Mpu6500& imu) {
    Status status = Status::ERROR;
    for (int attempt = 1; attempt <= CALIBRATION_ATTEMPTS; ++attempt) {
        std::printf("  gyro calibration, attempt %d/%d...\n", attempt, CALIBRATION_ATTEMPTS);
        status = cal::calibrate_gyro(imu);
        if (status == Status::OK)
            return status;
        std::printf("  failed: %s (board moved?), retrying in 1 s\n", example::status_text(status));
        sleep_ms(1000);
    }
    return status;
}

// Board must lie flat with the chip facing up (Z up). This also "zeroes" the level:
// the tilt of the surface ends up in the offset, so this position reads 0 / 0 afterwards.
Status calibrate_accel_with_retries(mpu6500::Mpu6500& imu) {
    Status status = Status::ERROR;
    for (int attempt = 1; attempt <= CALIBRATION_ATTEMPTS; ++attempt) {
        std::printf("  accel calibration, attempt %d/%d...\n", attempt, CALIBRATION_ATTEMPTS);
        status = cal::calibrate_accel(imu);
        if (status == Status::OK)
            return status;
        std::printf("  failed: %s (board moved or not lying Z-up?), retrying in 1 s\n",
                    example::status_text(status));
        sleep_ms(1000);
    }
    return status;
}

} // namespace

int main() {
    stdio_init_all();
    sleep_ms(STARTUP_DELAY_MS);

    std::printf("\n=== MPU6500 tilt demo ===\n\n");

    // --- setup ---------------------------------------------------------------
    i2c_config::init_test_i2c();
    bus::pico::I2CBus bus{i2c0, i2c_config::DEVICE_ADDR, I2C_TIMEOUT_US};

    cfg::Config config{};
    config.measurement.gyro.filter = cfg::GyroFilter::Hz41;
    config.measurement.accel.filter = cfg::AccelFilter::Hz41;
    config.measurement.sample_divider = SAMPLE_DIVIDER;

    mpu6500::Mpu6500 imu(bus, sleep_ms, config);

    const Status init_status = imu.init();
    example::print_status("init", init_status);
    if (init_status != Status::OK)
        example::halt("init failed", init_status);

    // --- gyro calibration (without it the gyro drifts ~8 deg/s) ---------------
    std::printf("\nPlace the board flat and do not touch it.\n");
    countdown();
    const Status calibration_status = calibrate_gyro_with_retries(imu);
    if (calibration_status != Status::OK)
        example::halt("gyro calibration failed", calibration_status);
    const Vec3 gyro_offset = imu.config().calibration.gyro_offset_dps;
    std::printf(
        "  gyro offset %+.3f %+.3f %+.3f dps\n", gyro_offset.x, gyro_offset.y, gyro_offset.z);

    // --- accel calibration (removes the ~0.14 g Z offset and zeroes the level) -
    Vec3 accel_before{};
    if (imu.read_accel(accel_before) == Status::OK) {
        const orientation::Angles before = orientation::tilt_from_accel(accel_before);
        std::printf("\n  before accel calibration: roll %+.2f, pitch %+.2f deg, |a| %.3f g\n",
                    before.roll_deg,
                    before.pitch_deg,
                    magnitude(accel_before));
    }

    const Status accel_status = calibrate_accel_with_retries(imu);
    if (accel_status != Status::OK)
        example::halt("accel calibration failed", accel_status);
    const Vec3 accel_offset = imu.config().calibration.accel_offset_g;
    std::printf(
        "  accel offset %+.4f %+.4f %+.4f g\n", accel_offset.x, accel_offset.y, accel_offset.z);

    Vec3 accel_after{};
    if (imu.read_accel(accel_after) == Status::OK) {
        const orientation::Angles after = orientation::tilt_from_accel(accel_after);
        std::printf("  after  accel calibration: roll %+.2f, pitch %+.2f deg, |a| %.3f g\n",
                    after.roll_deg,
                    after.pitch_deg,
                    magnitude(accel_after));
    }

    std::printf("\nThe FILTERED column is the tilt to use (smooth and drift-free).\n");
    std::printf("Now tilt the board slowly, then shake it.\n");
    std::printf(
        "Slow tilt: both columns agree. Shaking: accel-only jumps, filtered stays smooth.\n");
    std::printf("\nroll  = tilt left/right (around X)\n");
    std::printf("pitch = tilt forward/back (around Y)\n\n");

    // --- loop ----------------------------------------------------------------
    orientation::ComplementaryFilter filter; // default time constant and tolerance

    absolute_time_t next_wakeup = get_absolute_time();
    uint64_t last_us = time_us_64();
    int loop_count = 0;

    for (;;) {
        // wait until the next 10 ms slot, so the period does not drift
        next_wakeup = delayed_by_us(next_wakeup, LOOP_PERIOD_US);
        sleep_until(next_wakeup);

        // real time since the previous read, in seconds
        const uint64_t now_us = time_us_64();
        const float dt_s = static_cast<float>(now_us - last_us) * 1e-6f;
        last_us = now_us;

        Sample sample{};
        const Status status = imu.read_all(sample);
        if (status != Status::OK) {
            std::printf("  read failed: %s\n", example::status_text(status));
            continue;
        }

        const orientation::Angles filtered = filter.update(sample.accel_g, sample.gyro_dps, dt_s);

        if (++loop_count % PRINT_EVERY == 0) {
            std::printf("\r  roll %+6.1f deg   pitch %+6.1f deg   tilt %5.1f deg   ",
                        filtered.roll_deg,
                        filtered.pitch_deg,
                        total_tilt_deg(filtered));
            std::fflush(stdout);
        }
    }
}

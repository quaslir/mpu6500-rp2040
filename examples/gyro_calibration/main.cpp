#include "bus_pico/i2c_bus.hpp"
#include "example_utils.hpp"
#include "i2c_config.hpp"
#include "mpu6500/calibration.hpp"
#include "mpu6500/mpu6500.hpp"
#include <cstdint>
#include <cstdio>
#include <pico/stdlib.h>

namespace {

constexpr uint32_t STARTUP_DELAY_MS = 3000; // time to open the serial terminal
constexpr uint32_t I2C_TIMEOUT_US = 30000;
constexpr uint8_t SAMPLE_DIVIDER = 9; // 1000 / (1 + 9) = 100 Hz
constexpr uint32_t SAMPLE_PERIOD_MS = SAMPLE_DIVIDER + 1;

constexpr uint16_t CALIBRATION_SAMPLES = 200; // 2 s at 100 Hz
constexpr int CALIBRATION_ATTEMPTS = 3;
constexpr int COUNTDOWN_S = 3;

constexpr uint16_t CHECK_SAMPLES = 100; // samples used for the before/after comparison
constexpr uint32_t LIVE_PRINT_PERIOD_MS = 200;

// Average of `samples` gyro readings, with or without the stored offset.
Status measure_mean(const mpu6500::Mpu6500& imu, uint16_t samples, bool raw, Vec3& mean) {
    Vec3 sum{};
    for (uint16_t i = 0; i < samples; ++i) {
        Vec3 gyro{};
        const Status status = raw ? imu.read_gyro_raw(gyro) : imu.read_gyro(gyro);
        if (status != Status::OK)
            return status;
        sum += gyro;
        sleep_ms(SAMPLE_PERIOD_MS);
    }
    mean.x = sum.x / samples;
    mean.y = sum.y / samples;
    mean.z = sum.z / samples;
    return Status::OK;
}

void print_vec(const char* label, const Vec3& v) {
    std::printf("  %-24s x=%+8.3f  y=%+8.3f  z=%+8.3f dps\n", label, v.x, v.y, v.z);
}

void countdown() {
    for (int s = COUNTDOWN_S; s > 0; --s) {
        std::printf("  starting in %d...\n", s);
        sleep_ms(1000);
    }
}

Status run_calibration(mpu6500::Mpu6500& imu) {
    const uint32_t duration_ms = CALIBRATION_SAMPLES * SAMPLE_PERIOD_MS;
    Status status = Status::ERROR;

    for (int attempt = 1; attempt <= CALIBRATION_ATTEMPTS; ++attempt) {
        std::printf("  calibrating (attempt %d/%d, ~%lu ms)...\n",
                    attempt,
                    CALIBRATION_ATTEMPTS,
                    static_cast<unsigned long>(duration_ms));

        status = mpu6500::calibration::calibrate_gyro(imu,
                                                      mpu6500::calibration::DEFAULT_GYRO_OPTIONS);
        if (status == Status::OK)
            return status;

        std::printf("  failed: %s (board moved or bus error), retrying in 1 s\n",
                    example::status_text(status));
        sleep_ms(1000);
    }
    return status;
}

} // namespace

int main() {
    stdio_init_all();
    sleep_ms(STARTUP_DELAY_MS);

    std::printf("\n=== MPU6500 gyro calibration ===\n\n");

    // --- setup ---------------------------------------------------------------
    i2c_config::init_test_i2c();
    bus::pico::I2CBus bus{i2c0, i2c_config::DEVICE_ADDR, I2C_TIMEOUT_US};

    mpu6500::config::Config config{};
    config.measurement.gyro.filter = mpu6500::config::GyroFilter::Hz41;
    config.measurement.accel.filter = mpu6500::config::AccelFilter::Hz41;
    config.measurement.sample_divider = SAMPLE_DIVIDER;

    mpu6500::Mpu6500 imu(bus, sleep_ms, config);

    uint8_t id{};
    const Status id_status = imu.who_am_i(id);
    std::printf("WHO_AM_I: 0x%02x (%s)\n", id, example::status_text(id_status));

    const Status init_status = imu.init();
    example::print_status("init", init_status);
    if (init_status != Status::OK)
        example::halt("init failed", init_status);

    std::printf("Filter 41 Hz, sample rate %u Hz\n\n", 1000u / (SAMPLE_DIVIDER + 1u));
    std::printf("Place the board on a flat surface and DO NOT TOUCH IT until told.\n");
    countdown();

    // --- 1. before -----------------------------------------------------------
    std::printf("\n[1] Before calibration (average of %u samples):\n", CHECK_SAMPLES);
    Vec3 before{};
    const Status before_status = measure_mean(imu, CHECK_SAMPLES, false, before);
    if (before_status != Status::OK)
        example::halt("reading gyro failed", before_status);
    print_vec("read_gyro", before);
    std::printf("  -> this is the gyro bias: the board is still, so it should be 0\n");

    // --- 2. calibration ------------------------------------------------------
    std::printf("\n[2] Calibration:\n");
    const Status calibration_status = run_calibration(imu);
    if (calibration_status != Status::OK)
        example::halt("gyro calibration failed", calibration_status);
    print_vec("measured offset", imu.config().calibration.gyro_offset_dps);

    // --- 3. after ------------------------------------------------------------
    std::printf("\n[3] After calibration (average of %u samples):\n", CHECK_SAMPLES);
    Vec3 after{};
    Vec3 after_raw{};
    const Status after_status = measure_mean(imu, CHECK_SAMPLES, false, after);
    const Status after_raw_status = measure_mean(imu, CHECK_SAMPLES, true, after_raw);
    if (after_status != Status::OK || after_raw_status != Status::OK)
        example::halt("reading gyro failed", Status::ERROR);
    print_vec("read_gyro (corrected)", after);
    print_vec("read_gyro_raw", after_raw);
    std::printf("  -> corrected values should be close to 0, raw values unchanged\n");

    // --- 4. live -------------------------------------------------------------
    std::printf("\n[4] Live data: you can move the board now.\n");
    std::printf("    Rotate it slowly and watch the corrected values react,\n");
    std::printf("    then put it down: they should return to about 0.\n\n");
    std::printf("  %-32s | %s\n", "corrected [dps]", "raw [dps]");

    for (;;) {
        Vec3 gyro{};
        Vec3 gyro_raw{};
        const Status status = imu.read_gyro(gyro);
        const Status raw_status = imu.read_gyro_raw(gyro_raw);

        if (status != Status::OK || raw_status != Status::OK) {
            std::printf("  read failed: %s / %s\n",
                        example::status_text(status),
                        example::status_text(raw_status));
        } else {
            std::printf("  %+8.2f %+8.2f %+8.2f        | %+8.2f %+8.2f %+8.2f\n",
                        gyro.x,
                        gyro.y,
                        gyro.z,
                        gyro_raw.x,
                        gyro_raw.y,
                        gyro_raw.z);
        }
        sleep_ms(LIVE_PRINT_PERIOD_MS);
    }
}

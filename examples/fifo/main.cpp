// FIFO demo: the chip buffers samples at 100 Hz, we read them in batches every 100 ms.
// Expect about 10 frames per read. Every few seconds a long pause forces an overflow:
// that read reports OVERFLOW with 0 frames, and the reads after it are normal again.

#include "bus_pico/i2c_bus.hpp"
#include "example_utils.hpp"
#include "i2c_config.hpp"
#include "mpu6500/calibration.hpp"
#include "mpu6500/mpu6500.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <pico/stdlib.h>

namespace {

namespace cfg = mpu6500::config;
namespace cal = mpu6500::calibration;

constexpr uint32_t STARTUP_DELAY_MS = 3000; // time to open the serial terminal
constexpr uint32_t I2C_TIMEOUT_US = 30000;
constexpr uint8_t SAMPLE_DIVIDER = 9; // 1000 / (1 + 9) = 100 Hz

constexpr uint32_t READ_PERIOD_MS = 100;        // ~10 frames per read at 100 Hz
constexpr int READS_PER_REPORT = 10;            // print the measured rate once per second
constexpr int READS_BETWEEN_OVERFLOWS = 50;     // force an overflow every 5 s
constexpr uint32_t OVERFLOW_PAUSE_MS = 1000;    // buffer holds 512 / 12 = 42 frames = 420 ms
constexpr std::size_t MAX_FRAMES_PER_READ = 64; // more than the FIFO can ever hold (42)

void print_frames(int index, std::size_t frames, const mpu6500::Sample& last) {
    std::printf("  read %4d: %2u frames | last: accel z %+6.3f g | gyro %+6.2f %+6.2f %+6.2f dps\n",
                index,
                static_cast<unsigned>(frames),
                last.accel_g.z,
                last.gyro_dps.x,
                last.gyro_dps.y,
                last.gyro_dps.z);
}

} // namespace

int main() {
    stdio_init_all();
    sleep_ms(STARTUP_DELAY_MS);

    std::printf("\n=== MPU6500 FIFO demo ===\n\n");

    // --- setup ---------------------------------------------------------------
    i2c_config::init_test_i2c();
    bus::pico::I2CBus bus{i2c0, i2c_config::DEVICE_ADDR, I2C_TIMEOUT_US};

    cfg::Config config{};
    config.measurement.gyro.filter = cfg::GyroFilter::Hz41;
    config.measurement.accel.filter = cfg::AccelFilter::Hz41;
    config.measurement.sample_divider = SAMPLE_DIVIDER;
    config.fifo.enabled = true;
    config.fifo.sources.accel = true;
    config.fifo.sources.gyro = true;
    config.fifo.sources.temperature = false; // frame = 6 + 6 = 12 bytes
    config.fifo.mode = cfg::FifoMode::Overwrite;

    mpu6500::Mpu6500 imu(bus, sleep_ms, config);

    const bus::Status init_status = imu.init();
    example::print_status("init", init_status);
    if (init_status != bus::Status::OK)
        example::halt("init failed", init_status);

    // --- gyro calibration, so the gyro columns read about 0 at rest ----------
    std::printf("\nKeep the board still, calibrating the gyro...\n");
    const bus::Status calibration_status = cal::calibrate_gyro(imu);
    example::print_status("calibrate_gyro", calibration_status);
    if (calibration_status != bus::Status::OK)
        std::printf("  continuing without gyro calibration\n");

    // The FIFO kept filling (and overflowing) during calibration: start clean.
    const bus::Status reset_status = imu.fifo_reset();
    example::print_status("fifo_reset", reset_status);
    if (reset_status != bus::Status::OK)
        example::halt("fifo reset failed", reset_status);

    const float rate_hz = imu.gyro_sample_rate_hz();
    std::printf("\nSample rate %.1f Hz, reading every %lu ms -> expect about %.0f frames/read.\n",
                rate_hz,
                static_cast<unsigned long>(READ_PERIOD_MS),
                rate_hz * static_cast<float>(READ_PERIOD_MS) / 1000.0f);
    std::printf("Every %d reads the demo pauses %lu ms to force an overflow.\n\n",
                READS_BETWEEN_OVERFLOWS,
                static_cast<unsigned long>(OVERFLOW_PAUSE_MS));

    // --- loop ----------------------------------------------------------------
    std::array<mpu6500::Sample, MAX_FRAMES_PER_READ> samples{};

    absolute_time_t next_read = get_absolute_time();
    uint64_t window_start_us = time_us_64();
    std::size_t frames_in_window = 0;
    int reads_in_window = 0;

    for (int read_index = 1;; ++read_index) {
        if (read_index % READS_BETWEEN_OVERFLOWS == 0) {
            std::printf("\n  -- pausing %lu ms, the FIFO will overflow --\n",
                        static_cast<unsigned long>(OVERFLOW_PAUSE_MS));
            sleep_ms(OVERFLOW_PAUSE_MS);
            next_read = get_absolute_time();
            // the pause would distort the rate measurement, start a new window
            window_start_us = time_us_64();
            frames_in_window = 0;
            reads_in_window = 0;
        } else {
            next_read = delayed_by_ms(next_read, READ_PERIOD_MS);
            sleep_until(next_read);
        }

        mpu6500::FifoReadResult result{};
        const bus::Status status = imu.read_fifo(samples, result);
        if (status != bus::Status::OK) {
            std::printf("  read %4d: failed (%s)\n", read_index, example::status_text(status));
            continue;
        }

        if (result.overflowed) {
            std::printf("  read %4d: OVERFLOW, FIFO was reset, %u frames returned\n\n",
                        read_index,
                        static_cast<unsigned>(result.frames));
        } else if (result.frames == 0) {
            std::printf("  read %4d: no frames\n", read_index);
        } else {
            print_frames(read_index, result.frames, samples[result.frames - 1]);
        }

        frames_in_window += result.frames;
        if (++reads_in_window == READS_PER_REPORT) {
            const uint64_t now_us = time_us_64();
            const float seconds = static_cast<float>(now_us - window_start_us) * 1e-6f;
            std::printf("  -> measured %.1f frames/s (expected %.1f)\n",
                        static_cast<float>(frames_in_window) / seconds,
                        rate_hz);
            window_start_us = now_us;
            frames_in_window = 0;
            reads_in_window = 0;
        }
    }
}

// Interrupt demo: the MPU6500 sends a short pulse on its INT pin for every new sample (100 Hz).
// The GPIO interrupt only raises a flag, the main loop asks the driver what happened
// and reads the sample. Expect about 100 events per second.

#include "bus_pico/i2c_bus.hpp"
#include "example_utils.hpp"
#include "i2c_config.hpp"
#include "mpu6500/calibration.hpp"
#include "mpu6500/config.hpp"
#include "mpu6500/mpu6500.hpp"
#include "mpu6500/sample.hpp"
#include <cstdint>
#include <cstdio>
#include <pico/stdlib.h>

namespace {

namespace cfg = mpu6500::config;
namespace cal = mpu6500::calibration;

constexpr uint GPIO_INT = 14; // MPU6500 INT pin -> Pico GPIO, change to your wiring
constexpr uint32_t STARTUP_DELAY_MS = 3000;
constexpr uint32_t I2C_TIMEOUT_US = 30000;
constexpr uint8_t SAMPLE_DIVIDER = 0;            // 1000 / (1 + 9) = 100 Hz
constexpr uint64_t REPORT_PERIOD_US = 1'000'000; // print once per second

// Set by the GPIO interrupt, cleared by the main loop. No I2C in the interrupt.
volatile bool pending_int = false;

void irq_callback(uint gpio, uint32_t events) {
    if (gpio == GPIO_INT && (events & GPIO_IRQ_EDGE_RISE))
        pending_int = true;
}

} // namespace

int main() {
    stdio_init_all();
    sleep_ms(STARTUP_DELAY_MS);

    std::printf("\n=== MPU6500 interrupt demo ===\n\n");

    // --- sensor setup ----------------------------------------------------------
    i2c_config::init_test_i2c();
    bus::pico::I2CBus bus{i2c0, i2c_config::DEVICE_ADDR, I2C_TIMEOUT_US};

    cfg::Config config{};
    config.measurement.gyro.filter = cfg::GyroFilter::Hz184;
    config.measurement.accel.filter = cfg::AccelFilter::Hz184;
    config.measurement.sample_divider = SAMPLE_DIVIDER;
    config.interrupts.sources.raw_data_ready = true;
    config.interrupts.mode = cfg::IntMode::Pulse;        // 50 us pulse per event, releases itself
    config.interrupts.level = cfg::IntLevel::ActiveHigh; // matches GPIO_IRQ_EDGE_RISE below
    config.interrupts.drive = cfg::IntDrive::PushPull;

    mpu6500::Mpu6500 imu{bus, sleep_ms, config};

    const Status init_status = imu.init();
    example::print_status("init", init_status);
    if (init_status != Status::OK)
        example::halt("init failed", init_status);

    // --- calibration (board flat, chip up, do not touch) -----------------------
    std::printf("\nPlace the board flat, chip facing up, and do not touch it...\n");
    sleep_ms(2000);
    const Status gyro_status = cal::calibrate_gyro(imu);
    example::print_status("calibrate_gyro", gyro_status);
    const Status accel_status = cal::calibrate_accel(imu);
    example::print_status("calibrate_accel", accel_status);
    if (gyro_status != Status::OK || accel_status != Status::OK)
        std::printf("  calibration failed, continuing with uncalibrated data\n");

    // --- GPIO interrupt setup --------------------------------------------------
    gpio_init(GPIO_INT);
    gpio_set_dir(GPIO_INT, GPIO_IN);
    gpio_pull_down(GPIO_INT); // keeps the pin low if the wire is disconnected
    gpio_set_irq_enabled_with_callback(GPIO_INT, GPIO_IRQ_EDGE_RISE, true, irq_callback);
    std::printf("\nGPIO %u configured, rising edge interrupt enabled\n", GPIO_INT);

    // In pulse mode the pin releases itself, but flags collected during calibration are
    // still pending in the driver: take them once so the loop starts clean.
    InterruptFlags flags{};
    const Status clear_status = imu.take_interrupt_flags(flags);
    example::print_status("take_interrupt_flags", clear_status);
    std::printf("\n");

    // --- loop ------------------------------------------------------------------
    Sample last_sample{};
    uint32_t events = 0;
    uint32_t samples = 0;
    uint32_t failed_reads = 0;
    uint64_t last_report_us = time_us_64();

    for (;;) {
        if (pending_int) {
            pending_int = false; // clear first: an interrupt during the reads below is kept
            ++events;

            const Status flags_status = imu.take_interrupt_flags(flags);
            if (flags_status != Status::OK) {
                ++failed_reads;
            } else if (flags.raw_data_ready) {
                if (imu.read_all(last_sample) == Status::OK)
                    ++samples;
                else
                    ++failed_reads;
            }
        }

        const uint64_t now_us = time_us_64();
        if (now_us - last_report_us >= REPORT_PERIOD_US) {
            std::printf("events %3lu/s  samples %3lu/s  failed %lu | "
                        "accel %+6.3f %+6.3f %+6.3f g | gyro %+6.2f %+6.2f %+6.2f dps\n",
                        static_cast<unsigned long>(events),
                        static_cast<unsigned long>(samples),
                        static_cast<unsigned long>(failed_reads),
                        last_sample.accel_g.x,
                        last_sample.accel_g.y,
                        last_sample.accel_g.z,
                        last_sample.gyro_dps.x,
                        last_sample.gyro_dps.y,
                        last_sample.gyro_dps.z);

            if (events == 0)
                std::printf("  WARNING: no interrupts in the last second, check the INT wire "
                            "and GPIO_INT\n");

            events = 0;
            samples = 0;
            failed_reads = 0;
            last_report_us = now_us;
        }

        tight_loop_contents();
    }
}

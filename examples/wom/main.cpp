// Wake-on-Motion demo.
// The MPU6500 runs in low-power accel mode: the gyro is off and the accelerometer wakes up
// ~31 times per second. On every wake-up the chip compares the new sample with the previous
// one; if any axis changed by more than the threshold, it pulses the INT pin.
// Leave the board alone: nothing happens. Tap or move it: motion events are printed.

#include "bus_pico/i2c_bus.hpp"
#include "example_utils.hpp"
#include "i2c_config.hpp"
#include "mpu6500/config.hpp"
#include "mpu6500/mpu6500.hpp"
#include "mpu6500/sample.hpp"
#include <cstdint>
#include <cstdio>
#include <pico/stdlib.h>

namespace {

namespace cfg = mpu6500::config;

constexpr uint GPIO_INT = 14; // MPU6500 INT pin -> Pico GPIO, change to your wiring
constexpr uint32_t STARTUP_DELAY_MS = 3000;
constexpr uint32_t I2C_TIMEOUT_US = 3000;
constexpr uint16_t THRESHOLD_MG = 100; // change between two samples that counts as motion
constexpr cfg::LowPowerAccelRate WAKE_RATE = cfg::LowPowerAccelRate::Hz31_25;

constexpr uint64_t MOTION_END_US = 500'000;      // no event for 0.5 s -> motion is over
constexpr uint64_t IDLE_REPORT_US = 5'000'000;   // "still idle" line every 5 s

// Set by the GPIO interrupt, cleared by the main loop. No I2C in the interrupt.
static volatile bool pending_int = false;

void irq_callback(uint gpio, uint32_t events) {
    if (gpio == GPIO_INT && (events & GPIO_IRQ_EDGE_RISE))
        pending_int = true;
}

uint32_t ms_now() {
    return to_ms_since_boot(get_absolute_time());
}

} // namespace

int main() {
    stdio_init_all();
    sleep_ms(STARTUP_DELAY_MS);

    std::printf("\n=== MPU6500 wake-on-motion demo ===\n\n");

    // --- sensor setup ----------------------------------------------------------
    i2c_config::init_test_i2c();
    bus::pico::I2CBus bus{i2c0, i2c_config::DEVICE_ADDR, I2C_TIMEOUT_US};

    cfg::Config config{};
    config.power.mode = cfg::PowerMode::LowPowerAccel; // gyro off, accel duty-cycled
    config.power.low_power_rate = WAKE_RATE;
    config.wake_on_motion.enabled = true;
    config.wake_on_motion.threshold_mg = THRESHOLD_MG;
    config.interrupts.sources.wake_on_motion = true;
    config.interrupts.mode = cfg::IntMode::Pulse; // one short pulse per motion event
    config.interrupts.level = cfg::IntLevel::ActiveHigh;
    config.interrupts.drive = cfg::IntDrive::PushPull;

    mpu6500::Mpu6500 imu{bus, sleep_ms, config};

    const Status init_status = imu.init();
    example::print_status("init", init_status);
    if (init_status != Status::OK)
        example::halt("init failed", init_status);

    std::printf("  low-power accel at %.2f Hz, motion threshold %u mg\n",
                imu.accel_sample_rate_hz(),
                THRESHOLD_MG);

    // --- GPIO interrupt setup --------------------------------------------------
    gpio_init(GPIO_INT);
    gpio_set_dir(GPIO_INT, GPIO_IN);
    gpio_pull_down(GPIO_INT); // keeps the pin low if the wire is disconnected
    gpio_set_irq_enabled_with_callback(GPIO_INT, GPIO_IRQ_EDGE_RISE, true, irq_callback);
    std::printf("  GPIO %u configured, rising edge interrupt enabled\n", GPIO_INT);

    // Drop anything that was flagged while the board was being picked up and plugged in.
    InterruptFlags flags{};
    const Status clear_status = imu.take_interrupt_flags(flags);
    example::print_status("take_interrupt_flags", clear_status);

    std::printf("\nLeave the board still: nothing should be printed.\n");
    std::printf("Tap or move it: a motion burst is printed with its length and event count.\n\n");

    // --- loop ------------------------------------------------------------------
    bool in_motion = false;
    uint32_t motion_start_ms = 0;
    uint32_t events_in_burst = 0;
    uint32_t bursts_total = 0;
    uint64_t last_event_us = time_us_64();
    uint64_t last_idle_report_us = time_us_64();

    for (;;) {
        if (pending_int) {
            pending_int = false; // clear first: an interrupt during the reads below is kept

            const Status flags_status = imu.take_interrupt_flags(flags);
            if (flags_status != Status::OK) {
                std::printf("[%7lu ms] take_interrupt_flags failed: %s\n",
                            static_cast<unsigned long>(ms_now()),
                            example::status_text(flags_status));
            } else if (flags.wake_on_motion) {
                last_event_us = time_us_64();
                ++events_in_burst;

                if (!in_motion) {
                    in_motion = true;
                    motion_start_ms = ms_now();
                    ++bursts_total;

                    Vec3 accel{};
                    const Status read_status = imu.read_accel(accel);
                    if (read_status == Status::OK)
                        std::printf("[%7lu ms] MOTION #%lu started | "
                                    "accel %+6.3f %+6.3f %+6.3f g\n",
                                    static_cast<unsigned long>(motion_start_ms),
                                    static_cast<unsigned long>(bursts_total),
                                    accel.x,
                                    accel.y,
                                    accel.z);
                    else
                        std::printf("[%7lu ms] MOTION #%lu started (accel read failed: %s)\n",
                                    static_cast<unsigned long>(motion_start_ms),
                                    static_cast<unsigned long>(bursts_total),
                                    example::status_text(read_status));
                }
            }
        }

        const uint64_t now_us = time_us_64();

        // No event for a while: the burst is over.
        if (in_motion && now_us - last_event_us > MOTION_END_US) {
            const uint32_t end_ms = ms_now();
            std::printf("[%7lu ms] motion stopped after %lu ms, %lu events\n",
                        static_cast<unsigned long>(end_ms),
                        static_cast<unsigned long>(end_ms - motion_start_ms -
                                                   static_cast<uint32_t>(MOTION_END_US / 1000)),
                        static_cast<unsigned long>(events_in_burst));
            in_motion = false;
            events_in_burst = 0;
            last_idle_report_us = now_us;
        }

        // Proof of life while nothing happens.
        if (!in_motion && now_us - last_idle_report_us >= IDLE_REPORT_US) {
            std::printf("[%7lu ms] still, %lu motion bursts so far\n",
                        static_cast<unsigned long>(ms_now()),
                        static_cast<unsigned long>(bursts_total));
            last_idle_report_us = now_us;
        }

        tight_loop_contents();
    }
}

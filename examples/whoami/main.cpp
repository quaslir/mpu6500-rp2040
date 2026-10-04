// Output rates: what the driver predicts vs what the chip really does.
// For every filter / divider combination the demo prints the rates returned by
// gyro_sample_rate_hz() / accel_sample_rate_hz() and three measured rates:
//   - data ready: how often INT_STATUS reports new data (register update rate);
//   - gyro / accel: how often the value read from the chip actually changes.
// The result is a Markdown table for docs/measurements.md.
//
// Build with IMU_BUS=SPI: measuring kHz rates needs fast reads. Over I2C only the
// low rates (up to a few hundred Hz) can be measured.

#if defined(IMU_BUS_SPI)
#include "bus_pico/spi_bus.hpp"
#include "spi_config.hpp"
#else
#include "bus_pico/i2c_bus.hpp"
#include "i2c_config.hpp"
#endif

#include "example_utils.hpp"
#include "mpu6500/config.hpp"
#include "mpu6500/mpu6500.hpp"
#include "mpu6500/sample.hpp"
#include <array>
#include <cstdint>
#include <cstdio>
#include <pico/stdlib.h>

namespace {

namespace cfg = mpu6500::config;

constexpr uint32_t STARTUP_DELAY_MS = 3000;
constexpr uint32_t SETTLE_MS = 50;              // let the new filter settle
constexpr uint64_t MEASURE_US = 500'000;        // one measurement pass
constexpr float MATCH_TOLERANCE = 0.05f;        // 5 %

#if defined(IMU_BUS_SPI)
constexpr const char* BUS_NAME = "SPI";
constexpr uint32_t WRITE_CLOCK_HZ = 1'000'000;  // datasheet limit for register writes
constexpr uint32_t READ_CLOCK_HZ = 8'000'000;   // reads only
#else
constexpr const char* BUS_NAME = "I2C";
constexpr uint32_t WRITE_CLOCK_HZ = 400'000;
constexpr uint32_t READ_CLOCK_HZ = 400'000;
#endif

struct Step {
    const char* name;
    cfg::GyroFilter gyro;
    cfg::AccelFilter accel;
    uint8_t divider;
};

constexpr std::array<Step, 10> STEPS{{
    {"DLPF 184, div 0", cfg::GyroFilter::Hz184, cfg::AccelFilter::Hz184, 0},
    {"DLPF 184, div 1", cfg::GyroFilter::Hz184, cfg::AccelFilter::Hz184, 1},
    {"DLPF 92, div 4", cfg::GyroFilter::Hz92, cfg::AccelFilter::Hz92, 4},
    {"DLPF 41, div 9", cfg::GyroFilter::Hz41, cfg::AccelFilter::Hz41, 9},
    {"DLPF 20, div 19", cfg::GyroFilter::Hz20, cfg::AccelFilter::Hz20, 19},
    {"DLPF 5, div 99", cfg::GyroFilter::Hz5, cfg::AccelFilter::Hz5, 99},
    {"gyro 250, accel 460, div 9 (ignored)", cfg::GyroFilter::Hz250, cfg::AccelFilter::Hz460, 9},
    {"gyro 3600, accel 460", cfg::GyroFilter::Hz3600, cfg::AccelFilter::Hz460, 0},
    {"gyro bypass 3600, accel bypass", cfg::GyroFilter::Bypass3600Hz,
     cfg::AccelFilter::Bypass1130Hz, 0},
    {"gyro bypass 8800, accel bypass", cfg::GyroFilter::Bypass8800Hz,
     cfg::AccelFilter::Bypass1130Hz, 0},
}};

struct Rate {
    float hz = 0.0f;          // measured events per second
    float max_hz = 0.0f;      // reads per second: rates above about half of it are not reliable
    uint32_t errors = 0;
};

// Bus clock ---------------------------------------------------------------------------

void set_clock(uint32_t hz) {
#if defined(IMU_BUS_SPI)
    spi_set_baudrate(spi0, hz);
#else
    i2c_set_baudrate(i2c0, hz);
#endif
}

// Measurements ------------------------------------------------------------------------

// How often the value returned by `read` changes. Identical consecutive values are not
// counted, so a very narrow filter (little noise) can make this an underestimate.
template <typename Read> Rate count_changes(Read read) {
    Rate rate{};
    math::Vec3 previous{};
    math::Vec3 current{};
    if (read(previous) != bus::Status::OK) {
        ++rate.errors;
        return rate;
    }

    uint32_t reads = 0;
    uint32_t changes = 0;
    const uint64_t start = time_us_64();
    uint64_t now = start;
    while (now - start < MEASURE_US) {
        if (read(current) == bus::Status::OK) {
            ++reads;
            if (current.x != previous.x || current.y != previous.y || current.z != previous.z)
                ++changes;
            previous = current;
        } else {
            ++rate.errors;
        }
        now = time_us_64();
    }
    const float seconds = static_cast<float>(now - start) * 1e-6f;
    rate.hz = static_cast<float>(changes) / seconds;
    rate.max_hz = static_cast<float>(reads) / seconds;
    return rate;
}

// How often INT_STATUS reports new data. Every read clears the flag, so each new sample
// is counted once as long as we poll faster than the sample rate.
Rate count_data_ready(mpu6500::Mpu6500& imu) {
    Rate rate{};
    mpu6500::InterruptFlags flags{};
    (void)imu.take_interrupt_flags(flags); // start clean, result not needed

    uint32_t polls = 0;
    uint32_t events = 0;
    const uint64_t start = time_us_64();
    uint64_t now = start;
    while (now - start < MEASURE_US) {
        if (imu.take_interrupt_flags(flags) == bus::Status::OK) {
            ++polls;
            if (flags.raw_data_ready)
                ++events;
        } else {
            ++rate.errors;
        }
        now = time_us_64();
    }
    const float seconds = static_cast<float>(now - start) * 1e-6f;
    rate.hz = static_cast<float>(events) / seconds;
    rate.max_hz = static_cast<float>(polls) / seconds;
    return rate;
}

// Table ---------------------------------------------------------------------------------

const char* verdict(float predicted, const Rate& measured) {
    if (measured.errors > 0)
        return "bus errors";
    if (predicted > measured.max_hz / 2.0f)
        return "too fast to measure";
    const float diff = measured.hz - predicted;
    const float relative = diff < 0.0f ? -diff / predicted : diff / predicted;
    return relative <= MATCH_TOLERANCE ? "ok" : "MISMATCH";
}

void print_header() {
    std::printf("\n| Config | Gyro predicted, Hz | Accel predicted, Hz | Data ready, Hz | "
                "Gyro measured, Hz | Accel measured, Hz | Gyro | Accel |\n");
    std::printf("|---|---:|---:|---:|---:|---:|---|---|\n");
}

void print_row(const Step& step,
               float gyro_predicted,
               float accel_predicted,
               const Rate& data_ready,
               const Rate& gyro,
               const Rate& accel) {
    std::printf("| %s | %.1f | %.1f | %.1f | %.1f | %.1f | %s | %s |\n",
                step.name,
                gyro_predicted,
                accel_predicted,
                data_ready.hz,
                gyro.hz,
                accel.hz,
                verdict(gyro_predicted, gyro),
                verdict(accel_predicted, accel));
}

} // namespace

int main() {
    stdio_init_all();
    sleep_ms(STARTUP_DELAY_MS);

    std::printf("\n=== MPU6500 output rates (%s) ===\n", BUS_NAME);

    // --- setup ---------------------------------------------------------------
#if defined(IMU_BUS_SPI)
    spi_config::init_test_spi();
    bus::pico::SPIBus bus{spi0, spi_config::CS};
#else
    i2c_config::init_test_i2c();
    bus::pico::I2CBus bus{i2c0, i2c_config::DEVICE_ADDR, 30000};
#endif

    cfg::Config config{};
#if defined(IMU_BUS_SPI)
    config.use_i2c = false;
#endif
    mpu6500::Mpu6500 imu{bus, sleep_ms, config};

    set_clock(WRITE_CLOCK_HZ);
    const bus::Status init_status = imu.init();
    example::print_status("init", init_status);
    if (init_status != bus::Status::OK)
        example::halt("init failed", init_status);

    std::printf("\nEach measurement runs for %lu ms. Keep the board still or move it, both work:\n",
                static_cast<unsigned long>(MEASURE_US / 1000));
    std::printf("the change counters only need the values to differ between samples.\n");
    std::printf("'too fast to measure' means the bus cannot read often enough for that rate.\n");

    // --- table ---------------------------------------------------------------
    print_header();
    for (const Step& step : STEPS) {
        set_clock(WRITE_CLOCK_HZ); // configuration is written at the slow clock
        const bus::Status g = imu.set_gyro_filter(step.gyro);
        const bus::Status a = imu.set_accel_filter(step.accel);
        const bus::Status d = imu.set_sample_rate_divider(step.divider);
        if (g != bus::Status::OK || a != bus::Status::OK || d != bus::Status::OK) {
            std::printf("| %s | configuration failed |||||||\n", step.name);
            continue;
        }
        sleep_ms(SETTLE_MS);

        const float gyro_predicted = imu.gyro_sample_rate_hz();
        const float accel_predicted = imu.accel_sample_rate_hz();

        set_clock(READ_CLOCK_HZ); // measurements only read
        const Rate data_ready = count_data_ready(imu);
        const Rate gyro = count_changes(
            [&](math::Vec3& v) { return imu.read_gyro_uncorrected(v); });
        const Rate accel = count_changes(
            [&](math::Vec3& v) { return imu.read_accel_uncorrected(v); });

        print_row(step, gyro_predicted, accel_predicted, data_ready, gyro, accel);
    }

    set_clock(WRITE_CLOCK_HZ);
    std::printf("\nDone.\n");
    for (;;)
        tight_loop_contents();
}

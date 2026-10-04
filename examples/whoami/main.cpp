// Benchmark: how long driver operations take on the current bus.
// Every operation is called many times in a row and timed with time_us_32().
// The result is printed as a Markdown table, ready to paste into docs/measurements.md.
//
// Build with IMU_BUS=SPI for SPI, otherwise I2C is used. On I2C the table is measured
// at 100 kHz and 400 kHz. On SPI it is measured at 1 MHz (the datasheet limit for
// register writes); read-only operations are additionally measured at a faster clock.

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
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <pico/stdlib.h>
#include <span>

namespace {

namespace cfg = mpu6500::config;

constexpr uint32_t STARTUP_DELAY_MS = 3000;
constexpr uint32_t READ_ITERATIONS = 1000;
constexpr uint32_t FIFO_ITERATIONS = 50;
constexpr uint32_t INIT_ITERATIONS = 3;
constexpr uint32_t FIFO_WAIT_TIMEOUT_US = 200'000;
constexpr std::size_t FIFO_FRAME_BYTES = 12;     // accel + gyro
constexpr std::size_t MAX_FIFO_FRAMES = 64;
constexpr std::array<std::size_t, 3> FIFO_FRAMES{1, 10, 40}; // 40 * 12 = 480 of 512 bytes

#if defined(IMU_BUS_SPI)
constexpr const char* BUS_NAME = "SPI";
constexpr std::array<uint32_t, 1> CLOCKS_HZ{1'000'000};
constexpr uint32_t FAST_READ_CLOCK_HZ = 8'000'000; // reads only, datasheet allows up to 20 MHz
#else
constexpr const char* BUS_NAME = "I2C";
constexpr std::array<uint32_t, 2> CLOCKS_HZ{100'000, 400'000};
#endif

// Bus clock -------------------------------------------------------------------------

uint32_t set_clock(uint32_t hz) {
#if defined(IMU_BUS_SPI)
    return spi_set_baudrate(spi0, hz);
#else
    return i2c_set_baudrate(i2c0, hz);
#endif
}

// Theoretical time the bytes spend on the wire, for one register read transaction.
// I2C: START, address+W, register, repeated START, address+R, data, STOP; 9 bits per byte.
// SPI: one address byte plus the data bytes, 8 bits each.
float wire_us(std::size_t data_bytes, uint32_t clock_hz) {
#if defined(IMU_BUS_SPI)
    const float bits = 8.0f * static_cast<float>(1 + data_bytes);
#else
    const float bits = 9.0f * static_cast<float>(3 + data_bytes) + 3.0f;
#endif
    return bits * 1e6f / static_cast<float>(clock_hz);
}

// Statistics ------------------------------------------------------------------------

struct Stats {
    uint32_t min_us = UINT32_MAX;
    uint32_t max_us = 0;
    uint64_t sum_us = 0;
    uint32_t count = 0;
    uint32_t errors = 0;

    void add(uint32_t us) {
        min_us = us < min_us ? us : min_us;
        max_us = us > max_us ? us : max_us;
        sum_us += us;
        ++count;
    }
    float mean_us() const {
        return count ? static_cast<float>(sum_us) / static_cast<float>(count) : 0.0f;
    }
};

// Calls op() `iterations` times, op returns bus::Status.
template <typename Op> Stats measure(uint32_t iterations, Op op) {
    Stats stats{};
    for (uint32_t i = 0; i < iterations; ++i) {
        const uint32_t start = time_us_32();
        const bus::Status status = op();
        const uint32_t elapsed = time_us_32() - start;
        if (status == bus::Status::OK)
            stats.add(elapsed);
        else
            ++stats.errors;
    }
    return stats;
}

// Table -----------------------------------------------------------------------------

void print_header() {
    std::printf("\n| operation | bus | clock | bytes | mean, us | min, us | max, us | "
                "wire, us | overhead | errors |\n");
    std::printf("|---|---|---|---|---|---|---|---|---|---|\n");
}

void print_row(const char* name, uint32_t clock_hz, std::size_t bytes, float wire, const Stats& s) {
    const float mean = s.mean_us();
    const float overhead = mean > 0.0f ? (mean - wire) / mean * 100.0f : 0.0f;
    std::printf("| %s | %s | %lu kHz | %u | %.1f | %lu | %lu | %.1f | %.0f%% | %lu |\n",
                name,
                BUS_NAME,
                static_cast<unsigned long>(clock_hz / 1000),
                static_cast<unsigned>(bytes),
                mean,
                static_cast<unsigned long>(s.count ? s.min_us : 0),
                static_cast<unsigned long>(s.max_us),
                wire,
                overhead,
                static_cast<unsigned long>(s.errors));
}

// Benchmarks ------------------------------------------------------------------------

void bench_reads(mpu6500::Mpu6500& imu, uint32_t clock_hz) {
    mpu6500::Sample sample{};
    math::Vec3 vec{};
    float temp{};
    uint8_t id{};
    mpu6500::InterruptFlags flags{};

    print_row("read_all",
              clock_hz,
              14,
              wire_us(14, clock_hz),
              measure(READ_ITERATIONS, [&] { return imu.read_all(sample); }));
    print_row("read_accel",
              clock_hz,
              6,
              wire_us(6, clock_hz),
              measure(READ_ITERATIONS, [&] { return imu.read_accel(vec); }));
    print_row("read_gyro",
              clock_hz,
              6,
              wire_us(6, clock_hz),
              measure(READ_ITERATIONS, [&] { return imu.read_gyro(vec); }));
    print_row("read_temp",
              clock_hz,
              2,
              wire_us(2, clock_hz),
              measure(READ_ITERATIONS, [&] { return imu.read_temp(temp); }));
    print_row("who_am_i",
              clock_hz,
              1,
              wire_us(1, clock_hz),
              measure(READ_ITERATIONS, [&] { return imu.who_am_i(id); }));
    print_row("take_interrupt_flags",
              clock_hz,
              1,
              wire_us(1, clock_hz),
              measure(READ_ITERATIONS, [&] { return imu.take_interrupt_flags(flags); }));
}

// Waits until the FIFO holds at least `frames` frames.
bool wait_for_frames(mpu6500::Mpu6500& imu, std::size_t frames) {
    const uint32_t start = time_us_32();
    uint16_t count{};
    while (time_us_32() - start < FIFO_WAIT_TIMEOUT_US) {
        if (imu.fifo_frame_count(count) == bus::Status::OK && count >= frames)
            return true;
    }
    return false;
}

void bench_fifo(mpu6500::Mpu6500& imu, uint32_t clock_hz) {
    std::array<mpu6500::Sample, MAX_FIFO_FRAMES> buffer{};

    for (const std::size_t frames : FIFO_FRAMES) {
        Stats stats{};
        for (uint32_t i = 0; i < FIFO_ITERATIONS; ++i) {
            if (imu.fifo_reset() != bus::Status::OK || !wait_for_frames(imu, frames)) {
                ++stats.errors;
                continue;
            }
            mpu6500::FifoReadResult result{};
            const uint32_t start = time_us_32();
            const bus::Status status =
                imu.read_fifo(std::span{buffer}.first(frames), result);
            const uint32_t elapsed = time_us_32() - start;
            if (status == bus::Status::OK && result.frames == frames && !result.overflowed)
                stats.add(elapsed);
            else
                ++stats.errors;
        }

        // read_fifo = INT_STATUS (1 byte) + FIFO_COUNT (2 bytes) + data, three transactions.
        const std::size_t data_bytes = frames * FIFO_FRAME_BYTES;
        const float wire =
            wire_us(1, clock_hz) + wire_us(2, clock_hz) + wire_us(data_bytes, clock_hz);

        char name[32];
        std::snprintf(name, sizeof(name), "read_fifo %u frames", static_cast<unsigned>(frames));
        print_row(name, clock_hz, data_bytes + 3, wire, stats);
    }
}

void bench_init(mpu6500::Mpu6500& imu) {
    Stats stats = measure(INIT_ITERATIONS, [&] { return imu.init(); });
    std::printf("\ninit: mean %.1f ms, min %.1f ms, max %.1f ms, errors %lu "
                "(includes 2 x %u ms reset waits)\n",
                stats.mean_us() / 1000.0f,
                static_cast<float>(stats.count ? stats.min_us : 0) / 1000.0f,
                static_cast<float>(stats.max_us) / 1000.0f,
                static_cast<unsigned long>(stats.errors),
                100u);
}

} // namespace

int main() {
    stdio_init_all();
    sleep_ms(STARTUP_DELAY_MS);

    std::printf("\n=== MPU6500 benchmark (%s) ===\n", BUS_NAME);
    std::printf("Every read is repeated %lu times, FIFO reads %lu times.\n",
                static_cast<unsigned long>(READ_ITERATIONS),
                static_cast<unsigned long>(FIFO_ITERATIONS));

    // --- setup ---------------------------------------------------------------
#if defined(IMU_BUS_SPI)
    spi_config::init_test_spi();
    bus::pico::SPIBus bus{spi0, spi_config::CS};
#else
    i2c_config::init_test_i2c();
    bus::pico::I2CBus bus{i2c0, i2c_config::DEVICE_ADDR, 100000};
#endif

    // 1 kHz, so the FIFO fills fast enough for the FIFO benchmark.
    cfg::Config config{};
#if defined(IMU_BUS_SPI)
    config.use_i2c = false;
#endif
    config.measurement.gyro.filter = cfg::GyroFilter::Hz184;
    config.measurement.accel.filter = cfg::AccelFilter::Hz184;
    config.measurement.sample_divider = 0;
    config.fifo.enabled = true; // accel + gyro, 12 bytes per frame

    mpu6500::Mpu6500 imu{bus, sleep_ms, config};

    set_clock(CLOCKS_HZ[0]);
    const bus::Status init_status = imu.init();
    example::print_status("init", init_status);
    if (init_status != bus::Status::OK)
        example::halt("init failed", init_status);

    // --- operation timing ----------------------------------------------------
    std::printf("\nwire = theoretical time on the bus, overhead = everything else "
                "(SDK, driver, decoding).\n");
    print_header();
    for (const uint32_t clock : CLOCKS_HZ) {
        const uint32_t actual = set_clock(clock);
        bench_reads(imu, actual);
        bench_fifo(imu, actual);
    }

#if defined(IMU_BUS_SPI)
    // Reads only: register writes must stay at or below 1 MHz.
    const uint32_t fast = set_clock(FAST_READ_CLOCK_HZ);
    bench_reads(imu, fast);
    set_clock(CLOCKS_HZ[0]);
#endif

    // --- init ------------------------------------------------------------------
    set_clock(CLOCKS_HZ[0]);
    bench_init(imu);

    std::printf("\nDone.\n");
    for (;;)
        tight_loop_contents();
}

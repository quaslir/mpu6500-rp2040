#include "bus_pico/i2c_bus.hpp"
#include "i2c_config.hpp"
#include "mpu6500/mpu6500.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <pico/stdlib.h>

namespace {

constexpr int SAMPLES_PER_STEP = 200;       // 2 s per filter at 100 Hz
constexpr uint32_t SAMPLE_PERIOD_MS = 10;   // matches divider 9 -> 100 Hz
constexpr uint32_t SETTLE_TIME_MS = 100;    // let the new filter settle before measuring
constexpr uint32_t STARTUP_DELAY_MS = 3000; // time to open the serial terminal

// Collects min, max, mean and standard deviation of one axis
struct AxisStats {
    float sum{};
    float sum_sq{};
    float min{std::numeric_limits<float>::max()};
    float max{std::numeric_limits<float>::lowest()};
    int count{};

    void add(float v) {
        sum += v;
        sum_sq += v * v;
        min = v < min ? v : min;
        max = v > max ? v : max;
        ++count;
    }

    float mean() const { return count ? sum / count : 0.0f; }

    float stddev() const {
        if (count < 2)
            return 0.0f;
        const float m = mean();
        const float variance = sum_sq / count - m * m;
        return variance > 0.0f ? std::sqrt(variance) : 0.0f;
    }

    float peak_to_peak() const { return count ? max - min : 0.0f; }
};

struct Vec3Stats {
    AxisStats x, y, z;

    void add(const Vec3& v) {
        x.add(v.x);
        y.add(v.y);
        z.add(v.z);
    }
};

struct FilterStep {
    const char* name;
    mpu6500::config::GyroFilter gyro;
    mpu6500::config::AccelFilter accel;
};

constexpr std::array<FilterStep, 7> FILTER_STEPS{{
    {"widest (gyro 250 / accel 460 Hz)",
     mpu6500::config::GyroFilter::Hz250,
     mpu6500::config::AccelFilter::Hz460},
    {"184 Hz", mpu6500::config::GyroFilter::Hz184, mpu6500::config::AccelFilter::Hz184},
    {"92 Hz", mpu6500::config::GyroFilter::Hz92, mpu6500::config::AccelFilter::Hz92},
    {"41 Hz", mpu6500::config::GyroFilter::Hz41, mpu6500::config::AccelFilter::Hz41},
    {"20 Hz", mpu6500::config::GyroFilter::Hz20, mpu6500::config::AccelFilter::Hz20},
    {"10 Hz", mpu6500::config::GyroFilter::Hz10, mpu6500::config::AccelFilter::Hz10},
    {"5 Hz", mpu6500::config::GyroFilter::Hz5, mpu6500::config::AccelFilter::Hz5},
}};

const char* status_text(Status status) {
    switch (status) {
        case Status::OK:
            return "OK";
        case Status::TIMEOUT:
            return "TIMEOUT";
        case Status::NACK:
            return "NACK";
        case Status::ERROR:
            return "ERROR";
    }
    return "UNKNOWN";
}

void print_status(const char* what, Status status) {
    std::printf("  %-28s %s\n", what, status_text(status));
}

[[noreturn]] void halt(const char* reason, Status status) {
    for (;;) {
        std::printf("FATAL: %s (%s). Check wiring and restart.\n", reason, status_text(status));
        sleep_ms(2000);
    }
}

bool apply_filter(mpu6500::Mpu6500& imu, const FilterStep& step) {
    const Status gyro_status = imu.set_gyro_filter(step.gyro);
    const Status accel_status = imu.set_accel_filter(step.accel);
    if (gyro_status != Status::OK || accel_status != Status::OK) {
        print_status("set_gyro_filter", gyro_status);
        print_status("set_accel_filter", accel_status);
        return false;
    }
    return true;
}

void measure_and_print(mpu6500::Mpu6500& imu, const FilterStep& step) {
    Vec3Stats accel{};
    Vec3Stats gyro{};
    int failed_reads = 0;

    for (int i = 0; i < SAMPLES_PER_STEP; ++i) {
        Sample sample{};
        if (imu.read_all(sample) == Status::OK) {
            accel.add(sample.accel_g);
            gyro.add(sample.gyro_dps);
        } else {
            ++failed_reads;
        }
        sleep_ms(SAMPLE_PERIOD_MS);
    }

    // accel noise in milli-g, gyro noise in deg/s
    std::printf("%-34s | %6.2f %6.2f %6.2f | %6.3f %6.3f %6.3f | %+7.3f | %6.3f",
                step.name,
                accel.x.stddev() * 1000.0f,
                accel.y.stddev() * 1000.0f,
                accel.z.stddev() * 1000.0f,
                gyro.x.stddev(),
                gyro.y.stddev(),
                gyro.z.stddev(),
                accel.z.mean(),
                gyro.z.peak_to_peak());
    if (failed_reads > 0)
        std::printf("  (%d failed reads)", failed_reads);
    std::printf("\n");
}

void print_table_header() {
    std::printf("\n%-34s | %-20s | %-20s | %-7s | %s\n",
                "filter",
                "accel noise [mg]",
                "gyro noise [dps]",
                "acc Z",
                "gyro Z");
    std::printf("%-34s | %6s %6s %6s | %6s %6s %6s | %-7s | %s\n",
                "",
                "x",
                "y",
                "z",
                "x",
                "y",
                "z",
                "mean g",
                "p-p dps");
    std::printf("-----------------------------------+----------------------+----------------------+"
                "---------+--------\n");
}

} // namespace

int main() {
    stdio_init_all();

    std::printf("\n=== MPU6500 filter comparison ===\n");
    std::printf("Bus: I2C, address 0x%02x\n\n", i2c_config::DEVICE_ADDR);

    i2c_config::init_test_i2c();

    mpu6500::config::Config config{};
    config.starting_sample_divider = 9; // 1000 / (1 + 9) = 100 Hz

    bus::pico::I2CBus bus{i2c0, i2c_config::DEVICE_ADDR, 30000};
    mpu6500::Mpu6500 imu(bus, sleep_ms, config);

    uint8_t id{};
    const Status id_status = imu.who_am_i(id);
    std::printf("WHO_AM_I: 0x%02x (%s)\n", id, status_text(id_status));

    const Status init_status = imu.init();
    print_status("init", init_status);
    if (init_status != Status::OK)
        halt("init failed", init_status);
    std::printf("\nSettings: accel range code %u, gyro range code %u, sample divider %u\n",
                static_cast<unsigned>(imu.accel_range()),
                static_cast<unsigned>(imu.gyro_range()),
                static_cast<unsigned>(imu.sample_divider()));
    std::printf("Each filter is measured over %d samples (%d ms).\n",
                SAMPLES_PER_STEP,
                SAMPLES_PER_STEP * static_cast<int>(SAMPLE_PERIOD_MS));
    std::printf("KEEP THE BOARD STILL: the numbers show sensor noise.\n");
    std::printf("Lower noise = stronger filtering (but slower reaction).\n");

    for (int round = 1;; ++round) {
        std::printf("\n--- round %d ---", round);
        print_table_header();

        for (const FilterStep& step : FILTER_STEPS) {
            if (!apply_filter(imu, step)) {
                std::printf("%-34s | could not set filter\n", step.name);
                continue;
            }
            sleep_ms(SETTLE_TIME_MS);
            measure_and_print(imu, step);
        }
    }
}

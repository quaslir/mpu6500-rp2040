// Interactive console for the MPU6500 driver.
// Type commands over USB serial (e.g. picocom), "help" lists them.

#include "bus_pico/i2c_bus.hpp"
#include "example_utils.hpp"
#include "i2c_config.hpp"
#include "mpu6500/mpu6500.hpp"
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <pico/stdlib.h>

namespace {

namespace cfg = mpu6500::config;

constexpr uint32_t STARTUP_DELAY_MS = 3000;
constexpr uint32_t I2C_TIMEOUT_US = 30000;
constexpr uint32_t DEFAULT_STREAM_PERIOD_MS = 200;
constexpr std::size_t LINE_MAX = 64;
constexpr int MAX_TOKENS = 5;

// ---------------------------------------------------------------------------
// Name <-> value tables, used both for parsing commands and printing status
// ---------------------------------------------------------------------------
template <typename T> struct Named {
    const char* name;
    T value;
};

constexpr Named<cfg::AccelRange> ACCEL_RANGES[] = {
    {"2", cfg::AccelRange::G2},
    {"4", cfg::AccelRange::G4},
    {"8", cfg::AccelRange::G8},
    {"16", cfg::AccelRange::G16},
};

constexpr Named<cfg::GyroRange> GYRO_RANGES[] = {
    {"250", cfg::GyroRange::Dps250},
    {"500", cfg::GyroRange::Dps500},
    {"1000", cfg::GyroRange::Dps1000},
    {"2000", cfg::GyroRange::Dps2000},
};

constexpr Named<cfg::GyroFilter> GYRO_FILTERS[] = {
    {"250", cfg::GyroFilter::Hz250},
    {"184", cfg::GyroFilter::Hz184},
    {"92", cfg::GyroFilter::Hz92},
    {"41", cfg::GyroFilter::Hz41},
    {"20", cfg::GyroFilter::Hz20},
    {"10", cfg::GyroFilter::Hz10},
    {"5", cfg::GyroFilter::Hz5},
    {"3600", cfg::GyroFilter::Hz3600},
    {"bypass3600", cfg::GyroFilter::Bypass3600Hz},
    {"bypass8800", cfg::GyroFilter::Bypass8800Hz},
};

constexpr Named<cfg::AccelFilter> ACCEL_FILTERS[] = {
    {"460", cfg::AccelFilter::Hz460},
    {"184", cfg::AccelFilter::Hz184},
    {"92", cfg::AccelFilter::Hz92},
    {"41", cfg::AccelFilter::Hz41},
    {"20", cfg::AccelFilter::Hz20},
    {"10", cfg::AccelFilter::Hz10},
    {"5", cfg::AccelFilter::Hz5},
    {"bypass1130", cfg::AccelFilter::Bypass1130Hz},
};

constexpr Named<cfg::ClockSource> CLOCK_SOURCES[] = {
    {"internal", cfg::ClockSource::Internal20MHz},
    {"auto", cfg::ClockSource::Auto},
    {"stop", cfg::ClockSource::Stopped},
};

constexpr Named<cfg::LowPowerAccelRate> LOW_POWER_RATES[] = {
    {"0.24", cfg::LowPowerAccelRate::Hz0_24},
    {"0.49", cfg::LowPowerAccelRate::Hz0_49},
    {"0.98", cfg::LowPowerAccelRate::Hz0_98},
    {"1.95", cfg::LowPowerAccelRate::Hz1_95},
    {"3.91", cfg::LowPowerAccelRate::Hz3_91},
    {"7.81", cfg::LowPowerAccelRate::Hz7_81},
    {"15.63", cfg::LowPowerAccelRate::Hz15_63},
    {"31.25", cfg::LowPowerAccelRate::Hz31_25},
    {"62.5", cfg::LowPowerAccelRate::Hz62_5},
    {"125", cfg::LowPowerAccelRate::Hz125},
    {"250", cfg::LowPowerAccelRate::Hz250},
    {"500", cfg::LowPowerAccelRate::Hz500},
};

template <typename T, std::size_t N>
const Named<T>* find_by_name(const Named<T> (&table)[N], const char* name) {
    for (const auto& entry : table) {
        if (std::strcmp(entry.name, name) == 0)
            return &entry;
    }
    return nullptr;
}

template <typename T, std::size_t N> const char* name_of(const Named<T> (&table)[N], T value) {
    for (const auto& entry : table) {
        if (entry.value == value)
            return entry.name;
    }
    return "?";
}

template <typename T, std::size_t N> void print_options(const Named<T> (&table)[N]) {
    for (const auto& entry : table)
        std::printf(" %s", entry.name);
    std::printf("\n");
}

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
struct ConsoleState {
    bool streaming = false;
    uint32_t stream_period_ms = DEFAULT_STREAM_PERIOD_MS;
    uint32_t last_stream_ms = 0;
};

const char* on_off(bool value) {
    return value ? "on" : "off";
}

bool parse_on_off(const char* text, bool& value) {
    if (text == nullptr)
        return false;
    if (std::strcmp(text, "on") == 0) {
        value = true;
        return true;
    }
    if (std::strcmp(text, "off") == 0) {
        value = false;
        return true;
    }
    return false;
}

// Parses a whole integer (decimal, 0x.. hex, may be negative) within [min, max].
bool parse_long(const char* text, long min, long max, long& value) {
    if (text == nullptr)
        return false;
    char* end = nullptr;
    const long parsed = std::strtol(text, &end, 0);
    if (end == text || *end != '\0' || parsed < min || parsed > max)
        return false;
    value = parsed;
    return true;
}

void print_result(const char* what, Status status) {
    std::printf("%s: %s\n", what, example::status_text(status));
}

uint32_t now_ms() {
    return to_ms_since_boot(get_absolute_time());
}

void print_sample(const Sample& s) {
    std::printf("accel %+7.3f %+7.3f %+7.3f g | gyro %+8.2f %+8.2f %+8.2f dps | temp %6.2f C\n",
                s.accel_g.x,
                s.accel_g.y,
                s.accel_g.z,
                s.gyro_dps.x,
                s.gyro_dps.y,
                s.gyro_dps.z,
                s.temperature_c);
}

void print_axes(const cfg::EnabledAxes& a) {
    std::printf("  axes      accel x=%s y=%s z=%s | gyro x=%s y=%s z=%s\n",
                on_off(a.accel_x),
                on_off(a.accel_y),
                on_off(a.accel_z),
                on_off(a.gyro_x),
                on_off(a.gyro_y),
                on_off(a.gyro_z));
}

void print_status(const mpu6500::Mpu6500& imu, const ConsoleState& state) {
    const cfg::Config& c = imu.config();
    const Vec3& ao = c.calibration.accel_offset_g;
    const Vec3& go = c.calibration.gyro_offset_dps;
    const RawVec3& hw = c.calibration.gyro_hw_offset;

    std::printf("--- status ---\n");
    std::printf("  accel     range +-%s g, filter %s\n",
                name_of(ACCEL_RANGES, c.measurement.accel.range),
                name_of(ACCEL_FILTERS, c.measurement.accel.filter));
    std::printf("  gyro      range +-%s dps, filter %s\n",
                name_of(GYRO_RANGES, c.measurement.gyro.range),
                name_of(GYRO_FILTERS, c.measurement.gyro.filter));
    std::printf("  rate      divider %u (%s), gyro %.2f Hz, accel %.2f Hz\n",
                c.measurement.sample_divider,
                imu.divider_effective() ? "effective" : "ignored by current gyro filter",
                imu.gyro_sample_rate_hz(),
                imu.accel_sample_rate_hz());
    std::printf("  power     sleep=%s standby=%s temp=%s clock=%s\n",
                on_off(c.power.sleeping),
                on_off(c.power.gyro_standby),
                on_off(c.power.temperature_enabled),
                name_of(CLOCK_SOURCES, c.power.clock_source));
    if (imu.is_low_power())
        std::printf("  low power on, wake-up rate %s Hz (gyro off)\n",
                    name_of(LOW_POWER_RATES, imu.low_power_rate()));
    else
        std::printf("  low power off\n");
    print_axes(c.power.enabled_axes);
    std::printf("  offsets   accel %+.4f %+.4f %+.4f g | gyro %+.3f %+.3f %+.3f dps\n",
                ao.x,
                ao.y,
                ao.z,
                go.x,
                go.y,
                go.z);
    std::printf("  hw offset gyro %d %d %d (raw)\n", hw.x, hw.y, hw.z);
    std::printf("  stream    %s, every %lu ms\n",
                on_off(state.streaming),
                static_cast<unsigned long>(state.stream_period_ms));
}

void print_help() {
    std::printf("--- commands ---\n");
    std::printf("  help                          this list\n");
    std::printf("  status                        show current settings\n");
    std::printf("  whoami                        read chip ID\n");
    std::printf("  init                          reset chip and re-apply settings\n");
    std::printf("  read                          one sample\n");
    std::printf("  stream on|off                 continuous samples\n");
    std::printf("  rate <ms>                     stream period (10..5000)\n");
    std::printf("\n");
    std::printf("  arange <g>                   :");
    print_options(ACCEL_RANGES);
    std::printf("  grange <dps>                 :");
    print_options(GYRO_RANGES);
    std::printf("  afilter <hz>                 :");
    print_options(ACCEL_FILTERS);
    std::printf("  gfilter <hz>                 :");
    print_options(GYRO_FILTERS);
    std::printf("  div <0..255>                  sample rate divider\n");
    std::printf("  hz <4..1000>                  sample rate in Hz (gyro filter 184..5)\n");
    std::printf("\n");
    std::printf("  sleep on|off                  whole chip sleep\n");
    std::printf("  standby on|off                gyro standby\n");
    std::printf("  temp on|off                   temperature sensor\n");
    std::printf("  clock <src>                  :");
    print_options(CLOCK_SOURCES);
    std::printf("  axis ax|ay|az|gx|gy|gz on|off enable/disable one axis\n");
    std::printf("  lp <hz>                      :");
    print_options(LOW_POWER_RATES);
    std::printf("  lp off                        leave low-power accel mode\n");
    std::printf("\n");
    std::printf("  sigreset [g] [a] [t]          reset signal paths (none given = all)\n");
    std::printf("  sensreset                     reset all paths and clear data registers\n");
    std::printf("\n");
    std::printf("  calgyro                       gyro calibration (keep still)\n");
    std::printf("  calaccel                      accel calibration (flat, Z up)\n");
    std::printf("  clear                         clear software offsets\n");
    std::printf("  ghwoff <x> <y> <z>            gyro hardware offset, raw units\n");
}

// ---------------------------------------------------------------------------
// Line input: non-blocking, with echo and backspace
// ---------------------------------------------------------------------------
bool poll_line(char* line, std::size_t& length) {
    int c = getchar_timeout_us(0);
    while (c != PICO_ERROR_TIMEOUT) {
        if (c == '\r' || c == '\n') {
            if (length > 0) {
                line[length] = '\0';
                std::printf("\n");
                return true;
            }
        } else if (c == '\b' || c == 127) {
            if (length > 0) {
                --length;
                std::printf("\b \b");
            }
        } else if (length < LINE_MAX - 1 && std::isprint(c)) {
            line[length++] = static_cast<char>(c);
            std::putchar(c);
        }
        c = getchar_timeout_us(0);
    }
    return false;
}

int tokenize(char* line, char* tokens[], int max_tokens) {
    int count = 0;
    char* token = std::strtok(line, " \t");
    while (token != nullptr && count < max_tokens) {
        tokens[count++] = token;
        token = std::strtok(nullptr, " \t");
    }
    return count;
}

void print_prompt() {
    std::printf("> ");
}

// ---------------------------------------------------------------------------
// Command handling
// ---------------------------------------------------------------------------
bool set_axis(cfg::EnabledAxes& axes, const char* name, bool enabled) {
    if (std::strcmp(name, "ax") == 0)
        axes.accel_x = enabled;
    else if (std::strcmp(name, "ay") == 0)
        axes.accel_y = enabled;
    else if (std::strcmp(name, "az") == 0)
        axes.accel_z = enabled;
    else if (std::strcmp(name, "gx") == 0)
        axes.gyro_x = enabled;
    else if (std::strcmp(name, "gy") == 0)
        axes.gyro_y = enabled;
    else if (std::strcmp(name, "gz") == 0)
        axes.gyro_z = enabled;
    else
        return false;
    return true;
}

void handle_command(mpu6500::Mpu6500& imu, ConsoleState& state, char* line) {
    char* tokens[MAX_TOKENS] = {};
    const int count = tokenize(line, tokens, MAX_TOKENS);
    if (count == 0)
        return;

    const char* cmd = tokens[0];
    const char* arg1 = count > 1 ? tokens[1] : nullptr;
    const char* arg2 = count > 2 ? tokens[2] : nullptr;
    const char* arg3 = count > 3 ? tokens[3] : nullptr;

    // --- general -------------------------------------------------------------
    if (std::strcmp(cmd, "help") == 0) {
        print_help();
    } else if (std::strcmp(cmd, "status") == 0) {
        print_status(imu, state);
    } else if (std::strcmp(cmd, "whoami") == 0) {
        uint8_t id{};
        const Status status = imu.who_am_i(id);
        std::printf("WHO_AM_I: 0x%02x (%s)\n", id, example::status_text(status));
    } else if (std::strcmp(cmd, "init") == 0) {
        print_result("init", imu.init());
    } else if (std::strcmp(cmd, "read") == 0) {
        Sample sample{};
        const Status status = imu.read_all(sample);
        if (status == Status::OK)
            print_sample(sample);
        else
            print_result("read", status);
    } else if (std::strcmp(cmd, "stream") == 0) {
        bool on = false;
        if (!parse_on_off(arg1, on)) {
            std::printf("usage: stream on|off\n");
            return;
        }
        state.streaming = on;
        state.last_stream_ms = now_ms();
        std::printf("stream %s\n", on_off(on));
    } else if (std::strcmp(cmd, "rate") == 0) {
        long ms = 0;
        if (!parse_long(arg1, 10, 5000, ms)) {
            std::printf("usage: rate <10..5000>\n");
            return;
        }
        state.stream_period_ms = static_cast<uint32_t>(ms);
        std::printf("stream period %ld ms\n", ms);

        // --- measurement settings --------------------------------------------
    } else if (std::strcmp(cmd, "arange") == 0) {
        const auto* entry = arg1 ? find_by_name(ACCEL_RANGES, arg1) : nullptr;
        if (entry == nullptr) {
            std::printf("usage: arange");
            print_options(ACCEL_RANGES);
            return;
        }
        print_result("set_accel_range", imu.set_accel_range(entry->value));
    } else if (std::strcmp(cmd, "grange") == 0) {
        const auto* entry = arg1 ? find_by_name(GYRO_RANGES, arg1) : nullptr;
        if (entry == nullptr) {
            std::printf("usage: grange");
            print_options(GYRO_RANGES);
            return;
        }
        print_result("set_gyro_range", imu.set_gyro_range(entry->value));
    } else if (std::strcmp(cmd, "afilter") == 0) {
        const auto* entry = arg1 ? find_by_name(ACCEL_FILTERS, arg1) : nullptr;
        if (entry == nullptr) {
            std::printf("usage: afilter");
            print_options(ACCEL_FILTERS);
            return;
        }
        print_result("set_accel_filter", imu.set_accel_filter(entry->value));
    } else if (std::strcmp(cmd, "gfilter") == 0) {
        const auto* entry = arg1 ? find_by_name(GYRO_FILTERS, arg1) : nullptr;
        if (entry == nullptr) {
            std::printf("usage: gfilter");
            print_options(GYRO_FILTERS);
            return;
        }
        print_result("set_gyro_filter", imu.set_gyro_filter(entry->value));
        if (!imu.divider_effective())
            std::printf("note: divider is ignored with this filter, gyro runs at %.0f Hz\n",
                        imu.gyro_sample_rate_hz());
    } else if (std::strcmp(cmd, "div") == 0) {
        long divider = 0;
        if (!parse_long(arg1, 0, 255, divider)) {
            std::printf("usage: div <0..255>\n");
            return;
        }
        print_result("set_sample_rate_divider",
                     imu.set_sample_rate_divider(static_cast<uint8_t>(divider)));
    } else if (std::strcmp(cmd, "hz") == 0) {
        long hz = 0;
        if (!parse_long(arg1, 4, 1000, hz)) {
            std::printf("usage: hz <4..1000>\n");
            return;
        }
        if (!imu.divider_effective())
            std::printf("divider is ignored by the current gyro filter, use gfilter 184..5\n");
        print_result("set_sample_rate_hz", imu.set_sample_rate_hz(static_cast<uint16_t>(hz)));
        std::printf("divider %u, real rate gyro %.2f Hz, accel %.2f Hz\n",
                    imu.config().measurement.sample_divider,
                    imu.gyro_sample_rate_hz(),
                    imu.accel_sample_rate_hz());

        // --- power -----------------------------------------------------------
    } else if (std::strcmp(cmd, "sleep") == 0) {
        bool on = false;
        if (!parse_on_off(arg1, on)) {
            std::printf("usage: sleep on|off\n");
            return;
        }
        print_result("set_sleep", imu.set_sleep(on));
        if (on)
            std::printf("chip sleeps: data registers stop updating\n");
    } else if (std::strcmp(cmd, "standby") == 0) {
        bool on = false;
        if (!parse_on_off(arg1, on)) {
            std::printf("usage: standby on|off\n");
            return;
        }
        print_result("set_gyro_standby", imu.set_gyro_standby(on));
    } else if (std::strcmp(cmd, "temp") == 0) {
        bool on = false;
        if (!parse_on_off(arg1, on)) {
            std::printf("usage: temp on|off\n");
            return;
        }
        print_result("set_temperature_enabled", imu.set_temperature_enabled(on));
    } else if (std::strcmp(cmd, "clock") == 0) {
        const auto* entry = arg1 ? find_by_name(CLOCK_SOURCES, arg1) : nullptr;
        if (entry == nullptr) {
            std::printf("usage: clock");
            print_options(CLOCK_SOURCES);
            return;
        }
        if (entry->value == cfg::ClockSource::Stopped)
            std::printf("warning: stopped clock halts the chip, use 'clock auto' to resume\n");
        print_result("set_clock_source", imu.set_clock_source(entry->value));
    } else if (std::strcmp(cmd, "axis") == 0) {
        bool on = false;
        cfg::EnabledAxes axes = imu.config().power.enabled_axes;
        if (arg1 == nullptr || !parse_on_off(arg2, on) || !set_axis(axes, arg1, on)) {
            std::printf("usage: axis ax|ay|az|gx|gy|gz on|off\n");
            return;
        }
        print_result("set_enabled_axes", imu.set_enabled_axes(axes));
        print_axes(imu.config().power.enabled_axes);
    } else if (std::strcmp(cmd, "lp") == 0) {
        if (arg1 != nullptr && std::strcmp(arg1, "off") == 0) {
            print_result("exit_low_power_accel", imu.exit_low_power_accel());
            return;
        }
        const auto* entry = arg1 ? find_by_name(LOW_POWER_RATES, arg1) : nullptr;
        if (entry == nullptr) {
            std::printf("usage: lp off | lp");
            print_options(LOW_POWER_RATES);
            return;
        }
        print_result("enter_low_power_accel", imu.enter_low_power_accel(entry->value));
        if (imu.is_low_power())
            std::printf("accel updates at %s Hz, gyro is off\n", entry->name);

        // --- resets ----------------------------------------------------------
    } else if (std::strcmp(cmd, "sigreset") == 0) {
        bool gyro = false;
        bool accel = false;
        bool temp = false;
        const char* args[] = {arg1, arg2, arg3};
        for (const char* arg : args) {
            if (arg == nullptr)
                continue;
            if (std::strcmp(arg, "g") == 0)
                gyro = true;
            else if (std::strcmp(arg, "a") == 0)
                accel = true;
            else if (std::strcmp(arg, "t") == 0)
                temp = true;
            else {
                std::printf("usage: sigreset [g] [a] [t]\n");
                return;
            }
        }
        if (!gyro && !accel && !temp)
            gyro = accel = temp = true;
        print_result("reset_signal_paths", imu.reset_signal_paths(gyro, accel, temp));
        std::printf("reset: gyro=%s accel=%s temp=%s\n", on_off(gyro), on_off(accel), on_off(temp));
    } else if (std::strcmp(cmd, "sensreset") == 0) {
        print_result("reset_sensor_registers", imu.reset_sensor_registers());

        // --- calibration -----------------------------------------------------
    } else if (std::strcmp(cmd, "calgyro") == 0) {
        std::printf("keep the board still...\n");
        print_result("calibrate_gyro", imu.calibrate_gyro());
        const Vec3& o = imu.config().calibration.gyro_offset_dps;
        std::printf("gyro offset %+.3f %+.3f %+.3f dps\n", o.x, o.y, o.z);
    } else if (std::strcmp(cmd, "calaccel") == 0) {
        std::printf("board flat, chip up, keep still...\n");
        print_result("calibrate_accel", imu.calibrate_accel());
        const Vec3& o = imu.config().calibration.accel_offset_g;
        std::printf("accel offset %+.4f %+.4f %+.4f g\n", o.x, o.y, o.z);
    } else if (std::strcmp(cmd, "clear") == 0) {
        imu.clear_offsets();
        std::printf("software offsets cleared\n");
    } else if (std::strcmp(cmd, "ghwoff") == 0) {
        long x = 0;
        long y = 0;
        long z = 0;
        if (!parse_long(arg1, -32768, 32767, x) || !parse_long(arg2, -32768, 32767, y) ||
            !parse_long(arg3, -32768, 32767, z)) {
            const RawVec3& hw = imu.config().calibration.gyro_hw_offset;
            std::printf("gyro hw offset %d %d %d (raw)\n", hw.x, hw.y, hw.z);
            std::printf("usage: ghwoff <x> <y> <z>   (-32768..32767)\n");
            return;
        }
        const RawVec3 offset{
            static_cast<int16_t>(x), static_cast<int16_t>(y), static_cast<int16_t>(z)};
        print_result("set_gyro_hw_offset", imu.set_gyro_hw_offset(offset));
    } else {
        std::printf("unknown command '%s', type 'help'\n", cmd);
    }
}

} // namespace

int main() {
    stdio_init_all();
    sleep_ms(STARTUP_DELAY_MS);

    std::printf("\n=== MPU6500 interactive console ===\n");

    i2c_config::init_test_i2c();
    bus::pico::I2CBus bus{i2c0, i2c_config::DEVICE_ADDR, I2C_TIMEOUT_US};

    cfg::Config config{};
    config.measurement.gyro.filter = cfg::GyroFilter::Hz41;
    config.measurement.accel.filter = cfg::AccelFilter::Hz41;
    config.measurement.sample_divider = 9;

    mpu6500::Mpu6500 imu(bus, sleep_ms, config);

    const Status init_status = imu.init();
    print_result("init", init_status);
    if (init_status != Status::OK)
        example::halt("init failed", init_status);

    ConsoleState state{};
    print_help();
    print_status(imu, state);
    print_prompt();

    char line[LINE_MAX] = {};
    std::size_t length = 0;

    for (;;) {
        if (poll_line(line, length)) {
            handle_command(imu, state, line);
            length = 0;
            print_prompt();
        }

        if (state.streaming && now_ms() - state.last_stream_ms >= state.stream_period_ms) {
            state.last_stream_ms = now_ms();
            Sample sample{};
            const Status status = imu.read_all(sample);
            if (status == Status::OK)
                print_sample(sample);
            else
                print_result("read", status);
        }

        sleep_ms(1);
    }
}

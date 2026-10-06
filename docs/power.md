# Power

The driver has three power modes and a few finer switches (gyro standby, temperature sensor,
individual axes, clock source).

## Modes

| Mode | Accelerometer | Gyroscope | Temperature | Typical use |
|---|---|---|---|---|
| `Normal` | on | on | as configured | normal measurement |
| `Sleep` | off | off | off | lowest power, no data |
| `LowPowerAccel` | wakes up at `low_power_rate`, one sample per wake-up | off | off | battery devices, wake-on-motion |

```cpp
namespace cfg = mpu6500::config;

// at startup
config.power.mode = cfg::PowerMode::LowPowerAccel;
config.power.low_power_rate = cfg::LowPowerAccelRate::Hz31_25;

// at runtime
imu.set_power_mode(cfg::PowerMode::Normal);
imu.set_low_power_rate(cfg::LowPowerAccelRate::Hz7_81);
```

`set_low_power_rate()` can be called in any mode. The rate is stored and used the next time
`LowPowerAccel` is entered.

## Low-power wake-up rates

| `LowPowerAccelRate` | Rate |
|---|---:|
| `Hz0_24` | 0.24 Hz |
| `Hz0_49` | 0.49 Hz |
| `Hz0_98` | 0.98 Hz |
| `Hz1_95` | 1.95 Hz |
| `Hz3_91` | 3.91 Hz |
| `Hz7_81` | 7.81 Hz |
| `Hz15_63` | 15.63 Hz |
| `Hz31_25` | 31.25 Hz |
| `Hz62_5` | 62.5 Hz |
| `Hz125` | 125 Hz |
| `Hz250` | 250 Hz |
| `Hz500` | 500 Hz |

Lower rates use less power but react more slowly, which matters for wake-on-motion.

## What low-power mode changes

The configuration always keeps what **you** asked for. The mode is applied on top of it when
the registers are written:

| Setting | Normal | Sleep | LowPowerAccel |
|---|---|---|---|
| Chip sleep | off | **on** | off |
| Cycle (duty-cycled accel) | off | off | **on** |
| Gyro standby | from config | from config | **off** (required for cycle mode) |
| Temperature sensor | from config | from config | **off** |
| Gyro axes | from config | from config | **off** |
| Accel axes | from config | from config | from config |
| Accel filter | from config | from config | **bypass** (required by the chip) |

Leaving `LowPowerAccel` writes the configuration as it is, so the accel filter, temperature
sensor and gyro axes come back to the values you set. Nothing needs to be saved or restored.

Settings you change **while** in low-power mode are stored but not written if the mode
overrides them. For example, `set_accel_filter(Hz41)` in low-power mode keeps the chip on
bypass and applies 41 Hz when you return to `Normal`. `config()` always shows your settings,
not the overridden ones.

## Sample rates per mode

| Mode | `gyro_sample_rate_hz()` | `accel_sample_rate_hz()` |
|---|---|---|
| `Normal` | from filter and divider | from filter and divider |
| `Sleep` | 0 | 0 |
| `LowPowerAccel` | 0 | the wake-up rate |

## Finer switches

These work in every mode; low-power mode overrides some of them as shown above.

| Function | Config field | Default | Effect |
|---|---|---|---|
| `set_gyro_standby(bool)` | `power.gyro_standby` | `false` | gyro drive stays on but sensing is off: lower power, fast restart |
| `set_temperature_enabled(bool)` | `power.temperature_enabled` | `false` | temperature sensor on or off |
| `set_enabled_axes(axes)` | `power.enabled_axes` | all on | turn single accel or gyro axes off |
| `set_clock_source(src)` | `power.clock_source` | `Auto` | clock: `Auto` (PLL when ready), `Internal20MHz`, `Stopped` |

The temperature sensor is **off by default**. Enable it if you read `temperature_c`.

`ClockSource::Stopped` halts the chip's timing generator: no data until the clock is set
back to `Auto`.

## Restrictions

- **Calibration needs `Normal` mode.** In sleep no data is produced; in low-power mode the
  accel filter is bypassed and the gyro is off. `calibrate_gyro()` and `calibrate_accel()`
  return `ERROR` in other modes.
- **No gyro data in low-power mode.** `read_gyro()` returns values, but they are not updated.
- **FIFO in low-power mode** only makes sense with accelerometer sources.

## Current consumption

_To be measured: module current in each mode (normal, accel only, gyro standby, sleep,
low-power at 0.98 / 31.25 / 500 Hz, with wake-on-motion). The breakout module's power LED
and regulator have to be accounted for, otherwise they dominate the low-power readings.
Compare with the current table in the MPU6500 product specification._

## Example

The `console` example switches modes interactively: `power normal|sleep|lp`, `lprate <hz>`,
`standby on|off`, `temp on|off`, `axis <name> on|off`. `examples/wom` runs in low-power mode
with wake-on-motion.

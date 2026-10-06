# Configuration

The whole chip setup is described by one `mpu6500::config::Config`. It is passed to the
constructor, written to the chip by `init()`, and kept by the driver as its current state.

```cpp
namespace cfg = mpu6500::config;

cfg::Config config{};                       // chip reset defaults, everything optional off
config.measurement.gyro.filter = cfg::GyroFilter::Hz41;
config.measurement.accel.filter = cfg::AccelFilter::Hz41;
config.measurement.sample_divider = 9;      // 100 Hz

mpu6500::Mpu6500 imu{bus, sleep_ms, config};
imu.init();

const cfg::Config& now = imu.config();      // read-only view of the current state
```

- Change settings at runtime **only through setters** (`set_gyro_filter()`, ...). They write
  the register and update the stored configuration.
- `config()` returns a const reference, so the stored state cannot drift from the chip.
- `init()` resets the chip and writes the whole configuration again. Calling it twice is
  safe.

## Overview

| Group | Field | Default |
|---|---|---|
| — | `use_i2c` | `true` |
| `measurement` | `accel.range` | `G2` |
| | `accel.filter` | `Hz460` |
| | `gyro.range` | `Dps250` |
| | `gyro.filter` | `Hz250` |
| | `sample_divider` | `0` |
| `power` | `mode` | `Normal` |
| | `low_power_rate` | `Hz0_24` |
| | `gyro_standby` | `false` |
| | `temperature_enabled` | `false` |
| | `clock_source` | `Auto` |
| | `enabled_axes` | all on |
| `calibration` | `accel_offset_g` | 0, 0, 0 |
| | `gyro_offset_dps` | 0, 0, 0 |
| | `gyro_hw_offset` | 0, 0, 0 |
| `fifo` | `enabled` | `false` |
| | `sources` | accel + gyro |
| | `mode` | `Overwrite` |
| `interrupts` | `sources` | all off |
| | `level` | `ActiveHigh` |
| | `drive` | `PushPull` |
| | `mode` | `Pulse` |
| `wake_on_motion` | `enabled` | `false` |
| | `threshold_mg` | `100` |

## Bus

| Field | Meaning |
|---|---|
| `use_i2c` | `true` on I2C. Set to `false` on SPI: `init()` then turns the chip's I2C interface off, as required by the datasheet |

## Measurement

### Ranges

| `AccelRange` | Range | Sensitivity |
|---|---|---|
| `G2` | ±2 g | 16384 LSB/g |
| `G4` | ±4 g | 8192 LSB/g |
| `G8` | ±8 g | 4096 LSB/g |
| `G16` | ±16 g | 2048 LSB/g |

| `GyroRange` | Range | Sensitivity |
|---|---|---|
| `Dps250` | ±250 °/s | 131 LSB/(°/s) |
| `Dps500` | ±500 °/s | 65.5 LSB/(°/s) |
| `Dps1000` | ±1000 °/s | 32.8 LSB/(°/s) |
| `Dps2000` | ±2000 °/s | 16.4 LSB/(°/s) |

A smaller range gives finer resolution; a larger range avoids clipping fast motion.

### Gyro filter

The gyro filter also sets the internal sample rate, and with it whether the divider works.

| `GyroFilter` | Bandwidth | Delay | Internal rate | Divider |
|---|---:|---:|---:|---|
| `Hz250` | 250 Hz | 0.97 ms | 8 kHz | ignored |
| `Hz184` | 184 Hz | 2.9 ms | 1 kHz | applies |
| `Hz92` | 92 Hz | 3.9 ms | 1 kHz | applies |
| `Hz41` | 41 Hz | 5.9 ms | 1 kHz | applies |
| `Hz20` | 20 Hz | 9.9 ms | 1 kHz | applies |
| `Hz10` | 10 Hz | 17.85 ms | 1 kHz | applies |
| `Hz5` | 5 Hz | 33.48 ms | 1 kHz | applies |
| `Hz3600` | 3600 Hz | 0.17 ms | 8 kHz | ignored |
| `Bypass3600Hz` | 3600 Hz | 0.11 ms | 32 kHz | ignored |
| `Bypass8800Hz` | 8800 Hz | 0.064 ms | 32 kHz | ignored |

Note that the default, `Hz250`, runs at 8 kHz and ignores the divider.

### Accelerometer filter

| `AccelFilter` | Bandwidth | Delay | Rate |
|---|---:|---:|---:|
| `Hz460` | 460 Hz | 1.94 ms | 1 kHz |
| `Hz184` | 184 Hz | 5.80 ms | 1 kHz |
| `Hz92` | 92 Hz | 7.80 ms | 1 kHz |
| `Hz41` | 41 Hz | 11.80 ms | 1 kHz |
| `Hz20` | 20 Hz | 19.80 ms | 1 kHz |
| `Hz10` | 10 Hz | 35.70 ms | 1 kHz |
| `Hz5` | 5 Hz | 66.96 ms | 1 kHz |
| `Bypass1130Hz` | 1130 Hz | 0.75 ms | 4 kHz |

Narrower filters mean less noise but more delay. See the `filters` example for measured
noise per setting.

### Sample rate

| Gyro filter | Gyro rate | Accel rate |
|---|---|---|
| `Hz184` … `Hz5` | 1000 / (1 + divider) | 1000 / (1 + divider), or 4 kHz with accel bypass |
| `Hz250`, `Hz3600` | 8 kHz | 1 kHz, or 4 kHz with accel bypass |
| `Bypass3600Hz`, `Bypass8800Hz` | 32 kHz | 1 kHz, or 4 kHz with accel bypass |

| Divider | Rate (filters 184…5) |
|---:|---:|
| 0 | 1000 Hz |
| 1 | 500 Hz |
| 4 | 200 Hz |
| 9 | 100 Hz |
| 19 | 50 Hz |
| 99 | 10 Hz |
| 249 | 4 Hz |

Helpers:

| Function | Returns |
|---|---|
| `gyro_sample_rate_hz()` | the gyro output rate for the current configuration and mode |
| `accel_sample_rate_hz()` | the accelerometer output rate |
| `divider_effective()` | whether the divider applies with the current gyro filter |
| `set_sample_rate_hz(hz)` | sets the divider for 4…1000 Hz; `ERROR` if the divider does not apply |

Pick a filter bandwidth below half the sample rate: 41 Hz at 100 Hz, 92 Hz at 200 Hz,
184 Hz at 500 Hz and 1 kHz. These rules are verified on hardware in
[measurements.md](measurements.md).

## Power

See [power.md](power.md). Note that `temperature_enabled` is `false` by default: enable it
to read temperature.

## Calibration

| Field | Unit | Set by |
|---|---|---|
| `accel_offset_g` | g | `calibrate_accel()` or by hand |
| `gyro_offset_dps` | °/s | `calibrate_gyro()` or by hand |
| `gyro_hw_offset` | raw counts | `set_gyro_hw_offset()` or by hand |

Offsets measured once can be put into the configuration, so no calibration is needed at
every start. See [calibration.md](calibration.md).

## FIFO, interrupts, wake-on-motion

See [fifo.md](fifo.md) and [interrupts.md](interrupts.md).

## Combinations that do not make sense

`init()` writes these without complaint, but they do nothing useful:

| Combination | Problem |
|---|---|
| FIFO enabled, all sources off | the buffer never fills, `read_fifo()` returns `ERROR` |
| `interrupts.sources.fifo_overflow` with the FIFO disabled | event never happens |
| `interrupts.sources.wake_on_motion` with `wake_on_motion.enabled = false` | the chip never detects motion |
| Wake-on-motion in a mode other than `LowPowerAccel` | not the intended use; tested only in low-power mode |
| Divider set with gyro filter `Hz250`, `Hz3600` or bypass | divider is ignored |
| Filter bandwidth above half the sample rate | filter lets through more than the rate can represent |
| `use_i2c = true` on SPI | the I2C interface stays on and can misread SPI traffic |

## Typical configurations

**Simple 100 Hz reading**

```cpp
config.measurement.gyro.filter = cfg::GyroFilter::Hz41;
config.measurement.accel.filter = cfg::AccelFilter::Hz41;
config.measurement.sample_divider = 9;
```

**1 kHz with the FIFO (SPI recommended)**

```cpp
config.use_i2c = false;
config.measurement.gyro.filter = cfg::GyroFilter::Hz184;
config.measurement.accel.filter = cfg::AccelFilter::Hz184;
config.measurement.sample_divider = 0;
config.fifo.enabled = true;
```

**Data-ready interrupt at 200 Hz**

```cpp
config.measurement.gyro.filter = cfg::GyroFilter::Hz92;
config.measurement.accel.filter = cfg::AccelFilter::Hz92;
config.measurement.sample_divider = 4;
config.interrupts.sources.raw_data_ready = true;
config.interrupts.mode = cfg::IntMode::Pulse;
```

**Wake-on-motion on battery**

```cpp
config.power.mode = cfg::PowerMode::LowPowerAccel;
config.power.low_power_rate = cfg::LowPowerAccelRate::Hz31_25;
config.wake_on_motion.enabled = true;
config.wake_on_motion.threshold_mg = 100;
config.interrupts.sources.wake_on_motion = true;
```

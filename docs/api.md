# API reference

Every public function and type of the driver. For background see
[configuration.md](configuration.md), [fifo.md](fifo.md), [interrupts.md](interrupts.md),
[power.md](power.md) and [calibration.md](calibration.md).

## Conventions

- Functions that talk to the chip return `bus::Status`: `OK`, `TIMEOUT`, `NACK` or `ERROR`.
  All of them are `[[nodiscard]]`.
- `ERROR` is also used for driver-level problems: wrong power mode, invalid argument, board
  moved during calibration.
- Output values are passed by reference and are only valid when the result is `OK`.
- Units: acceleration in g, angular rate in °/s, temperature in °C, threshold in mg,
  rates in Hz.
- Setters write the register first and update the stored configuration only on success
  (power setters: see their notes).
- Over SPI a write cannot be confirmed by the bus: `OK` means the bytes were sent.

## Contents

- [Types](#types)
- [Construction and lifecycle](#construction-and-lifecycle)
- [Reading data](#reading-data)
- [Measurement settings](#measurement-settings)
- [Sample rate](#sample-rate)
- [Offsets](#offsets)
- [Power](#power)
- [FIFO](#fifo)
- [Interrupts and wake-on-motion](#interrupts-and-wake-on-motion)
- [Resets](#resets)
- [Calibration functions](#calibration-functions)
- [Error handling macro](#error-handling-macro)

---

## Types

### `bus::Status`

```cpp
enum class Status : uint8_t { OK, TIMEOUT, NACK, ERROR };
```

| Value | Meaning |
|---|---|
| `OK` | success |
| `TIMEOUT` | the bus transfer did not finish in time (I2C) |
| `NACK` | the device did not acknowledge (I2C: wrong address, not connected, bus disturbed) |
| `ERROR` | any other failure, including driver-level errors |

### `mpu6500::WaitFunction`

```cpp
using WaitFunction = void (*)(uint32_t delay_ms);
```

Blocking delay in milliseconds, used after resets and during calibration. On the Pico pass
`sleep_ms`.

### `mpu6500::Sample`

```cpp
struct Sample {
    math::Vec3 accel_g;      // g
    math::Vec3 gyro_dps;     // °/s
    float temperature_c;     // °C
};
```

One measurement of all sensors. Filled by `read_all()`, `read_all_uncorrected()` and
`read_fifo()`.

### `mpu6500::InterruptFlags`

```cpp
struct InterruptFlags {
    bool raw_data_ready;
    bool fifo_overflow;
    bool wake_on_motion;
};
```

Pending events, filled by `take_interrupt_flags()`.

### `mpu6500::FifoReadResult`

```cpp
struct FifoReadResult {
    std::size_t frames;   // frames written to the output span
    bool overflowed;      // the FIFO overflowed since the last read
};
```

### `mpu6500::config::Config`

The complete configuration. All fields and defaults are listed in
[configuration.md](configuration.md).

---

## Construction and lifecycle

### Constructor

```cpp
Mpu6500(bus::Bus& bus, WaitFunction wait, const config::Config& config);
```

Stores the bus, the delay function and a copy of the configuration. Does **not** talk to the
chip; call `init()` next.

- The bus must be set up by the application (pins, clock, pull-ups) and must outlive the
  driver.
- The driver cannot be copied.

### `init`

```cpp
bus::Status init();
```

Brings the chip into the configured state:

1. reads `WHO_AM_I` and returns `ERROR` if it is not `0x70`;
2. resets the device and the signal paths, waiting 100 ms after each (datasheet requirement);
3. disables the chip's I2C interface if `use_i2c` is `false`;
4. writes the whole configuration (measurement, offsets, wake-on-motion, power, FIFO,
   interrupts) and clears pending interrupt flags.

Takes about 200 ms. Can be called again at any time to reset the chip and reapply the stored
configuration, including all changes made with setters.

### `who_am_i`

```cpp
bus::Status who_am_i(uint8_t& id);
```

Reads the `WHO_AM_I` register. The MPU6500 returns `0x70`. Useful as a bus check.

### `config`

```cpp
const config::Config& config() const;
```

Read-only view of the current configuration, including every change made through setters.
Shows the settings you asked for; in low-power mode some of them are overridden in the chip
(see [power.md](power.md)).

### `wait`

```cpp
WaitFunction wait() const;
```

Returns the delay function passed to the constructor. Used by the calibration functions.

---

## Reading data

All read functions are `const` and do not change the configuration.

### `read_all`

```cpp
bus::Status read_all(Sample& sample) const;
```

Reads accelerometer, temperature and gyroscope in one burst (14 bytes) and subtracts the
software offsets. The fastest way to get a consistent set of all values from the same
sample.

Temperature is only meaningful when `power.temperature_enabled` is `true`.

### `read_accel`

```cpp
bus::Status read_accel(math::Vec3& accel_g) const;
```

Reads the accelerometer (6 bytes) in g, with the software offset subtracted.

### `read_gyro`

```cpp
bus::Status read_gyro(math::Vec3& gyro_dps) const;
```

Reads the gyroscope (6 bytes) in °/s, with the software offset subtracted.

### `read_temp`

```cpp
bus::Status read_temp(float& temperature_c) const;
```

Reads the temperature sensor (2 bytes) in °C. Requires `power.temperature_enabled`.

### `read_all_uncorrected`, `read_accel_uncorrected`, `read_gyro_uncorrected`

```cpp
bus::Status read_all_uncorrected(Sample& sample) const;
bus::Status read_accel_uncorrected(math::Vec3& accel_g) const;
bus::Status read_gyro_uncorrected(math::Vec3& gyro_dps) const;
```

Same as the functions above but **without** subtracting the software offsets. Values are
still converted to g and °/s. Used by calibration and for comparing before/after.

---

## Measurement settings

### `set_accel_range`

```cpp
bus::Status set_accel_range(config::AccelRange range);
```

Sets the accelerometer full-scale range: ±2, ±4, ±8 or ±16 g. Following reads are scaled
accordingly.

### `set_gyro_range`

```cpp
bus::Status set_gyro_range(config::GyroRange range);
```

Sets the gyroscope full-scale range: ±250, ±500, ±1000 or ±2000 °/s.

### `set_accel_filter`

```cpp
bus::Status set_accel_filter(config::AccelFilter filter);
```

Sets the accelerometer low-pass filter. In `LowPowerAccel` mode the chip requires bypass:
the new filter is stored and applied when the mode changes back, and the register is not
written now.

### `set_gyro_filter`

```cpp
bus::Status set_gyro_filter(config::GyroFilter filter);
```

Sets the gyroscope low-pass filter. This also sets the internal sample rate (1, 8 or
32 kHz) and decides whether the divider works. Check `divider_effective()` after changing it.

---

## Sample rate

### `set_sample_rate_divider`

```cpp
bus::Status set_sample_rate_divider(uint8_t divider);
```

Sets `SMPLRT_DIV`. Output rate = 1000 / (1 + divider) Hz, **only** when the gyro filter is
`Hz184` … `Hz5`. With other gyro filters the value is stored but has no effect.

### `set_sample_rate_hz`

```cpp
bus::Status set_sample_rate_hz(uint16_t hz);
```

Computes and sets the divider for a rate between 4 and 1000 Hz (rounded to the nearest
possible rate). Returns `ERROR` if the rate is out of range or the divider does not apply
with the current gyro filter.

### `gyro_sample_rate_hz`

```cpp
float gyro_sample_rate_hz() const;
```

Gyroscope output rate for the current configuration: from filter and divider, 8 kHz or
32 kHz when the divider is ignored, 0 in `Sleep` and `LowPowerAccel`. Computed, does not
read the chip.

### `accel_sample_rate_hz`

```cpp
float accel_sample_rate_hz() const;
```

Accelerometer output rate: from filter and divider, 1 kHz when the divider is ignored, 4 kHz
in bypass, the wake-up rate in `LowPowerAccel`, 0 in `Sleep`.

### `divider_effective`

```cpp
bool divider_effective() const;
```

`true` if the sample rate divider applies with the current gyro filter (`Hz184` … `Hz5`).

---

## Offsets

### `set_accel_offset`

```cpp
void set_accel_offset(const math::Vec3& offset_g);
```

Sets the software accelerometer offset in g. Subtracted from every following accelerometer
read. Does not access the chip.

### `set_gyro_offset`

```cpp
void set_gyro_offset(const math::Vec3& offset_dps);
```

Sets the software gyroscope offset in °/s. Does not access the chip.

### `clear_offsets`

```cpp
void clear_offsets();
```

Sets both software offsets to zero. The hardware gyro offset is not changed.

### `set_gyro_hw_offset`

```cpp
bus::Status set_gyro_hw_offset(const math::RawVec3& offset);
```

Writes the chip's gyro offset registers (`XG/YG/ZG_OFFSET`) in raw counts. The chip adds
them before the data registers, so they also apply to FIFO data. The scale is not given by
the register map; verify on hardware. See [calibration.md](calibration.md).

---

## Power

See [power.md](power.md) for the modes and what they override.

### `set_power_mode`

```cpp
bus::Status set_power_mode(config::PowerMode mode);
```

Switches between `Normal`, `Sleep` and `LowPowerAccel` and rewrites all power-related
registers for the new mode. Leaving `LowPowerAccel` restores your accel filter, temperature
sensor and gyro axes automatically.

### `set_low_power_rate`

```cpp
bus::Status set_low_power_rate(config::LowPowerAccelRate rate);
```

Sets the wake-up rate of `LowPowerAccel` mode (0.24 … 500 Hz). Stored in any mode; written to
the chip when the mode is `LowPowerAccel`.

### `set_gyro_standby`

```cpp
bus::Status set_gyro_standby(bool enabled);
```

Gyro standby: drive on, sensing off. Lower power with a fast restart. Overridden (off) in
`LowPowerAccel` mode.

### `set_temperature_enabled`

```cpp
bus::Status set_temperature_enabled(bool enabled);
```

Turns the temperature sensor on or off. Off by default. Overridden (off) in `LowPowerAccel`
mode.

### `set_enabled_axes`

```cpp
bus::Status set_enabled_axes(const config::EnabledAxes& axes);
```

Turns individual accelerometer and gyroscope axes on or off. In `LowPowerAccel` mode the gyro
axes are always off.

### `set_clock_source`

```cpp
bus::Status set_clock_source(config::ClockSource source);
```

Selects the clock: `Auto` (PLL when ready, recommended), `Internal20MHz` or `Stopped`
(halts the chip until changed back).

Note for the power setters except `set_clock_source`: the stored configuration is updated
before the registers are written. If the bus fails, the stored value and the chip can
differ until the next successful power setter or `init()`.

---

## FIFO

See [fifo.md](fifo.md).

### `read_fifo`

```cpp
bus::Status read_fifo(std::span<Sample> samples, FifoReadResult& result);
```

Reads up to `samples.size()` whole frames from the FIFO, oldest first, converts them to
`Sample` and subtracts the software offsets. Fields of disabled sources stay zero.

- `result.frames`: number of frames written to `samples`.
- `result.overflowed`: the FIFO overflowed since the last read. In `Overwrite` mode the FIFO
  is reset and 0 frames are returned (the data was misaligned); in `StopWhenFull` mode the
  buffered frames are returned.
- Returns `OK` with 0 frames if the FIFO is disabled or `samples` is empty.
- Returns `ERROR` if the FIFO is enabled but no sources are selected.

### `fifo_frame_count`

```cpp
bus::Status fifo_frame_count(uint16_t& frames) const;
```

Number of whole frames currently in the FIFO. 0 if the FIFO is disabled.

### `fifo_reset`

```cpp
bus::Status fifo_reset();
```

Empties the FIFO and drops a pending overflow flag. Call it before the first read after a
long pause (calibration, reconfiguration), otherwise the first read returns old data or an
overflow.

---

## Interrupts and wake-on-motion

See [interrupts.md](interrupts.md).

### `take_interrupt_flags`

```cpp
bus::Status take_interrupt_flags(InterruptFlags& flags);
```

Reads the interrupt status from the chip and reports all pending events.

- Releases a latched INT pin. In latched mode call it after **every** interrupt.
- Clears the user-owned flags (`raw_data_ready`, `wake_on_motion`) after reporting them.
- Reports `fifo_overflow` but does **not** clear it: the next `read_fifo()` must still see it.

### `set_wake_on_motion`

```cpp
bus::Status set_wake_on_motion(bool enabled);
```

Turns the chip's wake-on-motion logic on or off. To get the event on the INT pin, also
enable `interrupts.sources.wake_on_motion` in the configuration. Intended for
`LowPowerAccel` mode.

### `set_wom_threshold`

```cpp
bus::Status set_wom_threshold(uint16_t threshold_mg);
```

Sets the wake-on-motion threshold in mg: the change between two consecutive samples that
counts as motion. Range 0 … 1020 mg in 4 mg steps; larger values are clamped to 1020 mg.

---

## Resets

### `reset_signal_paths`

```cpp
bus::Status reset_signal_paths(bool gyro = true, bool accel = true, bool temp = true);
```

Resets the digital signal paths of the selected sensors. Settings are kept, the data
registers are not cleared.

### `reset_sensor_registers`

```cpp
bus::Status reset_sensor_registers();
```

Resets all signal paths and clears the data registers (`SIG_COND_RST`). Settings are kept.
For a full reset of the chip use `init()`.

---

## Calibration functions

Namespace `mpu6500::calibration`. Free functions that use the public driver API. All of them
need the board to be still and the power mode to be `Normal`; otherwise they return `ERROR`.

### `MeasureOptions`

```cpp
struct MeasureOptions {
    uint16_t samples;         // samples to average
    uint16_t warmup_samples;  // samples read and discarded first
    float max_spread;         // max (max - min) per axis, in the sensor's unit
};
```

| Constant | Value |
|---|---|
| `DEFAULT_GYRO_OPTIONS` | 200 samples, 10 warm-up, 2.0 °/s spread |
| `DEFAULT_ACCEL_OPTIONS` | 200 samples, 10 warm-up, 0.05 g spread |
| `GRAVITY_Z_UP` | (0, 0, 1) g |
| `MIN_GRAVITY_G`, `MAX_GRAVITY_G` | 0.8 g, 1.2 g |

### `calibrate_gyro`

```cpp
bus::Status calibrate_gyro(Mpu6500& imu,
                           const MeasureOptions& options = DEFAULT_GYRO_OPTIONS);
```

Measures the average gyro reading at rest and stores it as the software gyro offset.

### `calibrate_accel`

```cpp
bus::Status calibrate_accel(Mpu6500& imu,
                            const math::Vec3& expected_gravity_g = GRAVITY_Z_UP,
                            const MeasureOptions& options = DEFAULT_ACCEL_OPTIONS);
```

Measures the average accelerometer reading at rest, subtracts the expected gravity vector
and stores the result as the software accel offset. Returns `ERROR` if the averaged
magnitude is outside 0.8 … 1.2 g. The tilt of the surface is included in the offset.

### `measure_gyro_offset`

```cpp
bus::Status measure_gyro_offset(const Mpu6500& imu,
                                math::Vec3& offset_dps,
                                const MeasureOptions& options = DEFAULT_GYRO_OPTIONS);
```

Like `calibrate_gyro`, but only returns the measured offset without storing it.

### `measure_accel_offset`

```cpp
bus::Status measure_accel_offset(const Mpu6500& imu,
                                 math::Vec3& offset_g,
                                 const math::Vec3& expected_gravity_g = GRAVITY_Z_UP,
                                 const MeasureOptions& options = DEFAULT_ACCEL_OPTIONS);
```

Like `calibrate_accel`, but only returns the measured offset without storing it.

### `measure_mean`

```cpp
bus::Status measure_mean(const Mpu6500& imu,
                         Sensor sensor,
                         math::Vec3& mean,
                         const MeasureOptions& options);
```

The averaging procedure behind all calibration functions: warm-up, then `samples` reads of
`Sensor::Gyro` or `Sensor::Accel` at the configured sample rate (offsets not subtracted),
then the spread check. Returns the average in `mean`.

---

## Error handling macro

### `MPU_RETURN_IF_ERROR`

```cpp
MPU_RETURN_IF_ERROR(expr);
```

Evaluates `expr` once; if the result is not `bus::Status::OK`, returns it from the enclosing
function. Can be used in application code that returns `bus::Status`:

```cpp
bus::Status setup(mpu6500::Mpu6500& imu) {
    MPU_RETURN_IF_ERROR(imu.init());
    MPU_RETURN_IF_ERROR(imu.set_gyro_range(mpu6500::config::GyroRange::Dps500));
    return bus::Status::OK;
}
```

Avoid template arguments with commas inside the macro argument; put such expressions in a
variable first.

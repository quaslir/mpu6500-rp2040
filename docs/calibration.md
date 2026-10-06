# Calibration

Every MPU6500 has small offsets: at rest the gyro does not read exactly 0 °/s and the
accelerometer does not read exactly (0, 0, 1) g. On the test module the gyro offset was about
+8 / +1.8 / −3.4 °/s and the accel offset about −0.1 / +0.01 / +0.13 g. Without calibration a
gyro offset of 8 °/s turns into 480° of error per minute when integrated.

The driver supports two kinds of offset.

| Kind | Where it is applied | Units | Set by |
|---|---|---|---|
| Software offset | subtracted by the driver on every read | g, °/s | `calibrate_gyro()`, `calibrate_accel()`, `set_*_offset()` |
| Hardware gyro offset | added by the chip before the data registers | raw counts | `set_gyro_hw_offset()` |

The software offset is the main tool. It is stored in physical units, so changing the range
does not invalidate it.

## Requirements

- The power mode must be `Normal`. In sleep there is no data, in low-power mode the gyro is
  off and the accel filter is bypassed. Calibration returns `ERROR` in other modes.
- The board must be **still** for the whole measurement.
- For the accelerometer, the board must lie in a known orientation (by default flat, chip
  facing up).

## Gyro

```cpp
namespace cal = mpu6500::calibration;

if (cal::calibrate_gyro(imu) == bus::Status::OK) {
    const math::Vec3& o = imu.config().calibration.gyro_offset_dps;
}
```

At rest the true rotation rate is zero, so the average reading **is** the offset. It is
stored in `calibration.gyro_offset_dps` and subtracted from every following read.

## Accelerometer

```cpp
cal::calibrate_accel(imu);                                    // flat, chip up
cal::calibrate_accel(imu, math::Vec3{0.0f, 0.0f, -1.0f});     // upside down
```

At rest the accelerometer measures only gravity. The function averages the readings and
subtracts the expected gravity vector (default `GRAVITY_Z_UP`, that is (0, 0, 1) g). The rest
is the offset.

Before accepting the result it checks that the length of the averaged vector is between
`MIN_GRAVITY_G` (0.8 g) and `MAX_GRAVITY_G` (1.2 g). Outside that, the board was moving or the
data is wrong, and `ERROR` is returned.

**The surface tilt ends up in the offset.** If the table is tilted by 1°, that tilt is
calibrated away and the board reads 0° roll and pitch on that table. For absolute levelness
calibrate on a surface you know is level.

## How a measurement works

Both calibrations use the same averaging procedure (`measure_mean`):

1. **Warm-up:** read and discard `warmup_samples` samples, so filters settle.
2. **Measure:** read `samples` samples at the configured sample rate and average them.
3. **Spread check:** if any axis varies by more than `max_spread` between its minimum and
   maximum, the board moved, and `ERROR` is returned without changing the offset.

| Options | samples | warmup_samples | max_spread |
|---|---:|---:|---:|
| `DEFAULT_GYRO_OPTIONS` | 200 | 10 | 2.0 °/s |
| `DEFAULT_ACCEL_OPTIONS` | 200 | 10 | 0.05 g |

The duration follows from the sample rate: at 100 Hz, 210 samples take about 2.1 s; at 20 Hz
they take about 10.5 s. Calibrate at a higher rate to keep the board-still time short.

Custom options:

```cpp
cal::MeasureOptions opts{.samples = 500, .warmup_samples = 20, .max_spread = 1.0f};
cal::calibrate_gyro(imu, opts);
```

Measure without storing the result:

```cpp
math::Vec3 offset{};
cal::measure_gyro_offset(imu, offset);
cal::measure_accel_offset(imu, offset);
```

## When calibration fails

| Cause | What to do |
|---|---|
| board moved or vibrated | keep it still and retry; examples retry up to three times |
| power mode is not `Normal` | switch to `Normal` first |
| accel: board not in the expected orientation | lay it flat chip-up, or pass the real gravity vector |
| bus error | check wiring; the status tells `TIMEOUT` / `NACK` |

## Reusing offsets

Offsets of one board change little between power-ups. Calibrate once, print the offsets, and
put them into the configuration:

```cpp
config.calibration.gyro_offset_dps = {7.965f, 1.796f, -3.403f};
config.calibration.accel_offset_g = {-0.0822f, 0.0090f, 0.1337f};
```

The `accel_calibration` example prints this line for you. The gyro offset drifts with
temperature, so for best results recalibrate the gyro at start-up anyway; it only needs two
seconds of rest.

## Manual offsets

| Function | Effect |
|---|---|
| `set_gyro_offset(v)` | set the software gyro offset in °/s |
| `set_accel_offset(v)` | set the software accel offset in g |
| `clear_offsets()` | set both software offsets to zero |
| `read_*_uncorrected()` | read without subtracting the software offset |

## Hardware gyro offset

`set_gyro_hw_offset(RawVec3)` writes the chip's `XG/YG/ZG_OFFSET` registers. The chip adds
these values before the data reaches the data registers, so they also apply to the FIFO and
cost nothing at read time.

The values are raw counts. The register map does not state their scale; check it on the
hardware (write a known value and see how the output moves) before relying on it with
ranges other than the one you calibrated in. For most uses the software offset is simpler.

## Limitations

- Only **offsets** are calibrated. Scale and cross-axis errors are not; a six-position
  accelerometer calibration would cover the accelerometer scale.
- The gyro offset changes with temperature. There is no temperature compensation.
- The accelerometer hardware offset registers are not used.

## Examples

`examples/gyro_calibration` and `examples/accel_calibration` show the readings before and
after, the measured offsets and a live view. The console has `calgyro`, `calaccel` and
`clear`.

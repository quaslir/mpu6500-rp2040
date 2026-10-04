## Output rates: driver prediction vs chip

Checks that `gyro_sample_rate_hz()` and `accel_sample_rate_hz()` return what the chip
really does, for every way the rate can be set: low-pass filter with divider, and the
filter settings that ignore the divider.

### Method

Example: `examples/rates`. For each configuration the chip is reconfigured at 1 MHz
(SPI write limit), then three rates are measured for 0.5 s each:

| Column | How it is measured |
|---|---|
| Data ready | `INT_STATUS` is polled as fast as possible; every set data-ready flag is one new sample |
| Gyro measured | the gyro is read in a loop; every read that differs from the previous one counts |
| Accel measured | same for the accelerometer |

The change counters see each sample as long as the bus reads faster than the sensor
updates. On SPI the reads run at 8 MHz (read-only, allowed up to 20 MHz by the datasheet).

### SPI (reads at 8 MHz)

| Config | Gyro predicted, Hz | Accel predicted, Hz | Data ready, Hz | Gyro measured, Hz | Accel measured, Hz | Match |
|---|---:|---:|---:|---:|---:|---|
| DLPF 184, div 0 | 1000 | 1000 | 1000.0 | 1000.0 | 1000.0 | yes |
| DLPF 184, div 1 | 500 | 500 | 500.0 | 500.0 | 500.0 | yes |
| DLPF 92, div 4 | 200 | 200 | 200.0 | 200.0 | 200.0 | yes |
| DLPF 41, div 9 | 100 | 100 | 100.0 | 100.0 | 100.0 | yes |
| DLPF 20, div 19 | 50 | 50 | 50.0 | 50.0 | 50.0 | yes |
| DLPF 5, div 99 | 10 | 10 | 10.0 | 10.0 | 10.0 | yes |
| Gyro 250, accel 460, div 9 (ignored) | 8000 | 1000 | 7788.0 | 7969.8 | 1000.0 | yes |
| Gyro 3600, accel 460 | 8000 | 1000 | 7998.0 | 7999.8 | 1000.0 | yes |
| Gyro bypass 3600, accel bypass | 32000 | 4000 | 31993.9 | 31995.6 | 3999.9 | yes |
| Gyro bypass 8800, accel bypass | 32000 | 4000 | 31993.9 | 31995.4 | 3997.9 | yes |

### I2C (400 kHz)

Same configurations over I2C. Rates above about 4 kHz cannot be seen: one gyro read takes
about 247 µs (about 4050 reads per second) and one `INT_STATUS` read about 111 µs (about
9000 per second), so the counters stop at the bus limit.

| Config | Gyro predicted, Hz | Accel predicted, Hz | Data ready, Hz | Gyro measured, Hz | Accel measured, Hz | Result |
|---|---:|---:|---:|---:|---:|---|
| DLPF 184, div 0 | 1000 | 1000 | 999.8 | 999.8 | 999.6 | match |
| DLPF 184, div 1 | 500 | 500 | 499.9 | 500.0 | 499.8 | match |
| DLPF 92, div 4 | 200 | 200 | 200.0 | 199.9 | 200.0 | match |
| DLPF 41, div 9 | 100 | 100 | 100.0 | 100.0 | 100.0 | match |
| DLPF 20, div 19 | 50 | 50 | 50.0 | 50.0 | 50.0 | match |
| DLPF 5, div 99 | 10 | 10 | 10.0 | 10.0 | 10.0 | match |
| Gyro 250, accel 460, div 9 (ignored) | 8000 | 1000 | 7872.5 | 4063.2 | 999.6 | gyro: bus-limited, accel: match |
| Gyro 3600, accel 460 | 8000 | 1000 | 7998.5 | 4063.6 | 999.6 | gyro: bus-limited, accel: match |
| Gyro bypass 3600, accel bypass | 32000 | 4000 | 8984.0 | 4063.5 | 4000.2 | gyro: bus-limited, accel: at the limit |
| Gyro bypass 8800, accel bypass | 32000 | 4000 | 8984.0 | 4063.5 | 4000.3 | gyro: bus-limited, accel: at the limit |

### Observations

- **The driver's rate functions are correct for every configuration**, from 10 Hz to
  32 kHz. All measured rates are within 0.5 % of the prediction, except data ready in the
  "gyro 250" row (2.7 % low, still within tolerance).
- **The divider is really ignored** when the gyro filter is 250 Hz, 3600 Hz or a bypass:
  with divider 9 the gyro runs at 8 kHz and the accelerometer at 1 kHz, not 100 Hz. This
  confirms the fix in `accel_sample_rate_hz()`, which earlier applied the divider here.
- **Gyro and accelerometer can run at different rates.** With the gyro at 8 or 32 kHz the
  data registers update at the gyro rate, but the accelerometer values change only at
  1 kHz (DLPF) or 4 kHz (bypass).
- **Only SPI can follow the high rates.** Over I2C the measurement stops at about 4 kHz for
  gyro reads and about 9 kHz for status polling. This matches the bus timing measured in
  "Bus timing".

# MPU6500 driver for RP2040

A C++20 driver for the InvenSense MPU6500 6-axis IMU (3-axis accelerometer, 3-axis gyroscope,
temperature sensor) on the Raspberry Pi Pico, over I2C or SPI.

The driver core does not depend on the Pico SDK: it talks to the chip through a small
`bus::Bus` interface. The `bus_pico` library implements that interface for the RP2040.

## Features

- **I2C and SPI** through one interface, chosen at runtime by the bus object you pass in
- **Full configuration in one struct**: ranges, digital low-pass filters, sample rate divider,
  power, FIFO, interrupts and wake-on-motion, written to the chip by `init()`
- **Sample rate helpers** that know when the divider is ignored (8 kHz and 32 kHz modes),
  verified on hardware from 10 Hz to 32 kHz
- **FIFO** with batched reads, frame decoding for any combination of sources and correct
  overflow handling (no misaligned frames after an overflow)
- **Interrupts**: data ready, FIFO overflow and wake-on-motion, pulse or latched INT pin,
  with a flag cache so that no event is lost when several parts of the code read the status
- **Power modes**: normal, sleep and low-power accelerometer, switched with one call
- **Wake-on-motion** with a threshold in milligravity
- **Calibration**: software gyro and accel offsets, hardware gyro offset registers
- **Units in the API**: g, °/s, °C, mg and Hz instead of raw register values

## Hardware

Tested with:

| Part | Details |
|---|---|
| MCU | Raspberry Pi Pico (RP2040) |
| Sensor | MPU6500 breakout module, 3.3 V |
| I2C | 100 kHz and 400 kHz, address 0x68 |
| SPI | 1 MHz for register writes, up to 8 MHz for reads |

Default wiring used by the examples:

| Signal | I2C | SPI |
|---|---|---|
| SDA / SDI | GP4 | GP3 (MOSI) |
| SCL / SCLK | GP5 | GP2 |
| AD0 / SDO | GND (address 0x68) | GP4 (MISO) |
| NCS | 3.3 V | GP15 |
| INT | GP14 (optional) | GP14 (optional) |

Full wiring, pull-ups and notes in [docs/wiring.md](docs/wiring.md).


## Using the library in your project

Add the repository as a subdirectory after the Pico SDK is initialised:

```cmake
add_subdirectory(mpu6500-rp2040)

target_link_libraries(your_app PRIVATE
    mpu6500     # the driver
    bus_pico    # I2C / SPI implementation for RP2040
    pico_stdlib
    hardware_i2c
    hardware_spi
)
```

## Building the examples

```bash
mkdir build && cd build
make -j
```

Each example produces a `.uf2` in `build/examples/<name>/`. Flash it in BOOTSEL mode or with
`picotool load -f <file>.uf2`. Output goes to USB serial.

| Example | What it shows |
|---|---|
| `whoami` | Bus check: reads the chip ID |
| `read_data` | Reading accel, gyro and temperature in a loop |
| `console` | Interactive serial console for every driver setting |
| `filters` | Noise for each low-pass filter setting |
| `gyro_calibration` | Gyro offset calibration, before and after |
| `accel_calibration` | Accel offset calibration, before and after |
| `tilt` | Roll and pitch with a complementary filter |
| `fifo` | Batched FIFO reads, rate check and overflow handling |
| `interrupts` | Data-ready interrupt on the INT pin |
| `wom` | Wake-on-motion in low-power mode |
| `benchmark` | Timing of every driver operation, as a Markdown table |
| `rates` | Predicted vs measured output rates, as a Markdown table |

## Performance

Measured on hardware, full tables in [docs/measurements.md](docs/measurements.md).

| `read_all` (accel + temp + gyro, 14 bytes) | Time |
|---|---:|
| I2C 100 kHz | 1674 µs |
| I2C 400 kHz | 465 µs |
| SPI 1 MHz | 158 µs |
| SPI 8 MHz (reads only) | 34 µs |

At 1 kHz, reading every sample by interrupt takes about 58 % of the time on I2C 400 kHz and
about 18 % on SPI 1 MHz. Above 1 kHz use SPI with the FIFO.

## Documentation

| Document | Contents |
|---|---|
| [docs/wiring.md](docs/wiring.md) | Wiring for I2C and SPI, pull-ups, INT pin |
| [docs/architecture.md](docs/architecture.md) | Layers, design decisions, porting to another MCU |
| [docs/configuration.md](docs/configuration.md) | Every `Config` field, defaults, sample rate rules |
| [docs/fifo.md](docs/fifo.md) | How the FIFO works, overflow, how often to read |
| [docs/interrupts.md](docs/interrupts.md) | Interrupt sources, pulse vs latched, GPIO setup |
| [docs/power.md](docs/power.md) | Power modes and what low-power mode changes |
| [docs/calibration.md](docs/calibration.md) | Software and hardware offsets |
| [docs/measurements.md](docs/measurements.md) | Bus timing and output rate measurements |
| [docs/known-issues.md](docs/known-issues.md) | Problems found during development and their fixes |

## Project structure

```
lib/bus          bus::Bus interface and bus::Status
lib/bus_pico     I2C and SPI implementation for RP2040
lib/math         Vec3 and RawVec3
lib/orientation  complementary filter used by the tilt example
mpu6500/         the driver
  include/       public headers
  src/driver/    Mpu6500 class methods
  src/registers/ register map, bit masks, register access
  src/codec/     decoding of bytes and FIFO frames, no bus access
examples/        example programs
docs/            documentation
```

## Limitations

- **Not implemented:** DMP, FSYNC, the auxiliary I2C master (external magnetometer), self-test.
- **Errors:** driver errors that are not bus errors (wrong mode, board moved during
  calibration) are all reported as `bus::Status::ERROR`.
- **SPI cannot detect failed writes:** there is no acknowledge on SPI. `init()` checks
  `WHO_AM_I`, but a wrongly wired SPI bus can still return `OK` from configuration calls.
- **Long I2C FIFO reads:** a large FIFO batch at 100 kHz can take longer than the I2C
  timeout. Use 400 kHz, smaller batches, or a longer timeout.
- **INT pin over I2C:** with the INT wire routed next to SDA/SCL, INT edges can disturb the
  I2C bus at high rates. See [docs/known-issues.md](docs/known-issues.md).

## License

See [LICENSE](LICENSE).

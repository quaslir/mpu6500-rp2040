## Bus timing

How long driver operations take over SPI and I2C, measured on the hardware.

### Setup

| Parameter | Value |
|---|---|
| MCU | RP2040 (Raspberry Pi Pico), default clock |
| Bus driver | Pico SDK, blocking calls (no DMA) |
| Sensor rate | 1 kHz (gyro/accel filter 184 Hz, divider 0) |
| FIFO frame | accel + gyro, 12 bytes |
| Iterations | 1000 per read, 50 per FIFO read, 3 for `init` |
| Timer | `time_us_32()`, 1 µs resolution |
| I2C timeout | 100 ms per transaction (see "Observations") |
| Example | `examples/benchmark`, built with `IMU_BUS=SPI` or `IMU_BUS=I2C` |

**Wire time** is the theoretical time the bits spend on the bus:

- SPI: 8 bits per byte, plus one address byte;
- I2C: 9 bits per byte (8 data + ACK) for address+W, register, address+R and every data
  byte, plus START, repeated START and STOP.

**Efficiency** is wire time divided by the measured mean time. 100 % would mean no overhead.

`read_fifo` makes three transactions: `INT_STATUS` (1 byte), `FIFO_COUNT` (2 bytes) and the
frame data. In the tables below, "+ 3" are those two control reads.

### SPI at 1 MHz (actual 992 kHz)

1 MHz is the datasheet limit for register writes, so the whole driver runs at this clock.

| Operation | Bytes on bus | Mean, µs | Min, µs | Max, µs | Wire, µs | Efficiency |
|---|---:|---:|---:|---:|---:|---:|
| `read_all` | 14 | 157.5 | 157 | 203 | 121.0 | 77 % |
| `read_accel` | 6 | 73.4 | 73 | 122 | 56.4 | 77 % |
| `read_gyro` | 6 | 73.5 | 73 | 96 | 56.4 | 77 % |
| `read_temp` | 2 | 30.3 | 30 | 70 | 24.2 | 80 % |
| `who_am_i` | 1 | 19.2 | 19 | 35 | 16.1 | 84 % |
| `take_interrupt_flags` | 1 | 19.6 | 19 | 33 | 16.1 | 82 % |
| `read_fifo`, 1 frame | 12 + 3 | 195.2 | 192 | 329 | 145.2 | 74 % |
| `read_fifo`, 10 frames | 120 + 3 | 1343.7 | 1343 | 1362 | 1016.1 | 76 % |
| `read_fifo`, 40 frames | 480 + 3 | 5180.0 | 5179 | 5187 | 3919.1 | 76 % |

### SPI at 8 MHz (actual 7.81 MHz), read-only operations

The datasheet allows up to 20 MHz for reading sensor and interrupt registers, but not for
writes. Only operations that never write were measured at this clock.

| Operation | Bytes on bus | Mean, µs | Min, µs | Max, µs | Wire, µs | Efficiency |
|---|---:|---:|---:|---:|---:|---:|
| `read_all` | 14 | 33.9 | 33 | 46 | 15.4 | 45 % |
| `read_accel` | 6 | 16.7 | 16 | 34 | 7.2 | 43 % |
| `read_gyro` | 6 | 16.8 | 16 | 60 | 7.2 | 43 % |
| `read_temp` | 2 | 7.0 | 6 | 43 | 3.1 | 44 % |
| `who_am_i` | 1 | 4.3 | 4 | 34 | 2.0 | 47 % |
| `take_interrupt_flags` | 1 | 4.6 | 4 | 33 | 2.0 | 43 % |

### I2C at 100 kHz

| Operation | Bytes on bus | Mean, µs | Min, µs | Max, µs | Wire, µs | Efficiency |
|---|---:|---:|---:|---:|---:|---:|
| `read_all` | 14 | 1673.7 | 1673 | 1735 | 1560.0 | 93 % |
| `read_accel` | 6 | 899.0 | 897 | 943 | 840.0 | 93 % |
| `read_gyro` | 6 | 899.0 | 898 | 917 | 840.0 | 93 % |
| `read_temp` | 2 | 514.6 | 514 | 528 | 480.0 | 93 % |
| `who_am_i` | 1 | 418.3 | 412 | 419 | 390.0 | 93 % |
| `take_interrupt_flags` | 1 | 418.3 | 413 | 452 | 390.0 | 93 % |
| `read_fifo`, 1 frame | 12 + 3 | 2424.4 | 2422 | 2523 | 2250.0 | 93 % |
| `read_fifo`, 10 frames | 120 + 3 | 12923.9 | 12922 | 12953 | 11970.0 | 93 % |
| `read_fifo`, 40 frames | 480 + 3 | 47926.5 | 47924 | 47947 | 44370.0 | 93 % |

Measured with a 100 ms I2C timeout. With the original 30 ms timeout the 40-frame read
failed every time: 480 bytes need about 44 ms on the wire. See "Observations".

### I2C at 400 kHz (actual 399 kHz)

| Operation | Bytes on bus | Mean, µs | Min, µs | Max, µs | Wire, µs | Efficiency |
|---|---:|---:|---:|---:|---:|---:|
| `read_all` | 14 | 464.8 | 464 | 488 | 390.6 | 84 % |
| `read_accel` | 6 | 247.4 | 247 | 268 | 210.3 | 85 % |
| `read_gyro` | 6 | 247.5 | 247 | 256 | 210.3 | 85 % |
| `read_temp` | 2 | 138.5 | 138 | 148 | 120.2 | 87 % |
| `who_am_i` | 1 | 111.0 | 110 | 147 | 97.7 | 88 % |
| `take_interrupt_flags` | 1 | 111.1 | 110 | 128 | 97.7 | 88 % |
| `read_fifo`, 1 frame | 12 + 3 | 666.1 | 665 | 702 | 563.4 | 85 % |
| `read_fifo`, 10 frames | 120 + 3 | 3608.6 | 3607 | 3632 | 2997.3 | 83 % |
| `read_fifo`, 40 frames | 480 + 3 | 13417.7 | 13416 | 13442 | 11110.2 | 83 % |

### Comparison

Mean time per operation, µs.

| Operation | I2C 100 kHz | I2C 400 kHz | SPI 1 MHz | SPI 8 MHz |
|---|---:|---:|---:|---:|
| `read_all` | 1673.7 | 464.8 | 157.5 | 33.9 |
| `read_accel` | 899.0 | 247.4 | 73.4 | 16.7 |
| `who_am_i` | 418.3 | 111.0 | 19.2 | 4.3 |
| `read_fifo`, 10 frames | 12923.9 | 3608.6 | 1343.7 | — |
| Speed-up of `read_all` vs I2C 100 kHz | 1× | 3.6× | 10.6× | 49× |

### Bus budget

Share of bus time needed to keep up with the sensor. Over 100 % means the bus cannot keep up.

**One sample per interrupt** (`take_interrupt_flags` + `read_all` per sample):

| Sample rate | I2C 100 kHz (2092 µs) | I2C 400 kHz (576 µs) | SPI 1 MHz (177 µs) | SPI 8 MHz (38.5 µs) |
|---:|---:|---:|---:|---:|
| 100 Hz | 21 % | 5.8 % | 1.8 % | 0.4 % |
| 500 Hz | 105 % | 29 % | 8.9 % | 1.9 % |
| 1 kHz | 209 % | 58 % | 18 % | 3.9 % |

**FIFO**, 1 kHz, read every 10 ms (10 frames per read):

| Bus | Time per read | Bus time |
|---|---:|---:|
| I2C 100 kHz | 12.9 ms | 129 %: cannot keep up |
| I2C 400 kHz | 3.6 ms | 36 % |
| SPI 1 MHz | 1.3 ms | 13 % |

**Highest FIFO frame rate the bus can sustain** (accel + gyro, from the 40-frame batch,
all bus time spent on reading):

| Bus | Time per frame | Max frame rate |
|---|---:|---:|
| I2C 100 kHz | 1198 µs | about 830 Hz |
| I2C 400 kHz | 335 µs | about 3 kHz |
| SPI 1 MHz | 130 µs | about 7.7 kHz |

### init

| Bus | Mean | Min | Max |
|---|---:|---:|---:|
| SPI 1 MHz | 201.1 ms | 201.1 ms | 201.2 ms |
| I2C 100 kHz | 218.7 ms | 218.7 ms | 218.8 ms |

200 ms of this are the two 100 ms waits after `DEVICE_RESET` and `SIGNAL_PATH_RESET`
required by the datasheet. Writing the whole configuration takes about 1 ms over SPI and
about 19 ms over I2C at 100 kHz.

### Observations

- **No errors** on any bus once the I2C timeout was raised to 100 ms.
- **I2C is efficient but slow.** 83–93 % of the time is spent on the wire: the RP2040 I2C
  block handles bytes in hardware and the CPU is never the bottleneck. The bus itself is
  the limit.
- **SPI at 1 MHz loses about 2.5 clock periods per byte.** Efficiency stays at 74–84 %
  from 1 to 483 bytes, so the overhead is per byte, not per call: the gap the RP2040 SPI
  block adds between bytes in blocking mode.
- **SPI at 8 MHz is limited by the CPU.** About 2.3 µs per byte against 1.0 µs of wire
  time. A higher clock would gain little; DMA would be the next step.
- **Long I2C reads hit the timeout.** The driver reads a whole FIFO batch in one
  transaction. At 100 kHz, 40 frames (480 bytes) take about 48 ms. With the original 30 ms
  I2C timeout this read failed 50 times out of 50; with 100 ms it passes. A fixed timeout
  is fragile: better scale it with the transfer size, or read the FIFO in smaller chunks.
- **I2C at 100 kHz cannot carry 1 kHz accel + gyro.** The data alone is 12 kB/s; the bus
  delivers about 10 kB/s. During the 48 ms read, 48 new frames arrive while 40 are read,
  so in continuous use the FIFO would overflow. The 40-frame result is valid as a timing
  measurement only.
- **Jitter.** Maximum times are up to 2× the mean on short SPI reads (for example 35 µs vs
  19 µs): other interrupts on the Pico (USB, timer). Real-time code should budget for the
  maximum, not the mean.
- **FIFO reads should be batched.** The two control reads cost about 50 µs on SPI and about
  220 µs on I2C 400 kHz per call, whatever the batch size.

### Recommendations

| Use case | Bus | Method |
|---|---|---|
| Up to 100 Hz, simple wiring | I2C, 100 or 400 kHz | interrupt per sample or polling |
| Up to 500 Hz | I2C 400 kHz or SPI | interrupt per sample or FIFO |
| 1 kHz | SPI, or I2C 400 kHz with FIFO | FIFO |
| Above 1 kHz | SPI | FIFO, faster read clock, ideally DMA |

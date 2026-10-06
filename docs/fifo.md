# FIFO

The MPU6500 has a 512-byte FIFO buffer. On every sample the chip writes the selected data
into it by itself, and the application reads it in batches whenever it suits. Nothing is lost
as long as the buffer is read before it fills up.

## Why use it

Without the FIFO, the data registers hold only the **latest** sample. To get every sample,
the application must read exactly once per sample period: read too late and a sample is
lost, read too early and the same sample is read twice, and there is no way to tell.

With the FIFO:

- **no samples are lost** while the buffer does not overflow;
- **the CPU is free**: one read every 20–100 ms instead of one read per sample;
- **timing is exact**: samples are taken on the chip's clock with a fixed period, so the time
  of each sample is known even if it is read later;
- **less bus overhead per sample**: one long transfer is cheaper than many short ones.

## Configuration

```cpp
namespace cfg = mpu6500::config;

cfg::Config config{};
config.measurement.gyro.filter = cfg::GyroFilter::Hz184;
config.measurement.accel.filter = cfg::AccelFilter::Hz184;
config.measurement.sample_divider = 0;          // 1 kHz

config.fifo.enabled = true;
config.fifo.sources.accel = true;               // default
config.fifo.sources.gyro = true;                // default
config.fifo.sources.temperature = false;        // default
config.fifo.mode = cfg::FifoMode::Overwrite;    // default
```

| Field | Default | Meaning |
|---|---|---|
| `fifo.enabled` | `false` | FIFO on or off |
| `fifo.sources.accel` | `true` | write the 3 accelerometer axes (6 bytes) |
| `fifo.sources.temperature` | `false` | write the temperature (2 bytes) |
| `fifo.sources.gyro` | `true` | write the 3 gyro axes (6 bytes) |
| `fifo.mode` | `Overwrite` | what happens when the buffer is full, see below |

The gyro is enabled as a whole: per-axis gyro sources are not supported, which keeps the
frame layout simple.

When `fifo.enabled` is `false`, the driver writes no sources at all, so the chip does not
buffer anything in the background.

## Frames

One sample in the FIFO is a **frame**. Its blocks are always in the same order: accel,
temperature, gyro. A disabled block is simply missing and the following blocks move up.

| Sources | Frame size | accel at | temp at | gyro at | Frames in 512 bytes |
|---|---:|---:|---:|---:|---:|
| accel + temp + gyro | 14 | 0 | 6 | 8 | 36 |
| accel + gyro | 12 | 0 | — | 6 | 42 |
| accel + temp | 8 | 0 | 6 | — | 64 |
| temp + gyro | 8 | — | 0 | 2 | 64 |
| accel | 6 | 0 | — | — | 85 |
| gyro | 6 | — | — | 0 | 85 |
| temp | 2 | — | 0 | — | 256 |

There are no markers between frames in the buffer. The driver cuts the byte stream into
frames using the size computed from the configuration.

## Reading

```cpp
std::array<mpu6500::Sample, 64> samples{};
mpu6500::FifoReadResult result{};

if (imu.read_fifo(samples, result) == bus::Status::OK) {
    for (std::size_t i = 0; i < result.frames; ++i) {
        const mpu6500::Sample& s = samples[i];
        // s.accel_g, s.gyro_dps, s.temperature_c
    }
    if (result.overflowed) {
        // data was lost since the last read
    }
}
```

`read_fifo()`:

- reads **whole frames only**; a partial frame stays in the buffer for the next read;
- reads at most as many frames as fit into the span you pass;
- converts every frame to `Sample` in g, °/s and °C and subtracts the software offsets,
  exactly like `read_all()`;
- leaves fields of disabled sources at zero;
- returns `OK` with zero frames when the FIFO is disabled or the span is empty.

| Function | What it does |
|---|---|
| `read_fifo(span, result)` | read and decode up to `span.size()` frames |
| `fifo_frame_count(count)` | number of whole frames currently in the buffer |
| `fifo_reset()` | empty the buffer and drop a stale overflow flag |

The oldest frame comes first (`samples[0]`), the newest last.

## Overflow

If the buffer is not read in time, it fills up. What happens next depends on the mode.

### `Overwrite` (default)

The chip keeps writing and drops the **oldest bytes**. 512 is not a multiple of most frame
sizes (for example 12 or 14), so after an overflow the frame boundaries in the buffer are
shifted: reading on would turn the middle of one frame into the start of the next and
return plausible-looking garbage.

The driver therefore:

1. sees the overflow flag on the next `read_fifo()`;
2. resets the FIFO;
3. returns **zero frames** with `result.overflowed = true`.

The data since the previous read is lost, but every frame returned after that is correct.

### `StopWhenFull`

The chip stops writing when the buffer is full. The data stays aligned, so `read_fifo()`
returns the buffered (oldest) frames and sets `result.overflowed = true`. Every sample taken
after the buffer filled up and before the read is missing: there is a gap in time after the
last returned frame.

Use it to capture a burst of consecutive samples (for example a vibration snapshot). For a
continuous stream use `Overwrite` and read often enough.

## How often to read

Time until the buffer is full = frames in 512 bytes / sample rate.

| Sources | 100 Hz | 200 Hz | 500 Hz | 1 kHz |
|---|---:|---:|---:|---:|
| accel + gyro (12 bytes) | 420 ms | 210 ms | 84 ms | 42 ms |
| accel + temp + gyro (14 bytes) | 360 ms | 180 ms | 72 ms | 36 ms |
| accel only (6 bytes) | 850 ms | 425 ms | 170 ms | 85 ms |

Rule of thumb: read at least every **half** of the fill time. At 1 kHz with accel + gyro that
is every 20 ms or more often. Pass a span large enough for everything that can be in the
buffer (64 samples is enough for any layout except temperature only).

## Bus limits

The bus must be able to move the data faster than the chip produces it. Measured on RP2040
(see [measurements.md](measurements.md)):

| Bus | Time per 12-byte frame | Highest sustainable rate |
|---|---:|---:|
| I2C 100 kHz | about 1.2 ms | about 830 Hz |
| I2C 400 kHz | about 335 µs | about 3 kHz |
| SPI 1 MHz | about 130 µs | about 7.7 kHz |

Each `read_fifo()` call also costs two short control reads (interrupt status and FIFO
count): about 50 µs on SPI and about 220 µs on I2C 400 kHz. Read in batches rather than one
frame at a time.

**Long reads over I2C:** a batch is read in one transaction. At 100 kHz, 40 frames take
about 48 ms, longer than a 30 ms I2C timeout. Use a timeout that covers the largest batch,
smaller batches, or 400 kHz. See [known-issues.md](known-issues.md).

## Starting and restarting

- `init()` resets the FIFO and enables it if configured.
- After anything that stalls reading for a while (calibration, long prints, changing
  settings), call `fifo_reset()` before the first read. Otherwise the first read returns old
  data or an overflow.
- `fifo_reset()` also drops the overflow flag that was set while the buffer was full.
  `FIFO_RST` alone does not clear that flag.

## FIFO and interrupts

The FIFO overflow flag belongs to `read_fifo()`. `take_interrupt_flags()` reports it to the
user but does not clear it, so the next `read_fifo()` still sees it and resets the buffer.
See [interrupts.md](interrupts.md).

The MPU6500 has **no FIFO threshold interrupt** ("half full"), only "overflowed". Do not use
the overflow interrupt to trigger reads: by then data is already lost. Read on a timer, or
on every N-th data-ready interrupt.

## Example

`examples/fifo` reads the FIFO every 100 ms at 100 Hz, prints the number of frames and the
measured rate, and forces an overflow every few seconds to show that the data after it is
correct.

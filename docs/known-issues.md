# Known issues and findings

Problems found while building and testing the driver on hardware: what was seen, how it
was tracked down, what caused it and what was done about it.

## Summary

| # | Issue | Area | Status |
|---|---|---|---|
| 1 | INT pin activity breaks the I2C bus | hardware / wiring | workaround |
| 2 | SPI writes were silently turned into reads | `bus_pico` | fixed |
| 3 | Long FIFO reads over I2C exceed the bus timeout | `bus_pico` / driver | workaround |
| 4 | SPI clock limits for writes and reads | hardware constraint | documented |
| 5 | Accelerometer rate reported wrong when the divider is ignored | driver | fixed |
| 6 | Reading `INT_STATUS` lost other events | driver | fixed by design |
| 7 | False FIFO overflow on the first read after a reset | driver | fixed |
| 8 | FIFO kept buffering while disabled | driver | fixed |
| 9 | Low-power mode did not start when gyro standby was on | driver | fixed by design |

---

## 1. INT pin activity breaks the I2C bus

**Status:** workaround. Not seen on SPI.

### Symptoms

- With the INT wire connected and latched interrupt mode, the **first** read of
  `INT_STATUS` returned `NACK`, and every transfer after that returned `TIMEOUT`. The bus
  never recovered without a power cycle.
- `WHO_AM_I` worked right before that read, even with the INT pin high and the GPIO interrupt
  configured.
- In pulse mode at 100 Hz everything worked. At 500 Hz the bus died again, this time during
  calibration, before the GPIO interrupt was even set up.
- At 200 Hz there was an occasional single failed read; the bus survived it.

### How it was tracked down

A diagnostic version of the interrupt example checked the bus with `WHO_AM_I` after every
setup step and printed the INT pin level and function:

| Test | Result |
|---|---|
| GPIO setup steps one by one | bus OK after each of them |
| INT wire disconnected, latched mode | bus OK, flags read correctly |
| INT wire connected, latched mode | bus dies on the first `INT_STATUS` read |
| INT wire connected, pulse mode, 100 Hz | works, 100 events per second, no errors |
| INT wire connected, pulse mode, 500 Hz | bus dies during calibration |
| INT wire disconnected, 500 Hz | calibration and bus OK |
| 4.7 kΩ pull-ups added on SDA/SCL | no change |
| I2C at 400 kHz instead of 100 kHz | rare single errors, bus survives |

### Cause

The INT pin switching disturbs the I2C lines. The setup is a breadboard with jumper wires,
the INT wire running next to SDA and SCL.

- In **latched** mode the chip releases INT exactly while it answers the `INT_STATUS` read,
  so the edge always lands in the middle of an I2C transfer.
- In **pulse** mode the edges come at sample time and only sometimes overlap a transfer.
  The more samples per second and the longer the transfers, the more often they overlap.
- At 400 kHz each transfer is four times shorter, so overlaps are rarer.

Once one transfer is corrupted, the chip can hold SDA low and the RP2040 I2C block does not
recover on its own, so a single glitch kills the bus.

The exact coupling path (crosstalk between wires or ground bounce on the module) was not
measured; it is most likely one of the two.

### Workaround

- Use **pulse** mode instead of latched.
- Run I2C at **400 kHz**.
- Use **SPI** for high rates: its lines are push-pull and much harder to disturb.
- Wiring: a short separate ground wire between module and Pico, the INT wire short and away
  from SDA/SCL, optionally a 330 Ω – 1 kΩ series resistor in the INT line near the module.

### Possible improvement

I2C bus recovery in `bus_pico`: on timeout, switch SCL to GPIO, send nine clock pulses and a
STOP, then return the pins to I2C. A single glitch would then cost one read instead of the
whole bus.

---

## 2. SPI writes were silently turned into reads

**Status:** fixed in `lib/bus_pico/src/spi_bus.cpp`.

### Symptoms

Over SPI:

- `WHO_AM_I`, `init()`, calibration and all reads returned `OK` and gave sensible data;
- but no interrupt ever reached the INT pin;
- polling `INT_STATUS` every 1 ms showed "data ready" on **every** poll, although the sample
  rate was configured to 100 Hz.

### How it was tracked down

An SPI diagnostic example checked the chain chip → INT pin → wire → GPIO → interrupt:

- the chip reported data ready 1000 times per second instead of 100: the divider had not
  been written;
- the INT pin never went high, and a pull-up / pull-down test showed the wire was connected
  and actively driven low: the interrupt source had not been enabled.

So reads worked, but **no configuration had reached the chip**.

### Cause

The write address was built as:

```cpp
uint8_t reg_write = ~(reg & read_bit_);
```

All MPU6500 register addresses are below `0x80`, so `reg & 0x80` is always `0` and `~0` is
`0xFF`. Every write was sent as `0xFF`: bit 7 set means a **read** of register `0x7F`. The
chip answered with a byte and ignored the data.

The bus could not notice: SPI has no acknowledge, so a write that goes nowhere looks exactly
like a successful one.

### Fix

Clear the read bit instead of inverting the result:

```cpp
const uint8_t reg_write = static_cast<uint8_t>(reg & ~read_bit_);
```

### Lessons

- Over SPI, `OK` from a write only means the bytes left the Pico.
- Planned: **read-back verification** of configuration writes in the register layer (write,
  read back, compare under the mask; skipped for self-clearing bits such as `DEVICE_RESET`,
  `FIFO_RST`, `SIGNAL_PATH_RESET`). This would have made `init()` fail on the first write.
- A host unit test checking the address byte sent for a write would also have caught it.

---

## 3. Long FIFO reads over I2C exceed the bus timeout

**Status:** workaround (longer timeout). Planned fix: read the FIFO in chunks.

### Symptoms

In the benchmark, reading 40 FIFO frames over I2C at 100 kHz failed 50 times out of 50.
The same read at 400 kHz and over SPI worked.

### Cause

40 frames are 480 bytes. At 100 kHz with 9 bits per byte this takes about 48 ms on the wire.
The I2C bus was created with a 30 ms timeout per transaction, and `read_fifo()` reads the
whole batch in one transaction, so the Pico SDK aborted it every time.

With a 100 ms timeout the read takes 47.9 ms and passes.

### Workaround and fix

- Use a timeout that covers the largest transfer, or I2C at 400 kHz.
- Better: let `read_fifo()` read the data in chunks (for example 10 frames per transaction).
  `FIFO_R_W` can be read in any number of pieces; the data continues where the last read
  stopped.

### Related limit

I2C at 100 kHz cannot carry accel + gyro at 1 kHz at all: the data is 12 kB/s, the bus
delivers about 10 kB/s. The FIFO overflows no matter how it is read.

---

## 4. SPI clock limits

**Status:** hardware constraint, documented.

- The MPU6500 accepts **register writes only up to 1 MHz**. Configuration (`init()` and
  every setter) must run at 1 MHz or below.
- **Reads** of sensor and interrupt registers are allowed up to 20 MHz.
- Measured: at 8 MHz `read_all` takes 34 µs instead of 158 µs at 1 MHz, but only about 45 %
  of that time is spent on the wire. The blocking SDK loop becomes the limit, so going higher
  without DMA gains little.

A driver that switches between a slow clock for writes and a fast clock for reads is a
possible improvement.

---

## 5. Accelerometer rate reported wrong when the divider is ignored

**Status:** fixed in `accel_sample_rate_hz()`.

### Symptom

With gyro filter 250 Hz (8 kHz internal rate) and divider 9, the driver reported the
accelerometer at 100 Hz.

### Cause

The sample rate divider only works when the gyro filter is 184…5 Hz. In 8 kHz and 32 kHz
modes it is ignored for the whole chip, and the accelerometer runs at its own 1 kHz. The
function applied the divider anyway.

### Fix and verification

The divider is applied only when `divider_effective()` is true. The `rates` example confirms
on hardware: in this configuration the accelerometer changes 1000 times per second.

---

## 6. Reading `INT_STATUS` lost other events

**Status:** fixed by design (flag cache).

### Cause

Reading `INT_STATUS` clears **all** flags. `read_fifo()` read it to check for overflow and
cleared the data-ready and wake-on-motion flags with it. Code waiting for those events would
never see them; in the other direction, a user reading the flags would hide an overflow from
`read_fifo()`, which would then read misaligned frames.

### Fix

All reads go through `poll_int_status()`, which ORs the flags into a cache. Each flag has one
owner that clears it. Details in [architecture.md](architecture.md#4-interrupt-flags-have-owners).

---

## 7. False FIFO overflow on the first read after a reset

**Status:** fixed in `fifo_reset()` and `apply_config()`.

### Symptom

In the FIFO example the first read after calibration always reported an overflow and
returned 0 frames, losing 100 ms of good data.

### Cause

During calibration the FIFO filled up and the chip set the overflow flag. `FIFO_RST` empties
the buffer but does **not** clear that flag; only reading `INT_STATUS` does.

### Fix

After resetting the FIFO, read `INT_STATUS` once and drop the overflow flag. The order
matters: **reset first, then read**. Reading first leaves a window in which a full FIFO
overflows again and sets the flag before the reset.

---

## 8. FIFO kept buffering while disabled

**Status:** fixed in `apply_config()`.

### Symptom

With the FIFO disabled, the first `take_interrupt_flags()` in the interrupt example reported
`fifo_overflow = 1`.

### Cause

`USER_CTRL.FIFO_EN = 0` only disables FIFO **access** over the serial interface. Whether the
chip writes into the buffer is controlled by the `FIFO_EN` register (0x23). `apply_config()`
wrote the sources from the configuration (accel and gyro by default) even when the FIFO was
disabled, so the buffer kept filling and overflowing in the background.

Writing `FifoSources{}` does not help: the default member values of `FifoSources` are accel
and gyro on.

### Fix

When `fifo.enabled` is false, `apply_config()` writes a `FifoSources` value with all three
sources explicitly off, so nothing is buffered and no overflow is reported while the FIFO is
not in use.

---

## 9. Low-power mode did not start when gyro standby was on

**Status:** fixed by design (power modes as an overlay).

### Cause

The datasheet requires `GYRO_STANDBY` to be cleared for `CYCLE` mode to work. The first
implementation of low-power mode did not touch `GYRO_STANDBY`, so with standby enabled the
chip stayed out of cycle mode while the driver reported low power. It also did not restore
sleep when leaving low-power mode, and a failure halfway through left the state inconsistent.

### Fix

`apply_power()` computes all power registers from the mode on every change and forces
`GYRO_STANDBY` off in low-power mode. There is no backup to restore. Details in
[architecture.md](architecture.md#3-power-modes-are-an-overlay-not-a-backup).

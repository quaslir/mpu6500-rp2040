# Interrupts

The MPU6500 can signal events on its INT pin. The driver configures which events are
signalled and how the pin behaves; reacting to the pin is done by the application on the
microcontroller.

## Sources

| Source | `config.interrupts.sources` | Fires when |
|---|---|---|
| Data ready | `raw_data_ready` | a new sample is in the data registers |
| FIFO overflow | `fifo_overflow` | the FIFO has overflowed (data already lost) |
| Wake-on-motion | `wake_on_motion` | acceleration changed by more than the threshold |

All sources are off by default.

- **Data ready** replaces reading on a timer: read exactly when the chip has a new sample,
  without missing or duplicating samples.
- **FIFO overflow** is an alarm, not a trigger: there is no "FIFO half full" interrupt on
  this chip. See [fifo.md](fifo.md).
- **Wake-on-motion** works in low-power accelerometer mode. See "Wake-on-motion" below.

FSYNC and the auxiliary I2C master interrupts are not supported.

## INT pin behaviour

| Field | Values | Default | Meaning |
|---|---|---|---|
| `interrupts.level` | `ActiveHigh`, `ActiveLow` | `ActiveHigh` | active level of the pin |
| `interrupts.drive` | `PushPull`, `OpenDrain` | `PushPull` | push-pull if only one MCU is connected; open-drain needs an external pull-up |
| `interrupts.mode` | `Pulse`, `Latched` | `Pulse` | how long the pin stays active |

### Pulse vs latched

**Pulse:** on every event the pin goes active for about 50 µs and returns by itself.

```
events:  ↓            ↓            ↓
INT:  ___|‾|__________|‾|__________|‾|___
         50 µs
```

**Latched:** the pin goes active and stays active **until `INT_STATUS` is read**.

```
events:  ↓                        ↓
INT:  ___|‾‾‾‾‾‾‾‾|_______________|‾‾‾‾‾‾‾|___
                  ↑                       ↑
           take_interrupt_flags()  take_interrupt_flags()
```

| | Pulse | Latched |
|---|---|---|
| Can an event be missed | yes, if the edge is not caught | no, the pin waits |
| Can the pin get stuck | no | yes, if the flags are never read |
| Must read flags after every event | no | yes |
| Detect by | edge only | edge or level |
| Pin changes during an I2C read | no | yes, when the flags are read |

Recommendation: **pulse** for frequent events such as data ready; **latched** for rare,
important events (wake-on-motion) on a clean board. On the test setup, latched mode
disturbed the I2C bus; see [known-issues.md](known-issues.md).

The driver always clears flags **only** by reading `INT_STATUS` (`INT_ANYRD_2CLEAR` = 0).
Other reads, such as `read_all()`, do not release a latched pin.

## Reading the flags

```cpp
mpu6500::InterruptFlags flags{};
if (imu.take_interrupt_flags(flags) == bus::Status::OK) {
    if (flags.raw_data_ready) { /* read the sample */ }
    if (flags.wake_on_motion) { /* motion detected */ }
    if (flags.fifo_overflow)  { /* data was lost, read_fifo() will reset the FIFO */ }
}
```

`take_interrupt_flags()` reads `INT_STATUS` (and so releases a latched pin), reports all
pending flags and clears only the ones that belong to the user.

### Why flags have owners

Reading `INT_STATUS` clears **all** flags in the chip. If the FIFO code and the user code
both read it, one of them would lose events. The driver keeps a cache:

- every read of `INT_STATUS` ORs the flags into the cache; nothing is cleared there by the
  read itself;
- each flag is cleared from the cache only by its owner.

| Flag | Owner | Cleared by |
|---|---|---|
| Data ready | user | `take_interrupt_flags()` |
| Wake-on-motion | user | `take_interrupt_flags()` |
| FIFO overflow | FIFO code | `read_fifo()` / `fifo_reset()` |

So a FIFO overflow seen in `take_interrupt_flags()` is still there for the next
`read_fifo()`, which resets the buffer. `init()` clears the whole cache.

## Setting up the GPIO interrupt on the Pico

The driver does not touch the Pico GPIO. The application does:

```cpp
constexpr uint GPIO_INT = 14;
volatile bool pending_int = false;

void irq_callback(uint gpio, uint32_t events) {
    if (gpio == GPIO_INT && (events & GPIO_IRQ_EDGE_RISE))
        pending_int = true; // only a flag: no bus access here
}

// after imu.init():
gpio_init(GPIO_INT);
gpio_set_dir(GPIO_INT, GPIO_IN);
gpio_pull_down(GPIO_INT);
gpio_set_irq_enabled_with_callback(GPIO_INT, GPIO_IRQ_EDGE_RISE, true, irq_callback);
```

Use `GPIO_IRQ_EDGE_RISE` for `ActiveHigh` and `GPIO_IRQ_EDGE_FALL` for `ActiveLow`.

### Rules for the handler

- **Never** access the bus or call the driver from the interrupt handler. I2C and SPI reads
  are blocking and take hundreds of microseconds, and the main loop may be in the middle of
  its own transfer.
- Only set a `volatile` flag (or increment a counter).

### Startup order

1. `imu.init()`;
2. configure the GPIO interrupt;
3. call `take_interrupt_flags()` once.

Step 3 matters in latched mode: while the GPIO was being configured, the chip already set a
flag and the pin is high. Without a new rising edge no interrupt would ever come. Reading
the flags releases the pin. In pulse mode it simply drops flags collected during setup.

### Main loop

```cpp
for (;;) {
    if (pending_int) {
        pending_int = false;               // clear first, see below
        mpu6500::InterruptFlags flags{};
        if (imu.take_interrupt_flags(flags) == bus::Status::OK && flags.raw_data_ready) {
            mpu6500::Sample s{};
            imu.read_all(s);
        }
    }
    // other work
}
```

Clear `pending_int` **before** reading the flags. If an interrupt arrives during the read,
the handler sets it again and the next loop iteration handles it. Clearing it after the read
would erase that new interrupt.

## Wake-on-motion

In low-power accelerometer mode the chip wakes up at `power.low_power_rate`, takes one
accelerometer sample and compares it with the previous one. If any axis changed by more
than the threshold, the wake-on-motion flag is set.

```cpp
config.power.mode = cfg::PowerMode::LowPowerAccel;
config.power.low_power_rate = cfg::LowPowerAccelRate::Hz31_25;
config.wake_on_motion.enabled = true;            // comparison logic in the chip
config.wake_on_motion.threshold_mg = 100;        // 0 to 1020 mg, 4 mg steps
config.interrupts.sources.wake_on_motion = true; // route the event to the INT pin
```

| Field | Meaning |
|---|---|
| `wake_on_motion.enabled` | turns the comparison logic on (`ACCEL_INTEL_CTRL`) |
| `wake_on_motion.threshold_mg` | change between two samples that counts as motion |
| `interrupts.sources.wake_on_motion` | puts the event on the INT pin |

The logic can run without the pin: the flag is then seen by polling `take_interrupt_flags()`.

At runtime: `set_wake_on_motion(bool)` and `set_wom_threshold(mg)`.

Notes:

- The chip compares **consecutive** samples, so it reacts to **changes** in acceleration
  (taps, jerks, starting to move), not to a constant tilt. A very slow tilt may not trigger.
- While the board moves, an event comes on almost every wake-up. Group events into
  "motion bursts" in the application (see `examples/wom`).
- Too many events from small vibrations: raise the threshold. No events from clear motion:
  lower it or raise the wake-up rate.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| No interrupts at all | wrong GPIO number, INT wire on another module pin, source not enabled, edge does not match `level` |
| Interrupts stop after a while (latched) | `take_interrupt_flags()` not called for some event, pin stuck active |
| Fewer events than the sample rate | main loop busy longer than one period, pulses merge into one flag |
| I2C errors as soon as interrupts run | INT wire disturbing SDA/SCL: use pulse mode, 400 kHz, shorter wiring or SPI |
| `fifo_overflow` set although the FIFO is unused | FIFO sources written while disabled (fixed in the driver) |

`examples/interrupts` shows data-ready interrupts; `examples/wom` shows wake-on-motion.

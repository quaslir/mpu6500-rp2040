# Wiring

How the MPU-6500 breakout board is connected to the Raspberry Pi Pico for both supported buses.

The MPU-6500 shares its pins between SPI and I²C, so most pins have two names (I²C name / SPI name). Breakout boards usually print only the I²C names.

Pin numbers in the "Pico pin" column are GPIO numbers (GPx). The "Physical pin" column is the pin number on the Pico board itself.

---

## SPI

Uses **SPI0**. Pin constants are in `examples/common/spi_config.hpp`.

| MPU-6500 pin | Function | Pico pin | Physical pin |
|---|---|---|---|
| VCC | Power (3.3 V) | 3V3(OUT) | 36 |
| GND | Ground | GND | 38 |
| SCL / SCLK | Clock (SCK) | GP2 (SPI0 SCK) | 4 |
| SDA / SDI | Data Pico → sensor (MOSI) | GP3 (SPI0 TX) | 5 |
| AD0 / SDO | Data sensor → Pico (MISO) | GP4 (SPI0 RX) | 6 |
| NCS | Chip select (active low) | GP15 (GPIO) | 20 |

**Bus settings**

- Mode 0 (CPOL = 0, CPHA = 0), 8 bits, MSB first
- Clock: 1 MHz (safe for all registers)
- Chip select is driven manually as a GPIO, not by the SPI peripheral
- Register read: address with bit 7 set (`0x80`); write: bit 7 cleared

---

## I²C

Uses **I2C0**. Pin constants are in `examples/common/i2c_config.hpp`.

| MPU-6500 pin | Function | Pico pin | Physical pin |
|---|---|---|---|
| VCC | Power (3.3 V) | 3V3(OUT) | 36 |
| GND | Ground | GND | 38 |
| SCL / SCLK | Clock (SCL) | GP5 (I2C0 SCL) | 7 |
| SDA / SDI | Data (SDA) | GP4 (I2C0 SDA) | 6 |
| AD0 / SDO | Address select | GND | 8 |
| nCS | Must be high in I²C mode | 3V3(OUT) | 36 |

**Bus settings**

- Device address: `0x68` (AD0 → GND). With AD0 → 3V3 the address is `0x69`
- Clock: see `i2c_config.hpp` (the MPU-6500 supports up to 400 kHz)
- Pull-ups on SDA and SCL: enabled in software (`gpio_pull_up`); some breakout boards also have their own resistors

---

## Other pins on the breakout board

| Pin | Connection | Reason |
|---|---|---|
| FSYNC | GND (if present on the board) | External sync input, not used |
| INT | Not connected | Reserved for the data-ready interrupt later |
| EDA / ECL | Not connected | Auxiliary I²C bus for an external magnetometer, not used |

---

## Switching between SPI and I²C

Four wires change; VCC and GND stay the same:

| Wire | SPI | I²C |
|---|---|---|
| SCL / SCLK | GP2 | GP5 |
| SDA / SDI | GP3 | GP4 |
| AD0 / SDO | GP4 | GND |
| nCS | GP15 | 3V3 |

---

## Notes

- **Power from 3V3, not VBUS (5 V).** The RP2040 pins are not 5 V tolerant, and pull-up resistors on the breakout board would put VCC on the data lines.
- **Most common SPI mistake:** swapping MOSI and MISO. SDI *receives* data, so it goes to the Pico's TX pin; SDO *sends* data, so it goes to the Pico's RX pin.
- **nCS in I²C mode:** if nCS is left floating or low, the chip may stay in SPI mode and ignore I²C.

---

## Verification

Run the `whoami` example. On both buses the expected output over USB serial is:

```
0x70
```

`0x70` is the MPU-6500 chip ID from the `WHO_AM_I` register (`0x75`). It is not the I²C address.

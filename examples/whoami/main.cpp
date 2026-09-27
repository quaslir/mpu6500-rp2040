#include "bus_pico/spi_bus.hpp"
#include "mpu6500/mpu6500.hpp"
#include <cstdint>
#include <cstdio>
#include <hardware/gpio.h>
#include <hardware/structs/io_bank0.h>
#include <iostream>
#include <hardware/i2c.h>
#include <hardware/spi.h>
#include <pico/stdio.h>
#include <pico/time.h>
#include "pico/stdlib.h"
#include "spi_config.hpp"
int main() {
    stdio_init_all();

    spi_config::init_test_spi();
    mpu6500::Config config;
    bus::pico::SPIBus bus{spi0, spi_config::CS};
    mpu6500::Mpu6500 mpu6500(bus, sleep_ms, config);
    uint8_t id = 0;
    (void) mpu6500.who_am_i(id);
    std::printf("0x%02x\n", id);
    return 0;
}

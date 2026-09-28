#include "bus_pico/i2c_bus.hpp"
#include "i2c_config.hpp"
#include "mpu6500/mpu6500.hpp"
#include <cstdint>
#include <cstdio>
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <pico/stdio.h>

int main() {
    stdio_init_all();
    i2c_config::init_test_i2c();
    mpu6500::Config config{};
    bus::pico::I2CBus bus{i2c0, i2c_config::DEVICE_ADDR, 10000};
    mpu6500::Mpu6500 mpu6500(bus, sleep_ms, config);
    uint8_t id{0};
    (void)mpu6500.who_am_i(id);
    std::printf("0x%02x\n", id);
    return 0;
}

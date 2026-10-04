#include "bus_pico/i2c_bus.hpp"
#include "i2c_config.hpp"
#include "mpu6500/mpu6500.hpp"
#include <cstdint>
#include <cstdio>
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <iostream>
#include <pico/stdio.h>
int main() {
    stdio_init_all();
    i2c_config::init_test_i2c();
    mpu6500::config::Config config{};
    bus::pico::I2CBus bus{i2c0, i2c_config::DEVICE_ADDR, 30000};
    mpu6500::Mpu6500 mpu6500(bus, sleep_ms, config);
    uint8_t id{0};
    (void)mpu6500.who_am_i(id);
    std::printf("0x%02x\n", id);

    bus::Status status = mpu6500.init();
    std::cout << (status == bus::Status::OK ? "Mpu6500 ready" : "Init failed") << std::endl;

    std::printf("%-26s | %-32s | %s\n", "accel [g]", "gyro [dps]", "temp [C]");

    for (;;) {
        mpu6500::Sample sample{};
        const bus::Status status = mpu6500.read_all(sample);

        if (status != bus::Status::OK) {
            std::printf("read_all failed: status %d\n", static_cast<int>(status));
        } else {
            std::printf("%+7.3f %+7.3f %+7.3f    | %+9.2f %+9.2f %+9.2f    | %6.2f\n",
                        sample.accel_g.x,
                        sample.accel_g.y,
                        sample.accel_g.z,
                        sample.gyro_dps.x,
                        sample.gyro_dps.y,
                        sample.gyro_dps.z,
                        sample.temperature_c);
        }

        sleep_ms(500);
    }
    return 0;
}

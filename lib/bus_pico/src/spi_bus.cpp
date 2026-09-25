#include "bus_pico/spi_bus.hpp"

#include <array>
#include <cstdint>
#include <hardware/gpio.h>
#include <hardware/spi.h>
namespace bus::pico {
SPIBus::SPIBus(spi_inst_t* spi, uint8_t cs, uint8_t read_bit)
    : spi_(spi), cs_(cs), read_bit_(read_bit) {}
void SPIBus::init() {
    gpio_put(cs_, 1);
}
Status SPIBus::read_regs(uint8_t reg, std::span<uint8_t> buffer) {
    if (buffer.empty())
        return Status::ERROR;
    uint8_t read_reg = reg | read_bit_;
    gpio_put(cs_, 0);
    spi_write_blocking(spi_, &read_reg, 1);
    spi_read_blocking(spi_, 0, buffer.data(), buffer.size());
    gpio_put(cs_, 1);
    return Status::OK;
}
Status SPIBus::write_reg(uint8_t reg, uint8_t data) {
    uint8_t reg_write = ~(reg & read_bit_);
    std::array<uint8_t, 2> payload{reg_write, data};
    gpio_put(cs_, 0);
    spi_write_blocking(spi_, payload.data(), payload.size());
    gpio_put(cs_, 1);

    return Status::OK;
}
} // namespace bus::pico

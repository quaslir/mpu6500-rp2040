#pragma once
#include "bus/bus.hpp"
#include <cstdint>
#include <span>

#include "hardware/spi.h"
namespace bus::pico {
class SPIBus : public Bus {
public:
    SPIBus(spi_inst_t* spi, uint8_t cs, uint8_t read_bit = 0x80);
    void init(); // Initializes a bus; expects the user to have initialized the CS GPIO.
    Status read_regs(uint8_t reg, std::span<uint8_t> buffer) override;
    Status write_reg(uint8_t reg, uint8_t data) override;

private:
    spi_inst_t* spi_;
    uint8_t cs_;
    uint8_t read_bit_;
};
} // namespace bus::pico

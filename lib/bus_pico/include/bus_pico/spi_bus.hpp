#pragma once
#include "bus/bus.hpp"
#include <cstdint>
#include <span>

#include "hardware/spi.h"
namespace bus::pico {
class SpiBus : public Bus {
public:
    SpiBus(spi_inst_t* spi, uint8_t cs, uint8_t read_bit = 0x80);
    void init();
    Status read_regs(uint8_t reg, std::span<uint8_t> buffer) override;
    Status write_reg(uint8_t reg, uint8_t data) override;

private:
    spi_inst_t* spi_;
    uint8_t cs_;
    uint8_t read_bit_;
};
} // namespace bus::pico

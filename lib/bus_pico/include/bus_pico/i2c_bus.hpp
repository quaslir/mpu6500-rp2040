#pragma once
#include "bus/bus.hpp"
#include "bus/status.hpp"
#include <cstdint>
#include <hardware/i2c.h>
#include <span>
namespace bus::pico {
class I2CBus : public Bus {
public:
    I2CBus(i2c_inst_t* i2c, uint8_t addr, uint32_t timeout_us);
    bus::Status read_regs(uint8_t reg, std::span<uint8_t> buffer) override;
    bus::Status write_reg(uint8_t reg, uint8_t data) override;

private:
    i2c_inst_t* i2c_;
    uint8_t addr_;
    uint32_t timeout_us_;
};
} // namespace bus::pico

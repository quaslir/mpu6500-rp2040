#pragma once
#include "bus/status.hpp"
#include <cstdint>
#include <span>

namespace bus {

class Bus {
public:
    [[nodiscard]] virtual Status read_regs(uint8_t reg, std::span<uint8_t> buffer) = 0;
    [[nodiscard]] virtual Status write_reg(uint8_t reg, uint8_t data) = 0;

    virtual ~Bus() = default;
};

} // namespace bus

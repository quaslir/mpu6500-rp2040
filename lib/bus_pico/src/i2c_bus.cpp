#include "bus_pico/i2c_bus.hpp"

#include "bus/status.hpp"
#include <array>
#include <cstdint>
#include <hardware/i2c.h>
#include <pico/error.h>

namespace {
Status to_status(int result, int expected) {
    if (result < 0) {
        switch (result) {
            case PICO_ERROR_GENERIC:
                return Status::NACK;
            case PICO_ERROR_TIMEOUT:
                return Status::TIMEOUT;
            default:
                return Status::ERROR;
        }
    }

    return result == expected ? Status::OK : Status::ERROR;
}
} // namespace

namespace bus::pico {

I2CBus::I2CBus(i2c_inst_t* i2c, uint8_t addr, uint32_t timeout_us)
    : i2c_(i2c), addr_(addr), timeout_us_(timeout_us) {}
Status I2CBus::read_regs(uint8_t reg, std::span<uint8_t> buffer) {
    if (buffer.empty())
        return Status::ERROR;
    int bytes = i2c_write_timeout_us(i2c_, addr_, &reg, 1, true, timeout_us_);
    Status write_status = to_status(bytes, 1);
    if (write_status != Status::OK)
        return write_status;

    bytes = i2c_read_timeout_us(i2c_, addr_, buffer.data(), buffer.size(), false, timeout_us_);
    Status read_status = to_status(bytes, static_cast<int>(buffer.size()));

    return read_status;
}
Status I2CBus::write_reg(uint8_t reg, uint8_t data) {
    std::array<uint8_t, 2> payload{reg, data};
    int bytes =
        i2c_write_timeout_us(i2c_, addr_, payload.data(), payload.size(), false, timeout_us_);
    return to_status(bytes, static_cast<int>(payload.size()));
}

} // namespace bus::pico

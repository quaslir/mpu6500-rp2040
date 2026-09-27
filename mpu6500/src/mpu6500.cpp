#include "mpu6500/mpu6500.hpp"

#include "bus/bus.hpp"
#include "registers.hpp"
#include <cstdint>
#include <span>
namespace mpu6500 {
Mpu6500::Mpu6500(bus::Bus& bus, WaitFunction wait, const Config& config)
    : bus_(bus), wait_(wait), use_i2c_(config.use_i2c),
      accel_range_(config.starting_accelerometer_range),
      gyro_range_(config.starting_gyroscope_range) {}

Status Mpu6500::who_am_i(uint8_t& id) {
  return bus_.read_regs(
        WHOAMI, std::span<uint8_t>(&id, 1));
}

} // namespace mpu6500

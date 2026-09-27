#include "mpu6500/mpu6500.hpp"

#include "bus/bus.hpp"
#include "bus/status.hpp"
#include "registers.hpp"
#include <cstdint>
#include <span>
namespace mpu6500 {
Mpu6500::Mpu6500(bus::Bus& bus, WaitFunction wait, const Config& config)
    : bus_(bus), wait_(wait), use_i2c_(config.use_i2c),
      accel_range_(config.starting_accelerometer_range),
      gyro_range_(config.starting_gyroscope_range) {}

Status Mpu6500::who_am_i(uint8_t& id) {
    return bus_.read_regs(WHOAMI, std::span<uint8_t>(&id, 1));
}

Status Mpu6500::init() {
    uint8_t id{};
    Status who_am_i_result = who_am_i(id);
    if (who_am_i_result != Status::OK)
        return who_am_i_result;
    if (id != EXPECTED_DEVICE_ID)
        return Status::ERROR;

    Status reset_result = bus_.write_reg(PWR_MGMT_1, PWR_MGMT_1_RESET);
    if (reset_result != Status::OK)
        return reset_result;
    wait_(INIT_WAIT_INTERVAL_MS);

    Status signal_path_reset_result = bus_.write_reg(SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_ALL);
    if (signal_path_reset_result != Status::OK)
        return signal_path_reset_result;
    wait_(INIT_WAIT_INTERVAL_MS);

    if (!use_i2c_) { // disable I2C if config was stated that SPI is used. If user uses I2C, NACK
                     // will be a result of following writing.
        Status user_ctrl_result = bus_.write_reg(USER_CTRL, USER_CTRL_I2C_IF_DIS);
        if (user_ctrl_result != Status::OK)
            return user_ctrl_result;
    }

    Status normal_mode_result = bus_.write_reg(PWR_MGMT_1, PWR_MGMT_1_NORMAL);
    if (normal_mode_result != Status::OK)
        return normal_mode_result;

    Status set_accel_range_result = set_accel_range(accel_range_);
    if (set_accel_range_result != Status::OK)
        return set_accel_range_result;

    Status set_gyro_range_result = set_gyro_range(gyro_range_);
    if (set_gyro_range_result != Status::OK) {
        return set_gyro_range_result;
    }
    return Status::OK;
}

AccelRange Mpu6500::accel_range() const {
    return accel_range_;
}

GyroRange Mpu6500::gyro_range() const {
    return gyro_range_;
}

Status Mpu6500::set_accel_range(AccelRange range) {
    Status set_accel_range_result = bus_.write_reg(ACCEL_CONFIG, static_cast<uint8_t>(range) << 3);
    if (set_accel_range_result != Status::OK)
        return set_accel_range_result;
    accel_range_ = range;
    return Status::OK;
}

Status Mpu6500::set_gyro_range(GyroRange range) {
    Status set_gyro_range_result = bus_.write_reg(GYRO_CONFIG, static_cast<uint8_t>(range) << 3);
    if (set_gyro_range_result != Status::OK)
        return set_gyro_range_result;
    gyro_range_ = range;
    return Status::OK;
}

} // namespace mpu6500

#include "mpu6500/mpu6500.hpp"
#include "registers/bits.hpp"
namespace mpu6500 {
Status Mpu6500::poll_int_status() {
    uint8_t int_status{};
    MPU_RETURN_IF_ERROR(regs_.read_int_status(int_status));
    pending_int_flags_ |= int_status;
    return Status::OK;
}

Status Mpu6500::take_interrupt_flags(InterruptFlags& flags) {
    MPU_RETURN_IF_ERROR(poll_int_status());
    flags.fifo_overflow = pending_int_flags_ & bits::int_status::FIFO_OFLOW;
    flags.raw_data_ready = pending_int_flags_ & bits::int_status::RAW_DATA_RDY;
    flags.wake_on_motion = pending_int_flags_ & bits::int_status::WOM;
    if (flags.raw_data_ready) {
        pending_int_flags_ &= static_cast<uint8_t>(~bits::int_status::RAW_DATA_RDY);
    }
    if(flags.wake_on_motion) {
        pending_int_flags_ &= static_cast<uint8_t>(~bits::int_status::WOM);
    }
    return Status::OK;
}


Status Mpu6500::set_wake_on_motion(bool enabled) {
    MPU_RETURN_IF_ERROR(regs_.write_accel_intel(enabled));
    config_.wake_on_motion.enabled = enabled;
    return Status::OK;
}
 Status Mpu6500::set_wom_threshold(uint16_t threshold) {
     MPU_RETURN_IF_ERROR(regs_.write_wom_threshold(threshold));
     config_.wake_on_motion.threshold_mg = threshold;
     return Status::OK;
 }
}

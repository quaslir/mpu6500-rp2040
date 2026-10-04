#include "bits.hpp"
#include "bus/status.hpp"
#include "decode.hpp"
#include "device.hpp"
#include "fifo_frame.hpp"
#include "mpu6500/config.hpp"
#include "mpu6500/mpu6500.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace mpu6500 {
Status Mpu6500::fifo_reset() {
    MPU_RETURN_IF_ERROR(regs_.fifo_reset());
    MPU_RETURN_IF_ERROR(poll_int_status());
    pending_int_flags_ &= static_cast<uint8_t>(~bits::int_status::FIFO_OFLOW);
    return Status::OK;
}
[[nodiscard]] Status Mpu6500::fifo_frame_count(uint16_t& count) const {
    if (!config_.fifo.enabled) {
        count = 0;
        return Status::OK;
    }
    fifo_frame::FifoFrameLayout layout = fifo_frame::make_fifo_frame_layout(config_.fifo.sources);
    if (layout.size == 0)
        return Status::ERROR;
    uint16_t size{};
    MPU_RETURN_IF_ERROR(regs_.read_fifo_count(size));
    count = size / layout.size;
    return Status::OK;
}
[[nodiscard]] Status Mpu6500::read_fifo(std::span<Sample> samples,
                                        config::FifoReadResult& fifo_result) {

    fifo_result.frames = 0;
    fifo_result.overflowed = false;

    if (!config_.fifo.enabled || samples.empty())
        return Status::OK;
    fifo_frame::FifoFrameLayout layout = fifo_frame::make_fifo_frame_layout(config_.fifo.sources);
    if (layout.size == 0)
        return Status::ERROR;
    MPU_RETURN_IF_ERROR(poll_int_status());
    if (pending_int_flags_ & bits::int_status::FIFO_OFLOW) {
        fifo_result.overflowed = true;
        pending_int_flags_ &= static_cast<uint8_t>(~bits::int_status::FIFO_OFLOW);
        if (config_.fifo.mode == config::FifoMode::Overwrite) {
            MPU_RETURN_IF_ERROR(regs_.fifo_reset());

            return Status::OK;
        }
    }

    uint16_t available_frames{};
    MPU_RETURN_IF_ERROR(fifo_frame_count(available_frames));
    const size_t frames_to_read = std::min(static_cast<size_t>(available_frames), samples.size());
    if (frames_to_read == 0)
        return Status::OK;
    std::array<uint8_t, device::FIFO_SIZE_BYTES> buffer{0};
    MPU_RETURN_IF_ERROR(
        regs_.read_fifo_bytes(std::span{buffer}.first(frames_to_read * layout.size)));

    const float accel_sens = ::detail::accel_range_to_scale(config_.measurement.accel.range);
    const float gyro_sens = ::detail::gyro_range_to_scale(config_.measurement.gyro.range);

    for (size_t i = 0; i < frames_to_read; i++) {
        auto frame = std::span{buffer}.subspan(i * layout.size, layout.size);
        Sample sample = fifo_frame::parse_fifo_frame(frame, layout, accel_sens, gyro_sens);
        if (layout.has_accel) {
            sample.accel_g -= config_.calibration.accel_offset_g;
        }
        if (layout.has_gyro) {
            sample.gyro_dps -= config_.calibration.gyro_offset_dps;
        }

        samples[i] = sample;
    }
    fifo_result.frames = frames_to_read;
    return Status::OK;
}
} // namespace mpu6500

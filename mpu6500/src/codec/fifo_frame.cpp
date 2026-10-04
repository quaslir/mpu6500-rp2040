#include "codec/fifo_frame.hpp"

#include "codec/decode.hpp"
#include "codec/layout.hpp"
#include "mpu6500/sample.hpp"
#include <cstddef>
#include <cstdint>

namespace mpu6500::fifo_frame {
FifoFrameLayout make_fifo_frame_layout(const mpu6500::config::FifoSources& sources) {
    size_t offset{};
    FifoFrameLayout layout{};
    if (sources.accel) {
        layout.has_accel = true;
        layout.accel_offset = offset;
        offset += layout::VEC3_SIZE;
    }

    if (sources.temperature) {
        layout.has_temp = true;
        layout.temp_offset = offset;
        offset += layout::TEMP_SIZE;
    }

    if (sources.gyro) {
        layout.has_gyro = true;
        layout.gyro_offset = offset;
        offset += layout::VEC3_SIZE;
    }
    layout.size = offset;
    return layout;
}
Sample parse_fifo_frame(std::span<const uint8_t> frame,
                        const FifoFrameLayout& layout,
                        float acc_sens,
                        float gyro_sens) {
    Sample sample{};

    if (layout.has_accel) {
        sample.accel_g = detail::decode_vec3(
            frame.subspan(layout.accel_offset).first<layout::VEC3_SIZE>(), acc_sens);
    }
    if (layout.has_temp) {
        sample.temperature_c = detail::decode_temperature(
            frame.subspan(layout.temp_offset).first<layout::TEMP_SIZE>());
    }
    if (layout.has_gyro) {
        sample.gyro_dps = detail::decode_vec3(
            frame.subspan(layout.gyro_offset).first<layout::VEC3_SIZE>(), gyro_sens);
    }

    return sample;
}
} // namespace mpu6500::fifo_frame

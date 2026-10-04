#pragma once

#include "mpu6500/config.hpp"
#include "mpu6500/sample.hpp"
#include <cstddef>
#include <cstdint>
#include <span>

namespace mpu6500::fifo_frame {

struct FifoFrameLayout {
    size_t size{};
    bool has_accel{}, has_temp{}, has_gyro{};
    size_t accel_offset{}, temp_offset{}, gyro_offset{};
};
FifoFrameLayout make_fifo_frame_layout(const mpu6500::config::FifoSources& sources);
Sample parse_fifo_frame(const std::span<const uint8_t> frame,
                        const FifoFrameLayout& layout,
                        float acc_sens,
                        float gyro_sens);
} // namespace mpu6500::fifo_frame

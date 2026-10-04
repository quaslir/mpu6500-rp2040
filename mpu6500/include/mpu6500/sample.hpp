#pragma once
#include "math/vec3.hpp"
struct Sample {
    Vec3 accel_g{};
    Vec3 gyro_dps{};
    float temperature_c{};
};
struct InterruptFlags {
    bool raw_data_ready{};
    bool fifo_overflow{};
    bool wake_on_motion{};
};

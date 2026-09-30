#pragma once
#include "vec3.hpp"
struct Sample {
    Vec3 accel_g{};
    Vec3 gyro_dps{};
    float temperature_c{};
};

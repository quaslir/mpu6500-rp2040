#include "bus/status.hpp"
#include "mpu6500/calibration.hpp"
#include "mpu6500/config.hpp"
#include "mpu6500/mpu6500.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace mpu6500::calibration {

namespace {
bus::Status read_one(const Mpu6500& imu, Sensor sensor, math::Vec3& sample) {
    switch (sensor) {
        case Sensor::Accel:
            return imu.read_accel_uncorrected(sample);
        case Sensor::Gyro:
            return imu.read_gyro_uncorrected(sample);
        default:
            return bus::Status::ERROR;
    }
}
bus::Status measure_mean_impl(const Mpu6500& imu,
                         Sensor sensor,
                         math::Vec3& mean,
                         const MeasureOptions& options,
                         uint32_t period_ms) {

    if (options.samples == 0)
        return bus::Status::ERROR;
    WaitFunction wait = imu.wait();
    auto warm_up = [&imu, sensor, wait, period_ms](uint16_t samples) -> bus::Status {
        math::Vec3 sample{};
        for (uint16_t i = 0; i < samples; i++) {
            MPU_RETURN_IF_ERROR(read_one(imu, sensor, sample));
            wait(period_ms);
        }

        return bus::Status::OK;
    };

    MPU_RETURN_IF_ERROR(warm_up(options.warmup_samples));
    math::Vec3 sample{}, sum{};
    MPU_RETURN_IF_ERROR(read_one(imu, sensor, sample));
    math::Vec3 min_value = sample;
    math::Vec3 max_value = sample;
    wait(period_ms);
    for (uint16_t i = 0; i < options.samples; i++) {
        MPU_RETURN_IF_ERROR(read_one(imu, sensor, sample));
        sum += sample;
        min_value = component_min(min_value, sample);
        max_value = component_max(max_value, sample);
        wait(period_ms);
    }

    const math::Vec3 diff = max_value - min_value;
    if (diff.x > options.max_spread || diff.y > options.max_spread || diff.z > options.max_spread)
        return bus::Status::ERROR;

    mean.x = sum.x / static_cast<float>(options.samples);
    mean.y = sum.y / static_cast<float>(options.samples);
    mean.z = sum.z / static_cast<float>(options.samples);

    return bus::Status::OK;
}

uint32_t hz_to_period_ms(float hz) {
    auto rounded_period = std::ceil(1000.0f / hz);
    auto period_ms = std::max(rounded_period, 1.0f);
    return static_cast<uint32_t>(period_ms);
}

} // namespace

bus::Status measure_mean(const Mpu6500& imu, Sensor sensor, math::Vec3& mean, const MeasureOptions& options) {

    switch (sensor) {
        case Sensor::Gyro:
            if (imu.config().power.mode != config::PowerMode::Normal)
                return bus::Status::ERROR;
            return measure_mean_impl(
                imu, sensor, mean, options, hz_to_period_ms(imu.gyro_sample_rate_hz()));
        case Sensor::Accel:
            return measure_mean_impl(
                imu, sensor, mean, options, hz_to_period_ms(imu.accel_sample_rate_hz()));
        default:
            return bus::Status::ERROR;
    }
}

bus::Status measure_gyro_offset(const Mpu6500& imu, math::Vec3& offset, const MeasureOptions& options) {
    return measure_mean(imu, Sensor::Gyro, offset, options);
}
bus::Status calibrate_gyro(Mpu6500& imu, const MeasureOptions& options) {
    math::Vec3 offset{};
    MPU_RETURN_IF_ERROR(measure_gyro_offset(imu, offset, options));

    imu.set_gyro_offset(offset);
    return bus::Status::OK;
}

bus::Status measure_accel_offset(const Mpu6500& imu,
                            math::Vec3& offset,
                            const math::Vec3& expected_gravity_g,
                            const MeasureOptions& options) {
    math::Vec3 mean{};
    MPU_RETURN_IF_ERROR(measure_mean(imu, Sensor::Accel, mean, options));

    const float magnitude = std::sqrt(mean.x * mean.x + mean.y * mean.y + mean.z * mean.z);
    if (magnitude < MIN_GRAVITY_G || magnitude > MAX_GRAVITY_G) {
        return bus::Status::ERROR;
    }

    offset = mean - expected_gravity_g;

    return bus::Status::OK;
}
bus::Status
calibrate_accel(Mpu6500& imu, const math::Vec3& expected_gravity_g, const MeasureOptions& options) {
    math::Vec3 offset{};
    MPU_RETURN_IF_ERROR(measure_accel_offset(imu, offset, expected_gravity_g, options));

    imu.set_accel_offset(offset);
    return bus::Status::OK;
}

} // namespace mpu6500::calibration

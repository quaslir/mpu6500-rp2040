#include "mpu6500/mpu6500.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
namespace mpu6500 {
void Mpu6500::set_accel_offset(const Vec3& offset) {
    accel_offset_ = offset;
}
void Mpu6500::set_gyro_offset(const Vec3& offset) {
    gyro_offset_ = offset;
}

const Vec3& Mpu6500::accel_offset() const {
    return accel_offset_;
}
const Vec3& Mpu6500::gyro_offset() const {
    return gyro_offset_;
}

void Mpu6500::clear_offsets() {
    accel_offset_ = Vec3{};
    gyro_offset_ = Vec3{};
}

Status Mpu6500::measure_mean(calibration::Sensor sensor,
                             Vec3& mean,
                             const calibration::MeasureOptions& options) const {
    auto hz_to_period_ms = [](float hz) -> uint32_t {
        auto rounded_period = std::ceil(1000.0f / hz);
        auto period_ms = std::max(rounded_period, 1.0f);
        return static_cast<uint32_t>(period_ms);
    };

    switch (sensor) {
        case calibration::Sensor::Gyro:
            return measure_mean_impl(
                &Mpu6500::read_gyro_raw, mean, options, hz_to_period_ms(gyro_sample_rate_hz()));
        case calibration::Sensor::Accel:
            return measure_mean_impl(
                &Mpu6500::read_accel_raw, mean, options, hz_to_period_ms(accel_sample_rate_hz()));
        default:
            return Status::ERROR;
    }
}

Status Mpu6500::measure_mean_impl(ReadVec3Fn func,
                                  Vec3& mean,
                                  const calibration::MeasureOptions& options,
                                  uint32_t period_ms) const {
    if (options.samples == 0)
        return Status::ERROR;
    auto warm_up = [func, this, period_ms](uint16_t samples) -> Status {
        Vec3 sample{};
        for (uint16_t i = 0; i < samples; i++) {
            const Status read_result = (this->*func)(sample);
            if (read_result != Status::OK)
                return read_result;
            wait_(period_ms);
        }

        return Status::OK;
    };

    const Status warm_up_status = warm_up(options.warmup_samples);
    if (warm_up_status != Status::OK)
        return warm_up_status;
    Vec3 sample{}, sum{};
    const Status read_result_first = (this->*func)(sample);
    if (read_result_first != Status::OK)
        return read_result_first;
    Vec3 min_value = sample;
    Vec3 max_value = sample;
    wait_(period_ms);
    for (uint16_t i = 0; i < options.samples; i++) {
        const Status read_result = (this->*func)(sample);
        if (read_result != Status::OK)
            return read_result;
        sum += sample;
        min_value = component_min(min_value, sample);
        max_value = component_max(max_value, sample);
        wait_(period_ms);
    }

    const Vec3 diff = max_value - min_value;
    if (diff.x > options.max_spread || diff.y > options.max_spread || diff.z > options.max_spread)
        return Status::ERROR;

    mean.x = sum.x / static_cast<float>(options.samples);
    mean.y = sum.y / static_cast<float>(options.samples);
    mean.z = sum.z / static_cast<float>(options.samples);

    return Status::OK;
}

Status Mpu6500::measure_gyro_offset(Vec3& offset,
                                    const calibration::MeasureOptions& options) const {
    return measure_mean(calibration::Sensor::Gyro, offset, options);
}
Status Mpu6500::calibrate_gyro(const calibration::MeasureOptions& options) {
    Vec3 offset{};
    const Status measure_gyro_result = measure_gyro_offset(offset, options);
    if (measure_gyro_result != Status::OK)
        return measure_gyro_result;

    set_gyro_offset(offset);
    return Status::OK;
}

Status Mpu6500::measure_accel_offset(Vec3& offset,
                                     const Vec3& expected_gravity_g,
                                     const calibration::MeasureOptions& options) const {
    Vec3 mean{};
    const Status measure_mean_result = measure_mean(calibration::Sensor::Accel, mean, options);
    if (measure_mean_result != Status::OK) {
        return measure_mean_result;
    }

    const float magnitude = std::sqrt(mean.x * mean.x + mean.y * mean.y + mean.z * mean.z);
    if (magnitude < calibration::MIN_GRAVITY_G || magnitude > calibration::MAX_GRAVITY_G) {
        return Status::ERROR;
    }

    offset = mean - expected_gravity_g;

    return Status::OK;
}
Status Mpu6500::calibrate_accel(const Vec3& expected_gravity_g,
                                const calibration::MeasureOptions& options) {
    Vec3 offset{};
    const Status measure_accel_result = measure_accel_offset(offset, expected_gravity_g, options);
    if (measure_accel_result != Status::OK)
        return measure_accel_result;

    set_accel_offset(offset);
    return Status::OK;
}

} // namespace mpu6500

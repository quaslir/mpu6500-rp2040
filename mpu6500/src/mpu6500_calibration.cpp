#include "bus/status.hpp"
#include "mpu6500/mpu6500.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
namespace mpu6500 {
void Mpu6500::set_accel_offset(const Vec3& offset) {
    config_.calibration.accel_offset_g = offset;
}
void Mpu6500::set_gyro_offset(const Vec3& offset) {
    config_.calibration.gyro_offset_dps = offset;
}

void Mpu6500::clear_offsets() {
    config_.calibration.accel_offset_g = Vec3{};
    config_.calibration.gyro_offset_dps = Vec3{};
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
            if (low_mode_.active)
                return Status::ERROR;
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
            MPU_RETURN_IF_ERROR((this->*func)(sample));
            wait_(period_ms);
        }

        return Status::OK;
    };

    MPU_RETURN_IF_ERROR(warm_up(options.warmup_samples));
    Vec3 sample{}, sum{};
    MPU_RETURN_IF_ERROR((this->*func)(sample));
    Vec3 min_value = sample;
    Vec3 max_value = sample;
    wait_(period_ms);
    for (uint16_t i = 0; i < options.samples; i++) {
        MPU_RETURN_IF_ERROR((this->*func)(sample));
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
    MPU_RETURN_IF_ERROR(measure_gyro_offset(offset, options));

    set_gyro_offset(offset);
    return Status::OK;
}

Status Mpu6500::measure_accel_offset(Vec3& offset,
                                     const Vec3& expected_gravity_g,
                                     const calibration::MeasureOptions& options) const {
    Vec3 mean{};
    MPU_RETURN_IF_ERROR(measure_mean(calibration::Sensor::Accel, mean, options));

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
    MPU_RETURN_IF_ERROR(measure_accel_offset(offset, expected_gravity_g, options));

    set_accel_offset(offset);
    return Status::OK;
}

Status Mpu6500::set_gyro_hw_offset(const RawVec3& offset) {
    MPU_RETURN_IF_ERROR(regs_.write_gyro_hw_offset(offset));

    config_.calibration.gyro_hw_offset = offset;

    return Status::OK;
}

} // namespace mpu6500

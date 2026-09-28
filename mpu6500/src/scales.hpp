#pragma once

namespace mpu6500::scale {

inline constexpr float ACCEL_G2{16384.0f};
inline constexpr float ACCEL_G4{8192.0f};
inline constexpr float ACCEL_G8{4096.0f};
inline constexpr float ACCEL_G16{2048.0f};

inline constexpr float GYRO_DPS250{131.0f};
inline constexpr float GYRO_DPS500{65.5f};
inline constexpr float GYRO_DPS1000{32.8f};
inline constexpr float GYRO_DPS2000{16.4f};

inline constexpr float TEMP_SENSITIVITY{333.87f};
inline constexpr float TEMP_REFERENCE_C{21.0f};

} // namespace mpu6500::scale

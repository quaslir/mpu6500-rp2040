#pragma once

#include <cstdint>

namespace mpu6500::bits {

namespace pwr_mgmt_1 {
inline constexpr uint8_t DEVICE_RESET{0x80}; // bit 7, auto-clears
inline constexpr uint8_t SLEEP{0x40};        // bit 6
inline constexpr uint8_t CYCLE{0x20};        // bit 5, for low-power mode later
inline constexpr uint8_t GYRO_STANDBY{0x10}; // bit 4
inline constexpr uint8_t TEMP_DIS{0x08};     // bit 3, 1 = temperature sensor off

inline constexpr uint8_t CLKSEL_MASK{0x07};     // bits 2:0
inline constexpr uint8_t CLKSEL_INTERNAL{0x00}; // internal 20 MHz oscillator
inline constexpr uint8_t CLKSEL_AUTO{0x01};     // PLL if ready, else internal
inline constexpr uint8_t CLKSEL_STOP{0x07};     // clock stopped, chip does nothing

inline constexpr uint8_t RESET{DEVICE_RESET};
inline constexpr uint8_t NORMAL{CLKSEL_AUTO};
} // namespace pwr_mgmt_1

namespace pwr_mgmt_2 {
// 1 = axis disabled, 0 = axis enabled
inline constexpr uint8_t DIS_XA{0x20}; // bit 5
inline constexpr uint8_t DIS_YA{0x10}; // bit 4
inline constexpr uint8_t DIS_ZA{0x08}; // bit 3
inline constexpr uint8_t DIS_XG{0x04}; // bit 2
inline constexpr uint8_t DIS_YG{0x02}; // bit 1
inline constexpr uint8_t DIS_ZG{0x01}; // bit 0

inline constexpr uint8_t DIS_ACCEL_MASK{DIS_XA | DIS_YA | DIS_ZA};     // 0x38
inline constexpr uint8_t DIS_GYRO_MASK{DIS_XG | DIS_YG | DIS_ZG};      // 0x07
inline constexpr uint8_t DIS_ALL_MASK{DIS_ACCEL_MASK | DIS_GYRO_MASK}; // 0x3F, bits 7:6 untouched
} // namespace pwr_mgmt_2

namespace user_ctrl {
inline constexpr uint8_t I2C_IF_DIS{0x10};
inline constexpr uint8_t SIG_COND_RST{
    0x01}; // bit 0, resets all signal paths and clears data registers
inline constexpr uint8_t FIFO_EN = 0x40;
inline constexpr uint8_t FIFO_RST = 0x04;
} // namespace user_ctrl

namespace signal_path_reset {
inline constexpr uint8_t GYRO{0x04};  // bit 2
inline constexpr uint8_t ACCEL{0x02}; // bit 1
inline constexpr uint8_t TEMP{0x01};  // bit 0
inline constexpr uint8_t ALL{GYRO | ACCEL | TEMP};
} // namespace signal_path_reset

namespace fs_sel {
inline constexpr uint8_t SHIFT{3};
inline constexpr uint8_t MASK{0x18};
} // namespace fs_sel

namespace config {
inline constexpr uint8_t DLPF_CFG_MASK{0x07};     // bits 2:0, gyro/temp filter
inline constexpr uint8_t EXT_SYNC_SET_MASK{0x38}; // bits 5:3, FSYNC (unused, keep 0)
inline constexpr uint8_t FIFO_MODE{0x40};         // bit 6 (unused, keep 0)

} // namespace config

namespace accel_config2 {
inline constexpr uint8_t A_DLPF_CFG_MASK{0x07}; // bits 2:0, accel filter
inline constexpr uint8_t ACCEL_FCHOICE_B{0x08}; // bit 3, must be 0 or the filter is bypassed
} // namespace accel_config2

namespace gyro_config {
inline constexpr uint8_t FCHOICE_B_MASK{0x03};
inline constexpr uint8_t FCHOICE_B_BYPASS_8800HZ{0x01};
inline constexpr uint8_t FCHOICE_B_BYPASS_3600HZ{0x02};
inline constexpr uint8_t FCHOICE_B_USE_DLPF{0x00};
} // namespace gyro_config

namespace lp_accel_odr {
inline constexpr uint8_t LPOSC_CLKSEL_MASK{0x0F}; // bits 3:0, wake-up rate code 0..11
} // namespace lp_accel_odr

namespace fifo_en {
inline constexpr uint8_t TEMP{0x80};
inline constexpr uint8_t GYRO_X{0x40};
inline constexpr uint8_t GYRO_Y{0x20};
inline constexpr uint8_t GYRO_Z{0x10};
inline constexpr uint8_t ACCEL{0x08};
inline constexpr uint8_t GYRO_ALL{GYRO_X | GYRO_Y | GYRO_Z};
inline constexpr uint8_t MASK{TEMP | GYRO_ALL | ACCEL}; // bits 2..0 are SLV0..SLV2, untouched
} // namespace fifo_en

namespace int_status {
inline constexpr uint8_t FIFO_OFLOW{0x10};
inline constexpr uint8_t WOM{0x40};          // bit 6, wake-on-motion
inline constexpr uint8_t FSYNC{0x08};        // bit 3
inline constexpr uint8_t RAW_DATA_RDY{0x01}; // bit 0, new sample in data registers
} // namespace int_status

namespace fifo_count {
inline constexpr uint8_t HIGH_MASK{0x1F}; // FIFO_CNT[12:8]

} // namespace fifo_count

namespace int_pin_cfg {
inline constexpr uint8_t ACTL{0x80};         // 1 = INT pin active low
inline constexpr uint8_t OPEN{0x40};         // 1 = open drain, 0 = push-pull
inline constexpr uint8_t LATCH_INT_EN{0x20}; // 1 = held until INT_STATUS is read, 0 = 50 us pulse
inline constexpr uint8_t INT_ANYRD_2CLEAR{0x10}; // 1 = any read clears flags (driver keeps it 0)
inline constexpr uint8_t MASK{ACTL | OPEN | LATCH_INT_EN |
                              INT_ANYRD_2CLEAR}; // FSYNC/bypass untouched
} // namespace int_pin_cfg

namespace int_enable {
inline constexpr uint8_t WOM{0x40};
inline constexpr uint8_t FIFO_OFLOW{0x10};
inline constexpr uint8_t FSYNC{0x08};
inline constexpr uint8_t RAW_RDY{0x01};
inline constexpr uint8_t MASK{WOM | FIFO_OFLOW | FSYNC | RAW_RDY};
} // namespace int_enable

namespace accel_intel_ctrl {
inline constexpr uint8_t ACCEL_INTEL_EN{0x80};   // bit 7, enables the wake-on-motion logic
inline constexpr uint8_t ACCEL_INTEL_MODE{0x40}; // bit 6, 1 = compare with the previous sample
inline constexpr uint8_t MASK{ACCEL_INTEL_EN | ACCEL_INTEL_MODE}; // bits 5..0 reserved
} // namespace accel_intel_ctrl
} // namespace mpu6500::bits

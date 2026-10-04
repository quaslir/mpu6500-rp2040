#pragma once
#include <cstdint>

namespace bus {

#define MPU_RETURN_IF_ERROR(expr)                                                                  \
    do {                                                                                           \
        const ::bus::Status mpu_macro_status = (expr);                                             \
        if (mpu_macro_status != ::bus::Status::OK)                                                 \
            return mpu_macro_status;                                                               \
    } while (0)

enum class Status : uint8_t { OK = 0, TIMEOUT = 1, NACK = 2, ERROR = 3 };

} // namespace bus

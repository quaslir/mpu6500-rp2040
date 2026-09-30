#pragma once

#include "bus/status.hpp"
#include <cstdio>
#include <pico/stdlib.h>

namespace example {

inline const char* status_text(Status status) {
    switch (status) {
        case Status::OK:
            return "OK";
        case Status::TIMEOUT:
            return "TIMEOUT";
        case Status::NACK:
            return "NACK";
        case Status::ERROR:
            return "ERROR";
    }
    return "UNKNOWN";
}

inline void print_status(const char* what, Status status) {
    std::printf("  %-28s %s\n", what, status_text(status));
}

[[noreturn]] inline void halt(const char* reason, Status status) {
    for (;;) {
        std::printf("FATAL: %s (%s). Check wiring and restart.\n", reason, status_text(status));
        sleep_ms(2000);
    }
}

} // namespace example

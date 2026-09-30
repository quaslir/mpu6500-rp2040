#pragma once

struct Vec3 {
    float x{}, y{}, z{};

    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;

        return *this;
    }
};

inline Vec3 operator-(const Vec3& vec1, const Vec3& vec2) {
    return Vec3{.x = vec1.x - vec2.x, .y = vec1.y - vec2.y, .z = vec1.z - vec2.z};
}

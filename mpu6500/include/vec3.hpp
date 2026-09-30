#pragma once
#include <algorithm>
struct Vec3 {
    float x{}, y{}, z{};

    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;

        return *this;
    }
    Vec3& operator += (const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;

        return *this;
    }

    Vec3& operator /=(const Vec3& other) {
        if(other.x == 0 || other.y == 0 || other.z == 0) return *this;
        x /= other.x;
        y /= other.y;
        z /= other.z;

        return *this;
    }

    Vec3& operator *=(const Vec3& other) {
        x *= other.x;
        y *= other.y;
        z *= other.z;

        return *this;
    }
};

inline Vec3 operator-(const Vec3& vec1, const Vec3& vec2) {
Vec3 vec = vec1;
vec -= vec2;
return vec;
}
inline Vec3 operator+(const Vec3& vec1, const Vec3& vec2) {
    Vec3 vec = vec1;
    vec += vec2;
    return vec;
}

inline Vec3 operator / (const Vec3& vec1, const Vec3& vec2) {
    Vec3 vec = vec1;
    vec /= vec2;
    return vec;
}

inline Vec3 operator *(const Vec3& vec1, const Vec3& vec2) {
    Vec3 vec = vec1;
    vec *= vec2;
    return vec;
}


inline Vec3 component_min(const Vec3& vec1, const Vec3& vec2) {
    Vec3 vec{};
    vec.x = std::min(vec1.x, vec2.x);
    vec.y = std::min(vec1.y, vec2.y);
    vec.z = std::min(vec1.z, vec2.z);

    return vec;
}


inline Vec3 component_max(const Vec3& vec1, const Vec3& vec2) {
    Vec3 vec{};
    vec.x = std::max(vec1.x, vec2.x);
    vec.y = std::max(vec1.y, vec2.y);
    vec.z = std::max(vec1.z, vec2.z);

    return vec;
}

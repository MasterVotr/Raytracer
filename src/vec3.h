#pragma once

#include "util.h"

#include <ostream>

namespace raytracer {
class vec3 {
   public:
    float x, y, z;

   public:
    vec3() : x(0), y(0), z(0) {}
    vec3(float v) : x(v), y(v), z(v) {}
    vec3(float x0, float y0, float z0 = 0) : x(x0), y(y0), z(z0) {}
    vec3(const vec3& other) = default;
    vec3& operator=(const vec3& other) {
        if (this == &other)
            return *this;
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    vec3 operator*(float a) const { return vec3(x * a, y * a, z * a); }
    vec3 operator*(const vec3 r) const { return vec3(x * r.x, y * r.y, z * r.z); }
    vec3 operator/(const float r) const { return fabs(r) > epsilon ? vec3(x / r, y / r, z / r) : vec3(0, 0, 0); }
    vec3 operator+(const vec3& v) const { return vec3(x + v.x, y + v.y, z + v.z); }
    vec3 operator-(const vec3& v) const { return vec3(x - v.x, y - v.y, z - v.z); }
    vec3 operator-() const { return vec3(-x, -y, -z); }
    void operator+=(const vec3& v) { x += v.x, y += v.y, z += v.z; }
    void operator*=(float a) { x *= a, y *= a, z *= a; }
    void operator*=(const vec3& v) { x *= v.x, y *= v.y, z *= v.z; }
    bool operator==(const vec3& v) const { return (fabs(x - v.x) < epsilon && fabs(y - v.y) < epsilon && fabs(z - v.z) < epsilon); }
    float operator[](int i) const {
        if (i == 0)
            return x;
        else if (i == 1)
            return y;
        else if (i == 2)
            return z;
        else
            throw std::out_of_range("Index out of range");
    }
    float length() const { return sqrt(x * x + y * y + z * z); }
    float average() { return (x + y + z) / 3; }
    vec3 normalize() const { return (*this) / length(); }
};

using point3 = vec3;  // 3D point

// Vec3 Utility Functions

inline float dot(const vec3& v1, const vec3& v2) {
    return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
}

inline vec3 cross(const vec3& v1, const vec3& v2) {
    return vec3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
}

inline vec3 abs(const vec3& v) {
    return vec3(fabs(v.x), fabs(v.y), fabs(v.z));
}

inline std::ostream& operator<<(std::ostream& os, const vec3& vec) {
    os << vec.x << " " << vec.y << " " << vec.z;
    return os;
}

}  // namespace raytracer
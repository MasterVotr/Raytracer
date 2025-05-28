#pragma once

#include "src/util.h"

#include <ostream>

namespace raytracer {
class Vec3 {
   public:
    __host__ __device__ Vec3() : x(0), y(0), z(0) {}
    __host__ __device__ Vec3(float v) : x(v), y(v), z(v) {}
    __host__ __device__ Vec3(float x0, float y0, float z0 = 0) : x(x0), y(y0), z(z0) {}
    Vec3(const Vec3& other) = default;
    __host__ __device__ Vec3& operator=(const Vec3& other) {
        if (this == &other)
            return *this;
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    __host__ __device__ Vec3 operator*(float a) const { return Vec3(x * a, y * a, z * a); }
    __host__ __device__ Vec3 operator*(const Vec3 r) const { return Vec3(x * r.x, y * r.y, z * r.z); }
    __host__ __device__ Vec3 operator/(const float a) const { return fabs(a) > epsilon ? Vec3(x / a, y / a, z / a) : Vec3(0); }
    __host__ __device__ Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    __host__ __device__ Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    __host__ __device__ Vec3 operator-() const { return Vec3(-x, -y, -z); }
    __host__ __device__ void operator+=(const Vec3& v) { x += v.x, y += v.y, z += v.z; }
    __host__ __device__ void operator*=(float a) { x *= a, y *= a, z *= a; }
    __host__ __device__ void operator*=(const Vec3& v) { x *= v.x, y *= v.y, z *= v.z; }
    __host__ __device__ bool operator==(const Vec3& v) const {
        return (fabs(x - v.x) < epsilon && fabs(y - v.y) < epsilon && fabs(z - v.z) < epsilon);
    }
    __host__ __device__ float operator[](int i) const {
        if (i == 0)
            return x;
        else if (i == 1)
            return y;
        else if (i == 2)
            return z;
        else {
#ifdef __CUDA_ARCH__
            printf("Index out of range\n");
            return 0;
#else
            throw std::out_of_range("Index out of range");
#endif
        }
    }
    __host__ __device__ float length() const { return sqrt(x * x + y * y + z * z); }
    __host__ __device__ float average() { return (x + y + z) / 3; }
    __host__ __device__ Vec3 normalize() const { return (*this) / length(); }

    __device__ void print() const { printf("%f %f %f", x, y, z); }

    float x, y, z;
};

using Point3 = Vec3;  // 3D point

// Vec3 Utility Functions

__host__ __device__ inline float dot(const Vec3& v1, const Vec3& v2) {
    return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
}

__host__ __device__ inline Vec3 cross(const Vec3& v1, const Vec3& v2) {
    return Vec3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
}

__host__ __device__ inline Vec3 abs(const Vec3& v) {
    return Vec3(fabs(v.x), fabs(v.y), fabs(v.z));
}

inline std::ostream& operator<<(std::ostream& os, const Vec3& vec) {
    os << vec.x << " " << vec.y << " " << vec.z;
    return os;
}

}  // namespace raytracer
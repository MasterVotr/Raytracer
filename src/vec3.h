#pragma once

#include <ostream>

namespace raytracer {
class vec3 {
   public:
    double x, y, z;

   public:
    vec3() : x(0), y(0), z(0) {}
    vec3(double v) : x(v), y(v), z(v) {}
    vec3(double x0, double y0, double z0 = 0) : x(x0), y(y0), z(z0) {}

    vec3 operator*(double a) const { return vec3(x * a, y * a, z * a); }
    vec3 operator*(const vec3 r) const { return vec3(x * r.x, y * r.y, z * r.z); }
    vec3 operator/(const double r) const { return fabs(r) > epsilon ? vec3(x / r, y / r, z / r) : vec3(0, 0, 0); }
    vec3 operator+(const vec3& v) const { return vec3(x + v.x, y + v.y, z + v.z); }
    vec3 operator-(const vec3& v) const { return vec3(x - v.x, y - v.y, z - v.z); }
    vec3 operator-() const { return vec3(-x, -y, -z); }
    void operator+=(const vec3& v) { x += v.x, y += v.y, z += v.z; }
    void operator*=(double a) { x *= a, y *= a, z *= a; }
    void operator*=(const vec3& v) { x *= v.x, y *= v.y, z *= v.z; }
    double length() const { return sqrt(x * x + y * y + z * z); }
    double average() { return (x + y + z) / 3; }
    vec3 normalize() const { return (*this) / length(); }

   private:
    const double epsilon = 1e-9;  // Small value
};

using point3 = vec3;  // 3D point

// Vec3 Utility Functions

inline double dot(const vec3& v1, const vec3& v2) {
    return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
}

inline vec3 cross(const vec3& v1, const vec3& v2) {
    return vec3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
}

inline std::ostream& operator<<(std::ostream& os, vec3 vec) {
    os << vec.x << " " << vec.y << " " << vec.z;
    return os;
}

}  // namespace raytracer
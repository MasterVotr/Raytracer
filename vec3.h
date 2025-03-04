#pragma once

#include <ostream>

class vec3 {
   public:
    vec3();
    vec3(double v);
    vec3(double x0, double y0, double z0 = 0);
    vec3 operator*(double a) const;
    vec3 operator*(const vec3 r) const;
    vec3 operator/(const double r) const;
    vec3 operator+(const vec3& v) const;
    vec3 operator-(const vec3& v) const;
    vec3 operator-() const;
    void operator+=(const vec3& v);
    void operator*=(double a);
    void operator*=(const vec3& v);
    double length() const;
    double average();
    vec3 normalize() const;
    double dot(const vec3& other);
    vec3 cross(const vec3& other);

    inline friend std::ostream& operator<<(std::ostream& os, vec3 vec) {
        os << vec.x << " " << vec.y << " " << vec.z;
        return os;
    }

   private:
    double x, y, z;
};
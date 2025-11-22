#pragma once

#include <algorithm>

#include "src/vec3.h"

namespace raytracer {

struct AABB {
    Point3 min;
    Point3 max;
    Vec3 size;

    AABB() = default;
    AABB(const Point3& min, const Point3& max) : min(min), max(max), size(abs(max - min)) {}

    inline float volume() const { return size.x * size.y * size.z; }
    inline Point3 center() const { return min + (size / 2.0); }
    inline float surface_area() const { return 2.0f * (size.x * size.y + size.y * size.z + size.x * size.z); }
    inline void expand(const AABB& other) {
        min.x = std::min(min.x, other.min.x);
        min.y = std::min(min.y, other.min.y);
        min.z = std::min(min.z, other.min.z);
        max.x = std::max(max.x, other.max.x);
        max.y = std::max(max.y, other.max.y);
        max.z = std::max(max.z, other.max.z);
        size = max - min;
    }
    inline void expand(const Point3& point) {
        min.x = std::min(min.x, point.x);
        min.y = std::min(min.y, point.y);
        min.z = std::min(min.z, point.z);
        max.x = std::max(max.x, point.x);
        max.y = std::max(max.y, point.y);
        max.z = std::max(max.z, point.z);
        size = max - min;
    }
};

inline std::ostream& operator<<(std::ostream& os, AABB aabb) {
    os << "[" << aabb.min << "]-[" << aabb.max << "]";
    return os;
}

}  // namespace raytracer

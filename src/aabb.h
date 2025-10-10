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

    float volume() const { return (max.x - min.x) * (max.y - min.y) * (max.z - min.z); }
    void expand(const AABB& other) {
        min.x = std::min(min.x, other.min.x);
        min.y = std::min(min.y, other.min.y);
        min.z = std::min(min.z, other.min.z);
        max.x = std::max(max.x, other.max.x);
        max.y = std::max(max.y, other.max.y);
        max.z = std::max(max.z, other.max.z);
        size = max - min;
    }
};

}  // namespace raytracer

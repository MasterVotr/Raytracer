#pragma once

#include "src/vec3.h"

namespace raytracer {

struct AABB {
    Point3 min;
    Point3 max;
    Vec3 size;

    AABB() = default;
    AABB(const Point3& min, const Point3& max) : min(min), max(max), size(abs(max - min)) {}

    float volume() const { return (max.x - min.x) * (max.y - min.y) * (max.z - min.z); }
};

}  // namespace raytracer

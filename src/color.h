#pragma once

#include <iostream>

#include "src/util.h"
#include "src/vec3.h"

namespace raytracer {

using Color = Vec3;

inline void write_color(std::ostream& out, Color pixel_color) {
    out << static_cast<int>(255.999 * pixel_color.x) << ' ' << static_cast<int>(255.999 * pixel_color.y) << ' '
        << static_cast<int>(255.999 * pixel_color.z) << '\n';
}

__host__ __device__ inline Color clamp_color(const Color& c, float min = 0.0f, float max = 1.0f) {
    return Color(clamp(c.x, min, max), clamp(c.y, min, max), clamp(c.z, min, max));
}

}  // namespace raytracer
#pragma once

#include "util.h"
#include "vec3.h"

#include <iostream>

namespace raytracer {

using color = vec3;

void write_color(std::ostream& out, color pixel_color) {
    out << static_cast<int>(255.999 * pixel_color.x) << ' ' << static_cast<int>(255.999 * pixel_color.y) << ' '
        << static_cast<int>(255.999 * pixel_color.z) << '\n';
}

color clamp_color(const color& c, float min = 0.0f, float max = 1.0f) {
    return color(clamp(c.x, min, max), clamp(c.y, min, max), clamp(c.z, min, max));
}

}  // namespace raytracer
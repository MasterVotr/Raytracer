#pragma once

#include "vec3.h"

#include <array>

namespace raytracer {

struct Triangle {
    std::array<vec3, 3> vertices;
    size_t material_id;
};

inline std::ostream& operator<<(std::ostream& os, Triangle triangle) {
    os << triangle.vertices[0] << " " << triangle.vertices[1] << " " << triangle.vertices[2];
    return os;
}

}  // namespace raytracer
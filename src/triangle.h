#pragma once

#include "vec3.h"

#include <array>

namespace raytracer {

struct Triangle {
    vec3 vertices[3];
    size_t material_id;
    vec3 normal;

    bool operator==(const Triangle& other) const {
        return vertices[0] == other.vertices[0] && vertices[1] == other.vertices[1] && vertices[2] == other.vertices[2] ||
               vertices[0] == other.vertices[1] && vertices[1] == other.vertices[2] && vertices[2] == other.vertices[0] ||
               vertices[0] == other.vertices[2] && vertices[1] == other.vertices[0] && vertices[2] == other.vertices[1];
    }
};

inline std::ostream& operator<<(std::ostream& os, Triangle triangle) {
    os << triangle.vertices[0] << " " << triangle.vertices[1] << " " << triangle.vertices[2];
    return os;
}

vec3 calculate_triangle_normal(const Triangle& t) {
    vec3 u = t.vertices[1] - t.vertices[0];
    vec3 v = t.vertices[2] - t.vertices[1];
    return cross(u, v).normalize();
}

float calculate_triangle_area(const Triangle& t) {
    vec3 u = t.vertices[1] - t.vertices[0];
    vec3 v = t.vertices[2] - t.vertices[1];
    vec3 c = cross(u, v);
    float c_magnitude = c.length();
    return 0.5 * c_magnitude;
}

vec3 rand_point_on_triangle(const Triangle& t) {
    auto r1 = (float)rand() / (RAND_MAX);
    auto r2 = (float)rand() / (RAND_MAX);
    auto u = (r1 + r2 > 1) ? 1 - r1 : r1;
    auto v = (r1 + r2 > 1) ? 1 - r2 : r2;
    const auto& a = t.vertices[0];
    const auto& b = t.vertices[1];
    const auto& c = t.vertices[2];

    return vec3(a + (b - a) * u + (c - a) * v);
}

}  // namespace raytracer
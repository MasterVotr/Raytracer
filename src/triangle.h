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
    double c_magnitude = c.length();
    return 0.5 * c_magnitude;
}

}  // namespace raytracer
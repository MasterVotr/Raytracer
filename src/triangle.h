#pragma once

#include "vec3.h"
#include "vertex.h"

#include <array>

namespace raytracer {

struct Triangle {
    Vertex vertices[3];
    size_t material_id;
    vec3 normal;

    bool operator==(const Triangle& other) const {
        return (vertices[0] == other.vertices[0] && vertices[1] == other.vertices[1] && vertices[2] == other.vertices[2]) ||
               (vertices[0] == other.vertices[1] && vertices[1] == other.vertices[2] && vertices[2] == other.vertices[0]) ||
               (vertices[0] == other.vertices[2] && vertices[1] == other.vertices[0] && vertices[2] == other.vertices[1]);
    }
};

inline std::ostream& operator<<(std::ostream& os, Triangle triangle) {
    os << "[(" << triangle.vertices[0] << "), (" << triangle.vertices[1] << "), (" << triangle.vertices[2] << ")], mat_id:" << triangle.material_id
       << ", norm:" << triangle.normal;
    return os;
}

vec3 calculate_triangle_normal(const Triangle& t) {
    vec3 u = t.vertices[1].pos - t.vertices[0].pos;
    vec3 v = t.vertices[2].pos - t.vertices[1].pos;
    return cross(u, v).normalize();
}

float calculate_triangle_area(const Triangle& t) {
    vec3 u = t.vertices[1].pos - t.vertices[0].pos;
    vec3 v = t.vertices[2].pos - t.vertices[1].pos;
    vec3 c = cross(u, v);
    float c_magnitude = c.length();
    return 0.5 * c_magnitude;
}

vec3 rand_point_on_triangle(const Triangle& t) {
    auto r1 = (float)rand() / (RAND_MAX);
    auto r2 = (float)rand() / (RAND_MAX);
    auto u = (r1 + r2 > 1) ? 1 - r1 : r1;
    auto v = (r1 + r2 > 1) ? 1 - r2 : r2;
    const auto& a = t.vertices[0].pos;
    const auto& b = t.vertices[1].pos;
    const auto& c = t.vertices[2].pos;

    return vec3(a + (b - a) * u + (c - a) * v);
}

vec3 interpolate_normal_on_triangle(const Triangle& t, const vec3& intersection_point) {
    // Compute edges
    vec3 v0 = t.vertices[1].pos - t.vertices[0].pos;
    vec3 v1 = t.vertices[2].pos - t.vertices[0].pos;
    vec3 v2 = intersection_point - t.vertices[0].pos;

    // Compute dot products
    float d00 = dot(v0, v0);
    float d01 = dot(v0, v1);
    float d11 = dot(v1, v1);
    float d20 = dot(v2, v0);
    float d21 = dot(v2, v1);

    // Compute barycentric coordinates
    float denom = d00 * d11 - d01 * d01;
    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0f - v - w;

    // Interpolate normal
    vec3 normal = t.vertices[0].norm * u + t.vertices[1].norm * v + t.vertices[2].norm * w;

    return normal.normalize();
}

}  // namespace raytracer
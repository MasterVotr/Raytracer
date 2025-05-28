#pragma once

#include <ostream>

#include "src/vec3.h"
#include "src/vertex.h"

namespace raytracer {

struct Triangle {
    Vertex vertices[3];
    size_t material_id;
    Vec3 normal;

    __host__ __device__ bool operator==(const Triangle& other) const {
        return (vertices[0] == other.vertices[0] && vertices[1] == other.vertices[1] && vertices[2] == other.vertices[2]) ||
               (vertices[0] == other.vertices[1] && vertices[1] == other.vertices[2] && vertices[2] == other.vertices[0]) ||
               (vertices[0] == other.vertices[2] && vertices[1] == other.vertices[0] && vertices[2] == other.vertices[1]);
    }

    __device__ void print() const {
        printf("[");
        vertices[0].print();
        printf(", ");
        vertices[1].print();
        printf(", ");
        vertices[2].print();
        printf("], mat_id:%zu, norm:(", material_id);
        normal.print();
        printf(")");
    }
};

inline std::ostream& operator<<(std::ostream& os, Triangle triangle) {
    os << "[(" << triangle.vertices[0] << "), (" << triangle.vertices[1] << "), (" << triangle.vertices[2] << ")], mat_id:" << triangle.material_id
       << ", norm:" << triangle.normal;
    return os;
}

__host__ __device__ inline Vec3 calculate_triangle_normal(const Triangle& t) {
    Vec3 u = t.vertices[1].pos - t.vertices[0].pos;
    Vec3 v = t.vertices[2].pos - t.vertices[1].pos;
    return cross(u, v).normalize();
}

__host__ __device__ inline float calculate_triangle_area(const Triangle& t) {
    Vec3 u = t.vertices[1].pos - t.vertices[0].pos;
    Vec3 v = t.vertices[2].pos - t.vertices[1].pos;
    Vec3 c = cross(u, v);
    float c_magnitude = c.length();
    return 0.5 * c_magnitude;
}

__host__ __device__ inline Vec3 rand_point_on_triangle(const Triangle& t, unsigned int seed = 0, int i = 0) {
#ifdef __CUDA_ARCH__
    float r1 = device_rand(seed, i) / (RAND_MAX);
    float r2 = device_rand(seed, i + 1) / (RAND_MAX);
#else
    float r1 = (float)rand() / (RAND_MAX);
    float r2 = (float)rand() / (RAND_MAX);
#endif
    if (r1 + r2 > 1.0f) {
        r1 = 1.0f - r1;
        r2 = 1.0f - r2;
    }
    float u = r1;
    float v = r2;
    float w = 1.0f - u - v;
    const auto& a = t.vertices[0].pos;
    const auto& b = t.vertices[1].pos;
    const auto& c = t.vertices[2].pos;
    return a * w + b * u + c * v;
}

__host__ __device__ inline Vec3 interpolate_normal_on_triangle(const Triangle& t, const Vec3& intersection_point) {
    // Compute edges
    Vec3 v0 = t.vertices[1].pos - t.vertices[0].pos;
    Vec3 v1 = t.vertices[2].pos - t.vertices[0].pos;
    Vec3 v2 = intersection_point - t.vertices[0].pos;

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
    Vec3 normal = t.vertices[0].norm * u + t.vertices[1].norm * v + t.vertices[2].norm * w;

    return normal.normalize();
}

}  // namespace raytracer
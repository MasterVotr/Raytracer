#pragma once

#include "src/aabb.h"
#include "src/ray.h"
#include "src/triangle.h"
#include "src/util.h"
#include "src/vec3.h"

namespace raytracer {

// Calculates the time/parameter (t) at which the ray collides with the triangle, infinty if it misses
inline float collision_ray_triangle(Ray& r, const Triangle& triangle, bool cull_backfaces_ = true) {
    // Extract triangle vertices and ray properties
    const Vec3& a = triangle.vertices[0].pos;
    const Vec3& b = triangle.vertices[1].pos;
    const Vec3& c = triangle.vertices[2].pos;
    const Vec3& s = r.direction();
    const Point3 o = r.origin();

    // Compute edges of the triangle
    Vec3 e1 = b - a;  // Edge 1: vector from a to b
    Vec3 e2 = c - a;  // Edge 2: vector from a to c

    // Compute the determinant
    Vec3 p = cross(s, e2);  // Cross product of ray direction and edge 2
    float d = dot(e1, p);   // Determinant: dot product of edge 1 and p

    // Cull backfaces if enabled
    if (cull_backfaces_) {
        if (d < epsilon) {  // If determinant is near zero, the ray is parallel to the triangle
            r.t_distance() = infinity;
            return infinity;  // No intersection
        }
    } else {
        if (fabs(d) < epsilon) {  // For non-culling, check if determinant is close to zero
            r.t_distance() = infinity;
            return infinity;  // No intersection
        }
    }

    // Compute the inverse of the determinant
    float d_inv = 1.0 / d;

    // Compute the vector from vertex a to the ray origin
    Vec3 q = o - a;

    // Compute the barycentric coordinate u
    float u = d_inv * dot(q, p);
    if (u < 0.0 || u > 1.0) {  // If u is outside the range [0, 1], the intersection is outside the triangle
        r.t_distance() = infinity;
        return infinity;
    }

    // Compute the barycentric coordinate v
    Vec3 w = cross(q, e1);  // Cross product of q and edge 1
    float v = d_inv * dot(w, s);
    if (v < 0.0 || (u + v) > 1.0) {  // If v is outside the range [0, 1] or u + v > 1, the intersection is outside the triangle
        r.t_distance() = infinity;
        return infinity;
    }

    // Compute the distance t along the ray to the intersection point
    float t = d_inv * dot(e2, w);

    // Update the ray's t_distance and return the intersection distance
    r.t_distance() = t;
    return t;
}

// Calculates the latest axis entry a earliest axis exit and compares them -> "slab" technique
inline bool collision_ray_aabb(const Ray& r, const AABB& aabb) {
    float t_min = -infinity;
    float t_max = infinity;

    for (int i = 0; i < 3; ++i) {
        float inv_d = 1.0f / r.direction()[i];
        float t0 = (aabb.min[i] - r.origin()[i]) * inv_d;
        float t1 = (aabb.max[i] - r.origin()[i]) * inv_d;
        if (inv_d < 0.0f) {
            std::swap(t0, t1);
        }
        t_min = std::max(t_min, t0);
        t_max = std::min(t_max, t1);
    }
    return t_max >= t_min;
}

// ChatGPT generated function, I don't understand it very well
inline bool collision_triangle_aabb(const Triangle& t, const AABB& aabb) {
    const Vec3 box_center = (aabb.min + aabb.max) * 0.5f;
    const Vec3 box_half_size = (aabb.max - aabb.min) * 0.5f;

    // Triangle vertices relative to AABB center
    Vec3 v0 = t.vertices[0].pos - box_center;
    Vec3 v1 = t.vertices[1].pos - box_center;
    Vec3 v2 = t.vertices[2].pos - box_center;

    // Triangle edges
    Vec3 e0 = v1 - v0;
    Vec3 e1 = v2 - v1;
    Vec3 e2 = v0 - v2;

    auto axis_test = [&](const Vec3& axis, float& r, float& p0, float& p1, float& p2) {
        p0 = dot(v0, axis);
        p1 = dot(v1, axis);
        p2 = dot(v2, axis);
        r = box_half_size.x * std::abs(axis.x) + box_half_size.y * std::abs(axis.y) + box_half_size.z * std::abs(axis.z);
        float min_p = std::min({p0, p1, p2});
        float max_p = std::max({p0, p1, p2});
        return !(min_p > r || max_p < -r);
    };

    // 1. Test axes of the AABB
    for (int i = 0; i < 3; ++i) {
        float min_v = std::min({v0[i], v1[i], v2[i]});
        float max_v = std::max({v0[i], v1[i], v2[i]});
        if (min_v > box_half_size[i] || max_v < -box_half_size[i])
            return false;
    }

    // 2. Test axis perpendicular to triangle face
    Vec3 normal = t.normal;
    float p0 = dot(v0, normal);
    float r = box_half_size.x * std::abs(normal.x) + box_half_size.y * std::abs(normal.y) + box_half_size.z * std::abs(normal.z);
    if (std::abs(p0) > r)
        return false;

    // 3. Test 9 cross-product axes
    const Vec3 axes[] = {Vec3(0, -e0.z, e0.y), Vec3(0, -e1.z, e1.y), Vec3(0, -e2.z, e2.y), Vec3(e0.z, 0, -e0.x), Vec3(e1.z, 0, -e1.x),
                         Vec3(e2.z, 0, -e2.x), Vec3(-e0.y, e0.x, 0), Vec3(-e1.y, e1.x, 0), Vec3(-e2.y, e2.x, 0)};
    for (const auto& axis : axes) {
        float r_axis, p0, p1, p2;
        if (!axis_test(axis, r_axis, p0, p1, p2))
            return false;
    }

    return true;
}

// Compares the mins and maxes to detect collision with another aabb
inline bool collision_aabb_aabb(const AABB& a, const AABB& b) {
    return (a.min.x <= b.max.x && a.max.x >= b.min.x) && (a.min.y <= b.max.y && a.max.y >= b.min.y) && (a.min.z <= b.max.z && a.max.z >= b.min.z);
}

}  // namespace raytracer
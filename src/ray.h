#pragma once

#include "src/vec3.h"

namespace raytracer {

class Ray {
   public:
    __host__ __device__ Ray() {}
    __host__ __device__ Ray(Point3 orig, Vec3 dir, float t = -1.0) : orig_(orig), dir_(dir), t_(t) {}

    __host__ __device__ const Point3& origin() const { return orig_; }
    __host__ __device__ const Vec3& direction() const { return dir_; }
    __host__ __device__ float& t_distance() { return t_; }
    __host__ __device__ Point3 at(float t) const { return orig_ + dir_ * t; }

   private:
    Point3 orig_;
    Vec3 dir_;
    float t_;  // t when hit
};

}  // namespace raytracer
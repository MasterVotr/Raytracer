#pragma once

#include "src/vec3.h"

namespace raytracer {

class Ray {
   public:
    Ray() {}
    Ray(Point3 orig, Vec3 dir, float t = -1.0) : orig_(orig), dir_(dir), t_(t) {}

    inline const Point3& origin() const { return orig_; }
    inline const Vec3& direction() const { return dir_; }
    inline float& t_distance() { return t_; }
    inline Point3 at(float t) const { return orig_ + dir_ * t; }

   private:
    Point3 orig_;
    Vec3 dir_;
    float t_;  // t when hit
};

}  // namespace raytracer
#pragma once

#include "vec3.h"

namespace raytracer {

class ray {
   public:
    ray() {}
    ray(point3 orig, vec3 dir, float t = -1.0) : orig_(orig), dir_(dir), t_(t) {}

    const point3& origin() const { return orig_; }
    const vec3& direction() const { return dir_; }
    float& t_distance() { return t_; }

    point3 at(float t) const { return orig_ + dir_ * t; }

   private:
    point3 orig_;
    vec3 dir_;
    float t_;  // t when hit
};

}  // namespace raytracer
#pragma once

#include "vec3.h"

namespace raytracer {

class ray {
   public:
    ray() {}
    ray(point3 _orig, vec3 _dir) : orig(_orig), dir(_dir) {}

    const point3& origin() const { return orig; }
    const vec3 direction() const { return dir; }

    point3 at(double t) const { return orig + dir * t; }

   private:
    point3 orig;
    vec3 dir;
};

}  // namespace raytracer
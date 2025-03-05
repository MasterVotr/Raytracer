#pragma once

#include "triangle.h"

#include <vector>

namespace raytracer {

class Scene {
   public:
    Scene() {}
    void AddTriangle(const Triangle& t) { triangles_.emplace_back(t); }
    const std::vector<Triangle>& GetTriangles() { return triangles_; }

   private:
    std::vector<Triangle> triangles_;
};

}  // namespace raytracer
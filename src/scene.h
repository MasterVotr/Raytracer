#pragma once

#include "material.h"
#include "triangle.h"

#include <vector>

namespace raytracer {

class Scene {
   public:
    Scene() {}
    void AddTriangle(const Triangle& t) { triangles_.emplace_back(t); }
    void AddMaterial(const Material& m) { materials_.emplace_back(m); }
    const std::vector<Triangle>& GetTriangles() const { return triangles_; }
    const std::vector<Material>& GetMaterials() const { return materials_; }

   private:
    std::vector<Triangle> triangles_;
    std::vector<Material> materials_;
};

}  // namespace raytracer
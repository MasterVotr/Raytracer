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
    void AddLight(const Triangle& t) { lights_.emplace_back(t); }
    const std::vector<Triangle>& GetTriangles() const { return triangles_; }
    const std::vector<Material>& GetMaterials() const { return materials_; }
    const std::vector<Triangle>& GetLights() const { return lights_; }

   private:
    std::vector<Triangle> triangles_;
    std::vector<Material> materials_;
    std::vector<Triangle> lights_;
};

}  // namespace raytracer
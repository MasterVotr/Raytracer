#pragma once

#include <ostream>
#include <vector>

#include "include/json.hpp"

#include "src/ADS/Octree/octree.h"
#include "src/aabb.h"
#include "src/ray.h"
#include "src/triangle.h"

namespace raytracer {

class OctreeParametric : public Octree {
   public:
    OctreeParametric(const nlohmann::json& config) : Octree(config) {}
    virtual ~OctreeParametric() = default;

    virtual std::vector<std::shared_ptr<const Triangle>> Search(const Ray& r, bool first_hit = false) const override;
};

}  // namespace raytracer
#pragma once

#include <vector>

#include "include/json.hpp"
#include "src/ADS/ads.h"
#include "src/aabb.h"
#include "src/ray.h"
#include "src/triangle.h"

namespace raytracer {

// Bounding volume hierarchy acceleration data structure
class Bvh : public Ads {
   public:
    Bvh(const nlohmann::json& config) : Ads(config) {}
    virtual ~Bvh() = default;

    virtual void Build(const std::vector<std::shared_ptr<const Triangle>>& triangles);
    virtual std::vector<std::shared_ptr<const Triangle>> Search(const Ray& r, bool first_hit = false) const;
    virtual void PrintStats(std::ostream& os) const = 0;

   protected:
    struct BvhNode {
        bool isLeaf;
        size_t depth;
        AABB bounding_box;
        std::vector<size_t> triangle_indices;
        std::shared_ptr<BvhNode> left;
        std::shared_ptr<BvhNode> right;
    };

    virtual void config_setup(const nlohmann::json& config);
    std::shared_ptr<BvhNode> root;
};

}  // namespace raytracer

#pragma once

#include <memory>

#include "src/ADS/ads.h"
#include "src/aabb.h"

namespace raytracer {

// Bounding volume hierarchy acceleration data structure - naive implementation
class BvhNaive : public Ads {
   public:
    BvhNaive(const nlohmann::json& config);
    virtual ~BvhNaive() = default;

    virtual void Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) override;
    virtual std::vector<std::shared_ptr<const Triangle>> Search(const Ray& r, bool first_hit = false) const override;
    virtual void PrintStats(std::ostream& os) const override;

   protected:
    // Represents one node in the BVH tree
    struct BvhNode {
        bool isLeaf;
        int depth;
        AABB bounding_box;
        std::vector<size_t> triangle_indices;
        std::shared_ptr<BvhNode> left, right;
    };

    std::shared_ptr<BvhNode> root_;
};

}  // namespace raytracer
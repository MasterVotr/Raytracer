#pragma once

#include <memory>

#include "src/ADS/ADSStats/ads_stats.h"
#include "src/ADS/ads.h"

namespace raytracer {

// Top down bounding volume hierarchy acceleration data structure - naive implementation
class TDBvh : public Ads {
   public:
    TDBvh(const nlohmann::json& config);
    virtual ~TDBvh() = default;

    virtual void Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) override;
    virtual std::vector<std::shared_ptr<const Triangle>> Search(const Ray& r, bool first_hit = false) const override;
    virtual void PrintStats(std::ostream& os) const override;

   private:
    // Represents one node in the BVH tree
    struct BvhNode {
        bool isLeaf;
        size_t depth;
        AABB bounding_box;
        std::vector<size_t> triangle_indices;
        std::shared_ptr<BvhNode> left, right;
    };

    void ConfigSetup(const nlohmann::json& config);
    void CalculateStats() const;

    std::shared_ptr<BvhNode> root_;
    mutable AdsStats stats_;

    // Config variables
    size_t max_triangles_per_BB_;
    size_t max_depth_;
};

}  // namespace raytracer
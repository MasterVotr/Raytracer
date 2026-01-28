#pragma once

#include <memory>

#include "src/ADS/ADSStats/ads_stats.h"
#include "src/ADS/ads.h"
#include "src/vec3.h"

namespace raytracer {

// Bounding volume hierarchy acceleration data structure - naive implementation
class SBvh : public Ads {
   public:
    SBvh(const nlohmann::json& config);
    virtual ~SBvh() = default;

    virtual void Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) override;
    virtual std::vector<std::shared_ptr<const Triangle>> Search(const Ray& r, bool first_hit = false) const override;
    virtual void PrintStats(std::ostream& os) const override;

   protected:
    // Represents one node in the BVH tree
    struct BvhNode {
        Vec3 aabb_min = {infinity, infinity, infinity};
        Vec3 aabb_max = {-infinity, -infinity, -infinity};
        size_t t_begin = 0;  // or left child (right child is t_begin+1)
        size_t t_count = 0;
        bool is_leaf() const { return t_count > 0; }
    };

    // Helper methods
    float FindBestSplit(BvhNode& node, int& axis, float& split_pos, const AABB& cb, AABB& TB_L, AABB& TB_R,
                        const std::vector<Point3>& tcs, const std::vector<AABB>& tbs);
    void Subdivide(size_t node_idx, int depth, AABB cb, const std::vector<Point3>& tcs, const std::vector<AABB>& tbs);
    void UpdateNodeBounds(size_t node_idx, const std::vector<Point3>& tbs);

    void ConfigSetup(const nlohmann::json& config);
    void CalculateStats() const;

    std::vector<BvhNode> nodes_;  // root node at idx 0, child nodes are parent node_idx*2+1 and node_idx*2+2
    std::vector<uint> tri_idxs_;
    size_t next_bvh_node_idx_;

    // Config variables
    size_t max_triangles_per_BB_;
    int max_depth_;
    int bin_count_;

    // Statistics variables
    mutable AdsStats stats_;

    mutable size_t bins_duration_;
    mutable size_t partitioning_duration_;
};

}  // namespace raytracer
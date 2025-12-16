#pragma once

#include <memory>

#include "src/ADS/ads.h"
#include "src/vec3.h"

namespace raytracer {

// Bounding volume hierarchy acceleration data structure - naive implementation
class BvhSeq2 : public Ads {
   public:
    BvhSeq2(const nlohmann::json& config);
    virtual ~BvhSeq2() = default;

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

    // Helper structure for BVH stats
    struct BvhSeq2Stats {
        // Build stats
        size_t min_depth;
        size_t max_depth;
        float avg_depth;
        size_t min_triangles_in_leaf_nodes;
        size_t max_triangles_in_leaf_nodes;
        float avg_triangles_in_leaf_nodes;
        size_t nodes_count;
        size_t leaf_nodes_count;
        // Search stats
        size_t search_count;
        size_t search_min_nodes_visited;
        size_t search_max_nodes_visited;
        size_t search_nodes_visited;
        size_t search_min_return_count;
        size_t search_max_return_count;
        size_t search_return_count;
        long search_min_time;
        long search_max_time;
        long search_time;
        size_t search_min_leaves_visited;
        size_t search_max_leaves_visited;
        size_t search_leaves_visited;
    };

    // Helper methods
    float find_best_split(BvhNode& node, int& axis, float& split_pos, const AABB& cb, AABB& TB_L, AABB& TB_R,
                          const std::vector<Point3>& tcs, const std::vector<AABB>& tbs);
    void subdivide(size_t node_idx, int depth, AABB cb, const std::vector<Point3>& tcs, const std::vector<AABB>& tbs);
    void update_node_boudns(size_t node_idx, const std::vector<Point3>& tbs);

    void config_setup(const nlohmann::json& config);
    BvhSeq2Stats calculate_stats() const;
    void reset_stats() const;

    std::vector<BvhNode> nodes_;  // root node at idx 0, child nodes are parent node_idx*2+1 and node_idx*2+2
    std::vector<uint> tri_idxs_;
    size_t next_bvh_node_idx_;

    // Config variables
    size_t max_triangles_per_BB_;
    int max_depth_;
    int bin_count_;

    // Statistics variables
    mutable size_t search_count_;
    mutable size_t search_min_nodes_visited_;
    mutable size_t search_max_nodes_visited_;
    mutable size_t search_nodes_visited_;
    mutable size_t search_min_return_count_;
    mutable size_t search_max_return_count_;
    mutable size_t search_return_count_;
    mutable long search_min_time_;
    mutable long search_max_time_;
    mutable long search_time_;
    mutable size_t search_min_leaves_visited_;
    mutable size_t search_max_leaves_visited_;
    mutable size_t search_leaves_visited_;

    mutable size_t bins_duration_;
};

}  // namespace raytracer
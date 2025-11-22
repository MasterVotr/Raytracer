#pragma once

#include <memory>

#include "src/ADS/ads.h"

namespace raytracer {

// Bounding volume hierarchy acceleration data structure - naive implementation
class BvhVec : public Ads {
   public:
    BvhVec(const nlohmann::json& config);
    virtual ~BvhVec() = default;

    virtual void Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) override;
    virtual std::vector<std::shared_ptr<const Triangle>> Search(const Ray& r, bool first_hit = false) const override;
    virtual void PrintStats(std::ostream& os) const override;

   protected:
    // Represents one node in the BVH tree
    struct BvhNode {
        size_t depth = 0;
        AABB bounding_box;
        size_t t_begin = 0;
        size_t t_count = 0;
        size_t left_child_idx = 0;
        size_t right_child_idx = 0;
    };

    // Helper structure for BVH stats
    struct BvhVecStats {
        // Build stats
        size_t min_depth;
        size_t max_depth;
        float avg_depth;
        size_t min_triangles_in_leaf_nodes;
        size_t max_triangles_in_leaf_nodes;
        float avg_traiangles_in_leaf_nodes;
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

    void config_setup(const nlohmann::json& config);
    BvhVecStats calculate_stats() const;
    void reset_stats() const;

    std::vector<BvhNode> nodes_;  // root node at idx 0, child nodes are parent node_idx*2+1 and node_idx*2+2
    std::vector<size_t> triangle_indices_;

    // Config variables
    size_t max_triangles_per_BB_;
    size_t max_depth_;
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
};

}  // namespace raytracer
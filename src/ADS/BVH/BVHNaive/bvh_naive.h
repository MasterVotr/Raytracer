#pragma once

#include <memory>

#include "src/ADS/ads.h"

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
        size_t depth;
        AABB bounding_box;
        std::vector<size_t> triangle_indices;
        std::shared_ptr<BvhNode> left, right;
    };

    // Helper structure for BVH stats
    struct BvhNaiveStats {
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
    BvhNaiveStats calculate_stats() const;
    void reset_stats() const;

    std::shared_ptr<BvhNode> root_;

    // Config variables
    size_t max_triangles_per_BB_;
    size_t max_depth_;

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
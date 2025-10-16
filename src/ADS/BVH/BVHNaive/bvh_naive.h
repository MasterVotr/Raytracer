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
        int depth;
        AABB bounding_box;
        std::vector<size_t> triangle_indices;
        std::shared_ptr<BvhNode> left, right;
    };

    // Helper structure for BVH stats
    struct BvhNaiveStats {
        size_t max_depth;
        float avg_depth;
        size_t triangles_in_leaf_nodes;
        float avg_traiangles_in_leaf_nodes;
        size_t nodes_count;
        size_t leaf_nodes_count;
        size_t search_count;
        size_t search_node_count;
        size_t search_return_count;
        long long search_time;
        size_t search_leaves_visited;
    };

    void config_setup(const nlohmann::json& config);
    BvhNaiveStats calculate_stats() const;

    std::shared_ptr<BvhNode> root_;

    // Config variables
    size_t max_triangles_per_BB_;
    size_t max_depth_;

    // Statistics variables
    mutable size_t search_count_ = 0;
    mutable size_t search_node_count_ = 0;
    mutable float search_time_ = 0;
    mutable size_t search_return_count_ = 0;
    mutable size_t search_leaves_visited_ = 0;
};

}  // namespace raytracer
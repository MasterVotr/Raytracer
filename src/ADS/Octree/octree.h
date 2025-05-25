#pragma once

#include <ostream>
#include <vector>

#include "include/json.hpp"

#include "src/ADS/ads.h"
#include "src/aabb.h"
#include "src/ray.h"
#include "src/triangle.h"

namespace raytracer {

class Octree : public Ads {
   public:
    Octree(const nlohmann::json& config) : Ads(config) { config_setup(config); }
    virtual ~Octree() = default;

    virtual void Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) override;
    virtual std::vector<std::shared_ptr<const Triangle>> Search(const Ray& r, bool first_hit = false) const override;
    virtual void PrintStats(std::ostream& os) const override;

   protected:
    // Represents one node in the Octree
    struct OctreeNode {
        bool isLeaf;
        size_t depth;
        AABB bounding_box;
        std::vector<size_t> triangle_indices;
        std::shared_ptr<OctreeNode> octanes[8];
    };

    // Helper structure for Octree stats
    struct OctreeStats {
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

    virtual void config_setup(const nlohmann::json& config);
    OctreeStats calculate_stats() const;

    std::shared_ptr<OctreeNode> root;

    // Statistics variables
    mutable size_t max_triangles_per_BB_;
    mutable size_t max_depth_;
    mutable size_t search_count_ = 0;
    mutable size_t search_node_count_ = 0;
    mutable float search_time_ = 0;
    mutable size_t search_return_count_ = 0;
    mutable size_t search_leaves_visited_ = 0;
};

}  // namespace raytracer
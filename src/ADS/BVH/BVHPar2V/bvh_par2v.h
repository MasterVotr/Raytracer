#pragma once

#include <memory>

#include "src/ADS/ads.h"
#include "src/vec3.h"

#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#elif defined(__aarch64__)
#include <arm_neon.h>
#endif

#include "src/aabb.h"
#include "src/collision_detection.h"
#include "src/timer.h"

namespace raytracer {

// Bounding volume hierarchy acceleration data structure - naive implementation
class BvhPar2V : public Ads {
   public:
    BvhPar2V(const nlohmann::json& config);
    virtual ~BvhPar2V() = default;

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
    struct BvhStats {
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

    struct vAABB {
        __m128 min;
        __m128 max;
    };

    // Helper methods
    float find_best_split(BvhNode& node, int& axis, float& split_pos, const AABB& cb, AABB& TB_L, AABB& TB_R,
                          const std::vector<Point3>& tcs, const std::vector<vAABB>& tbs);
    void subdivide(size_t node_idx, int depth, AABB cb, const std::vector<Point3>& tcs, const std::vector<vAABB>& tbs);

    void config_setup(const nlohmann::json& config);
    BvhStats calculate_stats() const;
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
    mutable size_t bins_seq_duration_;
    mutable size_t bins_sync_duration_;
    mutable size_t partitioning_duration_;
};

}  // namespace raytracer
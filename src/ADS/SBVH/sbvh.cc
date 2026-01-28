#include "src/ADS/SBVH/sbvh.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <stack>

#include "src/aabb.h"
#include "src/collision_detection.h"
#include "src/logger.h"
#include "src/timer.h"

namespace raytracer {

SBvh::SBvh(const nlohmann::json& config) : Ads(config) { ConfigSetup(config); }

void SBvh::Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) {
    Ads::Build(triangles);
    stats_.Reset();
    Logger::debug("Building SAH BVH...");
    Timer build_t;
    bins_duration_ = 0;
    partitioning_duration_ = 0;

    // Clear nodes_ and tri_idxs
    size_t n = triangles_.size();
    nodes_.resize(2 * n - 1);
    tri_idxs_.resize(n);
    std::iota(tri_idxs_.begin(), tri_idxs_.end(), 0);
    Timer init_t;

    // Precalculate tbs and cbs
    std::vector<AABB> tbs;  // Triangle bboxes
    tbs.reserve(n);
    std::vector<Point3> tcs;  // Triangle centroids
    tcs.reserve(n);
    std::for_each(triangles_.begin(), triangles_.end(), [&](const auto& t) {
        tbs.emplace_back(calculate_triangle_aabb(*t));
        tcs.emplace_back(calculate_triangle_centroid(*t));
    });

    // Create root and launch recursive subdivide
    AABB vb;
    AABB cb;
    std::for_each(tbs.begin(), tbs.end(), [&](const auto& tb) { vb.expand(tb); });
    std::for_each(tcs.begin(), tcs.end(), [&](const auto& tc) { cb.expand(tc); });

    Logger::debug("Init time: {} ms", init_t.elapsed_ms());

    BvhNode& root = nodes_[0];
    root.t_begin = 0;
    root.aabb_min = vb.min;
    root.aabb_max = vb.max;
    root.t_count = n;
    next_bvh_node_idx_ = 1;

    Subdivide(0, 0, cb, tcs, tbs);
    stats_.build_time = build_t.elapsed_ms();

    Logger::debug("Bins time: {} ms", bins_duration_ / 1'000'000.0);
    Logger::debug("Partitioning time: {} ms", partitioning_duration_ / 1'000'000.0);
    CalculateStats();
}

std::vector<std::shared_ptr<const Triangle>> SBvh::Search(const Ray& r, bool first_hit) const {
    stats_.total_query_count++;
    uint32_t trav_steps = 0;
    uint32_t inci_ops = 0;

    std::vector<std::shared_ptr<const Triangle>> result;
    result.reserve(max_triangles_per_BB_);

    std::stack<std::pair<size_t, size_t>> s;
    s.emplace(0, 0);
    while (!s.empty()) {
        auto [node_idx, depth] = s.top();
        s.pop();

        const BvhNode& node = nodes_[node_idx];
        trav_steps++;

        if (node.is_leaf()) {
            inci_ops += static_cast<uint32_t>(node.t_count);
            for (size_t i = 0; i < node.t_count; i++) {
                size_t tri_idx = tri_idxs_[node.t_begin + i];
                result.emplace_back(triangles_[tri_idx]);
            }
            continue;
        }

        size_t left_child_idx = node.t_begin;
        const BvhNode& left_child = nodes_[left_child_idx];
        size_t right_child_idx = left_child_idx + 1;
        const BvhNode& right_child = nodes_[right_child_idx];
        if (collision_ray_aabb(r, {left_child.aabb_min, left_child.aabb_max})) {
            s.emplace(left_child_idx, depth + 1);
        }
        if (collision_ray_aabb(r, {right_child.aabb_min, right_child.aabb_max})) {
            s.emplace(right_child_idx, depth + 1);
        }
    }

    stats_.min_traversal_steps = std::min(stats_.min_traversal_steps, trav_steps);
    stats_.max_traversal_steps = std::max(stats_.max_traversal_steps, trav_steps);
    stats_.total_traversal_steps += trav_steps;
    stats_.min_incidence_operations = std::min(stats_.min_incidence_operations, inci_ops);
    stats_.max_incidence_operations = std::max(stats_.max_incidence_operations, inci_ops);
    stats_.total_incidence_operations += inci_ops;

    return result;
}

void SBvh::PrintStats(std::ostream& os) const {
    os << "SAH BVH\n";
    stats_.Print(os);
}

float SBvh::FindBestSplit(BvhNode& node, int& axis, float& split_pos, const AABB& cb, AABB& TB_L, AABB& TB_R,
                          const std::vector<Point3>& tcs, const std::vector<AABB>& tbs) {
    // Decide the longest cb axis
    axis = 0;
    if (cb.size.x >= cb.size.y && cb.size.x >= cb.size.z) {
        axis = 0;
    } else if (cb.size.y >= cb.size.z) {
        axis = 1;
    } else {
        axis = 2;
    }

    // Calculate bins
    Timer t_bins;
    std::vector<AABB> bbs(bin_count_, AABB());
    std::vector<size_t> ns(bin_count_, 0);

    float k_0 = cb.min[axis];
    float k_1 = bin_count_ * (1 - 1e-3) /
                cb.size[axis];  // Possible problem with bin_idx being K (bin_count) - epsilon was too big
    for (size_t t = node.t_begin; t < node.t_begin + node.t_count; t++) {
        size_t t_idx = tri_idxs_[t];
        size_t bin_idx = std::min(bin_count_ - 1, static_cast<int>(k_1 * (tcs[t_idx][axis] - k_0)));

        bbs[bin_idx].expand(tbs[t_idx]);
        ns[bin_idx]++;
    }
    bins_duration_ += t_bins.elapsed_ns();

    // Prefix sum calculation left->right
    float best_split_cost = infinity;
    int split_count = bin_count_ - 1;
    std::vector<size_t> N_Ls(split_count), N_Rs(split_count);
    std::vector<AABB> TB_Ls(split_count), TB_Rs(split_count);
    std::vector<float> A_Ls(split_count), A_Rs(split_count);

    N_Ls[0] = ns[0];
    TB_Ls[0] = bbs[0];
    A_Ls[0] = TB_Ls[0].surface_area();

    for (int i = 1; i < split_count; i++) {
        N_Ls[i] = N_Ls[i - 1] + ns[i];
        TB_Ls[i] = TB_Ls[i - 1];
        TB_Ls[i].expand(bbs[i]);
        A_Ls[i] = TB_Ls[i].surface_area();
    }

    // Prefix sum calculation right->left and best split
    N_Rs[split_count - 1] = ns[split_count];
    TB_Rs[split_count - 1] = bbs[split_count];
    A_Rs[split_count - 1] = TB_Rs[split_count - 1].surface_area();

    float scale = cb.size[axis] / bin_count_;  // size of one bin in axis
    for (int i = split_count - 2; i >= 0; i--) {
        N_Rs[i] = N_Rs[i + 1] + ns[i + 1];
        TB_Rs[i] = TB_Rs[i + 1];
        TB_Rs[i].expand(bbs[i + 1]);
        A_Rs[i] = TB_Rs[i].surface_area();

        // Best split
        float split_cost = N_Ls[i] * A_Ls[i] + N_Rs[i] * A_Rs[i];
        if (split_cost != 0 && split_cost < best_split_cost) {
            best_split_cost = split_cost;
            split_pos = cb.min[axis] + scale * (i + 1);

            TB_L = TB_Ls[i];
            TB_R = TB_Rs[i];
        }
    }

    return best_split_cost;
}

void SBvh::Subdivide(size_t node_idx, int depth, AABB cb, const std::vector<Point3>& tcs,
                     const std::vector<AABB>& tbs) {
    BvhNode& node = nodes_[node_idx];

    // Check for BVH criteria
    bool cb_too_small = cb.size.x < epsilon && cb.size.y < epsilon && cb.size.z < epsilon;
    if (node.t_count < max_triangles_per_BB_ || depth >= max_depth_ || cb_too_small) {
        return;
    }

    // Find best split
    int axis;
    float split_pos;
    AABB TB_L, TB_R;
    FindBestSplit(node, axis, split_pos, cb, TB_L, TB_R, tcs, tbs);

    // Triangle partitioning
    Timer t_partitioning;
    int i = node.t_begin;
    int j = node.t_begin + node.t_count - 1;
    while (i <= j) {
        if (tcs[tri_idxs_[i]][axis] < split_pos) {
            i++;
        } else {
            std::swap(tri_idxs_[i], tri_idxs_[j]);
            j--;
        }
    }

    // Recalculate N_L and N_R based on actual partitioning
    size_t N_L = i - node.t_begin;
    size_t N_R = node.t_count - N_L;

    // Abort split if one of the children is empty
    if (N_L == 0 || N_R == 0) {
        return;
    }

    // Create child nodes
    size_t left_child_idx = next_bvh_node_idx_++;
    nodes_[left_child_idx].t_begin = node.t_begin;
    nodes_[left_child_idx].t_count = N_L;
    nodes_[left_child_idx].aabb_min = TB_L.min;
    nodes_[left_child_idx].aabb_max = TB_L.max;

    size_t right_child_idx = next_bvh_node_idx_++;
    nodes_[right_child_idx].t_begin = node.t_begin + N_L;
    nodes_[right_child_idx].t_count = N_R;
    nodes_[right_child_idx].aabb_min = TB_R.min;
    nodes_[right_child_idx].aabb_max = TB_R.max;

    // Set current node to internal
    node.t_begin = left_child_idx;
    node.t_count = 0;

    // Calculate new centroid bboxes
    AABB CB_L;
    size_t t_start_L = nodes_[left_child_idx].t_begin;
    size_t t_end_L = nodes_[left_child_idx].t_begin + nodes_[left_child_idx].t_count;
    for (size_t i = t_start_L; i < t_end_L; i++) {
        CB_L.expand(tcs[tri_idxs_[i]]);
    }

    AABB CB_R;
    size_t t_start_R = nodes_[right_child_idx].t_begin;
    size_t t_end_R = nodes_[right_child_idx].t_begin + nodes_[right_child_idx].t_count;
    for (size_t i = t_start_R; i < t_end_R; i++) {
        CB_R.expand(tcs[tri_idxs_[i]]);
    }

    partitioning_duration_ += t_partitioning.elapsed_ns();

    // Subdivide recursively
    Subdivide(left_child_idx, depth + 1, CB_L, tcs, tbs);
    Subdivide(right_child_idx, depth + 1, CB_R, tcs, tbs);
}

void SBvh::CalculateStats() const {
    std::stack<std::pair<size_t, size_t>> s;
    s.emplace(0, 0);
    while (!s.empty()) {
        auto [node_idx, depth] = s.top();
        s.pop();

        const BvhNode& node = nodes_[node_idx];
        stats_.node_count++;

        if (node.is_leaf()) {
            stats_.leaf_node_count++;

            stats_.min_leaf_depth = std::min(stats_.min_leaf_depth, static_cast<uint32_t>(depth));
            stats_.max_leaf_depth = std::max(stats_.max_leaf_depth, static_cast<uint32_t>(depth));
            stats_.total_leaf_depth += depth;

            stats_.min_prims_per_leaf = std::min(stats_.min_prims_per_leaf, static_cast<uint32_t>(node.t_count));
            stats_.max_prims_per_leaf = std::max(stats_.max_prims_per_leaf, static_cast<uint32_t>(node.t_count));
            stats_.total_prims_per_leaf += node.t_count;

            continue;
        }

        stats_.inner_node_count++;

        size_t left_child_idx = node.t_begin;
        size_t right_child_idx = left_child_idx + 1;
        s.emplace(left_child_idx, depth + 1);
        s.emplace(right_child_idx, depth + 1);
    }

    stats_.memory_consumption = nodes_.size() * sizeof(BvhNode);
}

void SBvh::ConfigSetup(const nlohmann::json& config) {
    max_triangles_per_BB_ = config.at("max_triangles_per_BB");
    max_depth_ = config.at("max_depth");
    bin_count_ = config.at("bin_count");

    Logger::debug("SAH BVH configured");
}

}  // namespace raytracer
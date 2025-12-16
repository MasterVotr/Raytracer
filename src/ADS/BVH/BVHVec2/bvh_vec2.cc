#include "src/ADS/BVH/BVHVec2/bvh_vec2.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <stack>

#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#elif defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace raytracer {

BvhVec2::BvhVec2(const nlohmann::json& config) : Ads(config) {
    config_setup(config);
    reset_stats();
}

void BvhVec2::Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) {
    Ads::Build(triangles);
    reset_stats();
    std::clog << "Building vectorized2 BVH..." << std::endl;
    Timer build_t;
    bins_duration_ = 0;

    // Clear nodes_ and tri_idxs
    size_t n = triangles_.size();
    nodes_.resize(2 * n - 1);
    tri_idxs_.resize(n);
    std::iota(tri_idxs_.begin(), tri_idxs_.end(), 0);
    Timer init_t;

    // Precalculate tbs, cbs, root cb and cb
    std::vector<vAABB> tbs(n);   // Triangle bboxes
    std::vector<Point3> tcs(n);  // Triangle centroids
    __m128 third = _mm_set1_ps(1.0f / 3.0f);
    __m128 vb_min = _mm_set1_ps(infinity);
    __m128 vb_max = _mm_set1_ps(-infinity);
    __m128 cb_min = _mm_set1_ps(infinity);
    __m128 cb_max = _mm_set1_ps(-infinity);
    alignas(16) float f4_dto_1[4];
    alignas(16) float f4_dto_2[4];
    for (size_t i = 0; i < n; i++) {
        const auto& t = triangles_[i];
        /*
            3x load - triangles vertices
            2x min + 2x max - triangles aabbs
            min + max - grow voxel aabb
            add + mul - triangles centroids
            min + max - grow centroid aabb
            3x write - trinagles aabbs and centroids
        */
        __m128 v0 = _mm_setr_ps(t->vertices[0].pos.x, t->vertices[0].pos.y, t->vertices[0].pos.z, 0.0f);
        __m128 v1 = _mm_setr_ps(t->vertices[1].pos.x, t->vertices[1].pos.y, t->vertices[1].pos.z, 0.0f);
        __m128 v2 = _mm_setr_ps(t->vertices[2].pos.x, t->vertices[2].pos.y, t->vertices[2].pos.z, 0.0f);

        __m128 tbb_min = _mm_min_ps(_mm_min_ps(v0, v1), v2);
        __m128 tbb_max = _mm_max_ps(_mm_max_ps(v0, v1), v2);

        __m128 tc = _mm_mul_ps(_mm_add_ps(_mm_add_ps(v0, v1), v2), third);

        vb_min = _mm_min_ps(vb_min, tbb_min);
        vb_max = _mm_max_ps(vb_max, tbb_max);

        cb_min = _mm_min_ps(cb_min, tc);
        cb_max = _mm_max_ps(cb_max, tc);

        tbs[i].min = tbb_min;
        tbs[i].max = tbb_max;

        _mm_store_ps(f4_dto_1, tc);
        tcs[i] = Point3(f4_dto_1[0], f4_dto_1[1], f4_dto_1[2]);
    }
    _mm_store_ps(f4_dto_1, vb_min);
    _mm_store_ps(f4_dto_2, vb_max);
    AABB vb({f4_dto_1[0], f4_dto_1[1], f4_dto_1[2]}, {f4_dto_2[0], f4_dto_2[1], f4_dto_2[2]});
    _mm_store_ps(f4_dto_1, cb_min);
    _mm_store_ps(f4_dto_2, cb_max);
    AABB cb({f4_dto_1[0], f4_dto_1[1], f4_dto_1[2]}, {f4_dto_2[0], f4_dto_2[1], f4_dto_2[2]});

    std::cout << "  Init time: " << init_t.elapsed_ms() << " ms" << std::endl;

    // Create root and launch recursive subdivide
    BvhNode& root = nodes_[0];
    root.t_begin = 0;
    root.aabb_min = vb.min;
    root.aabb_max = vb.max;
    root.t_count = n;
    next_bvh_node_idx_ = 1;

    subdivide(0, 0, cb, tcs, tbs);

    std::cout << "  Bins time: " << bins_duration_ / 1000000.0 << " ms" << std::endl;
    std::cout << "Vectorized2 BVH building time: " << build_t.elapsed_ms() << " ms" << std::endl;
}

std::vector<std::shared_ptr<const Triangle>> BvhVec2::Search(const Ray& r, bool first_hit) const {
    search_count_++;
    size_t search_nodes_visited = 0;
    size_t search_leaves_visited = 0;
    Timer t_search("search_time");

    std::vector<std::shared_ptr<const Triangle>> result;
    result.reserve(max_triangles_per_BB_);

    std::stack<std::pair<size_t, size_t>> s;
    s.emplace(0, 0);
    while (!s.empty()) {
        auto [node_idx, depth] = s.top();
        s.pop();

        const BvhNode& node = nodes_[node_idx];
        search_nodes_visited++;

        if (node.is_leaf()) {
            search_leaves_visited++;
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

    long duration = static_cast<long>(t_search.elapsed_ns());
    search_time_ += duration;
    search_min_time_ = std::min(search_min_time_, duration);
    search_max_time_ = std::max(search_max_time_, duration);

    search_min_nodes_visited_ = std::min(search_min_nodes_visited_, search_nodes_visited);
    search_max_nodes_visited_ = std::max(search_max_nodes_visited_, search_nodes_visited);
    search_nodes_visited_ += search_nodes_visited;

    search_min_leaves_visited_ = std::min(search_min_leaves_visited_, search_leaves_visited);
    search_max_leaves_visited_ = std::max(search_max_leaves_visited_, search_leaves_visited);
    search_leaves_visited_ += search_leaves_visited;

    search_min_return_count_ = std::min(search_min_return_count_, result.size());
    search_max_return_count_ = std::max(search_max_return_count_, result.size());
    search_return_count_ += result.size();

    return result;
}

void BvhVec2::PrintStats(std::ostream& os) const {
    auto stats = calculate_stats();
    os << "BVH Build stats: " << "\n";
    os << "  Nodes count: " << stats.nodes_count << "\n";
    os << "  Leaf count: " << stats.leaf_nodes_count << "\n";
    os << "  Leaf depth:\n";
    os << "   - Min depth: " << stats.min_depth << "\n";
    os << "   - Max depth: " << stats.max_depth << "\n";
    os << "   - Avg depth: " << stats.avg_depth << "\n";
    os << "  Leaf tris:\n";
    os << "   - Min: " << stats.min_triangles_in_leaf_nodes << "\n";
    os << "   - Max: " << stats.max_triangles_in_leaf_nodes << "\n";
    os << "   - Avg: " << stats.avg_triangles_in_leaf_nodes << "\n\n";
    os << "BVH Search stats:\n";
    os << " - Tatal calls: " << stats.search_count << "\n";
    os << "  Nodes visited (ray-aabb tests):\n";
    os << "   - Min: " << stats.search_min_nodes_visited << "\n";
    os << "   - Max: " << stats.search_max_nodes_visited << "\n";
    os << "   - Avg: " << (float)stats.search_nodes_visited / stats.search_count << "\n";
    os << "   - Total: " << stats.search_nodes_visited << "\n";
    os << "  Leaves visited:\n";
    os << "   - Min: " << stats.search_min_leaves_visited << "\n";
    os << "   - Max: " << stats.search_max_leaves_visited << "\n";
    os << "   - Avg: " << (float)stats.search_leaves_visited / stats.search_count << "\n";
    os << "   - Total: " << stats.search_leaves_visited << "\n";
    os << "  Time:\n";
    os << "   - Min: " << stats.search_min_time / 1000.0f << " µs\n";
    os << "   - Max: " << stats.search_max_time / 1000.0f << " µs\n";
    os << "   - Avg: " << (float)stats.search_time / stats.search_count / 1000.0f << " µs\n";
    os << "   - Total time: " << stats.search_time / 1000000000.0f << " s" << "\n";
    os << "  Tris returned (ray-tri tests):\n";
    os << "   - Min: " << stats.search_min_return_count << "\n";
    os << "   - Max: " << stats.search_max_return_count << "\n";
    os << "   - Avg: " << (float)stats.search_return_count / stats.search_count << "\n";
    os << "   - Total tris returned: " << stats.search_return_count << "\n\n";
}

float BvhVec2::find_best_split(BvhNode& node, int& axis, float& split_pos, const AABB& cb, AABB& TB_L, AABB& TB_R,
                               const std::vector<Point3>& tcs, const std::vector<vAABB>& tbs) {
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
    std::vector<AABB> bbs(bin_count_);
    __m128 inf4 = _mm_setr_ps(infinity, infinity, infinity, infinity);
    __m128 neg_inf4 = _mm_setr_ps(-infinity, -infinity, -infinity, -infinity);
    std::vector<__m128> bbs_min(bin_count_, inf4);
    std::vector<__m128> bbs_max(bin_count_, neg_inf4);
    std::vector<size_t> ns(bin_count_, 0);

    float k_0 = cb.min[axis];
    float k_1 = bin_count_ * (1 - 1e-3) /
                cb.size[axis];  // Possible problem with bin_idx being K (bin_count) - epsilon was too big
    for (size_t t = node.t_begin; t < node.t_begin + node.t_count; t++) {
        size_t t_idx = tri_idxs_[t];
        size_t bin_idx = std::min(bin_count_ - 1, static_cast<int>(k_1 * (tcs[t_idx][axis] - k_0)));

        bbs_min[bin_idx] = _mm_min_ps(bbs_min[bin_idx], tbs[t_idx].min);
        bbs_max[bin_idx] = _mm_max_ps(bbs_max[bin_idx], tbs[t_idx].max);
        ns[bin_idx]++;
    }
    alignas(16) float f4_dto_1[4];
    alignas(16) float f4_dto_2[4];
    for (int b = 0; b < bin_count_; b++) {
        _mm_store_ps(f4_dto_1, bbs_min[b]);
        _mm_store_ps(f4_dto_2, bbs_max[b]);
        bbs[b] = {{f4_dto_1[0], f4_dto_1[1], f4_dto_1[2]}, {f4_dto_2[0], f4_dto_2[1], f4_dto_2[2]}};
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

void BvhVec2::subdivide(size_t node_idx, int depth, AABB cb, const std::vector<Point3>& tcs,
                        const std::vector<vAABB>& tbs) {
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
    float split_cost = find_best_split(node, axis, split_pos, cb, TB_L, TB_R, tcs, tbs);

    // Triangle partitioning
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

    // Subdivide recursively
    subdivide(left_child_idx, depth + 1, CB_L, tcs, tbs);
    subdivide(right_child_idx, depth + 1, CB_R, tcs, tbs);
}

BvhVec2::BvhStats BvhVec2::calculate_stats() const {
    BvhStats stats;
    stats.max_depth = 0;
    stats.min_depth = std::numeric_limits<size_t>::max();
    int total_leaf_depth = 0;
    stats.min_triangles_in_leaf_nodes = std::numeric_limits<size_t>::max();
    stats.max_triangles_in_leaf_nodes = 0;
    int total_triangles_in_leaf_nodes = 0;
    stats.nodes_count = 0;
    stats.leaf_nodes_count = 0;

    std::stack<std::pair<size_t, size_t>> s;
    s.emplace(0, 0);
    while (!s.empty()) {
        auto [node_idx, depth] = s.top();
        s.pop();

        const BvhNode& node = nodes_[node_idx];
        stats.nodes_count++;

        // size_t left_child_idx = node.is_leaf() ? 0 : node.t_begin;
        // size_t right_child_idx = node.is_leaf() ? 0 : node.t_begin + 1;
        // std::cout << "Node: " << node_idx << ", Depth: " << depth << ", T_count: " << node.t_count
        //           << ", Left: " << left_child_idx << ", Right: " << right_child_idx << std::endl;

        if (node.is_leaf()) {
            stats.leaf_nodes_count++;

            stats.min_depth = std::min(stats.min_depth, depth);
            stats.max_depth = std::max(stats.max_depth, depth);
            total_leaf_depth += depth;

            stats.min_triangles_in_leaf_nodes = std::min(stats.min_triangles_in_leaf_nodes, node.t_count);
            stats.max_triangles_in_leaf_nodes = std::max(stats.max_triangles_in_leaf_nodes, node.t_count);
            total_triangles_in_leaf_nodes += node.t_count;

            continue;
        }

        size_t left_child_idx = node.t_begin;
        size_t right_child_idx = left_child_idx + 1;
        s.emplace(left_child_idx, depth + 1);
        s.emplace(right_child_idx, depth + 1);
    }

    stats.avg_depth = (float)total_leaf_depth / stats.leaf_nodes_count;
    stats.avg_triangles_in_leaf_nodes = (float)total_triangles_in_leaf_nodes / stats.leaf_nodes_count;

    stats.search_count = search_count_;
    stats.search_nodes_visited = search_nodes_visited_;
    stats.search_min_nodes_visited = search_min_nodes_visited_;
    stats.search_max_nodes_visited = search_max_nodes_visited_;

    stats.search_min_return_count = search_min_return_count_;
    stats.search_max_return_count = search_max_return_count_;
    stats.search_return_count = search_return_count_;

    stats.search_min_time = search_min_time_;
    stats.search_max_time = search_max_time_;
    stats.search_time = search_time_;

    stats.search_min_leaves_visited = search_min_leaves_visited_;
    stats.search_max_leaves_visited = search_max_leaves_visited_;
    stats.search_leaves_visited = search_leaves_visited_;

    return stats;
}

void BvhVec2::reset_stats() const {
    search_count_ = 0;
    search_nodes_visited_ = 0;
    search_min_nodes_visited_ = std::numeric_limits<size_t>::max();
    search_max_nodes_visited_ = 0;
    search_min_return_count_ = std::numeric_limits<size_t>::max();
    search_max_return_count_ = 0;
    search_return_count_ = 0;
    search_min_time_ = std::numeric_limits<long long>::max();
    search_max_time_ = 0;
    search_time_ = 0;
    search_min_leaves_visited_ = std::numeric_limits<size_t>::max();
    search_max_leaves_visited_ = 0;
    search_leaves_visited_ = 0;
}

void BvhVec2::config_setup(const nlohmann::json& config) {
    std::clog << "Configuring vectorized2 BVH..." << std::flush;

    max_triangles_per_BB_ = config.at("max_triangles_per_BB");
    max_depth_ = config.at("max_depth");
    bin_count_ = config.at("bin_count");

    std::clog << "\rVectorized2 BVH configured     " << std::endl;
}

}  // namespace raytracer
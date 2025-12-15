#include "src/ADS/BVH/BVHSeq/bvh_seq.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <stack>

#include "src/aabb.h"
#include "src/collision_detection.h"

namespace raytracer {

namespace {

/*  Triangle to bin separation
    k - binning (splitting) axis
    K - number of bins
    1-epsilon - floating point error for bounds of cb
    binID_i = (K * (1 - epsilon) * (c_i_k - cb_min_k)) / (cb_max_k - cb_min_k)
    k_0 = cb_min_k
    k_1 = (K * (1 - epsilon)) / (cb_max_k - cb_min_k)
    binID_i = k_1 * (c_i_k - k_0)
*/
std::vector<size_t> calculate_bin_ids(const AABB& cb, const std::vector<size_t>& triangle_indices,
                                      const std::vector<Point3>& cs, size_t K, size_t t_begin, size_t t_count) {
    int k = 0;
    if (cb.size.x >= cb.size.y && cb.size.x >= cb.size.z) {
        k = 0;
    } else if (cb.size.y >= cb.size.z) {
        k = 1;
    } else {
        k = 2;
    }

    std::vector<size_t> binIDs;
    binIDs.reserve(t_count);

    if (cb.size[k] < epsilon) {
        std::fill_n(std::back_inserter(binIDs), t_count, 0);
        return binIDs;
    }

    float k_0 = cb.min[k];
    float k_1 = K * (1 - 1e-3) / cb.size[k];  // Possible problem with bin_idx being K - epsilon was too big

    for (size_t i = 0; i < t_count; ++i) {
        size_t tri_idx = triangle_indices[i + t_begin];
        size_t bin_idx = static_cast<size_t>(k_1 * (cs[tri_idx][k] - k_0));
        binIDs.emplace_back(bin_idx);
    }

    return binIDs;
}

/*  Bin setup
    generate K bins of equal width on the widest axis - domains
    n_i - number of primitives in each bin B_i
    bb_i - bin bounds - aabb of primitives inside bin B_i
    N_L_j - number of prmitives on the left side of the split j
    N_R_j - number of prmitives on the left side of the split j
    TB_L_j - triangle bounding box on the left side of the split j
    TB_R_j - triangle bounding box on the right side of the split j
    A_L_j - sourface area of the bb_i of all B_i on the left side of the split j
    A_R_j - sourface area of the bb_i of all B_i on the right side of the split j
    cost_j - const of split j - A_L_j * N_L_j + A_R_j * N_R_j
*/
size_t calculate_best_split_and_bins(size_t& N_L, size_t& N_R, AABB& TB_L, AABB& TB_R, const std::vector<AABB>& tbs,
                                     const std::vector<size_t>& triangle_indices, const std::vector<size_t>& binIDs,
                                     size_t K, size_t t_begin, size_t t_count) {
    std::vector<size_t> ns(K, 0);
    std::vector<AABB> bbs(K, AABB(infinity, -infinity));
    for (size_t i = 0; i < t_count; i++) {
        ns[binIDs[i]]++;
        size_t tri_idx = triangle_indices[i + t_begin];
        bbs[binIDs[i]].expand(tbs[tri_idx]);
    }

    size_t split_count = K - 1;
    std::vector<size_t> N_Ls(split_count, 0);
    std::vector<size_t> N_Rs(split_count, 0);
    std::vector<AABB> TB_Ls(split_count, AABB(infinity, -infinity));
    std::vector<AABB> TB_Rs(split_count, AABB(infinity, -infinity));
    std::vector<float> A_Ls(split_count, 0.0f);
    std::vector<float> A_Rs(split_count, 0.0f);

    // Calculate left sides triangle counts, AABBs and surface areas
    N_Ls[0] = ns[0];
    TB_Ls[0] = bbs[0];
    A_Ls[0] = TB_Ls[0].surface_area();

    for (size_t i = 1; i < split_count; i++) {
        N_Ls[i] = N_Ls[i - 1] + ns[i];
        TB_Ls[i] = TB_Ls[i - 1];
        TB_Ls[i].expand(bbs[i]);
        A_Ls[i] = TB_Ls[i].surface_area();
    }

    // Calculate right sides triangle counts, AABBs, surface areas and best split
    N_Rs[split_count - 1] = ns[K - 1];
    TB_Rs[split_count - 1] = bbs[K - 1];
    A_Rs[split_count - 1] = TB_Rs[split_count - 1].surface_area();
    float best_split_cost =
        (N_Ls[split_count - 1] != 0 && N_Rs[split_count - 1] != 0)
            ? A_Ls[split_count - 1] * N_Ls[split_count - 1] + A_Rs[split_count - 1] * N_Rs[split_count - 1]
            : infinity;
    size_t best_split = split_count - 1;

    for (int i = split_count - 2; i >= 0; i--) {
        N_Rs[i] = N_Rs[i + 1] + ns[i + 1];
        TB_Rs[i] = TB_Rs[i + 1];
        TB_Rs[i].expand(bbs[i + 1]);
        A_Rs[i] = TB_Rs[i].surface_area();

        float split_cost = (N_Ls[i] != 0 && N_Rs[i] != 0) ? A_Ls[i] * N_Ls[i] + A_Rs[i] * N_Rs[i] : infinity;
        if (split_cost < best_split_cost) {
            best_split = i;
            best_split_cost = split_cost;
        }
    }

    N_L = N_Ls[best_split];
    N_R = N_Rs[best_split];
    TB_L = TB_Ls[best_split];
    TB_R = TB_Rs[best_split];

    return best_split;
}

}  // namespace

BvhSeq::BvhSeq(const nlohmann::json& config) : Ads(config) {
    config_setup(config);
    reset_stats();
}

void BvhSeq::Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) {
    Ads::Build(triangles);
    reset_stats();
    std::clog << "Building sequential BVH..." << std::flush;
    auto start_time = std::chrono::high_resolution_clock::now();
    int64_t calculate_bin_ids_duration = 0;
    int64_t calculate_best_split_and_bins_duration = 0;
    int64_t rearange_triangles_duration = 0;
    int64_t calculate_centroids_duration = 0;

    if (triangles_.empty()) {
        std::cerr << "No triangles to build a BVH." << std::endl;
        return;
    }

    //  Initial Setup
    size_t n = triangles_.size();
    nodes_.resize(2 * n - 1);
    triangle_indices_.resize(n);
    std::iota(triangle_indices_.begin(), triangle_indices_.end(), 0);

    std::vector<AABB> tbs;  // triangle AABBS
    tbs.reserve(n);
    std::vector<Point3> cs;  // triangle centroids
    cs.reserve(n);
    std::for_each(triangles_.begin(), triangles_.end(), [&](const auto& t) {
        tbs.emplace_back(calculate_triangle_aabb(*t));
        cs.emplace_back(calculate_triangle_centroid(*t));
    });
    AABB vb;  // voxel aabb of the current node (all triangle aabbs)
    std::for_each(tbs.begin(), tbs.end(), [&](const auto& tb) { vb.expand(tb); });
    AABB cb;  // centroid aabb of the currents node (all triangles centroids aabb)
    std::for_each(cs.begin(), cs.end(), [&](const auto& c) { cb.expand(c); });

    auto init_done_time = std::chrono::high_resolution_clock::now();
    auto init_duration = std::chrono::duration_cast<std::chrono::microseconds>(init_done_time - start_time).count();

    struct StackElem {
        size_t node_idx;
        size_t depth;
        AABB vb;
        AABB cb;
        size_t t_begin;
        size_t t_count;
    };

    size_t next_free_node_idx = 0;
    std::stack<StackElem> s;
    s.emplace(next_free_node_idx++, 0, vb, cb, 0, n);

    auto build_loop_start = std::chrono::high_resolution_clock::now();
    while (!s.empty()) {
        auto [node_idx, depth, vb, cb, t_begin, t_count] = s.top();
        s.pop();

        nodes_[node_idx].depth = depth;
        nodes_[node_idx].bounding_box = vb;
        nodes_[node_idx].t_begin = t_begin;
        nodes_[node_idx].t_count = 0;
        // Node becomes a leaf if termination conditions are met -> the recursion stops
        bool cb_too_small = cb.size.x < epsilon && cb.size.y < epsilon && cb.size.z < epsilon;
        if (t_count < max_triangles_per_BB_ || nodes_[node_idx].depth >= max_depth_ || cb_too_small) {
            nodes_[node_idx].t_count = t_count;
            continue;
        }

        auto start_time_tmp = std::chrono::high_resolution_clock::now();
        std::vector<size_t> binIDs = calculate_bin_ids(cb, triangle_indices_, cs, bin_count_, t_begin, t_count);
        auto end_time_tmp = std::chrono::high_resolution_clock::now();
        calculate_bin_ids_duration +=
            std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_tmp - start_time_tmp).count();

        size_t N_L, N_R;  // child triangle counts
        AABB TB_L, TB_R;  // child triangle bounds
        AABB CB_L, CB_R;  // child centroid bounds

        start_time_tmp = std::chrono::high_resolution_clock::now();
        size_t best_split = calculate_best_split_and_bins(N_L, N_R, TB_L, TB_R, tbs, triangle_indices_, binIDs,
                                                          bin_count_, t_begin, t_count);
        end_time_tmp = std::chrono::high_resolution_clock::now();
        calculate_best_split_and_bins_duration +=
            std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_tmp - start_time_tmp).count();

        // Bin comparison print
        // std::cout << "Node " << node_idx << " depth=" << depth << " t_begin=" << t_begin << " t_count=" << t_count
        //           << " best_split=" << best_split << " N_L=" << N_L << " N_R=" << N_R << " binIDs=[";
        // for (size_t i = 0; i < std::min<size_t>(t_count, 10); ++i)  // print first 10 binIDs
        //     std::cout << binIDs[i] << " ";
        // std::cout << "]\n";

        // If the split fails make current node as a leaf
        if (N_L == 0 || N_R == 0) {
            nodes_[node_idx].t_count = t_count;
            continue;
        }

        /*  Rearange triangles using a two-pointer partition
              l - starts at the beginning
              r - starts at the end
            Loop until the pointers cross.
            If l point to a "right" triangle and r point to a "left" triangle -> swap.
        */
        start_time_tmp = std::chrono::high_resolution_clock::now();
        size_t l = t_begin;
        size_t r = t_begin + t_count - 1;
        while (l < r) {
            // Advance l forward until it finds a "right" triangle
            while (l < r && binIDs[l - t_begin] <= best_split) {
                l++;
            }
            // Advance r backward until it finds a "left" triangle
            while (l < r && binIDs[r - t_begin] > best_split) {
                r--;
            }
            if (l < r) {
                std::swap(triangle_indices_[l], triangle_indices_[r]);
                l++;
                r--;
            }
        }
        end_time_tmp = std::chrono::high_resolution_clock::now();
        rearange_triangles_duration +=
            std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_tmp - start_time_tmp).count();

        // Calculate centroid bounds for children.
        start_time_tmp = std::chrono::high_resolution_clock::now();
        for (size_t i = t_begin; i < t_begin + N_L; ++i) {
            size_t tri_idx = triangle_indices_[i];
            CB_L.expand(cs[tri_idx]);
        }
        for (size_t i = t_begin + N_L; i < t_begin + t_count; ++i) {
            size_t tri_idx = triangle_indices_[i];
            CB_R.expand(cs[tri_idx]);
        }
        end_time_tmp = std::chrono::high_resolution_clock::now();
        calculate_centroids_duration +=
            std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_tmp - start_time_tmp).count();

        // Push children
        size_t left_child_idx = next_free_node_idx++;
        size_t right_child_idx = next_free_node_idx++;
        nodes_[node_idx].left_child_idx = left_child_idx;
        nodes_[node_idx].right_child_idx = right_child_idx;
        s.emplace(left_child_idx, depth + 1, TB_L, CB_L, t_begin, N_L);
        s.emplace(right_child_idx, depth + 1, TB_R, CB_R, t_begin + N_L, N_R);
    }

    auto build_loop_end = std::chrono::high_resolution_clock::now();
    auto build_loop_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(build_loop_end - build_loop_start).count();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    std::cout << "Sequential BVH building time: " << duration / 1000.0 << " ms" << std::endl;
    std::cout << "  Init time (precalculate aabbs and centroids): " << init_duration / 1000.0 << " ms" << std::endl;
    std::cout << "  BVH build loop time: " << build_loop_duration / 1000.0 << " ms" << std::endl;

    std::cout << "    Calculate bin ids time: " << calculate_bin_ids_duration / 1000000.0 << " ms" << std::endl;
    std::cout << "    Calculate best split and bins time: " << calculate_best_split_and_bins_duration / 1000000.0
              << " ms" << std::endl;
    std::cout << "    Rearange triangles time: " << rearange_triangles_duration / 1000000.0 << " ms" << std::endl;
    std::cout << "    Calculate centroids time: " << calculate_centroids_duration / 1000000.0 << " ms" << std::endl;
    int64_t loop_overhead =
        (build_loop_duration * 1000.0) - (calculate_bin_ids_duration + calculate_best_split_and_bins_duration +
                                          rearange_triangles_duration + calculate_centroids_duration);
    std::cout << "    Loop & structure node creation overhead: " << loop_overhead / 1000000.0 << " ms" << std::endl;
}

std::vector<std::shared_ptr<const Triangle>> BvhSeq::Search(const Ray& r, bool first_hit) const {
    search_count_++;
    size_t search_nodes_visited = 0;
    size_t search_leaves_visited = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::shared_ptr<const Triangle>> result;
    result.reserve(max_triangles_per_BB_);

    std::stack<size_t> s;
    if (collision_ray_aabb(r, nodes_[0].bounding_box)) {
        s.emplace(0);
    }
    while (!s.empty()) {
        auto node_idx = s.top();
        s.pop();

        search_nodes_visited++;

        // If node is a leaf, add trinagles and stop recursion
        if (nodes_[node_idx].t_count) {
            search_leaves_visited++;
            for (size_t i = 0; i < nodes_[node_idx].t_count; i++) {
                size_t tri_idx = triangle_indices_[nodes_[node_idx].t_begin + i];
                result.emplace_back(triangles_[tri_idx]);
            }
            continue;
        }

        //.emplace children
        size_t left_child_idx = nodes_[node_idx].left_child_idx;
        size_t right_child_idx = nodes_[node_idx].right_child_idx;
        if (collision_ray_aabb(r, nodes_[left_child_idx].bounding_box)) {
            s.emplace(left_child_idx);
        }
        if (collision_ray_aabb(r, nodes_[right_child_idx].bounding_box)) {
            s.emplace(right_child_idx);
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
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

void BvhSeq::PrintStats(std::ostream& os) const {
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

BvhSeq::BvhSeqStats BvhSeq::calculate_stats() const {
    BvhSeqStats stats;
    stats.max_depth = 0;
    stats.min_depth = std::numeric_limits<size_t>::max();
    int total_leaf_depth = 0;
    stats.min_triangles_in_leaf_nodes = std::numeric_limits<size_t>::max();
    stats.max_triangles_in_leaf_nodes = 0;
    int total_triangles_in_leaf_nodes = 0;
    stats.nodes_count = 0;
    stats.leaf_nodes_count = 0;

    std::stack<size_t> s;
    s.push(0);
    while (!s.empty()) {
        auto node_idx = s.top();
        s.pop();

        stats.nodes_count++;

        // If node is a leaf, add trinagles and stop recursion
        if (nodes_[node_idx].t_count) {
            stats.leaf_nodes_count++;

            stats.min_depth = std::min(stats.min_depth, nodes_[node_idx].depth);
            stats.max_depth = std::max(stats.max_depth, nodes_[node_idx].depth);
            total_leaf_depth += nodes_[node_idx].depth;

            stats.min_triangles_in_leaf_nodes = std::min(stats.min_triangles_in_leaf_nodes, nodes_[node_idx].t_count);
            stats.max_triangles_in_leaf_nodes = std::max(stats.max_triangles_in_leaf_nodes, nodes_[node_idx].t_count);
            total_triangles_in_leaf_nodes += nodes_[node_idx].t_count;
            continue;
        }

        // Push children
        size_t left_child_idx = nodes_[node_idx].left_child_idx;
        size_t right_child_idx = nodes_[node_idx].right_child_idx;
        if (left_child_idx) {
            s.push(left_child_idx);
        }
        if (right_child_idx) {
            s.push(right_child_idx);
        }
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

void BvhSeq::reset_stats() const {
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

void BvhSeq::config_setup(const nlohmann::json& config) {
    std::clog << "Configuring sequential BVH..." << std::flush;

    max_triangles_per_BB_ = config.at("max_triangles_per_BB");
    max_depth_ = config.at("max_depth");
    bin_count_ = config.at("bin_count");

    std::clog << "\rSequential BVH configured     " << std::endl;
}

}  // namespace raytracer
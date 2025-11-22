#include "src/ADS/BVH/BVHNaive/bvh_naive.h"

#include <chrono>
#include <iostream>
#include <limits>
#include <stack>

#include "src/aabb.h"
#include "src/collision_detection.h"

namespace raytracer {

BvhNaive::BvhNaive(const nlohmann::json& config) : Ads(config) {
    config_setup(config);
    reset_stats();
}

void BvhNaive::Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) {
    Ads::Build(triangles);
    reset_stats();
    std::clog << "Building naive BVH..." << std::flush;
    auto start_time = std::chrono::high_resolution_clock::now();

    if (triangles_.empty()) {
        std::cerr << "No triangles to build a BVH." << std::endl;
        return;
    }

    // Precalculate triangle centroids and AABBs into an array
    std::vector<Point3> t_centroids;
    t_centroids.resize(triangles_.size());
    std::vector<AABB> t_aabbs;
    t_aabbs.resize(triangles_.size());
    for (size_t t_idx = 0; t_idx < triangles_.size(); t_idx++) {
        t_centroids[t_idx] = calculate_triangle_centroid(*triangles_[t_idx]);
        t_aabbs[t_idx] = calculate_triangle_aabb(*triangles_[t_idx]);
    }

    // Create the root BVH node (add triangles and creat an AABB)
    root_ = std::make_shared<BvhNode>();
    root_->triangle_indices.reserve(triangles_.size());
    root_->depth = 0;
    root_->isLeaf = false;
    for (size_t t_idx = 0; t_idx < triangles_.size(); t_idx++) {
        root_->bounding_box.expand(t_aabbs[t_idx]);
        root_->triangle_indices.emplace_back(t_idx);
    }

    // Recursively (stack implementation) divide the current node in to two - naively
    std::stack<std::shared_ptr<BvhNode>> s;
    s.push(root_);
    while (!s.empty()) {
        std::shared_ptr<BvhNode> curr_node = s.top();
        s.pop();

        // Check depth and triangle count
        if (curr_node->depth >= max_depth_ || curr_node->triangle_indices.size() < max_triangles_per_BB_) {
            curr_node->isLeaf = true;
            continue;
        }

        // Create children
        curr_node->left = std::make_shared<BvhNode>();
        curr_node->right = std::make_shared<BvhNode>();
        curr_node->left->depth = curr_node->depth + 1;
        curr_node->right->depth = curr_node->depth + 1;
        curr_node->left->isLeaf = false;
        curr_node->right->isLeaf = false;

        // Find the median to be used for splitting along the longest axis
        Vec3 aabb_size = curr_node->bounding_box.max - curr_node->bounding_box.min;
        int splitting_axis;
        if (aabb_size.x >= aabb_size.y && aabb_size.x >= aabb_size.z) {
            splitting_axis = 0;
        } else if (aabb_size.y >= aabb_size.z) {
            splitting_axis = 1;
        } else {
            splitting_axis = 2;
        }
        auto& t_idxs = curr_node->triangle_indices;
        size_t mid = t_idxs.size() / 2;
        std::nth_element(t_idxs.begin(), t_idxs.begin() + mid, t_idxs.end(), [&](size_t t_idx_a, size_t t_idx_b) {
            if (splitting_axis == 0) return t_centroids[t_idx_a].x < t_centroids[t_idx_b].x;
            if (splitting_axis == 1) return t_centroids[t_idx_a].y < t_centroids[t_idx_b].y;
            return t_centroids[t_idx_a].z < t_centroids[t_idx_b].z;
        });

        // Split along the split point
        curr_node->left->triangle_indices.assign(t_idxs.begin(), t_idxs.begin() + mid);
        curr_node->right->triangle_indices.assign(t_idxs.begin() + mid, t_idxs.end());
        curr_node->left->bounding_box = AABB();
        for (auto t_idx : curr_node->left->triangle_indices) {
            curr_node->left->bounding_box.expand(t_aabbs[t_idx]);
        }
        curr_node->right->bounding_box = AABB();
        for (auto t_idx : curr_node->right->triangle_indices) {
            curr_node->right->bounding_box.expand(t_aabbs[t_idx]);
        }

        // If the split failes restore current node as a leaf
        if (curr_node->left->triangle_indices.empty() || curr_node->right->triangle_indices.empty()) {
            curr_node->isLeaf = true;
            curr_node->left.reset();
            curr_node->right.reset();
            // std::cerr << "Bad split of a BVH node, (atleast) one node is empty!" << std::endl;
            continue;
        }

        // If children are not empty, they are added to the stack to be processed
        if (!curr_node->left->triangle_indices.empty()) {
            s.push(curr_node->left);
        } else {
            curr_node->left.reset();
        }
        if (!curr_node->right->triangle_indices.empty()) {
            s.push(curr_node->right);
        } else {
            curr_node->right.reset();
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    std::clog << "\rNaive BVH building time: " << duration / 1000.0 << " ms" << std::endl;
}

std::vector<std::shared_ptr<const Triangle>> BvhNaive::Search(const Ray& r, bool first_hit) const {
    search_count_++;
    size_t search_nodes_visited = 0;
    size_t search_leaves_visited = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::shared_ptr<const Triangle>> result;
    result.reserve(max_triangles_per_BB_);

    std::stack<std::shared_ptr<BvhNode>> s;
    if (collision_ray_aabb(r, root_->bounding_box)) {
        s.push(root_);
    }

    while (!s.empty()) {
        std::shared_ptr<BvhNode> curr_node = s.top();
        s.pop();

        search_nodes_visited++;

        // If current node is a leaf, add its triangles to the result
        if (curr_node->isLeaf) {
            search_leaves_visited++;
            for (const auto& t_idx : curr_node->triangle_indices) {
                result.emplace_back(triangles_[t_idx]);
            }
            continue;
        }

        // Add non empty children to the stack to be processed
        if (curr_node->left && collision_ray_aabb(r, curr_node->left->bounding_box)) {
            s.push(curr_node->left);
        }
        if (curr_node->right && collision_ray_aabb(r, curr_node->right->bounding_box)) {
            s.push(curr_node->right);
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

void BvhNaive::PrintStats(std::ostream& os) const {
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
    os << "   - Avg: " << stats.avg_traiangles_in_leaf_nodes << "\n\n";
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

BvhNaive::BvhNaiveStats BvhNaive::calculate_stats() const {
    BvhNaiveStats stats;
    stats.max_depth = 0;
    int total_leaf_depth = 0;
    stats.max_triangles_in_leaf_nodes = 0;
    int total_triangles_in_leaf_nodes = 0;
    stats.nodes_count = 0;
    stats.leaf_nodes_count = 0;

    std::stack<std::shared_ptr<BvhNode>> s;
    s.push(root_);

    while (!s.empty()) {
        std::shared_ptr<BvhNode> curr_node = s.top();
        s.pop();

        stats.nodes_count++;

        if (curr_node->isLeaf) {
            stats.leaf_nodes_count++;

            stats.min_depth = std::min(stats.max_depth, curr_node->depth);
            stats.max_depth = std::max(stats.max_depth, curr_node->depth);
            total_leaf_depth += curr_node->depth;

            stats.min_triangles_in_leaf_nodes =
                std::min(stats.min_triangles_in_leaf_nodes, curr_node->triangle_indices.size());
            stats.max_triangles_in_leaf_nodes =
                std::max(stats.max_triangles_in_leaf_nodes, curr_node->triangle_indices.size());
            total_triangles_in_leaf_nodes += curr_node->triangle_indices.size();

            continue;
        }

        // Add non empty children to the stack to be processed
        if (curr_node->left) {
            s.push(curr_node->left);
        }
        if (curr_node->right) {
            s.push(curr_node->right);
        }
    }

    stats.avg_depth = (float)total_leaf_depth / stats.leaf_nodes_count;
    stats.avg_traiangles_in_leaf_nodes = (float)total_triangles_in_leaf_nodes / stats.leaf_nodes_count;

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

void BvhNaive::reset_stats() const {
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

void BvhNaive::config_setup(const nlohmann::json& config) {
    std::clog << "Configuring naive BVH..." << std::flush;

    max_triangles_per_BB_ = config.at("max_triangles_per_BB");
    max_depth_ = config.at("max_depth");

    std::clog << "\rNaive BVH configured     " << std::endl;
}

}  // namespace raytracer
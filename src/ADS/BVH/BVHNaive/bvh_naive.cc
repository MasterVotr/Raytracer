#include "src/ADS/BVH/BVHNaive/bvh_naive.h"

#include <iostream>

#include "src/aabb.h"
#include "src/collision_detection.h"

namespace raytracer {

BvhNaive::BvhNaive(const nlohmann::json& config) : Ads(config) { config_setup(config); }

void BvhNaive::Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) {
    Ads::Build(triangles);
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
    root_ = std::make_shared<BvhNode>(new BvhNode);
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
        if (curr_node->depth >= max_depth_ || curr_node->triangle_indices.size() >= max_triangles_per_BB_) {
            curr_node->isLeaf = true;
            continue;
        }

        // Create children
        curr_node->left = std::make_shared<BvhNode>(new BvhNode);
        curr_node->right = std::make_shared<BvhNode>(new BvhNode);
        curr_node->left->depth = curr_node->depth + 1;
        curr_node->right->depth = curr_node->depth + 1;
        curr_node->left->isLeaf = false;
        curr_node->right->isLeaf = false;

        // Find the split point along the longest axis (naive approach)
        Vec3 aabb_size = curr_node->bounding_box.max - curr_node->bounding_box.min;
        Point3 half_point = curr_node->bounding_box.max;
        if (aabb_size.x > aabb_size.y) {
            if (aabb_size.x > aabb_size.z) {
                half_point.x -= aabb_size.x;
            } else {
                half_point.z -= aabb_size.z;
            }
        } else {
            if (aabb_size.y > aabb_size.z) {
                half_point.y -= aabb_size.y;
            } else {
                half_point.z -= aabb_size.z;
            }
        }
        AABB left_half = {curr_node->bounding_box.min, half_point};
        AABB right_half = {half_point, curr_node->bounding_box.max};

        // Split along the split point
        for (auto t_idx : curr_node->triangle_indices) {
            if (collision_point_aabb(t_centroids[t_idx], left_half) <= 0) {
                curr_node->left->triangle_indices.emplace_back(t_idx);
                curr_node->left->bounding_box.expand(t_aabbs[t_idx]);
            } else {
                curr_node->right->triangle_indices.emplace_back(t_idx);
                curr_node->right->bounding_box.expand(t_aabbs[t_idx]);
            }
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
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::clog << "Naive BVH building time: " << duration / 1000.0 << " seconds" << std::endl;
}

std::vector<std::shared_ptr<const Triangle>> BvhNaive::Search(const Ray& r, bool first_hit) const {
    // Recursively check if the BVH node collides with the given ray
    // Current BVH node is a leaf -> return triangles
    // Current BVH node is internal -> add children to the stack if they are colliding with the ray
    // If first_hit -> end on first leaf, othewise continue the whole DFS search
    search_count_++;
    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::shared_ptr<const Triangle>> result;
    result.reserve(max_triangles_per_BB_);

    std::stack<std::shared_ptr<BvhNode>> s;
    s.push(root_);

    while (!s.empty()) {
        std::shared_ptr<BvhNode> curr_node = s.top();
        s.pop();

        search_node_count_++;

        // If current node is a leaf, add its triangles to the result
        if (curr_node->isLeaf) {
            search_leaves_visited_++;
            for (const auto& t_idx : curr_node->triangle_indices) {
                result.emplace_back(triangles_[t_idx]);
            }
            // if (first_hit) { break; } TODO: test if this makes sense
            continue;
        }

        // Add non empty children to the stack to be processed
        if (curr_node->left && collision_ray_aabb(r, curr_node->left->bounding_box)) {
            s.push(curr_node->left);
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    search_time_ += duration;

    search_return_count_ += result.size();
    return result;
}

void BvhNaive::PrintStats(std::ostream& os) const {
    auto stats = calculate_stats();
    os << "BVH stats: " << "\n";
    os << "  Max depth: " << stats.max_depth << "\n";
    os << "  Nodes count: " << stats.nodes_count << "\n";
    os << "  Leaf nodes count: " << stats.leaf_nodes_count << "\n";
    os << "  Average depth of leaf nodes: " << stats.avg_depth << "\n";
    os << "  Max triangles in leaf nodes: " << stats.max_triangles_in_leaf_nodes << "\n";
    os << "  Average triangles in leaf nodes: " << stats.avg_traiangles_in_leaf_nodes << "\n";
    os << "  Search method call count: " << stats.search_count << "\n";
    os << "  Search node count: " << stats.search_node_count << "\n";
    os << "  Search time: " << stats.search_time / 1000000000.0f << " s" << "\n";
    os << "  Search return count: " << stats.search_return_count << "\n";
    os << "  Average search return count: " << (float)stats.search_return_count / stats.search_count << "\n";
    os << "  Search leaves visited: " << stats.search_leaves_visited << "\n";
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
            stats.max_depth = std::max(stats.max_depth, curr_node->depth);
            stats.max_triangles_in_leaf_nodes =
                std::max(stats.max_triangles_in_leaf_nodes, curr_node->triangle_indices.size());
            total_leaf_depth += curr_node->depth;
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
    stats.search_node_count = search_node_count_;
    stats.search_time = search_time_;
    stats.search_return_count = search_return_count_;
    stats.search_leaves_visited = search_leaves_visited_;

    return stats;
}

void BvhNaive::config_setup(const nlohmann::json& config) {
    std::clog << "Configuring naive BVH..." << std::flush;

    max_triangles_per_BB_ = config.at("max_triangles_per_BB");
    max_depth_ = config.at("max_depth");

    std::clog << "\rNaive BVH configured     " << std::endl;
}

}  // namespace raytracer
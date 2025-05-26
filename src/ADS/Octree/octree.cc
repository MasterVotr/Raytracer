#include "src/ADS/Octree/octree.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <ostream>
#include <queue>
#include <set>
#include <stdexcept>
#include <vector>

#include "include/json.hpp"

#include "src/ADS/ads.h"
#include "src/aabb.h"
#include "src/collision_detection.h"
#include "src/ray.h"
#include "src/triangle.h"
#include "src/vec3.h"

namespace raytracer {

void Octree::Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) {
    Ads::Build(triangles);
    std::clog << "Building octree..." << std::flush;
    auto start_time = std::chrono::high_resolution_clock::now();

    if (triangles_.empty()) {
        std::cerr << "No triangles to build an octree." << std::endl;
        return;
    }

    // Inflate the AABBs by an epsilon to makeup for float point errors
    float aabb_epsilon = 1e-6f;

    // Calculate min and max for each axis for the whole scene
    Point3 min = triangles_[0]->vertices[0].pos;
    Point3 max = triangles_[0]->vertices[0].pos;
    for (const auto& triangle : triangles_) {
        for (const auto& vertex : triangle->vertices) {
            min.x = vertex.pos.x < min.x ? vertex.pos.x : min.x;
            min.y = vertex.pos.y < min.y ? vertex.pos.y : min.y;
            min.z = vertex.pos.z < min.z ? vertex.pos.z : min.z;
            max.x = vertex.pos.x > max.x ? vertex.pos.x : max.x;
            max.y = vertex.pos.y > max.y ? vertex.pos.y : max.y;
            max.z = vertex.pos.z > max.z ? vertex.pos.z : max.z;
        }
    }

    // Initialize the root node
    root = std::make_shared<OctreeNode>();
    root->bounding_box = AABB(min - abs(min * Vec3(aabb_epsilon)), max + abs(max * Vec3(aabb_epsilon)));
    root->depth = 0;
    root->triangle_indices.reserve(triangles_.size());
    root->isLeaf = false;
    for (size_t i = 0; i < triangles_.size(); ++i) {
        root->triangle_indices.push_back(i);
    }

    if (triangles_.size() <= max_triangles_per_BB_) {
        root->isLeaf = true;
        std::clog << "\rBuilding octree done               " << std::endl;
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::clog << "Octree building time: " << duration / 1000.0 << " seconds" << std::endl;
        return;
    }

    std::queue<std::shared_ptr<OctreeNode>> q;
    q.emplace(root);
    while (!q.empty()) {
        std::shared_ptr<OctreeNode> current_octree_node = q.front();
        q.pop();

        if (current_octree_node->depth == max_depth_) {
            current_octree_node->isLeaf = true;
            continue;
        }

        // Sets for checking if all trinagles were asigned to at least one octane
        std::set<size_t> current_triangle_indices(current_octree_node->triangle_indices.begin(), current_octree_node->triangle_indices.end());
        std::set<size_t> octanes_triangle_indices;

        // Divide the current node into octanse, bitwise each octanes is represented with XYZ: 000, 001, 010, 011, 100, 101, 110, 111
        Vec3 half_size = abs(current_octree_node->bounding_box.max - current_octree_node->bounding_box.min) / 2.0;
        Vec3 min_aabb_epsilon = abs(current_octree_node->bounding_box.min) * aabb_epsilon;
        Vec3 max_aabb_epsilon = abs(current_octree_node->bounding_box.max) * aabb_epsilon;

        // Split if at least one of the octanes has less triangles than me
        bool split = false;
        for (size_t o = 0; o < 8; ++o) {
            current_octree_node->octanes[o] = nullptr;
            std::shared_ptr<OctreeNode> octane = std::make_shared<OctreeNode>();
            octane->depth = current_octree_node->depth + 1;

            // Calculate the aabb
            Point3 octane_min = Point3(current_octree_node->bounding_box.min.x + (half_size.x * ((o & 4) >> 2)),
                                       current_octree_node->bounding_box.min.y + (half_size.y * ((o & 2) >> 1)),
                                       current_octree_node->bounding_box.min.z + (half_size.z * (o & 1)));
            Point3 octane_max = octane_min + half_size;
            octane->bounding_box = AABB(octane_min - min_aabb_epsilon, octane_max + max_aabb_epsilon);

            // Assigning triangles to octane
            for (size_t t = 0; t < current_octree_node->triangle_indices.size(); t++) {
                if (collision_triangle_aabb(*triangles_[current_octree_node->triangle_indices[t]], octane->bounding_box)) {
                    octane->triangle_indices.emplace_back(current_octree_node->triangle_indices[t]);
                    octanes_triangle_indices.emplace(current_octree_node->triangle_indices[t]);
                }
            }

            if (current_octree_node->triangle_indices.size() != octane->triangle_indices.size()) {
                split = true;
            }

            // If the octane has no triangles, skip it
            if (octane->triangle_indices.empty()) {
                continue;
            }

            current_octree_node->octanes[o] = octane;
        }

        if (!split) {
            current_octree_node->isLeaf = true;
            continue;
        }

        if (current_triangle_indices != octanes_triangle_indices) {
            std::cerr << "Error: not all trinagles from parent (" << current_triangle_indices.size() << ") asigned to octanes("
                      << octanes_triangle_indices.size() << ")" << std::endl;
            std::cerr << "Triangles not in octanes:\n" << std::flush;
            for (const auto& index : current_triangle_indices) {
                if (octanes_triangle_indices.find(index) == octanes_triangle_indices.end()) {
                    std::cerr << "\t" << index << "(" << triangles_[index]->vertices[0].pos.x << ", " << triangles_[index]->vertices[0].pos.y << ", "
                              << triangles_[index]->vertices[0].pos.z << "), (" << triangles_[index]->vertices[1].pos.x << ", "
                              << triangles_[index]->vertices[1].pos.y << ", " << triangles_[index]->vertices[1].pos.z << "), ("
                              << triangles_[index]->vertices[2].pos.x << ", " << triangles_[index]->vertices[2].pos.y << ", "
                              << triangles_[index]->vertices[2].pos.z << ")" << std::endl;
                }
            }
            std::cerr << "Parent bounding box: (" << current_octree_node->bounding_box.min.x << ", " << current_octree_node->bounding_box.min.y
                      << ", " << current_octree_node->bounding_box.min.z << ") : (" << current_octree_node->bounding_box.max.x << ", "
                      << current_octree_node->bounding_box.max.y << ", " << current_octree_node->bounding_box.max.z << ")" << std::endl;
            std::cerr << "Octanes bounding boxes:" << std::endl;
            for (size_t o = 0; o < 8; ++o) {
                if (current_octree_node->octanes[o]) {
                    std::cerr << "\t" << o << ": (" << current_octree_node->octanes[o]->bounding_box.min.x << ", "
                              << current_octree_node->octanes[o]->bounding_box.min.y << ", " << current_octree_node->octanes[o]->bounding_box.min.z
                              << ") : (" << current_octree_node->octanes[o]->bounding_box.max.x << ", "
                              << current_octree_node->octanes[o]->bounding_box.max.y << ", " << current_octree_node->octanes[o]->bounding_box.max.z
                              << ")" << std::endl;
                }
            }
            std::cerr << std::endl;
            exit(1);
        }

        for (const auto& octane : current_octree_node->octanes) {
            if (!octane) {
                continue;
            }
            if (octane->triangle_indices.size() > max_triangles_per_BB_) {
                octane->isLeaf = false;
                q.push(octane);
            } else {
                octane->isLeaf = true;
            }
        }
    }

    std::clog << "\rBuilding octree done               " << std::endl;
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::clog << "Octree building time: " << duration / 1000.0f << " s" << std::endl;
}

std::vector<std::shared_ptr<const Triangle>> Octree::Search(const Ray& r, bool first_hit) const {
    search_count_++;
    std::vector<std::shared_ptr<const Triangle>> result;
    std::queue<std::shared_ptr<OctreeNode>> q;
    q.push(root);

    auto start_time = std::chrono::high_resolution_clock::now();

    while (!q.empty()) {
        auto current_node = q.front();
        q.pop();

        search_node_count_++;
        if (current_node->isLeaf) {
            search_leaves_visited_++;
            for (const auto& index : current_node->triangle_indices) {
                result.emplace_back(triangles_[index]);
            }
            continue;
        }

        for (const auto& octane : current_node->octanes) {
            if (octane && !octane->triangle_indices.empty() && collision_ray_aabb(r, octane->bounding_box)) {
                q.push(octane);
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    search_time_ += duration;
    search_return_count_ += result.size();

    return result;
}

void Octree::PrintStats(std::ostream& os) const {
    auto stats = calculate_stats();
    os << "Octree stats: " << "\n";
    os << "  Max depth: " << stats.max_depth << "\n";
    os << "  Nodes count: " << stats.nodes_count << "\n";
    os << "  Leaf nodes count: " << stats.leaf_nodes_count << "\n";
    os << "  Average depth of leaf nodes: " << stats.avg_depth << "\n";
    os << "  Max triangles in leaf nodes: " << stats.triangles_in_leaf_nodes << "\n";
    os << "  Average triangles in leaf nodes: " << stats.avg_traiangles_in_leaf_nodes << "\n";
    os << "  Search method call count: " << stats.search_count << "\n";
    os << "  Search node count: " << stats.search_node_count << "\n";
    os << "  Search time: " << stats.search_time / 1000000000.0f << " s" << "\n";
    os << "  Search return count: " << stats.search_return_count << "\n";
    os << "  Average search return count: " << (float)stats.search_return_count / stats.search_count << "\n";
    os << "  Search leaves visited: " << stats.search_leaves_visited << "\n";
}

Octree::OctreeStats Octree::calculate_stats() const {
    OctreeStats stats;
    stats.max_depth = 0;
    int total_leaf_depth = 0;
    stats.triangles_in_leaf_nodes = 0;
    int total_triangles_in_leaf_nodes = 0;
    stats.nodes_count = 0;
    stats.leaf_nodes_count = 0;

    std::queue<std::shared_ptr<OctreeNode>> q;
    q.push(root);
    while (!q.empty()) {
        auto current_node = q.front();
        q.pop();
        stats.nodes_count++;
        if (current_node->isLeaf) {
            continue;
        }
        for (const auto& octane : current_node->octanes) {
            if (octane) {
                if (octane->isLeaf) {
                    stats.leaf_nodes_count++;
                    stats.max_depth = std::max(stats.max_depth, octane->depth);
                    stats.triangles_in_leaf_nodes = std::max(stats.triangles_in_leaf_nodes, octane->triangle_indices.size());
                    total_leaf_depth += octane->depth;
                    total_triangles_in_leaf_nodes += octane->triangle_indices.size();
                }
                q.push(octane);
            }
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

void Octree::config_setup(const nlohmann::json& config) {
    std::clog << "Configuring octree..." << std::flush;

    max_triangles_per_BB_ = config.at("max_triangles_per_BB");
    max_depth_ = config.at("max_depth");

    std::clog << "\rOctree configured     " << std::endl;
}

}  // namespace raytracer
#pragma once

#include "../include/json.hpp"
#include "ray.h"
#include "triangle.h"
#include "vec3.h"

#include <iostream>
#include <memory>

namespace raytracer {

struct AABB {
    point3 min;
    point3 max;

    AABB() : min(0, 0, 0), max(0, 0, 0) {}
    AABB(const point3& min, const point3& max) : min(min), max(max) {}

    float volume() const { return (max.x - min.x) * (max.y - min.y) * (max.z - min.z); }

    bool collision_aabb_aabb(const AABB& other) const {
        return (min.x <= other.max.x && max.x >= other.min.x) && (min.y <= other.max.y && max.y >= other.min.y) &&
               (min.z <= other.max.z && max.z >= other.min.z);
    }

    // ChatGPT generated function, I understand it, but couldnt implement it mayself
    bool collision_ray_aabb(const ray& r) const {
        float t_min = -infinity;
        float t_max = infinity;

        for (int i = 0; i < 3; ++i) {
            float inv_d = 1.0f / r.direction()[i];
            float t0 = (min[i] - r.origin()[i]) * inv_d;
            float t1 = (max[i] - r.origin()[i]) * inv_d;
            if (inv_d < 0.0f) {
                std::swap(t0, t1);
            }
            t_min = std::max(t_min, t0);
            t_max = std::min(t_max, t1);
        }
        return t_max >= t_min;
    }

    // ChatGPT generated function, I don't understand it very well
    bool collision_triangle_aabb(const Triangle& t) const {
        const vec3 box_center = (min + max) * 0.5f;
        const vec3 box_half_size = (max - min) * 0.5f;

        // Triangle vertices relative to AABB center
        vec3 v0 = t.vertices[0].pos - box_center;
        vec3 v1 = t.vertices[1].pos - box_center;
        vec3 v2 = t.vertices[2].pos - box_center;

        // Triangle edges
        vec3 e0 = v1 - v0;
        vec3 e1 = v2 - v1;
        vec3 e2 = v0 - v2;

        auto axis_test = [&](const vec3& axis, float& r, float& p0, float& p1, float& p2) {
            p0 = dot(v0, axis);
            p1 = dot(v1, axis);
            p2 = dot(v2, axis);
            r = box_half_size.x * std::abs(axis.x) + box_half_size.y * std::abs(axis.y) + box_half_size.z * std::abs(axis.z);
            float min_p = std::min({p0, p1, p2});
            float max_p = std::max({p0, p1, p2});
            return !(min_p > r || max_p < -r);
        };

        // 1. Test axes of the AABB
        for (int i = 0; i < 3; ++i) {
            float min_v = std::min({v0[i], v1[i], v2[i]});
            float max_v = std::max({v0[i], v1[i], v2[i]});
            if (min_v > box_half_size[i] || max_v < -box_half_size[i])
                return false;
        }

        // 2. Test axis perpendicular to triangle face
        vec3 normal = t.normal;
        float p0 = dot(v0, normal);
        float r = box_half_size.x * std::abs(normal.x) + box_half_size.y * std::abs(normal.y) + box_half_size.z * std::abs(normal.z);
        if (std::abs(p0) > r)
            return false;

        // 3. Test 9 cross-product axes
        const vec3 axes[] = {vec3(0, -e0.z, e0.y), vec3(0, -e1.z, e1.y), vec3(0, -e2.z, e2.y), vec3(e0.z, 0, -e0.x), vec3(e1.z, 0, -e1.x),
                             vec3(e2.z, 0, -e2.x), vec3(-e0.y, e0.x, 0), vec3(-e1.y, e1.x, 0), vec3(-e2.y, e2.x, 0)};
        for (const auto& axis : axes) {
            float r_axis, p0, p1, p2;
            if (!axis_test(axis, r_axis, p0, p1, p2))
                return false;
        }

        return true;
    }
};

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

class Octree {
   public:
    Octree(const std::vector<Triangle>& triangles, const nlohmann::json& config) : triangles_(triangles), config_(config) {
        config_setup(config_.at("octree"));
    }

    void Build() {
        if (triangles_.empty()) {
            std::cerr << "No triangles to build an octree." << std::endl;
            return;
        }
        float aabb_epsilon = 1e-6f;

        std::clog << "Building octree..." << std::flush;
        auto start_time = std::chrono::high_resolution_clock::now();

        // Initialize the root node
        point3 min = triangles_[0].vertices[0].pos;
        point3 max = triangles_[0].vertices[0].pos;
        for (const auto& triangle : triangles_) {
            for (const auto& vertex : triangle.vertices) {
                if (vertex.pos.x < min.x)
                    min.x = vertex.pos.x;
                if (vertex.pos.y < min.y)
                    min.y = vertex.pos.y;
                if (vertex.pos.z < min.z)
                    min.z = vertex.pos.z;
                if (vertex.pos.x > max.x)
                    max.x = vertex.pos.x;
                if (vertex.pos.y > max.y)
                    max.y = vertex.pos.y;
                if (vertex.pos.z > max.z)
                    max.z = vertex.pos.z;
            }
        }

        root = std::make_shared<OctreeNode>();
        root->bounding_box = AABB(min - abs(min * vec3(aabb_epsilon)), max + abs(max * vec3(aabb_epsilon)));
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

            std::set<size_t> current_triangle_indices(current_octree_node->triangle_indices.begin(), current_octree_node->triangle_indices.end());
            std::set<size_t> octanes_triangle_indices;
            /*  Split the root node into 8 children
                bottom left back 000
                bottom right back 001
                bottom left front 010
                bottom right front 011
                top left back 100
                top right back 101
                top left front 110
                top right front 111
            */
            vec3 half_size = abs(current_octree_node->bounding_box.max - current_octree_node->bounding_box.min) / 2.0;
            vec3 min_aabb_epsilon = abs(current_octree_node->bounding_box.min) * aabb_epsilon;
            vec3 max_aabb_epsilon = abs(current_octree_node->bounding_box.max) * aabb_epsilon;
            bool split = false;  // I want to split if at least one of my octanes has less triangles than me
            for (size_t o = 0; o < 8; ++o) {
                current_octree_node->octanes[o] = nullptr;
                std::shared_ptr<OctreeNode> octane = std::make_shared<OctreeNode>();
                octane->depth = current_octree_node->depth + 1;

                // Calculate bounding boxes
                point3 octane_min = point3(current_octree_node->bounding_box.min.x + (half_size.x * ((o & 4) >> 2)),
                                           current_octree_node->bounding_box.min.y + (half_size.y * ((o & 2) >> 1)),
                                           current_octree_node->bounding_box.min.z + (half_size.z * (o & 1)));
                point3 octane_max = octane_min + half_size;
                octane->bounding_box = AABB(octane_min - min_aabb_epsilon, octane_max + max_aabb_epsilon);

                // Assigning triangles to octanes
                for (size_t t = 0; t < current_octree_node->triangle_indices.size(); t++) {
                    if (octane->bounding_box.collision_triangle_aabb(triangles_[current_octree_node->triangle_indices[t]])) {
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
                        std::cerr << "\t" << index << "(" << triangles_[index].vertices[0].pos.x << ", " << triangles_[index].vertices[0].pos.y
                                  << ", " << triangles_[index].vertices[0].pos.z << "), (" << triangles_[index].vertices[1].pos.x << ", "
                                  << triangles_[index].vertices[1].pos.y << ", " << triangles_[index].vertices[1].pos.z << "), ("
                                  << triangles_[index].vertices[2].pos.x << ", " << triangles_[index].vertices[2].pos.y << ", "
                                  << triangles_[index].vertices[2].pos.z << ")" << std::endl;
                    }
                }
                std::cerr << "Parent bounding box: (" << current_octree_node->bounding_box.min.x << ", " << current_octree_node->bounding_box.min.y
                          << ", " << current_octree_node->bounding_box.min.z << ") : (" << current_octree_node->bounding_box.max.x << ", "
                          << current_octree_node->bounding_box.max.y << ", " << current_octree_node->bounding_box.max.z << ")" << std::endl;
                std::cerr << "Octanes bounding boxes:" << std::endl;
                for (size_t o = 0; o < 8; ++o) {
                    if (current_octree_node->octanes[o]) {
                        std::cerr << "\t" << o << ": (" << current_octree_node->octanes[o]->bounding_box.min.x << ", "
                                  << current_octree_node->octanes[o]->bounding_box.min.y << ", "
                                  << current_octree_node->octanes[o]->bounding_box.min.z << ") : ("
                                  << current_octree_node->octanes[o]->bounding_box.max.x << ", "
                                  << current_octree_node->octanes[o]->bounding_box.max.y << ", "
                                  << current_octree_node->octanes[o]->bounding_box.max.z << ")" << std::endl;
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

    std::vector<Triangle> Search(const ray& r) {
        search_count_++;
        std::vector<Triangle> result;
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
                if (octane && !octane->triangle_indices.empty() && octane->bounding_box.collision_ray_aabb(r)) {
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

    void PrintStats() const {
        auto stats = calculate_stats();
        std::clog << "Octree stats: " << std::endl;
        std::clog << "  Max depth: " << stats.max_depth << std::endl;
        std::clog << "  Nodes count: " << stats.nodes_count << std::endl;
        std::clog << "  Leaf nodes count: " << stats.leaf_nodes_count << std::endl;
        std::clog << "  Average depth of leaf nodes: " << stats.avg_depth << std::endl;
        std::clog << "  Max triangles in leaf nodes: " << stats.triangles_in_leaf_nodes << std::endl;
        std::clog << "  Average triangles in leaf nodes: " << stats.avg_traiangles_in_leaf_nodes << std::endl;
        std::clog << "  Search method call count: " << stats.search_count << std::endl;
        std::clog << "  Search node count: " << stats.search_node_count << std::endl;
        std::clog << "  Search time: " << stats.search_time / 1000000.0f << " ms" << std::endl;
        std::clog << "  Search return count: " << stats.search_return_count << std::endl;
        std::clog << "  Average search return count: " << (float)stats.search_return_count / stats.search_count << std::endl;
        std::clog << "  Search leaves visited: " << stats.search_leaves_visited << std::endl;
    }

   private:
    struct OctreeNode {
        bool isLeaf;
        size_t depth;
        AABB bounding_box;
        std::vector<size_t> triangle_indices;
        std::shared_ptr<OctreeNode> octanes[8];
    };

    const nlohmann::json& config_;
    size_t max_triangles_per_BB_;
    size_t max_depth_;
    std::shared_ptr<OctreeNode> root;
    const std::vector<Triangle>& triangles_;
    size_t search_count_ = 0;
    size_t search_node_count_ = 0;
    float search_time_ = 0;
    size_t search_return_count_ = 0;
    size_t search_leaves_visited_ = 0;

   private:
    OctreeStats calculate_stats() const {
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
    void config_setup(const nlohmann::json& config) {
        std::clog << "Configuring octree..." << std::flush;

        max_triangles_per_BB_ = config.at("max_triangles_per_BB");
        max_depth_ = config.at("max_depth");

        std::clog << "\rOctree configured     " << std::endl;
    }
};

}  // namespace raytracer
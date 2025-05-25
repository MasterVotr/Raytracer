#include "src/ADS/Octree/OctreeParametric/octree_parametric.h"

#include <iostream>
#include <vector>

#include "include/json.hpp"

#include "src/ADS/ads.h"
#include "src/aabb.h"
#include "src/collision_detection.h"
#include "src/ray.h"
#include "src/triangle.h"

namespace raytracer {

namespace {

uint8_t first_node(const Vec3& t0, const Vec3& tm) {
    uint8_t first_node_index = 0;
    float t_max = std::max({t0.x, t0.y, t0.z});

    if (fabs(t_max - t0.x) < epsilon) {          // t_max == t0.x -> YZ entry plane
        first_node_index |= (tm.y < t0.x) << 1;  // bit affected 1
        first_node_index |= (tm.z < t0.x) << 2;  // bit affected 2
    } else if (fabs(t_max - t0.y) < epsilon) {   // t_max == t0.y -> XZ entry plane
        first_node_index |= (tm.x < t0.y) << 0;  // bit affected 0
        first_node_index |= (tm.z < t0.y) << 2;  // bit affected 2
    } else if (fabs(t_max - t0.z) < epsilon) {   // t_max == t0.z -> XY entry plane
        first_node_index |= (tm.x < t0.z) << 0;  // bit affected 0
        first_node_index |= (tm.y < t0.z) << 1;  // bit affected 1
    } else {
        throw std::runtime_error("First node could not be selected - unknown entry plane!");
    }

    return first_node_index;
};  // namespace internaluint8_t first_node(constvec3&t0,constvec3&tm)

uint8_t next_node(const Vec3& t, uint8_t yz, uint8_t xz, uint8_t xy) {
    float t_min = std::min({t.x, t.y, t.z});

    if (fabs(t_min - t.x) < epsilon) {  // t_min == t0.x -> YZ exit plane
        return yz;
    } else if (fabs(t_min - t.y) < epsilon) {  // t_min == t0.y -> XZ exit plane
        return xz;
    } else if (fabs(t_min - t.z) < epsilon) {  // t_min == t0.x -> XY exit plane
        return xy;
    } else {
        throw std::runtime_error("Next node could not be selected - unknown exit plane!");
    }
};

std::pair<Ray, uint8_t> adjust_ray_negative_direction(const Ray& r, const AABB& aabb) {
    uint8_t adjustment = 0;
    Vec3 adjusted_ray_origin = r.origin();
    Vec3 adjusted_ray_direction = r.direction();

    if (r.direction().x < 0.0f) {
        adjusted_ray_origin.x -= 2.0f * (r.origin().x - (aabb.min.x + aabb.size.x * 0.5f));
        adjusted_ray_direction.x = -r.direction().x;
        adjustment |= 4;
    }
    if (r.direction().y < 0.0f) {
        adjusted_ray_origin.y -= 2.0f * (r.origin().y - (aabb.min.y + aabb.size.y * 0.5f));
        adjusted_ray_direction.y = -r.direction().y;
        adjustment |= 2;
    }
    if (r.direction().z < 0.0f) {
        adjusted_ray_origin.z -= 2.0f * (r.origin().z - (aabb.min.z + aabb.size.z * 0.5f));
        adjusted_ray_direction.z = -r.direction().z;
        adjustment |= 1;
    }

    return {Ray(adjusted_ray_origin, adjusted_ray_direction), adjustment};
};

Vec3 calculate_tm(const Ray& r, const Vec3& t0, const Vec3& t1) {
    Vec3 tm;

    tm.x = (t0.x + t1.x) * 0.5 > r.origin().x ? infinity : -infinity;
    tm.y = (t0.y + t1.y) * 0.5 > r.origin().y ? infinity : -infinity;
    tm.z = (t0.z + t1.z) * 0.5 > r.origin().z ? infinity : -infinity;

    return tm;
};

}  // namespace

std::vector<std::shared_ptr<const Triangle>> OctreeParametric::Search(const Ray& r, bool first_hit) const {
    search_count_++;
    std::vector<std::shared_ptr<const Triangle>> result;

    auto start_time = std::chrono::high_resolution_clock::now();

    // Node with aabb enter/exit coordinates
    struct StackElem {
        std::shared_ptr<OctreeNode> node;
        Vec3 t0, t1;
    };

    // Adjust for negative directions of the ray
    auto [a_r, a] = adjust_ray_negative_direction(r, root->bounding_box);

    // std::clog << "root->bounding_box: min = (" << root->bounding_box.min << "), max = (" << root->bounding_box.max << ")" << std::endl;
    // std::clog << "Original ray: origin = " << r.origin() << ", direction = " << r.direction() << std::endl;
    // std::clog << "Adjusted ray: origin = " << a_r.origin() << ", direction = " << a_r.direction() << std::endl;
    // std::clog << "Adjustment mask (a): " << (int)a << std::endl;

    // Aabb enter/exit coordinates calculation for each axis
    Vec3 inv_dir;
    inv_dir.x = a_r.direction().x != 0.0f ? 1.0f / a_r.direction().x : infinity;
    inv_dir.y = a_r.direction().y != 0.0f ? 1.0f / a_r.direction().y : infinity;
    inv_dir.z = a_r.direction().z != 0.0f ? 1.0f / a_r.direction().z : infinity;
    Vec3 t0 = (root->bounding_box.min - a_r.origin()) * inv_dir;
    Vec3 t1 = (root->bounding_box.max - a_r.origin()) * inv_dir;

    // std::clog << "inv_dir: " << inv_dir << std::endl;
    // std::clog << "t0: " << t0 << ", t1: " << t1 << std::endl;

    // If latest axis entry is later than the earliest axis exit -> ray misses
    float t_entry = std::max({t0.x, t0.y, t0.z});
    float t_exit = std::min({t1.x, t1.y, t1.z});
    if (t_entry >= t_exit) {
        // std::clog << "!Ray in dir: " << r.direction() << " didnt hit the scene\n" << std::endl;
        return result;
    }

    std::stack<StackElem> s;
    s.push({root, t0, t1});

    while (!s.empty()) {
        auto [node, t0, t1] = s.top();
        s.pop();

        // ray missed aabb
        if (t1.x < 0.0f || t1.y < 0.0f || t1.z < 0.0f) {
            continue;
        }

        search_node_count_++;
        if (node->isLeaf) {
            search_leaves_visited_++;
            for (const auto& ti : node->triangle_indices) {
                result.emplace_back(triangles_[ti]);
                if (first_hit) {
                    auto end_time = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
                    search_time_ += duration;
                    return result;
                }
            }
            continue;
        }

        // Midpoint of the enter/exit time
        // Vec3 tm = calculate_tm(a_r, t0, t1);
        Vec3 tm = (t0 + t1) * 0.5f;

        uint8_t current_subnode_index = first_node(t0, tm);
        // std::clog << "Sequence of subnodes: " << std::flush;
        do {
            // std::clog << (int)(current_subnode_index ^ a) << "(" << (int)current_subnode_index << "^" << (int)a << ") " << std::flush;
            switch ((int)current_subnode_index) {
                case 0:
                    if (node->octanes[current_subnode_index ^ a]) {
                        s.push({node->octanes[current_subnode_index ^ a], {t0.x, t0.y, t0.z}, {tm.x, tm.y, tm.z}});
                        current_subnode_index = next_node({tm.x, tm.y, tm.z}, 4, 2, 1);
                    } else {
                        // std::clog << "Octane " << (int)(current_subnode_index ^ a) << " is not present" << std::endl;
                        current_subnode_index = 8;
                    }
                    break;
                case 1:
                    if (node->octanes[current_subnode_index ^ a]) {
                        s.push({node->octanes[current_subnode_index ^ a], {t0.x, t0.y, tm.z}, {tm.x, tm.y, t1.z}});
                        current_subnode_index = next_node({tm.x, tm.y, t1.z}, 5, 3, 8);
                    } else {
                        // std::clog << "Octane " << (int)(current_subnode_index ^ a) << " is not present" << std::endl;
                        current_subnode_index = 8;
                    }
                    break;
                case 2:
                    if (node->octanes[current_subnode_index ^ a]) {
                        s.push({node->octanes[current_subnode_index ^ a], {t0.x, tm.y, t0.z}, {tm.x, t1.y, tm.z}});
                        current_subnode_index = next_node({tm.x, t1.y, tm.z}, 6, 8, 3);
                    } else {
                        // std::clog << "Octane " << (int)(current_subnode_index ^ a) << " is not present" << std::endl;
                        current_subnode_index = 8;
                    }
                    break;
                case 3:
                    if (node->octanes[current_subnode_index ^ a]) {
                        s.push({node->octanes[current_subnode_index ^ a], {t0.x, tm.y, tm.z}, {tm.x, t1.y, t1.z}});
                        current_subnode_index = next_node({tm.x, t1.y, t1.z}, 7, 8, 8);
                    } else {
                        // std::clog << "Octane " << (int)(current_subnode_index ^ a) << " is not present" << std::endl;
                        current_subnode_index = 8;
                    }
                    break;
                case 4:
                    if (node->octanes[current_subnode_index ^ a]) {
                        s.push({node->octanes[current_subnode_index ^ a], {tm.x, t0.y, t0.z}, {t1.x, tm.y, tm.z}});
                        current_subnode_index = next_node({t1.x, tm.y, tm.z}, 8, 6, 5);
                    } else {
                        // std::clog << "Octane " << (int)(current_subnode_index ^ a) << " is not present" << std::endl;
                        current_subnode_index = 8;
                    }
                    break;
                case 5:
                    if (node->octanes[current_subnode_index ^ a]) {
                        s.push({node->octanes[current_subnode_index ^ a], {tm.x, t0.y, tm.z}, {t1.x, tm.y, t1.z}});
                        current_subnode_index = next_node({t1.x, tm.y, t1.z}, 8, 7, 8);
                    } else {
                        // std::clog << "Octane " << (int)(current_subnode_index ^ a) << " is not present" << std::endl;
                        current_subnode_index = 8;
                    }
                    break;
                case 6:
                    if (node->octanes[current_subnode_index ^ a]) {
                        s.push({node->octanes[current_subnode_index ^ a], {tm.x, tm.y, t0.z}, {t1.x, t1.y, tm.z}});
                        current_subnode_index = next_node({t1.x, t1.y, tm.z}, 8, 8, 7);
                    } else {
                        // std::clog << "Octane " << (int)(current_subnode_index ^ a) << " is not present" << std::endl;
                        current_subnode_index = 8;
                    }
                    break;
                case 7:
                    if (node->octanes[current_subnode_index ^ a]) {
                        s.push({node->octanes[current_subnode_index ^ a], {tm.x, tm.y, tm.z}, {t1.x, t1.y, t1.z}});
                    }
                    current_subnode_index = 8;
                    // std::clog << "7 end of the line" << std::endl;
                    break;
                default:
                    throw std::runtime_error("Next node could not be selected - unknown current_subnode_index!");
                    break;
            }
        } while (current_subnode_index < 8);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    search_time_ += duration;
    search_return_count_ += result.size();

    // std::clog << "Ray in dir: " << r.direction() << " will check collision with " << result.size() << " triangles\n" << std::endl;
    return result;
}

}  // namespace raytracer

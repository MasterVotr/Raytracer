#pragma once

#include <ostream>

#include "src/vec3.h"

namespace raytracer {

/**
 * @brief BVH construction and traversal statistics.
 *
 * Collects comprehensive statistics about BVH structure and performance:
 * - Construction metrics (build time, node counts, tree depth)
 * - Query performance (traversal steps, incidence operations)
 * - Memory consumption
 * - BVH quality metrics (cost estimate)
 */
class AdsStats {
   private:
    static constexpr double kIntersectionCost = 1.0;
    static constexpr double kTraversalCost = 1.2;

   public:
    // Construction statistics
    double build_time;                ///< Total build time in ms.
    uint32_t node_count;              ///< Node count in BVH (internal + leafs).
    uint32_t degenerate_nodes_count;  ///< Number of leaf more than one prim in them.
    uint32_t inner_node_count;        ///< Inner node count in BVH.
    uint32_t leaf_node_count;         ///< Leaf node count in BVH.
    uint32_t min_leaf_depth;          ///< Min leaf depth in BVH.
    uint32_t max_leaf_depth;          ///< Max leaf depth in BVH.
    uint32_t total_leaf_depth;        ///< Total leaf depth (for average calculation).
    uint32_t min_prims_per_leaf;      ///< Max number of primitives per leaf.
    uint32_t max_prims_per_leaf;      ///< Max number of primitives per leaf.
    uint32_t total_prims_per_leaf;    ///< Total number of primitives in all leaves.
    uint32_t memory_consumption;      ///< Tutal memory consumption of the BVH.

    // Query statistics
    uint32_t total_query_count;           ///< Count of all queries.
    uint32_t min_incidence_operations;    ///< Min number of incidence operations.
    uint32_t max_incidence_operations;    ///< Max number of incidence operations.
    uint32_t total_incidence_operations;  ///< Number of incidence operations.
    uint32_t min_traversal_steps;         ///< Min number of nodes visited per query.
    uint32_t max_traversal_steps;         ///< Max number of nodes visited per query.
    uint32_t total_traversal_steps;       ///< Number of traversal steps in all queires.

    /**
     * @brief Constructor initializing all statistics to default values.
     */
    AdsStats();

    /** @brief Reset all statistics to initial state (zero/max/min values). */
    void Reset();

    /**
     * @brief Print construction statistics in extractable format.
     *
     * Outputs BVH construction metrics with # prefix for easy extraction:
     *  - build_time: construction time in milliseconds
     *  - node_count: total number of nodes (inner + leaf)
     *  - degenerate_node_count: number of non-optimal leaf nodes
     *  - inner_node_count: number of interior nodes
     *  - leaf_node_count: number of leaf nodes
     *  - min/max/avg_leaf_depth: tree depth statistics
     *  - min/max/avg_prims_per_leaf: primitive distribution statistics
     *  - memory_bytes: total memory consumption
     *  - scene_bbox_min: minimum corner of scene bounding box (x, y, z)
     *  - scene_bbox_max: maximum corner of scene bounding box (x, y, z)
     *
     * @param out Output stream for formatted statistics
     */
    void PrintOnlyConstruction(std::ostream& out);

    /**
     * @brief Print query statistics in extractable format.
     *
     * Outputs ray tracing performance metrics with # prefix for easy extraction:
     *  - total_query_count: number of ray queries processed
     *  - min/max/avg_incidence_ops: primitive intersection tests per query
     *  - min/max/avg_traversal_steps: BVH nodes visited per query
     *  - bvh_cost: estimated traversal cost (weighted sum of operations)
     *
     * @param out Output stream for formatted statistics
     */
    void PrintOnlyQuery(std::ostream& out);

    /**
     * @brief Print complete statistics (construction + query) in extractable
     * format.
     *
     * Combines PrintOnlyConstruction() and PrintOnlyQuery() output with
     * blank line separator. All values prefixed with # for easy extraction.
     *
     * @param out Output stream for formatted statistics
     */
    void Print(std::ostream& out);
};

}  // namespace raytracer
#include "src/ADS/ADSStats/ads_stats.h"

#include <iomanip>

namespace raytracer {

AdsStats::AdsStats() { Reset(); }

void AdsStats::Reset() {
    build_time = 0.0;
    node_count = 0;
    degenerate_nodes_count = 0;
    inner_node_count = 0;
    leaf_node_count = 0;
    min_leaf_depth = std::numeric_limits<uint32_t>::max();
    max_leaf_depth = std::numeric_limits<uint32_t>::lowest();
    total_leaf_depth = 0;
    min_prims_per_leaf = std::numeric_limits<uint32_t>::max();
    max_prims_per_leaf = std::numeric_limits<uint32_t>::lowest();
    total_prims_per_leaf = 0;
    memory_consumption = 0;

    min_incidence_operations = std::numeric_limits<uint32_t>::max();
    max_incidence_operations = std::numeric_limits<uint32_t>::lowest();
    total_incidence_operations = 0;
    min_traversal_steps = std::numeric_limits<uint32_t>::max();
    max_traversal_steps = std::numeric_limits<uint32_t>::lowest();
    total_traversal_steps = 0;
}

void AdsStats::PrintOnlyConstruction(std::ostream& out) {
    out << "Construction statistics:\n";
    out << "# build_time = " << build_time << "\n";

    out << "# node_count = " << node_count << "\n";
    out << "# degenerate_node_count = " << degenerate_nodes_count << "\n";
    out << "# inner_node_count = " << inner_node_count << "\n";

    out << "# leaf_node_count = " << leaf_node_count << "\n";
    out << "# min_leaf_depth = " << min_leaf_depth << "\n";
    out << "# max_leaf_depth = " << max_leaf_depth << "\n";
    double avgLeafDepth = (leaf_node_count > 0) ? (double)total_leaf_depth / leaf_node_count : 0.0;
    out << "# avg_leaf_depth = " << avgLeafDepth << "\n";

    out << "# min_prims_per_leaf = " << min_prims_per_leaf << "\n";
    out << "# max_prims_per_leaf = " << max_prims_per_leaf << "\n";
    double avgPrimsPerLeaf = (leaf_node_count > 0) ? (double)total_prims_per_leaf / leaf_node_count : 0.0;
    out << "# avg_prims_per_leaf = " << avgPrimsPerLeaf << "\n";

    out << "# memory_bytes = " << memory_consumption << "\n";
}

void AdsStats::PrintOnlyQuery(std::ostream& out) {
    out << "Query statistics:\n";
    out << "# total_query_count = " << total_query_count << '\n';

    out << "# min_incidence_ops = " << min_incidence_operations << '\n';
    out << "# max_incidence_ops = " << max_incidence_operations << '\n';
    const double avgIncidenceOps = (total_query_count > 0) ? static_cast<double>(total_incidence_operations) /
                                                                 static_cast<double>(total_query_count)
                                                           : -1.0;
    out << "# avg_incidence_ops = " << avgIncidenceOps << '\n';

    out << "# min_traversal_steps = " << min_traversal_steps << '\n';
    out << "# max_traversal_steps = " << max_traversal_steps << '\n';
    const double avgTraversalSteps =
        (total_query_count > 0) ? static_cast<double>(total_traversal_steps) / static_cast<double>(total_query_count)
                                : -1.0;
    out << "# avg_traversal_steps = " << avgTraversalSteps << '\n';

    const double bvhCost = avgIncidenceOps * kIntersectionCost + avgTraversalSteps * kTraversalCost;
    out << "# bvh_cost = " << bvhCost << '\n';
}

void AdsStats::Print(std::ostream& out) {
    PrintOnlyConstruction(out);
    out << "\n";
    PrintOnlyQuery(out);
}

}  // namespace raytracer

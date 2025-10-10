#include "src/ADS/BVH/bvh.h"

#include <iostream>

namespace raytracer {

void Bvh::Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) {
    Ads::Build(triangles);
    std::clog << "Building octree..." << std::flush;
    auto start_time = std::chrono::high_resolution_clock::now();

    if (triangles_.empty()) {
        std::cerr << "No triangles to build an octree." << std::endl;
        return;
    }
}

std::vector<std::shared_ptr<const Triangle>> Bvh::Search(const Ray& r, bool first_hit) const {}

void Bvh::PrintStats(std::ostream& os) const {}

}  // namespace raytracer

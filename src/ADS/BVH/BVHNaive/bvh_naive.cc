#include "ADS/BVH/BVHNaive/bvh_naive.h"

#include <iostream>

namespace raytracer {

BvhNaive::BvhNaive(const nlohmann::json& config) : Ads(config) {}
BvhNaive::~BvhNaive() = default;

void BvhNaive::Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) {
    Ads::Build(triangles);
    std::clog << "Building naive BVH..." << std::flush;
    auto start_time = std::chrono::high_resolution_clock::now();

    if (triangles_.empty()) {
        std::cerr << "No triangles to build a BVH." << std::endl;
        return;
    }
}
std::vector<std::shared_ptr<const Triangle>> BvhNaive::Search(const Ray& r, bool first_hit = false) const {}
void BvhNaive::PrintStats(std::ostream& os) const {}

}  // namespace raytracer
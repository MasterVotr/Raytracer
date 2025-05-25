#pragma once

#include <vector>

#include "include/json.hpp"

#include "src/ray.h"
#include "src/triangle.h"

namespace raytracer {

// Abstract class for all acceleration data structures
// Build the structure using the Build function
// Search for potential triangles the given ray intersects using the Search function
// Statistics of this particular structure can be printed out using the PrintStats function
class Ads {
   public:
    Ads(const nlohmann::json& config) : config_(config) {}
    virtual ~Ads() = default;

    virtual void Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) { triangles_ = triangles; }
    virtual std::vector<std::shared_ptr<const Triangle>> Search(const Ray& r, bool first_hit = false) const = 0;
    virtual void PrintStats(std::ostream& os) const = 0;

   protected:
    const nlohmann::json& config_;
    std::vector<std::shared_ptr<const Triangle>> triangles_;
};

}  // namespace raytracer
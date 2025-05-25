#pragma once

#include <vector>

#include "include/json.hpp"

#include "src/ADS/ads.h"
#include "src/ray.h"
#include "src/triangle.h"

namespace raytracer {

// Dummy ADS implementation without any actual stucture
class DummyAds : public Ads {
   public:
    DummyAds(const nlohmann::json& config) : Ads(config) {}
    virtual std::vector<std::shared_ptr<const Triangle>> Search(const Ray& r, bool first_hit = false) const override {
        search_count_++;
        search_return_count_ += triangles_.size();
        return std::vector<std::shared_ptr<const Triangle>>(triangles_.begin(), triangles_.end());
    }
    virtual void PrintStats(std::ostream& os) const override {
        os << "DummyAds stats: " << "\n";
        os << "  Search method call count: " << search_count_ << "\n";
        os << "  Search return count: " << search_return_count_ << "\n";
        os << "  Average search return count: " << (float)search_return_count_ / search_count_ << "\n";
    }

   protected:
    // Statistics variables
    mutable size_t search_count_ = 0;
    mutable size_t search_return_count_ = 0;
};

}  // namespace raytracer
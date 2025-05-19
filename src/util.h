#pragma once

#include <algorithm>
#include <limits>

namespace raytracer {

// Constants
const float epsilon = 1e-9;  // Small value
const float infinity = std::numeric_limits<float>::infinity();

// Utility functions
template <typename T>
inline T clamp(T val, T low, T high) {
    return std::max(std::min(val, high), low);
}

}  // namespace raytracer
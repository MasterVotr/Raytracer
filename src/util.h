#pragma once

#include <algorithm>
#include <limits>

namespace raytracer {

// Constants
const float epsilon = 1e-9;  // Small value
const float infinity = std::numeric_limits<float>::infinity();
const float pi = 3.1415926535897932385;

// Utility functions
template <typename T>
inline T clamp(T val, T low, T high) {
    return std::max(std::min(val, high), low);
}

}  // namespace raytracer
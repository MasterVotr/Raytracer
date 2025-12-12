#pragma once

#include <algorithm>
#include <limits>

namespace raytracer {

// Constants
const float epsilon = 1e-9;  // Small value
const float infinity = std::numeric_limits<float>::max();

// Utility functions
template <typename T>
inline T clamp(T val, T low, T high) {
    return std::max(std::min(val, high), low);
}

// Print utils
template <typename T>
inline std::ostream& operator<<(std::ostream& os, const std::vector<T> vec) {
    os << "[ ";
    std::for_each(vec.begin(), vec.end(), [&os](const auto& e) { os << e << " "; });
    os << " ]";
    return os;
}


}  // namespace raytracer
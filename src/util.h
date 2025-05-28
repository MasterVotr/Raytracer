#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace raytracer {

// Constants
constexpr float epsilon = 1e-9f;  // Small value
#ifdef __CUDA_ARCH__
constexpr float infinity = INFINITY;
#else
constexpr float infinity = std::numeric_limits<float>::infinity();
#endif

// Utility functions
template <typename T>
__host__ __device__ inline T clamp(T val, T low, T high) {
#ifdef __CUDA_ARCH__
    // Device code
    return max(min(val, high), low);
#else
    // Host code
    return std::max(std::min(val, high), low);
#endif
}

__device__ inline float device_rand(unsigned int seed, int i) {
    //  hash-based PRNG for device code
    unsigned int x = seed * 1973 + i * 9277 + 0x68bc21eb;
    x = (x ^ (x >> 13)) * 1274126177u;
    return (x & 0x00FFFFFF) / float(0x01000000);
}

}  // namespace raytracer
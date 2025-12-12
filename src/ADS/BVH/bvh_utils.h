#pragma once

#include <algorithm>
#include <vector>

#include "src/aabb.h"

namespace raytracer {

struct BinningBuffers {
    std::vector<size_t> ns;
    std::vector<AABB> bbs;
    std::vector<size_t> N_Ls;
    std::vector<size_t> N_Rs;
    std::vector<AABB> TB_Ls;
    std::vector<AABB> TB_Rs;
    std::vector<float> A_Ls;
    std::vector<float> A_Rs;

    inline explicit BinningBuffers(size_t K) {
        ns.resize(K);
        bbs.resize(K);
        size_t split_count = K > 0 ? K - 1 : 0;
        N_Ls.resize(split_count);
        N_Rs.resize(split_count);
        TB_Ls.resize(split_count);
        TB_Rs.resize(split_count);
        A_Ls.resize(split_count);
        A_Rs.resize(split_count);
        reset();
    }

    inline void reset() {
        std::fill(ns.begin(), ns.end(), 0);
        std::fill(bbs.begin(), bbs.end(), AABB(infinity, -infinity));
        std::fill(N_Ls.begin(), N_Ls.end(), 0);
        std::fill(N_Rs.begin(), N_Rs.end(), 0);
        std::fill(TB_Ls.begin(), TB_Ls.end(), AABB(infinity, -infinity));
        std::fill(TB_Rs.begin(), TB_Rs.end(), AABB(infinity, -infinity));
        std::fill(A_Ls.begin(), A_Ls.end(), 0.0f);
        std::fill(A_Rs.begin(), A_Rs.end(), 0.0f);
    }
};

// SoA definitions
struct PointSoA {
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;

    explicit PointSoA(size_t n) {
        x.resize(n);
        y.resize(n);
        z.resize(n);
    }

    inline void fill(float value) {
        std::fill(x.begin(), x.end(), value);
        std::fill(y.begin(), y.end(), value);
        std::fill(z.begin(), z.end(), value);
    }
};

struct AABBSoA {
    std::vector<float> min_x;
    std::vector<float> min_y;
    std::vector<float> min_z;
    std::vector<float> max_x;
    std::vector<float> max_y;
    std::vector<float> max_z;

    explicit AABBSoA(size_t n) {
        min_x.resize(n);
        min_y.resize(n);
        min_z.resize(n);
        max_x.resize(n);
        max_y.resize(n);
        max_z.resize(n);
    }

    inline void fill(float min_val, float max_val) {
        std::fill(min_x.begin(), min_x.end(), min_val);
        std::fill(min_y.begin(), min_y.end(), min_val);
        std::fill(min_z.begin(), min_z.end(), min_val);
        std::fill(max_x.begin(), max_x.end(), max_val);
        std::fill(max_y.begin(), max_y.end(), max_val);
        std::fill(max_z.begin(), max_z.end(), max_val);
    }
};

}  // namespace raytracer
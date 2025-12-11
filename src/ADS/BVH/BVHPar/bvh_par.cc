#include "src/ADS/BVH/BVHPar/bvh_par.h"

#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#elif defined(__aarch64__)
#include <arm_neon.h>
#endif

#include <algorithm>
// #include <cassert>
#include <chrono>
#include <iostream>
#include <limits>
#include <stack>

#include "src/aabb.h"
#include "src/collision_detection.h"

namespace raytracer {

namespace {

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

    void fill(float value) {
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

    void fill(float min_val, float max_val) {
        std::fill(min_x.begin(), min_x.end(), min_val);
        std::fill(min_y.begin(), min_y.end(), min_val);
        std::fill(min_z.begin(), min_z.end(), min_val);
        std::fill(max_x.begin(), max_x.end(), max_val);
        std::fill(max_y.begin(), max_y.end(), max_val);
        std::fill(max_z.begin(), max_z.end(), max_val);
    }
};

struct BinningBuffers {
    std::vector<size_t> ns;
    std::vector<AABB> bbs;
    std::vector<size_t> N_Ls;
    std::vector<size_t> N_Rs;
    std::vector<AABB> TB_Ls;
    std::vector<AABB> TB_Rs;
    std::vector<float> A_Ls;
    std::vector<float> A_Rs;

    explicit BinningBuffers(size_t K) {
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

    void reset() {
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

/*  Triangle to bin separation
    k - binning (splitting) axis
    K - number of bins
    1-epsilon - floating point error for bounds of cb
    binID_i = (K * (1 - epsilon) * (c_i_k - cb_min_k)) / (cb_max_k - cb_min_k)
    k_0 = cb_min_k
    k_1 = (K * (1 - epsilon)) / (cb_max_k - cb_min_k)
    binID_i = k_1 * (c_i_k - k_0)
*/
std::vector<int> calculate_bin_ids(const AABB& cb, const PointSoA& cs, const std::vector<size_t>& triangle_indices,
                                   int K, size_t t_begin, size_t t_count) {
    int k = 0;
    const float* __restrict__ cs_k;
    if (cb.size.x >= cb.size.y && cb.size.x >= cb.size.z) {
        k = 0;
        cs_k = cs.x.data();
    } else if (cb.size.y >= cb.size.z) {
        k = 1;
        cs_k = cs.y.data();
    } else {
        k = 2;
        cs_k = cs.z.data();
    }

    if (cb.size[k] < epsilon) {
        return std::vector<int>(t_count, 0);
    }

    std::vector<int> binIDs(t_count);
    int* __restrict__ binIDs_ptr = binIDs.data();

    float k_0 = cb.min[k];
    float k_1 = K * (1 - 1e-3) / cb.size[k];  // Possible problem with bin_idx being K - epsilon was too big

#if defined(__x86_64__) || defined(_M_X64)
    size_t i = 0;
    const __m256 k0_vec = _mm256_set1_ps(k_0);
    const __m256 k1_vec = _mm256_set1_ps(k_1);
    // Process 8 elements at a time
    for (; i + 7 < t_count; i += 8) {
        // binID_i = k_1 * (c_i_k - k_0)
        const size_t* tri_idx = &triangle_indices[i + t_begin];
        // assert(tri_idx[i + 7] < cs.x.size());
        __m256 centroids = _mm256_i32gather_ps(cs_k, _mm256_loadu_si256((const __m256i*)tri_idx), 4);
        __m256 diff = _mm256_sub_ps(centroids, k0_vec);
        __m256 scaled = _mm256_mul_ps(diff, k1_vec);

        // Convert to int (truncation)
        __m256i bin_indices = _mm256_cvtps_epi32(scaled);

        // Store results
        _mm256_storeu_si256((__m256i*)&binIDs_ptr[i], bin_indices);
    }
#elif defined(__aarch64__)
    size_t i = 0;
    const float32x4_t k0_vec = vdupq_n_f32(k_0);
    const float32x4_t k1_vec = vdupq_n_f32(k_1);
    // Process 4 elements at a time
    for (; i + 3 < t_count; i += 4) {
        // binID_i = k_1 * (c_i_k - k_0)
        // assert(i + t_begin + 3 < triangle_indices.size());
        float32x4_t centroids = {cs_k[triangle_indices[i + t_begin + 0]], cs_k[triangle_indices[i + t_begin + 1]],
                                 cs_k[triangle_indices[i + t_begin + 2]], cs_k[triangle_indices[i + t_begin + 3]]};
        float32x4_t diff = vsubq_f32(centroids, k0_vec);
        float32x4_t scaled = vmulq_f32(diff, k1_vec);

        // Convert to int (truncation)
        int32x4_t bin_indices = vcvtq_s32_f32(scaled);

        // Store result
        vst1q_s32(&binIDs_ptr[i], bin_indices);
    }
#else
    size_t i = 0;
#endif
    // Process remaining elements
    for (; i < t_count; i++) {
        size_t tri_idx = triangle_indices[i + t_begin];
        // assert(tri_idx < cs.x.size());
        int bin_idx = static_cast<int>(k_1 * (cs_k[tri_idx] - k_0));
        binIDs_ptr[i] = std::min(bin_idx, K - 1);
    }

    return binIDs;
}

std::vector<int> calculate_bin_ids_par(const AABB& cb, const PointSoA& cs, const std::vector<size_t>& triangle_indices,
                                       int K, size_t t_begin, size_t t_count) {
    int k = 0;
    const float* __restrict__ cs_k;
    if (cb.size.x >= cb.size.y && cb.size.x >= cb.size.z) {
        k = 0;
        cs_k = cs.x.data();
    } else if (cb.size.y >= cb.size.z) {
        k = 1;
        cs_k = cs.y.data();
    } else {
        k = 2;
        cs_k = cs.z.data();
    }

    if (cb.size[k] < epsilon) {
        return std::vector<int>(t_count, 0);
    }

    std::vector<int> binIDs(t_count);
    int* __restrict__ binIDs_ptr = binIDs.data();

    float k_0 = cb.min[k];
    float k_1 = K * (1 - 1e-3) / cb.size[k];  // Possible problem with bin_idx being K - epsilon was too big

#if defined(__x86_64__) || defined(_M_X64)
    size_t i = 0;
    const __m256 k0_vec = _mm256_set1_ps(k_0);
    const __m256 k1_vec = _mm256_set1_ps(k_1);
    size_t simd_limit = t_count - (t_count % 8);
// Process 8 elements at a time
#pragma omp parallel for schedule(static)
    for (size_t i = 0; i < simd_limit; i += 8) {
        // binID_i = k_1 * (c_i_k - k_0)
        const size_t* tri_idx = &triangle_indices[i + t_begin];
        __m256 centroids = _mm256_i32gather_ps(cs_k, _mm256_loadu_si256((const __m256i*)tri_idx), 4);
        __m256 diff = _mm256_sub_ps(centroids, k0_vec);
        __m256 scaled = _mm256_mul_ps(diff, k1_vec);

        // Convert to int (truncation)
        __m256i bin_indices = _mm256_cvtps_epi32(scaled);

        // Store results
        _mm256_storeu_si256((__m256i*)&binIDs_ptr[i], bin_indices);
    }
    for (size_t i = simd_limit; i < t_count; i++) {
        size_t tri_idx = triangle_indices[i + t_begin];
        // assert(tri_idx < cs.x.size());
        int bin_idx = static_cast<int>(k_1 * (cs_k[tri_idx] - k_0));
        binIDs_ptr[i] = std::min(bin_idx, K - 1);
    }
#elif defined(__aarch64__)
    size_t i = 0;
    const float32x4_t k0_vec = vdupq_n_f32(k_0);
    const float32x4_t k1_vec = vdupq_n_f32(k_1);
    size_t simd_limit = t_count - (t_count % 4);
// Process 4 elements at a time
#pragma omp parallel for schedule(static)
    for (size_t i = 0; i < simd_limit; i += 4) {
        // binID_i = k_1 * (c_i_k - k_0)
        float32x4_t centroids = {cs_k[triangle_indices[i + t_begin + 0]], cs_k[triangle_indices[i + t_begin + 1]],
                                 cs_k[triangle_indices[i + t_begin + 2]], cs_k[triangle_indices[i + t_begin + 3]]};
        float32x4_t diff = vsubq_f32(centroids, k0_vec);
        float32x4_t scaled = vmulq_f32(diff, k1_vec);

        // Convert to int (truncation)
        int32x4_t bin_indices = vcvtq_s32_f32(scaled);

        // Store result
        vst1q_s32(&binIDs_ptr[i], bin_indices);
    }
    // Process remaining elements
    for (size_t i = simd_limit; i < t_count; i++) {
        size_t tri_idx = triangle_indices[i + t_begin];
        // assert(tri_idx < cs.x.size());
        int bin_idx = static_cast<int>(k_1 * (cs_k[tri_idx] - k_0));
        binIDs_ptr[i] = std::min(bin_idx, K - 1);
    }
#else
#pragma omp parallel for schedule(static)
    for (size_t i = 0; i < t_count; i++) {
        size_t tri_idx = triangle_indices[i + t_begin];
        // assert(tri_idx < cs.x.size());
        int bin_idx = static_cast<int>(k_1 * (cs_k[tri_idx] - k_0));
        binIDs_ptr[i] = std::min(bin_idx, K - 1);
    }
#endif

    return binIDs;
}

/*  Bin setup
    generate K bins of equal width on the widest axis - domains
    n_i - number of primitives in each bin B_i
    bb_i - bin bounds - aabb of primitives inside bin B_i
    N_L_j - number of prmitives on the left side of the split j
    N_R_j - number of prmitives on the left side of the split j
    TB_L_j - triangle bounding box on the left side of the split j
    TB_R_j - triangle bounding box on the right side of the split j
    A_L_j - sourface area of the bb_i of all B_i on the left side of the split j
    A_R_j - sourface area of the bb_i of all B_i on the right side of the split j
    cost_j - const of split j - A_L_j * N_L_j + A_R_j * N_R_j
*/
size_t calculate_best_split_and_bins(BinningBuffers& binning_buffers, size_t& N_L, size_t& N_R, AABB& TB_L, AABB& TB_R,
                                     const AABBSoA& tbs, const std::vector<size_t>& triangle_indices,
                                     std::vector<int>& binIDs, size_t K, size_t t_begin, size_t t_count,
                                     size_t horizontal_threshold) {
    // Setup binning buffers
    binning_buffers.reset();

    if (t_count > horizontal_threshold) {
#pragma omp parallel
        {
            static thread_local std::vector<size_t> local_ns;
            static thread_local std::vector<AABB> local_bbs;

            if (local_ns.size() < K) {
                local_ns.resize(K);
                local_bbs.resize(K);
            }
            std::fill(local_ns.begin(), local_ns.end(), 0);
            std::fill(local_bbs.begin(), local_bbs.end(), AABB(infinity, -infinity));

#pragma omp for schedule(static) nowait
            for (size_t i = 0; i < t_count; i++) {
                int bin_idx = binIDs[i];
                size_t tri_idx = triangle_indices[i + t_begin];
                // assert(tri_idx < tbs.min_x.size());
                local_ns[bin_idx]++;
                local_bbs[bin_idx].expand(Point3(tbs.min_x[tri_idx], tbs.min_y[tri_idx], tbs.min_z[tri_idx]));
                local_bbs[bin_idx].expand(Point3(tbs.max_x[tri_idx], tbs.max_y[tri_idx], tbs.max_z[tri_idx]));
            }

#pragma omp critical
            {
                for (size_t k = 0; k < K; k++) {
                    binning_buffers.ns[k] += local_ns[k];
                    binning_buffers.bbs[k].expand(local_bbs[k]);
                }
            }
        }
    } else {
        std::vector<size_t>& ns = binning_buffers.ns;
        std::vector<AABB>& bbs = binning_buffers.bbs;
        for (size_t i = 0; i < t_count; i++) {
            int bin_idx = binIDs[i];
            size_t tri_idx = triangle_indices[i + t_begin];
            ns[bin_idx]++;
            bbs[bin_idx].expand(Point3(tbs.min_x[tri_idx], tbs.min_y[tri_idx], tbs.min_z[tri_idx]));
            bbs[bin_idx].expand(Point3(tbs.max_x[tri_idx], tbs.max_y[tri_idx], tbs.max_z[tri_idx]));
        }
    }

    std::vector<size_t>& ns = binning_buffers.ns;
    std::vector<AABB>& bbs = binning_buffers.bbs;
    std::vector<size_t>& N_Ls = binning_buffers.N_Ls;
    std::vector<size_t>& N_Rs = binning_buffers.N_Rs;
    std::vector<AABB>& TB_Ls = binning_buffers.TB_Ls;
    std::vector<AABB>& TB_Rs = binning_buffers.TB_Rs;
    std::vector<float>& A_Ls = binning_buffers.A_Ls;
    std::vector<float>& A_Rs = binning_buffers.A_Rs;

    size_t split_count = K - 1;

    // Calculate left sides triangle counts, AABBs and surface areas
    N_Ls[0] = ns[0];
    TB_Ls[0] = bbs[0];
    A_Ls[0] = TB_Ls[0].surface_area();

    for (size_t i = 1; i < split_count; i++) {
        N_Ls[i] = N_Ls[i - 1] + ns[i];
        TB_Ls[i] = TB_Ls[i - 1];
        TB_Ls[i].expand(bbs[i]);
        A_Ls[i] = TB_Ls[i].surface_area();
    }

    // Calculate right sides triangle counts, AABBs, surface areas and best split
    N_Rs[split_count - 1] = ns[K - 1];
    TB_Rs[split_count - 1] = bbs[K - 1];
    A_Rs[split_count - 1] = TB_Rs[split_count - 1].surface_area();
    float best_split_cost =
        (N_Ls[split_count - 1] != 0 && N_Rs[split_count - 1] != 0)
            ? A_Ls[split_count - 1] * N_Ls[split_count - 1] + A_Rs[split_count - 1] * N_Rs[split_count - 1]
            : infinity;
    size_t best_split = split_count - 1;

    for (int i = split_count - 2; i >= 0; i--) {
        N_Rs[i] = N_Rs[i + 1] + ns[i + 1];
        TB_Rs[i] = TB_Rs[i + 1];
        TB_Rs[i].expand(bbs[i + 1]);
        A_Rs[i] = TB_Rs[i].surface_area();

        float split_cost = (N_Ls[i] != 0 && N_Rs[i] != 0) ? A_Ls[i] * N_Ls[i] + A_Rs[i] * N_Rs[i] : infinity;
        if (split_cost < best_split_cost) {
            best_split = i;
            best_split_cost = split_cost;
        }
    }

    N_L = N_Ls[best_split];
    N_R = N_Rs[best_split];
    TB_L = TB_Ls[best_split];
    TB_R = TB_Rs[best_split];

    return best_split;
}

}  // namespace

BvhPar::BvhPar(const nlohmann::json& config) : Ads(config) {
    config_setup(config);
    reset_stats();
}

void BvhPar::Build(const std::vector<std::shared_ptr<const Triangle>>& triangles) {
    Ads::Build(triangles);
    reset_stats();
    std::clog << "Building parallel BVH..." << std::flush;
    auto start_time = std::chrono::high_resolution_clock::now();
    int64_t calculate_bin_ids_duration = 0;
    int64_t calculate_best_split_and_bins_duration = 0;
    int64_t rearange_triangles_duration = 0;
    int64_t calculate_centroids_duration = 0;

    if (triangles_.empty()) {
        std::cerr << "No triangles to build a BVH." << std::endl;
        return;
    }

    /*  AoS -> SoA
            Point3 -> 4x 16 bytes float (1 empty)
            Triangle -> 3 x Point3
            AABB -> 2x Point3

            3x load - triangles vertices
            2x min + 2x max - triangles aabbs
            min + max - grow voxel aabb
            add + mul - triangles centroids
            min + max - grow centroid aabb
            3x write - trinagles aabbs and centroids
    */
    //  Initial Setup
    size_t n = triangles_.size();
    nodes_.resize(2 * n - 1);
    triangle_indices_.resize(n);
    std::iota(triangle_indices_.begin(), triangle_indices_.end(), 0);

    BinningBuffers binning_buffers(bin_count_);

    // Preprocess triangle vertices
    struct TrinagleVerticesSoA {
        std::vector<float> v0x, v0y, v0z;
        std::vector<float> v1x, v1y, v1z;
        std::vector<float> v2x, v2y, v2z;

        TrinagleVerticesSoA(size_t count) {
            v0x.resize(count);
            v0y.resize(count);
            v0z.resize(count);
            v1x.resize(count);
            v1y.resize(count);
            v1z.resize(count);
            v2x.resize(count);
            v2y.resize(count);
            v2z.resize(count);
        }
    };
    TrinagleVerticesSoA tri_verts(n);
#pragma omp parallel for schedule(static)
    for (size_t i = 0; i < n; i++) {
        tri_verts.v0x[i] = triangles_[i]->vertices[0].pos.x;
        tri_verts.v0y[i] = triangles_[i]->vertices[0].pos.y;
        tri_verts.v0z[i] = triangles_[i]->vertices[0].pos.z;
        tri_verts.v1x[i] = triangles_[i]->vertices[1].pos.x;
        tri_verts.v1y[i] = triangles_[i]->vertices[1].pos.y;
        tri_verts.v1z[i] = triangles_[i]->vertices[1].pos.z;
        tri_verts.v2x[i] = triangles_[i]->vertices[2].pos.x;
        tri_verts.v2y[i] = triangles_[i]->vertices[2].pos.y;
        tri_verts.v2z[i] = triangles_[i]->vertices[2].pos.z;
    }

    // voxel aabb of the current node (all triangle aabbs)
    float vb_min_x = infinity;
    float vb_min_y = infinity;
    float vb_min_z = infinity;
    float vb_max_x = -infinity;
    float vb_max_y = -infinity;
    float vb_max_z = -infinity;

    // centroid aabb of the currents node (all triangles centroids aabb)
    float cb_min_x = infinity;
    float cb_min_y = infinity;
    float cb_min_z = infinity;
    float cb_max_x = -infinity;
    float cb_max_y = -infinity;
    float cb_max_z = -infinity;

    AABBSoA tbs(n);
    PointSoA cs(n);
#if defined(__x86_64__) || defined(_M_X64)
    const __m256 const_third = _mm256_set1_ps(1.0f / 3.0f);

    size_t simd_limit = n - (n % 8);
// Process 8 triangles at a time
#pragma omp parallel for schedule(static)
    for (size_t t = 0; t < simd_limit; t += 8) {
        // Load 8 sets of 3 vertices
        __m256 v0x = _mm256_loadu_ps(tri_verts.v0x.data() + t);
        __m256 v0y = _mm256_loadu_ps(tri_verts.v0y.data() + t);
        __m256 v0z = _mm256_loadu_ps(tri_verts.v0z.data() + t);
        __m256 v1x = _mm256_loadu_ps(tri_verts.v1x.data() + t);
        __m256 v1y = _mm256_loadu_ps(tri_verts.v1y.data() + t);
        __m256 v1z = _mm256_loadu_ps(tri_verts.v1z.data() + t);
        __m256 v2x = _mm256_loadu_ps(tri_verts.v2x.data() + t);
        __m256 v2y = _mm256_loadu_ps(tri_verts.v2y.data() + t);
        __m256 v2z = _mm256_loadu_ps(tri_verts.v2z.data() + t);

        // AABB calculation
        __m256 min_x = _mm256_min_ps(v0x, _mm256_min_ps(v1x, v2x));
        __m256 min_y = _mm256_min_ps(v0y, _mm256_min_ps(v1y, v2y));
        __m256 min_z = _mm256_min_ps(v0z, _mm256_min_ps(v1z, v2z));
        __m256 max_x = _mm256_max_ps(v0x, _mm256_max_ps(v1x, v2x));
        __m256 max_y = _mm256_max_ps(v0y, _mm256_max_ps(v1y, v2y));
        __m256 max_z = _mm256_max_ps(v0z, _mm256_max_ps(v1z, v2z));

        // Store AABB results
        _mm256_storeu_ps(&tbs.min_x[t], min_x);
        _mm256_storeu_ps(&tbs.min_y[t], min_y);
        _mm256_storeu_ps(&tbs.min_z[t], min_z);
        _mm256_storeu_ps(&tbs.max_x[t], max_x);
        _mm256_storeu_ps(&tbs.max_y[t], max_y);
        _mm256_storeu_ps(&tbs.max_z[t], max_z);

        // Centroid calculation
        __m256 c_x = _mm256_mul_ps(const_third, _mm256_add_ps(v0x, _mm256_add_ps(v1x, v2x)));
        __m256 c_y = _mm256_mul_ps(const_third, _mm256_add_ps(v0y, _mm256_add_ps(v1y, v2y)));
        __m256 c_z = _mm256_mul_ps(const_third, _mm256_add_ps(v0z, _mm256_add_ps(v1z, v2z)));

        // Store Centroid results
        _mm256_storeu_ps(&cs.x[t], c_x);
        _mm256_storeu_ps(&cs.y[t], c_y);
        _mm256_storeu_ps(&cs.z[t], c_z);
    }
    // Process remaining elements
    for (size_t t = simd_limit; t < n; t++) {
        // AABB calculation
        tbs.min_x[t] = std::min({tri_verts.v0x[t], tri_verts.v1x[t], tri_verts.v2x[t]});
        tbs.min_y[t] = std::min({tri_verts.v0y[t], tri_verts.v1y[t], tri_verts.v2y[t]});
        tbs.min_z[t] = std::min({tri_verts.v0z[t], tri_verts.v1z[t], tri_verts.v2z[t]});
        tbs.max_x[t] = std::max({tri_verts.v0x[t], tri_verts.v1x[t], tri_verts.v2x[t]});
        tbs.max_y[t] = std::max({tri_verts.v0y[t], tri_verts.v1y[t], tri_verts.v2y[t]});
        tbs.max_z[t] = std::max({tri_verts.v0z[t], tri_verts.v1z[t], tri_verts.v2z[t]});

        // Centroid calculation
        cs.x[t] = (tri_verts.v0x[t] + tri_verts.v1x[t] + tri_verts.v2x[t]) / 3.0f;
        cs.y[t] = (tri_verts.v0y[t] + tri_verts.v1y[t] + tri_verts.v2y[t]) / 3.0f;
        cs.z[t] = (tri_verts.v0z[t] + tri_verts.v1z[t] + tri_verts.v2z[t]) / 3.0f;
    }
#elif defined(__aarch64__)
    const float32x4_t const_third = vdupq_n_f32(1.0f / 3.0f);

    size_t simd_limit = n - (n % 4);
// Process 4 triangles at a time
#pragma omp parallel for schedule(static)
    for (size_t t = 0; t < simd_limit; t += 4) {
        // Load 4 sets of 3 vertices
        float32x4_t v0x = vld1q_f32(tri_verts.v0x.data() + t);
        float32x4_t v0y = vld1q_f32(tri_verts.v0y.data() + t);
        float32x4_t v0z = vld1q_f32(tri_verts.v0z.data() + t);
        float32x4_t v1x = vld1q_f32(tri_verts.v1x.data() + t);
        float32x4_t v1y = vld1q_f32(tri_verts.v1y.data() + t);
        float32x4_t v1z = vld1q_f32(tri_verts.v1z.data() + t);
        float32x4_t v2x = vld1q_f32(tri_verts.v2x.data() + t);
        float32x4_t v2y = vld1q_f32(tri_verts.v2y.data() + t);
        float32x4_t v2z = vld1q_f32(tri_verts.v2z.data() + t);

        // AABB calculation
        float32x4_t min_x = vminq_f32(v0x, vminq_f32(v1x, v2x));
        float32x4_t min_y = vminq_f32(v0y, vminq_f32(v1y, v2y));
        float32x4_t min_z = vminq_f32(v0z, vminq_f32(v1z, v2z));
        float32x4_t max_x = vmaxq_f32(v0x, vmaxq_f32(v1x, v2x));
        float32x4_t max_y = vmaxq_f32(v0y, vmaxq_f32(v1y, v2y));
        float32x4_t max_z = vmaxq_f32(v0z, vmaxq_f32(v1z, v2z));

        // Store AABB results
        vst1q_f32(&tbs.min_x[t], min_x);
        vst1q_f32(&tbs.min_y[t], min_y);
        vst1q_f32(&tbs.min_z[t], min_z);
        vst1q_f32(&tbs.max_x[t], max_x);
        vst1q_f32(&tbs.max_y[t], max_y);
        vst1q_f32(&tbs.max_z[t], max_z);

        // Centroid calculation
        float32x4_t c_x = vmulq_f32(const_third, vaddq_f32(v0x, vaddq_f32(v1x, v2x)));
        float32x4_t c_y = vmulq_f32(const_third, vaddq_f32(v0y, vaddq_f32(v1y, v2y)));
        float32x4_t c_z = vmulq_f32(const_third, vaddq_f32(v0z, vaddq_f32(v1z, v2z)));

        // Store Centroid results
        vst1q_f32(&cs.x[t], c_x);
        vst1q_f32(&cs.y[t], c_y);
        vst1q_f32(&cs.z[t], c_z);
    }
    // Process remaining elements
    for (size_t t = simd_limit; t < n; t++) {
        // AABB calculation
        tbs.min_x[t] = std::min({tri_verts.v0x[t], tri_verts.v1x[t], tri_verts.v2x[t]});
        tbs.min_y[t] = std::min({tri_verts.v0y[t], tri_verts.v1y[t], tri_verts.v2y[t]});
        tbs.min_z[t] = std::min({tri_verts.v0z[t], tri_verts.v1z[t], tri_verts.v2z[t]});
        tbs.max_x[t] = std::max({tri_verts.v0x[t], tri_verts.v1x[t], tri_verts.v2x[t]});
        tbs.max_y[t] = std::max({tri_verts.v0y[t], tri_verts.v1y[t], tri_verts.v2y[t]});
        tbs.max_z[t] = std::max({tri_verts.v0z[t], tri_verts.v1z[t], tri_verts.v2z[t]});

        // Centroid calculation
        cs.x[t] = (tri_verts.v0x[t] + tri_verts.v1x[t] + tri_verts.v2x[t]) / 3.0f;
        cs.y[t] = (tri_verts.v0y[t] + tri_verts.v1y[t] + tri_verts.v2y[t]) / 3.0f;
        cs.z[t] = (tri_verts.v0z[t] + tri_verts.v1z[t] + tri_verts.v2z[t]) / 3.0f;
    }
#else
    std::clog("Unsupported vectorization architecture");
#pragma omp parallel for schedule(static)
    for (size_t = 0; t < n; t++) {
        // AABB calculation
        tbs.min_x[t] = std::min({tri_verts.v0x[t], tri_verts.v1x[t], tri_verts.v2x[t]});
        tbs.min_y[t] = std::min({tri_verts.v0y[t], tri_verts.v1y[t], tri_verts.v2y[t]});
        tbs.min_z[t] = std::min({tri_verts.v0z[t], tri_verts.v1z[t], tri_verts.v2z[t]});
        tbs.max_x[t] = std::max({tri_verts.v0x[t], tri_verts.v1x[t], tri_verts.v2x[t]});
        tbs.max_y[t] = std::max({tri_verts.v0y[t], tri_verts.v1y[t], tri_verts.v2y[t]});
        tbs.max_z[t] = std::max({tri_verts.v0z[t], tri_verts.v1z[t], tri_verts.v2z[t]});

        // Centroid calculation
        cs.x[t] = (tri_verts.v0x[t] + tri_verts.v1x[t] + tri_verts.v2x[t]) / 3.0f;
        cs.y[t] = (tri_verts.v0y[t] + tri_verts.v1y[t] + tri_verts.v2y[t]) / 3.0f;
        cs.z[t] = (tri_verts.v0z[t] + tri_verts.v1z[t] + tri_verts.v2z[t]) / 3.0f;
    }
#endif

#pragma omp parallel for schedule(static) reduction(min : vb_min_x, vb_min_y, vb_min_z) \
    reduction(max : vb_max_x, vb_max_y, vb_max_z)
    for (size_t i = 0; i < n; i++) {
        // Expanding AABB
        vb_min_x = std::min(vb_min_x, tbs.min_x[i]);
        vb_min_y = std::min(vb_min_y, tbs.min_y[i]);
        vb_min_z = std::min(vb_min_z, tbs.min_z[i]);
        vb_max_x = std::max(vb_max_x, tbs.max_x[i]);
        vb_max_y = std::max(vb_max_y, tbs.max_y[i]);
        vb_max_z = std::max(vb_max_z, tbs.max_z[i]);
    }
#pragma omp parallel for schedule(static) reduction(min : cb_min_x, cb_min_y, cb_min_z) \
    reduction(max : cb_max_x, cb_max_y, cb_max_z)
    for (size_t i = 0; i < n; i++) {
        // Expanding AABB
        cb_min_x = std::min(cb_min_x, cs.x[i]);
        cb_min_y = std::min(cb_min_y, cs.y[i]);
        cb_min_z = std::min(cb_min_z, cs.z[i]);
        cb_max_x = std::max(cb_max_x, cs.x[i]);
        cb_max_y = std::max(cb_max_y, cs.y[i]);
        cb_max_z = std::max(cb_max_z, cs.z[i]);
    }
    AABB vb({vb_min_x, vb_min_y, vb_min_z}, {vb_max_x, vb_max_y, vb_max_z});
    AABB cb({cb_min_x, cb_min_y, cb_min_z}, {cb_max_x, cb_max_y, cb_max_z});

    auto init_done_time = std::chrono::high_resolution_clock::now();
    auto init_duration = std::chrono::duration_cast<std::chrono::microseconds>(init_done_time - start_time).count();

    struct StackElem {
        size_t node_idx;
        size_t depth;
        AABB vb;
        AABB cb;
        size_t t_begin;
        size_t t_count;
    };

    size_t next_free_node_idx = 0;
    std::stack<StackElem> s;
    s.emplace(next_free_node_idx++, 0, vb, cb, 0, n);

    auto build_loop_start = std::chrono::high_resolution_clock::now();
    while (!s.empty()) {
        auto [node_idx, depth, vb, cb, t_begin, t_count] = s.top();
        s.pop();

        nodes_[node_idx].depth = depth;
        nodes_[node_idx].bounding_box = vb;
        nodes_[node_idx].t_begin = t_begin;
        nodes_[node_idx].t_count = 0;
        // Node becomes a leaf if termination conditions are met -> the recursion stops
        bool cb_too_small = cb.size.x < epsilon && cb.size.y < epsilon && cb.size.z < epsilon;
        if (t_count < max_triangles_per_BB_ || nodes_[node_idx].depth >= max_depth_ || cb_too_small) {
            nodes_[node_idx].t_count = t_count;
            continue;
        }

        auto start_time_tmp = std::chrono::high_resolution_clock::now();
        std::vector<int> binIDs;
        if (t_count > horizontal_threshold_) {
            binIDs = calculate_bin_ids_par(cb, cs, triangle_indices_, bin_count_, t_begin, t_count);
        } else {
            binIDs = calculate_bin_ids(cb, cs, triangle_indices_, bin_count_, t_begin, t_count);
        }
        // std::vector<int> binIDs = calculate_bin_ids(cb, cs, triangle_indices_, bin_count_, t_begin, t_count);
        auto end_time_tmp = std::chrono::high_resolution_clock::now();
        calculate_bin_ids_duration +=
            std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_tmp - start_time_tmp).count();

        size_t N_L, N_R;  // child triangle counts
        AABB TB_L, TB_R;  // child triangle bounds

        start_time_tmp = std::chrono::high_resolution_clock::now();
        size_t best_split = calculate_best_split_and_bins(binning_buffers, N_L, N_R, TB_L, TB_R, tbs, triangle_indices_,
                                                          binIDs, bin_count_, t_begin, t_count, horizontal_threshold_);
        end_time_tmp = std::chrono::high_resolution_clock::now();
        calculate_best_split_and_bins_duration +=
            std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_tmp - start_time_tmp).count();
        // Bin comparison print
        // std::cout << "Node " << node_idx << " depth=" << depth << " t_begin=" << t_begin << " t_count=" << t_count
        //           << " best_split=" << best_split << " N_L=" << N_L << " N_R=" << N_R << " binIDs=[";
        // for (size_t i = 0; i < std::min<size_t>(t_count, 10); ++i)  // print first 10 binIDs
        //     std::cout << binIDs[i] << " ";
        // std::cout << "]\n";

        // If the split fails make current node as a leaf
        if (N_L == 0 || N_R == 0) {
            nodes_[node_idx].t_count = t_count;
            // std::clog << "Splitting failed N_L=" << N_L << ", N_R=" << N_R << ", best_split=" << best_split
            //           << std::endl;
            continue;
        }

        /*  Rearange triangles using a two-pointer partition
              l - starts at the beginning
              r - starts at the end
            Loop until the pointers cross.
            If l point to a "right" triangle and r point to a "left" triangle -> swap.
        */
        start_time_tmp = std::chrono::high_resolution_clock::now();
        size_t l = t_begin;
        size_t r = t_begin + t_count - 1;
        while (l < r) {
            // Advance l forward until it finds a "right" triangle
            while (l < r && binIDs[l - t_begin] <= best_split) {
                l++;
            }
            // Advance r backward until it finds a "left" triangle
            while (l < r && binIDs[r - t_begin] > best_split) {
                r--;
            }
            if (l < r) {
                // assert(l < triangle_indices_.size());
                // assert(r < triangle_indices_.size());
                std::swap(triangle_indices_[l], triangle_indices_[r]);
                l++;
                r--;
            }
        }
        end_time_tmp = std::chrono::high_resolution_clock::now();
        rearange_triangles_duration +=
            std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_tmp - start_time_tmp).count();

        // Calculate centroid bounds for children.
        start_time_tmp = std::chrono::high_resolution_clock::now();
        AABB CB_L, CB_R;  // child centroid bounds
        float CB_L_min_x = infinity;
        float CB_L_min_y = infinity;
        float CB_L_min_z = infinity;
        float CB_L_max_x = -infinity;
        float CB_L_max_y = -infinity;
        float CB_L_max_z = -infinity;

        float CB_R_min_x = infinity;
        float CB_R_min_y = infinity;
        float CB_R_min_z = infinity;
        float CB_R_max_x = -infinity;
        float CB_R_max_y = -infinity;
        float CB_R_max_z = -infinity;
#pragma omp simd reduction(min : CB_L_min_x, CB_L_min_y, CB_L_min_z) reduction(max : CB_L_max_x, CB_L_max_y, CB_L_max_z)
        for (size_t i = t_begin; i < t_begin + N_L; ++i) {
            size_t tri_idx = triangle_indices_[i];
            // assert(tri_idx < cs.x.size());
            CB_L_min_x = std::min(CB_L_min_x, cs.x[tri_idx]);
            CB_L_min_y = std::min(CB_L_min_y, cs.y[tri_idx]);
            CB_L_min_z = std::min(CB_L_min_z, cs.z[tri_idx]);
            CB_L_max_x = std::max(CB_L_max_x, cs.x[tri_idx]);
            CB_L_max_y = std::max(CB_L_max_y, cs.y[tri_idx]);
            CB_L_max_z = std::max(CB_L_max_z, cs.z[tri_idx]);
        }
#pragma omp simd reduction(min : CB_R_min_x, CB_R_min_y, CB_R_min_z) reduction(max : CB_R_max_x, CB_R_max_y, CB_R_max_z)
        for (size_t i = t_begin + N_L; i < t_begin + t_count; ++i) {
            size_t tri_idx = triangle_indices_[i];
            // assert(tri_idx < cs.x.size());
            CB_R_min_x = std::min(CB_R_min_x, cs.x[tri_idx]);
            CB_R_min_y = std::min(CB_R_min_y, cs.y[tri_idx]);
            CB_R_min_z = std::min(CB_R_min_z, cs.z[tri_idx]);
            CB_R_max_x = std::max(CB_R_max_x, cs.x[tri_idx]);
            CB_R_max_y = std::max(CB_R_max_y, cs.y[tri_idx]);
            CB_R_max_z = std::max(CB_R_max_z, cs.z[tri_idx]);
        }
        CB_L = AABB({CB_L_min_x, CB_L_min_y, CB_L_min_z}, {CB_L_max_x, CB_L_max_y, CB_L_max_z});
        CB_R = AABB({CB_R_min_x, CB_R_min_y, CB_R_min_z}, {CB_R_max_x, CB_R_max_y, CB_R_max_z});
        end_time_tmp = std::chrono::high_resolution_clock::now();
        calculate_centroids_duration +=
            std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_tmp - start_time_tmp).count();

        // Push children
        size_t left_child_idx = next_free_node_idx++;
        size_t right_child_idx = next_free_node_idx++;
        nodes_[node_idx].left_child_idx = left_child_idx;
        nodes_[node_idx].right_child_idx = right_child_idx;
        s.emplace(left_child_idx, depth + 1, TB_L, CB_L, t_begin, N_L);
        s.emplace(right_child_idx, depth + 1, TB_R, CB_R, t_begin + N_L, N_R);
    }

    auto build_loop_end = std::chrono::high_resolution_clock::now();
    auto build_loop_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(build_loop_end - build_loop_start).count();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    std::cout << "Parallel BVH building time: " << duration / 1000.0 << " ms" << std::endl;
    std::cout << "  Init time (precalculate aabbs and centroids): " << init_duration / 1000.0 << " ms" << std::endl;
    std::cout << "  BVH build loop time: " << build_loop_duration / 1000.0 << " ms" << std::endl;

    std::cout << "    Calculate bin ids time: " << calculate_bin_ids_duration / 1000000.0 << " ms" << std::endl;
    std::cout << "    Calculate best split and bins time: " << calculate_best_split_and_bins_duration / 1000000.0
              << " ms" << std::endl;
    std::cout << "    Rearange triangles time: " << rearange_triangles_duration / 1000000.0 << " ms" << std::endl;
    std::cout << "    Calculate centroids time: " << calculate_centroids_duration / 1000000.0 << " ms" << std::endl;
    int64_t loop_overhead =
        (build_loop_duration * 1000.0) - (calculate_bin_ids_duration + calculate_best_split_and_bins_duration +
                                          rearange_triangles_duration + calculate_centroids_duration);
    std::cout << "    Loop & structure node creation overhead: " << loop_overhead / 1000000.0 << " ms" << std::endl;
}

std::vector<std::shared_ptr<const Triangle>> BvhPar::Search(const Ray& r, bool first_hit) const {
    search_count_++;
    size_t search_nodes_visited = 0;
    size_t search_leaves_visited = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::shared_ptr<const Triangle>> result;
    result.reserve(max_triangles_per_BB_);

    std::stack<size_t> s;
    if (collision_ray_aabb(r, nodes_[0].bounding_box)) {
        s.emplace(0);
    }
    while (!s.empty()) {
        auto node_idx = s.top();
        s.pop();

        search_nodes_visited++;

        // If node is a leaf, add trinagles and stop recursion
        if (nodes_[node_idx].t_count) {
            search_leaves_visited++;
            for (size_t i = 0; i < nodes_[node_idx].t_count; i++) {
                size_t tri_idx = triangle_indices_[nodes_[node_idx].t_begin + i];
                result.emplace_back(triangles_[tri_idx]);
            }
            continue;
        }

        // Push children
        size_t left_child_idx = nodes_[node_idx].left_child_idx;
        size_t right_child_idx = nodes_[node_idx].right_child_idx;
        if (collision_ray_aabb(r, nodes_[left_child_idx].bounding_box)) {
            s.emplace(left_child_idx);
        }
        if (collision_ray_aabb(r, nodes_[right_child_idx].bounding_box)) {
            s.emplace(right_child_idx);
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    search_time_ += duration;
    search_min_time_ = std::min(search_min_time_, duration);
    search_max_time_ = std::max(search_max_time_, duration);

    search_min_nodes_visited_ = std::min(search_min_nodes_visited_, search_nodes_visited);
    search_max_nodes_visited_ = std::max(search_max_nodes_visited_, search_nodes_visited);
    search_nodes_visited_ += search_nodes_visited;

    search_min_leaves_visited_ = std::min(search_min_leaves_visited_, search_leaves_visited);
    search_max_leaves_visited_ = std::max(search_max_leaves_visited_, search_leaves_visited);
    search_leaves_visited_ += search_leaves_visited;

    search_min_return_count_ = std::min(search_min_return_count_, result.size());
    search_max_return_count_ = std::max(search_max_return_count_, result.size());
    search_return_count_ += result.size();

    return result;
}

void BvhPar::PrintStats(std::ostream& os) const {
    auto stats = calculate_stats();
    os << "BVH Build stats: " << "\n";
    os << "  Nodes count: " << stats.nodes_count << "\n";
    os << "  Leaf count: " << stats.leaf_nodes_count << "\n";
    os << "  Leaf depth:\n";
    os << "   - Min depth: " << stats.min_depth << "\n";
    os << "   - Max depth: " << stats.max_depth << "\n";
    os << "   - Avg depth: " << stats.avg_depth << "\n";
    os << "  Leaf tris:\n";
    os << "   - Min: " << stats.min_triangles_in_leaf_nodes << "\n";
    os << "   - Max: " << stats.max_triangles_in_leaf_nodes << "\n";
    os << "   - Avg: " << stats.avg_traiangles_in_leaf_nodes << "\n\n";
    os << "BVH Search stats:\n";
    os << " - Tatal calls: " << stats.search_count << "\n";
    os << "  Nodes visited (ray-aabb tests):\n";
    os << "   - Min: " << stats.search_min_nodes_visited << "\n";
    os << "   - Max: " << stats.search_max_nodes_visited << "\n";
    os << "   - Avg: " << (float)stats.search_nodes_visited / stats.search_count << "\n";
    os << "   - Total: " << stats.search_nodes_visited << "\n";
    os << "  Leaves visited:\n";
    os << "   - Min: " << stats.search_min_leaves_visited << "\n";
    os << "   - Max: " << stats.search_max_leaves_visited << "\n";
    os << "   - Avg: " << (float)stats.search_leaves_visited / stats.search_count << "\n";
    os << "   - Total: " << stats.search_leaves_visited << "\n";
    os << "  Time:\n";
    os << "   - Min: " << stats.search_min_time / 1000.0f << " µs\n";
    os << "   - Max: " << stats.search_max_time / 1000.0f << " µs\n";
    os << "   - Avg: " << (float)stats.search_time / stats.search_count / 1000.0f << " µs\n";
    os << "   - Total time: " << stats.search_time / 1000000000.0f << " s" << "\n";
    os << "  Tris returned (ray-tri tests):\n";
    os << "   - Min: " << stats.search_min_return_count << "\n";
    os << "   - Max: " << stats.search_max_return_count << "\n";
    os << "   - Avg: " << (float)stats.search_return_count / stats.search_count << "\n";
    os << "   - Total tris returned: " << stats.search_return_count << "\n\n";
}

BvhPar::BvhParStats BvhPar::calculate_stats() const {
    BvhParStats stats;
    stats.max_depth = 0;
    stats.min_depth = std::numeric_limits<size_t>::max();
    int total_leaf_depth = 0;
    stats.max_triangles_in_leaf_nodes = 0;
    stats.min_triangles_in_leaf_nodes = std::numeric_limits<size_t>::max();
    int total_triangles_in_leaf_nodes = 0;
    stats.nodes_count = 0;
    stats.leaf_nodes_count = 0;

    std::stack<size_t> s;
    s.push(0);
    while (!s.empty()) {
        auto node_idx = s.top();
        s.pop();

        stats.nodes_count++;

        // If node is a leaf, add trinagles and stop recursion
        if (nodes_[node_idx].t_count) {
            stats.leaf_nodes_count++;

            // assert(nodes_[node_idx].depth >= 0);
            stats.min_depth = std::min(stats.min_depth, nodes_[node_idx].depth);
            stats.max_depth = std::max(stats.max_depth, nodes_[node_idx].depth);
            total_leaf_depth += nodes_[node_idx].depth;

            stats.min_triangles_in_leaf_nodes = std::min(stats.min_triangles_in_leaf_nodes, nodes_[node_idx].t_count);
            stats.max_triangles_in_leaf_nodes = std::max(stats.max_triangles_in_leaf_nodes, nodes_[node_idx].t_count);
            total_triangles_in_leaf_nodes += nodes_[node_idx].t_count;
            continue;
        }

        // Push children
        size_t left_child_idx = nodes_[node_idx].left_child_idx;
        size_t right_child_idx = nodes_[node_idx].right_child_idx;
        if (left_child_idx) {
            s.push(left_child_idx);
        }
        if (right_child_idx) {
            s.push(right_child_idx);
        }
    }

    stats.avg_depth = (float)total_leaf_depth / stats.leaf_nodes_count;
    stats.avg_traiangles_in_leaf_nodes = (float)total_triangles_in_leaf_nodes / stats.leaf_nodes_count;

    stats.search_count = search_count_;
    stats.search_nodes_visited = search_nodes_visited_;
    stats.search_min_nodes_visited = search_min_nodes_visited_;
    stats.search_max_nodes_visited = search_max_nodes_visited_;

    stats.search_min_return_count = search_min_return_count_;
    stats.search_max_return_count = search_max_return_count_;
    stats.search_return_count = search_return_count_;

    stats.search_min_time = search_min_time_;
    stats.search_max_time = search_max_time_;
    stats.search_time = search_time_;

    stats.search_min_leaves_visited = search_min_leaves_visited_;
    stats.search_max_leaves_visited = search_max_leaves_visited_;
    stats.search_leaves_visited = search_leaves_visited_;

    return stats;
}

void BvhPar::reset_stats() const {
    search_count_ = 0;
    search_nodes_visited_ = 0;
    search_min_nodes_visited_ = std::numeric_limits<size_t>::max();
    search_max_nodes_visited_ = 0;
    search_min_return_count_ = std::numeric_limits<size_t>::max();
    search_max_return_count_ = 0;
    search_return_count_ = 0;
    search_min_time_ = std::numeric_limits<long long>::max();
    search_max_time_ = 0;
    search_time_ = 0;
    search_min_leaves_visited_ = std::numeric_limits<size_t>::max();
    search_max_leaves_visited_ = 0;
    search_leaves_visited_ = 0;
}

void BvhPar::config_setup(const nlohmann::json& config) {
    std::clog << "Configuring parallel BVH..." << std::flush;

    max_triangles_per_BB_ = config.at("max_triangles_per_BB");
    max_depth_ = config.at("max_depth");
    bin_count_ = config.at("bin_count");
    horizontal_threshold_ = config.at("horizontal_threshold");

    std::clog << "\rParallel BVH configured     " << std::endl;
}

}  // namespace raytracer

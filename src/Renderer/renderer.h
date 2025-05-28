#pragma once

#include <memory>
#include <vector>

#include "include/json.hpp"

#include "src/ADS/ads.h"
#include "src/camera.h"
#include "src/color.h"
#include "src/material.h"
#include "src/ray.h"
#include "src/scene.h"
#include "src/triangle.h"
#include "src/util.h"
#include "src/vec3.h"

namespace raytracer {

class Renderer {
   public:
    enum RENDER_TYPE { DISTANCE, DIFFUSION, PHONG, BLINN_PHONG };
    enum SHADING_TYPE { FLAT, SMOOTH };
    enum ACCELERATION_DATA_STRUCTURE { NONE, OCTREE, OCTREE_PARAMETRIC };
    enum RENDER_MODE { CPU, GPU };

    Renderer(const nlohmann::json& config) : config_(config) { config_setup(config); };

    void RenderScene(const Scene& scene) const;

   private:
    void config_setup(const nlohmann::json& config);

    std::unique_ptr<Ads> setup_ads(const Scene& scene) const;
    std::vector<Ray> generate_rays(const Scene& scene) const;
    void render_scene_cpu(const Scene& scene) const;
    void render_scene_gpu(const Scene& scene) const;
    Color ray_color(const Scene& scene, const std::unique_ptr<Ads>& ads, Ray& r, int depth = 0) const;
    Color render_local_ilumination(const Scene& scene,
                                   const std::unique_ptr<Ads>& ads,
                                   const Triangle& triangle,
                                   const Material& material,
                                   const Point3& intersection_point,
                                   const Vec3& intersection_point_normal) const;
    bool is_shadowed(const std::unique_ptr<Ads>& ads, const Point3& ray_intersection_point, const Vec3& light_pos) const;
    void save_image_to_pmm(int img_width, int img_height, const std::vector<Color>& img) const;

    const nlohmann::json& config_;
    int max_depth_;
    int samples_per_triangle_;
    Color background_color_;
    RENDER_TYPE render_type_;
    float max_distance_;
    SHADING_TYPE shading_type_;
    bool cull_backfaces_;
    ACCELERATION_DATA_STRUCTURE acceleration_data_structure_;
    RENDER_MODE render_mode_;

    // Statistics variables
    mutable size_t ray_trinagle_collision_count_;
    mutable long long ray_trinagle_collision_duration_;
};

}  // namespace raytracer
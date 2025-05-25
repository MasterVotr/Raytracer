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

    Renderer(const nlohmann::json& config) : config_(config) { config_setup(config); };

    void RenderScene(const Scene& scene) const;

   private:
    std::unique_ptr<Ads> setup_ads(const Scene& scene) const;
    std::vector<Ray> generate_rays(const Scene& scene) const;
    Color ray_color(const Scene& scene, const std::unique_ptr<Ads>& ads, Ray& r, int depth = 0) const;
    Color render_distance(float t_pixel, float t_max) const;
    Color render_local_ilumination(const Scene& scene,
                                   const std::unique_ptr<Ads>& ads,
                                   const Triangle& triangle,
                                   const Material& material,
                                   const Point3& intersection_point,
                                   const Vec3& intersection_point_normal) const;
    Color render_phong(const Camera& camera,
                       const Material& material,
                       const Point3& intersection_point,
                       const Vec3& intersection_point_normal,
                       const Point3& light_pos,
                       const Color& I_l = 1.0) const;
    Color render_blinn_phong(const Camera& camera,
                             const Material& material,
                             const Point3& intersection_point,
                             const Vec3& intersection_point_normal,
                             const Point3& light_pos,
                             const Color& I_l = 1.0) const;
    Ray calculate_reflection_ray(const Ray& r, const Point3& ray_intersection_point, const Vec3& normal) const;
    Ray calculate_refraction_ray(const Ray& r, const Point3& ray_intersection_point, const Vec3& normal, float ior) const;
    bool is_shadowed(const std::unique_ptr<Ads>& ads, const Point3& ray_intersection_point, const Vec3& light_pos) const;

    void config_setup(const nlohmann::json& config);
    void save_image_to_pmm(int img_width, int img_height, std::vector<Color>& img) const;

    const nlohmann::json& config_;
    int max_depth_;
    int samples_per_triangle_;
    Color background_color_;
    RENDER_TYPE render_type_;
    float max_distance_;
    SHADING_TYPE shading_type_;
    bool cull_backfaces_;
    ACCELERATION_DATA_STRUCTURE acceleration_data_structure_;

    // Statistics variables
    mutable size_t ray_trinagle_collision_count_;
    mutable long long ray_trinagle_collision_duration_;
};

}  // namespace raytracer
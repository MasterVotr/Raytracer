#include "src/Renderer/renderer.h"

#include <ctime>
#include <fstream>
#include <iostream>

#include "include/json.hpp"
#include "renderer.h"
#include "src/ADS/SBVH/sbvh.h"
#include "src/ADS/TDBVH/tdbvh.h"
// #include "src/ADS/BVH/BVHPar/bvh_par.h"
// #include "src/ADS/BVH/BVHPar2/bvh_par2.h"
// #include "src/ADS/BVH/BVHPar2V/bvh_par2v.h"
// #include "src/ADS/BVH/BVHSeq/bvh_seq.h"
// #include "src/ADS/BVH/BVHVec/bvh_vec.h"
// #include "src/ADS/BVH/BVHVec2/bvh_vec2.h"
#include "src/ADS/DummyAds/dummy_ads.h"
#include "src/ADS/Octree/OctreeParametric/octree_parametric.h"
#include "src/ADS/Octree/octree.h"
#include "src/ADS/ads.h"
#include "src/camera.h"
#include "src/collision_detection.h"
#include "src/color.h"
#include "src/material.h"
#include "src/ray.h"
#include "src/scene.h"
#include "src/triangle.h"
#include "src/util.h"
#include "src/vec3.h"

namespace raytracer {

std::vector<float> Renderer::RenderScene(const Scene& scene) const {
    std::vector<float> img;
    img.reserve(scene.GetCamera().width * scene.GetCamera().height * 3);
    ray_trinagle_collision_count_ = 0;
    ray_trinagle_collision_duration_ = 0;
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dis(0.0f, 0.001f);

    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<Ray> rays = generate_rays(scene);
    std::unique_ptr<Ads> ads = setup_ads();
    ads->Build(scene.GetTriangles());

    for (size_t r = 0; r < rays.size(); r++) {
        if (r % scene.GetCamera().height == 0) {
            std::clog << "\rRendering scene... " << (r * 1.0 / rays.size()) * 100.0 << "%     " << std::flush;
        }

        Color pixel_color = ray_color(scene, ads, rays[r]);
        for (int i = 0; i < scene.GetCamera().samples_per_pixel - 1; i++) {
            float x_jitter = dis(gen);
            float y_jitter = dis(gen);
            Ray jitter_ray(rays[r].origin(), rays[r].direction() + Vec3(x_jitter, y_jitter, 0.0f));
            pixel_color += ray_color(scene, ads, jitter_ray);
        }
        pixel_color = pixel_color / (scene.GetCamera().samples_per_pixel);
        pixel_color = clamp_color(pixel_color);
        img.emplace_back(pixel_color.x);
        img.emplace_back(pixel_color.y);
        img.emplace_back(pixel_color.z);
    }
    std::clog << "\rRendering done               " << std::endl;

    ads->PrintStats(std::cout);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << "Ray-triangle collisions:\n";
    std::cout << "  - Avg count per ray: " << (float)ray_trinagle_collision_count_ / rays.size() << "\n";
    std::cout << "  - Avg duration per ray: " << (float)ray_trinagle_collision_duration_ / rays.size() / 1000.0f
              << " µs\n";
    std::cout << "  - Total count: " << ray_trinagle_collision_count_ << "\n";
    std::cout << "  - Total duration: " << ray_trinagle_collision_duration_ / 1000000000.0f << " s\n";
    std::cout << "Rendering time: " << duration / 1000.0 << " s\n";
    std::cout << std::endl;

    return img;
}

std::unique_ptr<Ads> Renderer::setup_ads() const {
    switch (acceleration_data_structure_) {
        case NONE:
            return std::make_unique<DummyAds>(config_.at("acceleration_data_structure"));
        case OCTREE:
            return std::make_unique<Octree>(config_.at("acceleration_data_structure"));
        case OCTREE_PARAMETRIC:
            return std::make_unique<OctreeParametric>(config_.at("acceleration_data_structure"));
        case TDBVH:
            return std::make_unique<TDBvh>(config_.at("acceleration_data_structure"));
        // case BVH_SEQ:
        //     return std::make_unique<BvhSeq>(config_.at("acceleration_data_structure"));
        case SBVH:
            return std::make_unique<SBvh>(config_.at("acceleration_data_structure"));
        // case BVH_VEC:
        //     return std::make_unique<BvhVec>(config_.at("acceleration_data_structure"));
        // case BVH_VEC2:
        //     return std::make_unique<BvhVec2>(config_.at("acceleration_data_structure"));
        // case BVH_PAR:
        //     return std::make_unique<BvhPar>(config_.at("acceleration_data_structure"));
        // case BVH_PAR2:
        //     return std::make_unique<BvhPar2>(config_.at("acceleration_data_structure"));
        // case BVH_PAR2V:
        //     return std::make_unique<BvhPar2V>(config_.at("acceleration_data_structure"));
        default:
            throw std::runtime_error("Unknown ADS!");
    }
}

std::vector<Ray> Renderer::generate_rays(const Scene& scene) const {
    float width = scene.GetCamera().width;
    float height = scene.GetCamera().height;
    Vec3 origin = scene.GetCamera().pos;
    Vec3 camera_dir = scene.GetCamera().dir;
    Vec3 camera_up = scene.GetCamera().up;
    float camera_fov = scene.GetCamera().fov;

    Vec3 b = cross(camera_dir, camera_up).normalize();
    float gw = 2.0 * std::tan(camera_fov / 2.0f);
    float gh = gw * (height / width);
    Vec3 qw = b * (gw / (width - 1));
    Vec3 qh = -camera_up * (gh / (height - 1));
    Vec3 p00 = camera_dir - (b * (gw / 2)) + (camera_up * (gh / 2));

    std::vector<Ray> rays;
    rays.reserve(width * height);

    for (int y = 0; y < height; y++) {
        std::clog << "\rCreating rays... " << (y * 1.0 / height) * 100.0 << "%" << std::flush;
        for (int x = 0; x < width; x++) {
            Vec3 dir = (p00 + (qw * x) + (qh * y)).normalize();
            rays.emplace_back(origin, dir);
        }
    }
    std::clog << "\rCreating rays done               " << std::endl;

    return rays;
}

Color Renderer::ray_color(const Scene& scene, const std::unique_ptr<Ads>& ads, Ray& r, int depth) const {
    // Ray intersection with the scene
    Color final_color(0.0);
    auto t_pixel = infinity;
    auto t_max = max_distance_;
    auto triangle_hit = -1;
    auto triangles = ads->Search(r);

    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < triangles.size(); i++) {
        auto t = collision_ray_triangle(r, *triangles[i], cull_backfaces_);
        // miss or too close
        if (t == infinity || t < epsilon) {
            continue;
        }
        // closer hit
        if (t < t_pixel) {
            t_pixel = t;
            triangle_hit = i;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    ray_trinagle_collision_duration_ += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    ray_trinagle_collision_count_ += triangles.size();

    // No point intersection
    if (t_pixel == infinity) {
        return background_color_;
    }

    const auto& triangle = triangles[triangle_hit];
    const auto& material = scene.GetMaterials()[triangle->material_id];
    auto ray_intersection_point = r.origin() + r.direction() * t_pixel;
    Vec3 normal = triangle->normal;
    if (shading_type_ == SMOOTH) {
        normal = interpolate_normal_on_triangle(*triangle, ray_intersection_point);
    }
    switch (render_type_) {
        case DISTANCE: {
            final_color += render_distance(t_pixel, t_max);
            break;
        }
        case DIFFUSION: {
            final_color += material.diffuse;
            break;
        }
        case PHONG:
        case BLINN_PHONG: {
            final_color += render_local_ilumination(scene, ads, *triangle, material, ray_intersection_point, normal);
            break;
        }
        default:
            std::cerr << "Invalid render type" << std::endl;
            exit(1);
    }

    final_color = clamp_color(final_color);

    if (final_color == Vec3(1.0)) {
        return final_color;  // Early exit if color is white
    }

    // Reflection and refraction if not already white
    if (depth < max_depth_) {
        // Reflection
        if (!(material.specular == Vec3(0.0))) {
            Ray reflection_ray = calculate_reflection_ray(r, ray_intersection_point, normal);
            auto I_R = ray_color(scene, ads, reflection_ray, depth + 1);
            final_color += I_R * material.specular;
        }
        // Refraction
        if (!(material.transmittance == Vec3(0.0))) {
            Ray refraction_ray = calculate_refraction_ray(r, ray_intersection_point, normal, material.ior);
            if (!(refraction_ray.direction() == Vec3(0.0f))) {
                auto I_T = ray_color(scene, ads, refraction_ray, depth + 1);
                final_color += I_T * material.transmittance;
            }
        }
    }

    return final_color;
}

Color Renderer::render_distance(float t_pixel, float t_max) const {
    float greyscale = 1.0f - (std::min(t_pixel, 1.5f * t_max) / (1.5f * t_max));
    return Color(greyscale, greyscale, greyscale);
}

Color Renderer::render_local_ilumination(const Scene& scene, const std::unique_ptr<Ads>& ads, const Triangle& triangle,
                                         const Material& material, const Point3& intersection_point,
                                         const Vec3& intersection_point_normal) const {
    Color final_color(0.0);
    for (size_t l = 0; l < scene.GetLights().size(); l++) {
        const auto& light = scene.GetLights()[l];
        const auto& light_material = scene.GetMaterials()[light->material_id];
        if (*scene.GetLights()[l] == triangle) {
            return light_material.emission;  // Early exit if the triangle is a light source
        }
        auto S_l = calculate_triangle_area(*light);
        Color accumulated_color(0.0);
        for (int s = 0; s < samples_per_triangle_; s++) {
            auto p_l = rand_point_on_triangle(*light);
            bool shadowed = is_shadowed(ads, intersection_point, p_l, &triangle);
            if (!shadowed) {
                auto& n_l = light->normal;
                auto d_l = (p_l - intersection_point).normalize();  // normalize
                auto s = samples_per_triangle_;
                auto d = (p_l - intersection_point).length();
                auto w = (S_l * std::max(0.0f, dot(n_l, (-d_l)))) / (s * d * d + epsilon);
                auto I_l = light_material.emission * w;
                switch (render_type_) {
                    case PHONG: {
                        accumulated_color += render_phong(scene.GetCamera(), material, intersection_point,
                                                          intersection_point_normal, p_l, I_l);
                        break;
                    }
                    case BLINN_PHONG: {
                        accumulated_color += render_blinn_phong(scene.GetCamera(), material, intersection_point,
                                                                intersection_point_normal, p_l, I_l);
                        break;
                    }
                    default: {
                        std::cerr << "Invalid local ilumination render type" << std::endl;
                        exit(1);
                    }
                }
            }
        }
        final_color += accumulated_color;
    }

    return final_color;
}

Color Renderer::render_phong(const Camera& camera, const Material& material, const Point3& intersection_point,
                             const Vec3& intersection_point_normal, const Point3& light_pos, const Color& I_l) const {
    // Compute the light direction, view direction and reflection direction
    auto d_l = (light_pos - intersection_point).normalize();
    auto d_v = (camera.pos - intersection_point).normalize();
    auto d_r = intersection_point_normal * 2.0f * dot(intersection_point_normal, d_l) - d_l;

    // Compute ambient, diffuse, specular, reflection and reflaction components
    auto I_a = I_l * Color(0.0f);  // useless Ambient color
    auto I_d = I_l * material.diffuse * std::max(0.0f, dot(intersection_point_normal, d_l));
    auto I_s = I_l * material.specular * std::pow(std::max(0.0f, dot(d_v, d_r)), material.shininess);
    auto I_e = material.emission;

    return I_a + I_d + I_s + I_e;
}

Color Renderer::render_blinn_phong(const Camera& camera, const Material& material, const Point3& intersection_point,
                                   const Vec3& intersection_point_normal, const Point3& light_pos,
                                   const Color& I_l) const {
    // Compute the light direction, view direction, and halfway vector
    auto d_l = (light_pos - intersection_point).normalize();
    auto d_v = (camera.pos - intersection_point).normalize();
    auto d_h = (d_l + d_v).normalize();  // Halfway vector for Blinn-Phong

    // Compute ambient, diffuse, specular, reflection, and refraction components
    auto I_a = I_l * Color(0.0f);  // Useless ambient color
    auto I_d = I_l * material.diffuse * std::max(0.0f, dot(intersection_point_normal, d_l));
    auto I_s =
        I_l * material.specular * std::pow(std::max(0.0f, dot(intersection_point_normal, d_h)), material.shininess);

    return I_a + I_d + I_s;
}

bool Renderer::is_shadowed(const std::unique_ptr<Ads>& ads, const Point3& ray_intersection_point, const Vec3& light_pos,
                           const Triangle* const hit_triangle) const {
    auto dist_light = (light_pos - ray_intersection_point).length();
    Vec3 shadow_ray_direction = (light_pos - ray_intersection_point).normalize();
    auto shadow_ray = Ray(ray_intersection_point, shadow_ray_direction);
    float shadow_t;
    auto triangles = ads->Search(shadow_ray);

    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < triangles.size(); i++) {
        shadow_t = collision_ray_triangle(shadow_ray, *triangles[i], cull_backfaces_);
        if (shadow_t > dist_light * epsilon && shadow_t <= dist_light * (1 - epsilon)) {
            auto end = std::chrono::high_resolution_clock::now();
            ray_trinagle_collision_duration_ +=
                std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
            ray_trinagle_collision_count_ += i + 1;
            return true;  // Shadowed
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    ray_trinagle_collision_duration_ += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    ray_trinagle_collision_count_ += triangles.size();
    return false;  // Not shadowed
}

Ray Renderer::calculate_reflection_ray(const Ray& r, const Point3& ray_intersection_point, const Vec3& normal) const {
    Vec3 d_v = -r.direction();
    auto& d_n = normal;
    auto d_r = d_n * 2.0 * dot(d_n, d_v) - d_v;
    Ray reflection_ray = Ray(ray_intersection_point, d_r.normalize());
    return reflection_ray;
}

Ray Renderer::calculate_refraction_ray(const Ray& r, const Point3& ray_intersection_point, const Vec3& normal,
                                       float ior) const {
    Vec3 d_v = -r.direction();  // Inverted direction
    auto& d_n = normal;
    float n1 = 1.0;  // Air
    float n2 = ior;
    float n = n1 / n2;
    float ndotv = dot(d_n, d_v);
    auto t = d_v * -n + d_n * (n * ndotv - sqrt(1 - n * n * (1 - ndotv * ndotv)));
    if (t.length() > epsilon) {
        return Ray(ray_intersection_point, t.normalize());
    }
    return Ray(ray_intersection_point, Vec3(0.0f));
}

void Renderer::config_setup(const nlohmann::json& config) {
    std::clog << "Configuring renderer..." << std::flush;
    background_color_ =
        Color(config.at("background_color")[0], config.at("background_color")[1], config.at("background_color")[2]);
    max_depth_ = config.at("max_depth");
    samples_per_triangle_ = config.at("samples_per_triangle");
    cull_backfaces_ = config.at("cull_backfaces");
    max_distance_ = config.at("max_distance");

    // Setup render type
    if (config.at("render_type") == "distance") {
        render_type_ = DISTANCE;
    } else if (config.at("render_type") == "diffusion") {
        render_type_ = DIFFUSION;
    } else if (config.at("render_type") == "phong") {
        render_type_ = PHONG;
    } else if (config.at("render_type") == "blinn_phong") {
        render_type_ = BLINN_PHONG;
    } else {
        throw std::runtime_error("Invalid render type");
    }

    // Setup shading type
    if (config.at("shading_type") == "flat") {
        shading_type_ = FLAT;
    } else if (config.at("shading_type") == "smooth") {
        shading_type_ = SMOOTH;
    } else {
        throw std::runtime_error("Invalid shading type");
    }

    // Setup acceleration data structure
    if (config.at("acceleration_data_structure").at("name") == "none") {
        acceleration_data_structure_ = NONE;
    } else if (config.at("acceleration_data_structure").at("name") == "octree") {
        acceleration_data_structure_ = OCTREE;
    } else if (config.at("acceleration_data_structure").at("name") == "octee_parametric") {
        acceleration_data_structure_ = OCTREE_PARAMETRIC;
    } else if (config.at("acceleration_data_structure").at("name") == "tdbvh") {
        acceleration_data_structure_ = TDBVH;
    } else if (config.at("acceleration_data_structure").at("name") == "sbvh") {
        acceleration_data_structure_ = SBVH;
        // } else if (config.at("acceleration_data_structure").at("name") == "bvh_seq") {
        //     acceleration_data_structure_ = BVH_SEQ;
        // } else if (config.at("acceleration_data_structure").at("name") == "bvh_vec") {
        //     acceleration_data_structure_ = BVH_VEC;
        // } else if (config.at("acceleration_data_structure").at("name") == "bvh_vec2") {
        //     acceleration_data_structure_ = BVH_VEC2;
        // } else if (config.at("acceleration_data_structure").at("name") == "bvh_par") {
        //     acceleration_data_structure_ = BVH_PAR;
        // } else if (config.at("acceleration_data_structure").at("name") == "bvh_par2") {
        //     acceleration_data_structure_ = BVH_PAR2;
        // } else if (config.at("acceleration_data_structure").at("name") == "bvh_par2v") {
        //     acceleration_data_structure_ = BVH_PAR2V;
    } else {
        throw std::runtime_error("Invalid data structure");
    }

    std::clog << "\rRenderer configured     " << std::endl;
}

}  // namespace raytracer
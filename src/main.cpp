#include "../include/json.hpp"
#include "color.h"
#include "obj_loader.h"
#include "ray.h"
#include "util.h"
#include "vec3.h"

#include <ctime>
#include <fstream>
#include <iostream>

namespace raytracer {

class Renderer {
   public:
    enum RENDER_TYPE { DISTANCE, DIFFUSION, PHONG, BLINN_PHONG };
    enum SHADING_TYPE { FLAT, SMOOTH };

    inline Renderer(const nlohmann::json& config) : config_(config) { config_setup(config_["renderer"]); };

    void RenderScene(const Scene& scene);

   private:
    const nlohmann::json& config_;
    unsigned int max_depth_;
    unsigned int samples_per_triangle_;
    color background_color_;
    RENDER_TYPE render_type_;
    float max_distance_;
    SHADING_TYPE shading_type_;
    bool cull_backfaces_;

   private:
    std::vector<ray> generate_rays(const Scene& scene);
    color ray_color(const Scene& scene, ray& ray, unsigned int depth = 0);
    color render_distance(float t_pixel, float t_max);
    color render_local_ilumination(const Scene& scene,
                                   const Triangle& triangle,
                                   const Material& material,
                                   const point3& intersection_point,
                                   const vec3& intersection_point_normal);
    color render_phong(const Camera& camera,
                       const Material& material,
                       const point3& intersection_point,
                       const vec3& intersection_point_normal,
                       const point3& light_pos,
                       const color& I_l = 1.0);
    color render_blinn_phong(const Camera& camera,
                             const Material& material,
                             const point3& intersection_point,
                             const vec3& intersection_point_normal,
                             const point3& light_pos,
                             const color& I_l = 1.0);
    ray calculate_reflection_ray(const ray& r, const point3& ray_intersection_point, const vec3& triangle_normal);
    ray calculate_refraction_ray(const ray& r, const point3& ray_intersection_point, const vec3& triangle_normal, float ior);
    bool is_shadowed(const Scene& scene, const point3& ray_intersection_point, const vec3& light_pos);
    float collision_ray_triangle(const Triangle& triangle, ray& ray);

    void config_setup(const nlohmann::json& config);
    void save_image_to_pmm(int img_width, int img_height, std::vector<color>& img);
};

void Renderer::RenderScene(const Scene& scene) {
    std::vector<color> img;
    img.reserve(scene.GetCamera().width * scene.GetCamera().height);

    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<ray> rays = generate_rays(scene);

    for (size_t r = 0; r < rays.size(); r++) {
        if (r % 100 == 0) {
            std::clog << "\rRendering scene... " << (r * 1.0 / rays.size()) * 100.0 << "%     " << std::flush;
        }

        color pixel_color = ray_color(scene, rays[r]);
        for (size_t i = 0; i < scene.GetCamera().samples_per_pixel - 1; i++) {
            float x_jitter = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 0.001f;
            float y_jitter = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 0.001f;
            ray jitter_ray(rays[r].origin(), rays[r].direction() + vec3(x_jitter, y_jitter, 0.0f));
            pixel_color += ray_color(scene, jitter_ray);
        }
        pixel_color = pixel_color / (scene.GetCamera().samples_per_pixel);
        pixel_color = clamp_color(pixel_color);
        img.emplace_back(pixel_color);
    }
    std::clog << "\rRendering done               " << std::endl;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::clog << "Rendering time: " << duration / 1000.0 << " seconds" << std::endl;

    save_image_to_pmm(scene.GetCamera().width, scene.GetCamera().height, img);
}

std::vector<ray> Renderer::generate_rays(const Scene& scene) {
    int width = scene.GetCamera().width;
    int height = scene.GetCamera().height;
    vec3 camera_pos = scene.GetCamera().pos;
    vec3 camera_dir = scene.GetCamera().dir;
    vec3 camera_up = scene.GetCamera().up;
    float camera_fov = scene.GetCamera().fov;

    float t = 1.0f;
    vec3 b = cross(camera_dir, camera_up);
    float gw = 2 * t * std::tan(camera_fov / 2.0f);
    float gh = gw * (height / width);
    vec3 qw = b * (gw / (width - 1));
    vec3 qh = camera_up * (gh / (height - 1));
    vec3 p00 = camera_dir * t - (b * (gw / 2)) + (camera_up * (gh / 2));

    std::vector<ray> rays;
    rays.reserve(width * height);

    for (int y = 0; y < height; y++) {
        std::clog << "\rCreating rays... " << (y * 1.0 / height) * 100.0 << "%" << std::flush;
        for (int x = 0; x < width; x++) {
            vec3 pxy = p00 + (qw * x) - (qh * y);
            vec3 rxy = pxy / pxy.length();
            rays.emplace_back(camera_pos, rxy);
        }
    }
    std::clog << "\rCreating rays done               " << std::endl;

    return rays;
}

color Renderer::ray_color(const Scene& scene, ray& r, unsigned int depth) {
    // Ray intersection with the scene
    color final_color(0.0);
    auto t_pixel = infinity;
    auto t_max = max_distance_;
    auto triangle_hit = -1;
    for (size_t i = 0; i < scene.GetTriangles().size(); i++) {
        auto t = collision_ray_triangle(scene.GetTriangles()[i], r);
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

    // No point intersection
    if (t_pixel == infinity) {
        return background_color_;
    }

    const auto& triangle = scene.GetTriangles()[triangle_hit];
    const auto& material = scene.GetMaterials()[triangle.material_id];
    auto ray_intersection_point = r.origin() + r.direction() * t_pixel;
    vec3 normal = triangle.normal;
    if (shading_type_ == SMOOTH) {
        normal = interpolate_normal_on_triangle(triangle, ray_intersection_point);
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
            final_color += render_local_ilumination(scene, triangle, material, ray_intersection_point, normal);
            break;
        }
        default:
            std::cerr << "Invalid render type" << std::endl;
            exit(1);
    }

    final_color = clamp_color(final_color);

    if (final_color == vec3(1.0)) {
        return final_color;  // Early exit if color is white
    }

    // Reflection and refraction if not already white
    if (depth < max_depth_) {
        // Reflection
        if (!(material.specular == vec3(0.0))) {
            ray reflection_ray = calculate_reflection_ray(r, ray_intersection_point, normal);
            auto I_R = ray_color(scene, reflection_ray, depth + 1);
            final_color += I_R * material.specular;
        }
        // Refraction
        if (!(material.transmittance == vec3(0.0))) {
            ray refraction_ray = calculate_refraction_ray(r, ray_intersection_point, normal, material.ior);
            if (!(refraction_ray.direction() == vec3(0.0f))) {
                auto I_T = ray_color(scene, refraction_ray, depth + 1);
                final_color += I_T * material.transmittance;
            }
        }
    }

    return final_color;
}

color Renderer::render_distance(float t_pixel, float t_max) {
    float greyscale = 1.0f - (std::min(t_pixel, 1.5f * t_max) / (1.5f * t_max));
    return color(greyscale, greyscale, greyscale);
}

color Renderer::render_local_ilumination(const Scene& scene,
                                         const Triangle& triangle,
                                         const Material& material,
                                         const point3& intersection_point,
                                         const vec3& intersection_point_normal) {
    color final_color(0.0);
    for (size_t l = 0; l < scene.GetLights().size(); l++) {
        const auto& light = scene.GetLights()[l];
        const auto& light_material = scene.GetMaterials()[light.material_id];
        if (scene.GetLights()[l] == triangle) {
            return light_material.emission;  // Early exit if the triangle is a light source
        }
        auto S_l = calculate_triangle_area(light);
        color accumulated_color(0.0);
        for (size_t s = 0; s < samples_per_triangle_; s++) {
            auto p_l = rand_point_on_triangle(light);
            bool shadowed = is_shadowed(scene, intersection_point, p_l);
            if (!shadowed) {
                auto& n_l = light.normal;
                auto d_l = (p_l - intersection_point).normalize();  // normalize
                auto s = samples_per_triangle_;
                auto d = (p_l - intersection_point).length();
                auto w = (S_l * std::max(0.0f, dot(n_l, (-d_l)))) / (s * d * d + epsilon);
                auto I_l = light_material.emission * w;
                switch (render_type_) {
                    case PHONG: {
                        accumulated_color += render_phong(scene.GetCamera(), material, intersection_point, intersection_point_normal, p_l, I_l);
                        break;
                    }
                    case BLINN_PHONG: {
                        accumulated_color += render_blinn_phong(scene.GetCamera(), material, intersection_point, intersection_point_normal, p_l, I_l);
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

color Renderer::render_phong(const Camera& camera,
                             const Material& material,
                             const point3& intersection_point,
                             const vec3& intersection_point_normal,
                             const point3& light_pos,
                             const color& I_l) {
    // Compute the light direction, view direction and reflection direction
    auto d_l = (light_pos - intersection_point).normalize();
    auto d_v = (camera.pos - intersection_point).normalize();
    auto d_r = intersection_point_normal * 2.0f * dot(intersection_point_normal, d_l) - d_l;

    // Compute ambient, diffuse, specular, reflection and reflaction components
    auto I_a = I_l * color(0.0f);  // useless Ambient color
    auto I_d = I_l * material.diffuse * std::max(0.0f, dot(intersection_point_normal, d_l));
    auto I_s = I_l * material.specular * std::pow(std::max(0.0f, dot(d_v, d_r)), material.shininess);
    auto I_e = material.emission;

    return I_a + I_d + I_s + I_e;
}

color Renderer::render_blinn_phong(const Camera& camera,
                                   const Material& material,
                                   const point3& intersection_point,
                                   const vec3& intersection_point_normal,
                                   const point3& light_pos,
                                   const color& I_l) {
    // Compute the light direction, view direction, and halfway vector
    auto d_l = (light_pos - intersection_point).normalize();
    auto d_v = (camera.pos - intersection_point).normalize();
    auto d_h = (d_l + d_v).normalize();  // Halfway vector for Blinn-Phong

    // Compute ambient, diffuse, specular, reflection, and refraction components
    auto I_a = I_l * color(0.0f);  // Useless ambient color
    auto I_d = I_l * material.diffuse * std::max(0.0f, dot(intersection_point_normal, d_l));
    auto I_s = I_l * material.specular * std::pow(std::max(0.0f, dot(intersection_point_normal, d_h)), material.shininess);

    return I_a + I_d + I_s;
}

bool Renderer::is_shadowed(const Scene& scene, const point3& ray_intersection_point, const vec3& light_pos) {
    auto dist_light = (light_pos - ray_intersection_point).length();
    auto shadow_ray = ray(ray_intersection_point, (light_pos - ray_intersection_point).normalize());
    float shadow_t;
    for (size_t i = 0; i < scene.GetTriangles().size(); i++) {
        shadow_t = collision_ray_triangle(scene.GetTriangles()[i], shadow_ray);
        // miss
        if (shadow_t == infinity || shadow_t < epsilon) {
            continue;
        }
        // closer hit
        if (shadow_t < dist_light) {
            return true;  // Shadowed
        }
    }
    return false;  // Not shadowed
}

float Renderer::collision_ray_triangle(const Triangle& triangle, ray& ray) {
    // Extract triangle vertices and ray properties
    const vec3& a = triangle.vertices[0].pos;
    const vec3& b = triangle.vertices[1].pos;
    const vec3& c = triangle.vertices[2].pos;
    const vec3& s = ray.direction();
    const point3 o = ray.origin();

    // Compute edges of the triangle
    vec3 e1 = b - a;  // Edge 1: vector from a to b
    vec3 e2 = c - a;  // Edge 2: vector from a to c

    // Compute the determinant
    vec3 p = cross(s, e2);  // Cross product of ray direction and edge 2
    float d = dot(e1, p);   // Determinant: dot product of edge 1 and p

    // Cull backfaces if enabled
    if (cull_backfaces_) {
        if (d < epsilon) {    // If determinant is near zero, the ray is parallel to the triangle
            return infinity;  // No intersection
        }
    } else {
        if (fabs(d) < epsilon) {  // For non-culling, check if determinant is close to zero
            return infinity;      // No intersection
        }
    }

    // Compute the inverse of the determinant
    float d_inv = 1.0 / d;

    // Compute the vector from vertex a to the ray origin
    vec3 q = o - a;

    // Compute the barycentric coordinate u
    float u = d_inv * dot(q, p);
    if (u < 0.0 || u > 1.0) {  // If u is outside the range [0, 1], the intersection is outside the triangle
        ray.t_distance() = infinity;
        return infinity;
    }

    // Compute the barycentric coordinate v
    vec3 r = cross(q, e1);  // Cross product of q and edge 1
    float v = d_inv * dot(r, s);
    if (v < 0.0 || (u + v) > 1.0) {  // If v is outside the range [0, 1] or u + v > 1, the intersection is outside the triangle
        return infinity;
    }

    // Compute the distance t along the ray to the intersection point
    float t = d_inv * dot(e2, r);

    // Update the ray's t_distance and return the intersection distance
    ray.t_distance() = t;
    return t;
}

ray Renderer::calculate_reflection_ray(const ray& r, const point3& ray_intersection_point, const vec3& triangle_normal) {
    vec3 d_v = -r.direction();
    auto& d_n = triangle_normal;
    auto d_r = d_n * 2.0 * dot(d_n, d_v) - d_v;
    ray reflection_ray = ray(ray_intersection_point, d_r.normalize());
    return reflection_ray;
}

ray Renderer::calculate_refraction_ray(const ray& r, const point3& ray_intersection_point, const vec3& triangle_normal, float ior) {
    vec3 d_v = -r.direction();  // Inverted direction
    auto& d_n = triangle_normal;
    float n1 = 1.0;  // Air
    float n2 = ior;
    float n = n1 / n2;
    float ndotv = dot(d_n, d_v);
    auto t = d_v * -n + d_n * (n * ndotv - sqrt(1 - n * n * (1 - ndotv * ndotv)));
    if (t.length() > epsilon) {
        return ray(ray_intersection_point, t.normalize());
    }
    return ray(ray_intersection_point, vec3(0.0f));
}

void Renderer::config_setup(const nlohmann::json& config) {
    std::clog << "Configuring renderer..." << std::flush;
    background_color_ = color(config.at("background_color")[0], config.at("background_color")[1], config.at("background_color")[2]);
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
        std::cerr << "Invalid render type" << std::endl;
        exit(1);
    }

    // Setup shading type
    if (config.at("shading_type") == "flat") {
        shading_type_ = FLAT;
    } else if (config.at("shading_type") == "smooth") {
        shading_type_ = SMOOTH;
    } else {
        std::cerr << "Invalid shadiing type" << std::endl;
        exit(1);
    }

    std::clog << "\rRenderer configured     " << std::endl;
}

void Renderer::save_image_to_pmm(int img_width, int img_height, std::vector<color>& img) {
    std::clog << "Saving image to output.pmm" << std::endl;
    std::ofstream output(config_["output"]["filename"]);
    output << "P3\n" << img_width << ' ' << img_height << "\n255\n";
    for (const auto& pixel_color : img) {
        write_color(output, pixel_color);
    }
    std::clog << "\rImage saved to" << config_["output"]["filename"] << "        \n";
    output.close();
}

}  // namespace raytracer

int main(int argc, char const* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file>" << std::endl;
        return 1;
    }
    std::clog << "Loading config..." << std::flush;
    std::ifstream ifs(argv[1]);
    nlohmann::json config = nlohmann::json::parse(ifs);
    ifs.close();
    std::clog << "\rConfig loaded     " << std::endl;

    try {
        if (config.at("seed") != -1) {
            srand(config.at("seed"));
        } else {
            srand(time(0));
        }
        raytracer::Renderer renderer(config);
        raytracer::Scene scene = raytracer::LoadScene(config, config.at("triangulate"));
        renderer.RenderScene(scene);
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

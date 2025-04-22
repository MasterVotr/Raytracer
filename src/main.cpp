#include "color.h"
#include "obj_loader.h"
#include "ray.h"
#include "util.h"
#include "vec3.h"

#include <ctime>
#include <fstream>
#include <iostream>

const bool cull_backfaces = true;

enum RENDER_TYPE { DISTANCE, DIFFUSION, PHONG, BLINN_PHONG };
const RENDER_TYPE render_type = PHONG;

// Viewport
const auto w = 800;
const auto h = 800;

// Camera
raytracer::point3 camera_pos(278.0, 273.0, -1000.0);
raytracer::vec3 camera_up(0.0, 1.0, 0.0);
raytracer::vec3 camera_dir(0.0, 0.0, 1.0);
double camera_fov = 0.6;

// Point Lights
raytracer::vec3 light_pos(275.0, 490.0, 275.0);
raytracer::color light_color(1.0, 1.0, 1.0);
uint samples_per_triangle = 32;

std::vector<raytracer::vec3> rand_points_on_triangle(uint num_of_points, const raytracer::Triangle& t) {
    std::vector<raytracer::vec3> points;
    for (size_t i = 0; i < num_of_points; i++) {
        auto r1 = (float)rand() / (RAND_MAX);
        auto r2 = (float)rand() / (RAND_MAX);
        auto u = (r1 + r2 > 1) ? 1 - r1 : r1;
        auto v = (r1 + r2 > 1) ? 1 - r2 : r2;
        const auto& a = t.vertices[0];
        const auto& b = t.vertices[1];
        const auto& c = t.vertices[2];
        points.emplace_back(raytracer::vec3(a + (b - a) * u + (c - a) * v));
    }
    return points;
}

double CollisionRayTriangle(const raytracer::Triangle& triangle, raytracer::ray& ray) {
    // Extract triangle vertices and ray properties
    const raytracer::vec3& a = triangle.vertices[0];
    const raytracer::vec3& b = triangle.vertices[1];
    const raytracer::vec3& c = triangle.vertices[2];
    const raytracer::vec3& s = ray.direction();
    const raytracer::point3 o = ray.origin();

    // Compute edges of the triangle
    raytracer::vec3 e1 = b - a;  // Edge 1: vector from a to b
    raytracer::vec3 e2 = c - a;  // Edge 2: vector from a to c

    // Compute the determinant
    raytracer::vec3 p = raytracer::cross(s, e2);  // Cross product of ray direction and edge 2
    double d = raytracer::dot(e1, p);             // Determinant: dot product of edge 1 and p

    // Cull backfaces if enabled
    if (cull_backfaces) {
        if (d < raytracer::epsilon) {    // If determinant is near zero, the ray is parallel to the triangle
            return raytracer::infinity;  // No intersection
        }
    } else {
        if (fabs(d) < raytracer::epsilon) {  // For non-culling, check if determinant is close to zero
            return raytracer::infinity;      // No intersection
        }
    }

    // Compute the inverse of the determinant
    double d_inv = 1.0 / d;

    // Compute the vector from vertex a to the ray origin
    raytracer::vec3 q = o - a;

    // Compute the barycentric coordinate u
    double u = d_inv * raytracer::dot(q, p);
    if (u < 0.0 || u > 1.0) {  // If u is outside the range [0, 1], the intersection is outside the triangle
        ray.t_distance() = raytracer::infinity;
        return raytracer::infinity;
    }

    // Compute the barycentric coordinate v
    raytracer::vec3 r = raytracer::cross(q, e1);  // Cross product of q and edge 1
    double v = d_inv * raytracer::dot(r, s);
    if (v < 0.0 || (u + v) > 1.0) {  // If v is outside the range [0, 1] or u + v > 1, the intersection is outside the triangle
        return raytracer::infinity;
    }

    // Compute the distance t along the ray to the intersection point
    double t = d_inv * raytracer::dot(e2, r);

    // Update the ray's t_distance and return the intersection distance
    ray.t_distance() = t;
    return t;
}

raytracer::color render_phong(const raytracer::Triangle& triangle,
                              const raytracer::Material& material,
                              const raytracer::point3& intersection_point,
                              const raytracer::color& I_l = 1.0,
                              const raytracer::color& I_R = 0.0,
                              const raytracer::color& I_T = 0.0) {
    // Compute the light direction, view direction and reflection direction
    auto d_l = (light_pos - intersection_point).normalize();
    auto d_v = (camera_pos - intersection_point).normalize();
    auto d_r = triangle.normal * 2.0 * raytracer::dot(triangle.normal, d_l) - d_l;

    // Compute ambient, diffuse, specular, reflection and reflaction components
    auto I_a = I_l * raytracer::color{0.0, 0.0, 0.0};  // useless Ambient color
    auto I_d = I_l * material.diffuse * std::max(0.0, raytracer::dot(triangle.normal, d_l));
    auto I_s = I_l * material.specular * std::pow(std::max(0.0, raytracer::dot(d_v, d_r)), material.shininess);
    auto I_e = material.emission;
    auto I_r = I_R * material.specular;
    auto I_t = I_T * material.transmittance;

    return I_a + I_d + I_s + I_e + I_r + I_t;
}

raytracer::color render_blinn_phong(const raytracer::Triangle& triangle,
                                    const raytracer::Material& material,
                                    const raytracer::point3& intersection_point,
                                    const raytracer::color& I_l = 1.0,
                                    const raytracer::color& I_R = 0.0,
                                    const raytracer::color& I_T = 0.0) {
    // Compute the light direction, view direction, and halfway vector
    auto d_l = (light_pos - intersection_point).normalize();
    auto d_v = (camera_pos - intersection_point).normalize();
    auto d_h = (d_l + d_v).normalize();  // Halfway vector for Blinn-Phong

    // Compute ambient, diffuse, specular, reflection, and refraction components
    auto I_a = I_l * raytracer::color{0.0, 0.0, 0.0};  // Useless ambient color
    auto I_d = I_l * material.diffuse * std::max(0.0, raytracer::dot(triangle.normal, d_l));
    auto I_s = I_l * material.specular * std::pow(std::max(0.0, raytracer::dot(triangle.normal, d_h)), material.shininess);
    auto I_r = I_R * material.specular;
    auto I_t = I_T * material.transmittance;

    return I_a + I_d + I_s + I_r + I_t;
}

bool is_shadowed(const raytracer::Scene& scene, const raytracer::point3& ray_intersection_point, const raytracer::vec3& light_pos) {
    auto dist_light = (light_pos - ray_intersection_point).length();
    auto shadow_ray = raytracer::ray(ray_intersection_point, (light_pos - ray_intersection_point).normalize());
    double shadow_t;
    for (size_t i = 0; i < scene.GetTriangles().size(); i++) {
        shadow_t = CollisionRayTriangle(scene.GetTriangles()[i], shadow_ray);
        // miss
        if (shadow_t == raytracer::infinity || shadow_t < raytracer::epsilon) {
            continue;
        }
        // closer hit
        if (shadow_t < dist_light) {
            return true;  // Shadowed
        }
    }
    return false;  // Not shadowed
}

raytracer::color ray_color(const raytracer::Scene& scene, raytracer::ray& ray) {
    const auto& triangles = scene.GetTriangles();
    // Color computation
    raytracer::color final_color(0.0, 0.0, 0.0);
    auto t_pixel = raytracer::infinity;
    auto t_max = 2000.0;
    auto triangle_hit = -1;
    for (size_t i = 0; i < scene.GetTriangles().size(); i++) {
        auto t = CollisionRayTriangle(scene.GetTriangles()[i], ray);
        // miss
        if (t == raytracer::infinity || t < raytracer::epsilon) {
            continue;
        }
        // closer hit
        if (t < t_pixel) {
            t_pixel = t;
            triangle_hit = i;
        }
    }

    // No point intersection
    if (t_pixel == raytracer::infinity) {
        return final_color;
    }

    const auto& triangle = scene.GetTriangles()[triangle_hit];
    const auto& material = scene.GetMaterials()[triangle.material_id];
    auto ray_intersection_point = ray.origin() + ray.direction() * t_pixel;
    switch (render_type) {
        case DISTANCE: {
            auto greyscale = 1 - (std::min(t_pixel, 1.5 * t_max) / (1.5 * t_max));
            final_color = raytracer::vec3(greyscale, greyscale, greyscale);
            break;
        }
        case DIFFUSION: {
            if (t_pixel != raytracer::infinity) {
                final_color = material.diffuse;
            }
            break;
        }
        case PHONG: {
            for (size_t i = 0; i < scene.GetLights().size(); i++) {
                const auto& light = scene.GetLights()[i];
                const auto& light_material = scene.GetMaterials()[light.material_id];
                auto p_ls = rand_points_on_triangle(samples_per_triangle, light);
                auto S_l = raytracer::calculate_triangle_area(light);
                raytracer::color accumulated_color(0.0, 0.0, 0.0);
                for (const auto& p_l : p_ls) {
                    bool shadowed = is_shadowed(scene, ray_intersection_point, p_l);
                    if (!shadowed) {
                        auto& n_l = light.normal;
                        auto d_l = (p_l - ray_intersection_point).normalize();  // normalize
                        auto s = samples_per_triangle;
                        auto d = (p_l - ray_intersection_point).length();
                        auto w = (S_l * std::max(0.0, raytracer::dot(n_l, (-d_l)))) / (s * d * d + raytracer::epsilon);
                        auto I_l = light_material.emission * w;
                        accumulated_color += render_phong(triangle, material, ray_intersection_point, I_l);
                    }
                }
                final_color += accumulated_color;
            }
            break;
        }
        case BLINN_PHONG: {
            for (size_t i = 0; i < scene.GetLights().size(); i++) {
                const auto& light = scene.GetLights()[i];
                const auto& light_material = scene.GetMaterials()[light.material_id];
                auto p_ls = rand_points_on_triangle(samples_per_triangle, light);
                auto S_l = raytracer::calculate_triangle_area(light);
                raytracer::color accumulated_color(0.0, 0.0, 0.0);
                for (const auto& p_l : p_ls) {
                    bool shadowed = is_shadowed(scene, ray_intersection_point, p_l);
                    if (!shadowed) {
                        auto& n_l = light.normal;
                        auto d_l = (p_l - ray_intersection_point).normalize();  // normalize
                        auto s = samples_per_triangle;
                        auto d = (p_l - ray_intersection_point).length();
                        auto w = (S_l * std::max(0.0, raytracer::dot(n_l, (-d_l)))) / (s * d * d + raytracer::epsilon);
                        auto I_l = light_material.emission * w;
                        accumulated_color += render_blinn_phong(triangle, material, ray_intersection_point, I_l);
                    }
                }
                final_color += accumulated_color;
            }
            break;
        }
        default:
            std::cerr << "Invalid render type" << std::endl;
            exit(1);
    }

    // TODO: Add reflection and refraction

    final_color.x = raytracer::clamp(final_color.x, 0.0, 1.0);
    final_color.y = raytracer::clamp(final_color.y, 0.0, 1.0);
    final_color.z = raytracer::clamp(final_color.z, 0.0, 1.0);
    return final_color;
}

static void SaveImageToPmm(int img_width, int img_height, std::vector<raytracer::color>& img) {
    std::clog << "Saving image to output.pmm" << std::endl;
    std::ofstream output("output.ppm");
    output << "P3\n" << img_width << ' ' << img_height << "\n255\n";
    for (const auto& pixel_color : img) {
        raytracer::write_color(output, pixel_color);
    }
    std::clog << "\rImage saved to output.pmm        \n";
    output.close();
}

void RenderScene(const raytracer::Scene& scene) {
    double t = 1.0;
    raytracer::vec3 b = raytracer::cross(camera_dir, camera_up);
    double gw = 2 * t * std::tan(camera_fov / 2.0);
    double gh = gw * (h / w);
    raytracer::vec3 qw = b * (gw / (w - 1));
    raytracer::vec3 qh = camera_up * (gh / (h - 1));
    raytracer::vec3 p00 = camera_dir * t - (b * (gw / 2)) + (camera_up * (gh / 2));
    raytracer::vec3 r00 = p00 / p00.length();

    std::vector<raytracer::color> img;

    for (int y = 0; y < h; y++) {
        std::clog << "\rRendering scene... " << (y * 1.0 / h) * 100.0 << "%" << std::flush;
        for (int x = 0; x < w; x++) {
            auto pxy = p00 + (qw * x) - (qh * y);
            auto rxy = pxy / pxy.length();
            raytracer::ray r(camera_pos, rxy);

            raytracer::color pixel_color = ray_color(scene, r);
            img.emplace_back(pixel_color);
        }
    }
    std::clog << "\rRendering done               " << std::endl;

    SaveImageToPmm(w, h, img);
}

int main(int argc, char const* argv[]) {
    srand(time(0));
    const char* basepath = "res/";
    auto scene = raytracer::LoadScene("res/CornellBox-Original.obj", basepath);

    RenderScene(scene);
}

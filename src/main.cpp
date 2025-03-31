#include "color.h"
#include "obj_loader.h"
#include "ray.h"
#include "util.h"
#include "vec3.h"

#include <fstream>
#include <iostream>

const bool cull_backfaces = true;

enum RENDER_TYPE { GREYSCALE, DIFFUSION, PHONG };
const RENDER_TYPE render_type = DIFFUSION;

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

raytracer::color ray_color(const raytracer::Scene& scene, raytracer::ray& ray) {
    const auto& triangles = scene.GetTriangles();
    // Color computation
    auto t_pixel = raytracer::infinity;
    auto t_max = 2000.0;
    auto triangle_hit = -1;
    for (size_t i = 0; i < scene.GetTriangles().size(); i++) {
        auto t = CollisionRayTriangle(scene.GetTriangles()[i], ray);
        // miss
        if (t == raytracer::infinity) {
            continue;
        }
        // closer hit
        if (t < t_pixel) {
            t_pixel = t;
            triangle_hit = i;
        }
    }

    // Compute color
    raytracer::color final_color(0.0, 0.0, 0.0);
    if (t_pixel == raytracer::infinity) {
        return final_color;
    }

    switch (render_type) {
        case GREYSCALE: {
            auto greyscale = 1 - (std::min(t_pixel, 1.5 * t_max) / (1.5 * t_max));
            final_color = raytracer::vec3(greyscale, greyscale, greyscale);
            break;
        }
        case DIFFUSION: {
            if (t_pixel != raytracer::infinity) {
                auto material_id = scene.GetTriangles()[triangle_hit].material_id;
                final_color = scene.GetMaterials()[material_id].diffuse;
            }
            break;
        }
        case PHONG: {
            auto material_id = scene.GetTriangles()[triangle_hit].material_id;
            const auto& material = scene.GetMaterials()[material_id];
            
            raytracer::vec3 I_a(0.0, 0.0, 0.0);  // Useless Ambient color

            auto I = I_a + I_d + I_s + I_r + I_t;
            final_color = I;
            break;
        }
        default:
            std::cerr << "Invalid render type" << std::endl;
            exit(1);
    }

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
    const char* basepath = "scenes/";
    auto scene = raytracer::LoadScene("scenes/CornellBox-Original.obj", basepath);

    RenderScene(scene);
}

/*struct Sphere {
    Sphere(float rad_, vec3 p_):
        rad(rad_), p(p_) {}

    float intersect(const Ray &r) const { // returns distance, 0 if nohit
        vec3 op = p - r.o; // Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0
        float t, eps=1e-4, b=op.dot(r.d), det=b*b-op.dot(op)+rad*rad;
        if (det<0) return 0; else det=sqrt(det);
        return (t=b-det)>eps ? t : ((t=b+det)>eps ? t : 0);
    }

    float rad;  // radius
    vec3 p;     // position
}; */

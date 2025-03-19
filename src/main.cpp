#define TINYOBJLOADER_IMPLEMENTATION
#include "color.h"
#include "obj_loader.h"
#include "ray.h"
#include "util.h"
#include "vec3.h"

#include <fstream>
#include <iostream>

// Viewport
const auto w = 1000;
const auto h = 1000;
const auto aspect_ratio = 16.0 / 9.0;

// Camera
raytracer::point3 o(278.0, 273.0, -1000.0);
raytracer::vec3 u(0.0, 1.0, 0.0);
raytracer::vec3 d(0.0, 0.0, 1.0);
float fov = 0.6;

double CollisionRayTriangle(const raytracer::Triangle& triangle, raytracer::ray& ray) {
    const raytracer::vec3& a = triangle.vertices[0];
    const raytracer::vec3& b = triangle.vertices[1];
    const raytracer::vec3& c = triangle.vertices[2];
    const raytracer::vec3& s = ray.direction();
    const raytracer::point3 o = ray.origin();

    raytracer::vec3 e1 = b - a;
    raytracer::vec3 e2 = c - a;
    raytracer::vec3 p = raytracer::cross(s, e2);
    double d = raytracer::dot(e1, p);

    if (d < raytracer::epsilon) {
        ray.t_distance() = raytracer::infinity;
        return raytracer::infinity;
    }

    double d_inv = 1 / d;
    raytracer::vec3 q = o - a;
    double u = d_inv * raytracer::dot(q, p);
    if (u < 0.0 && u > 1.0) {
        ray.t_distance() = raytracer::infinity;
        return raytracer::infinity;
    }

    raytracer::vec3 r = raytracer::cross(q, e1);
    double v = d_inv * raytracer::dot(r, s);
    if (v < 0.0 && u + v > 1.0) {
        ray.t_distance() = raytracer::infinity;
        return raytracer::infinity;
    }

    double t = d_inv * raytracer::dot(e2, r);
    ray.t_distance() = t;
    return t;
}

double global_t_min = raytracer::infinity;
double global_t_max = -raytracer::infinity;

raytracer::color ray_color(const std::vector<raytracer::Triangle>& scene, raytracer::ray& ray) {
    // Color computation
    auto t_pixel = CollisionRayTriangle(scene[0], ray);
    auto t_max = 60000.0;
    auto triangle_hit = -1;
    for (size_t i = 0; i < scene.size(); i++) {
        auto t = CollisionRayTriangle(scene[i], ray);
        // miss
        if (t == raytracer::infinity) {
            continue;
        } else {  // hit
        }
        // nearest hit
        if (t < t_pixel) {
            t_pixel = t;
            triangle_hit = i;
        }
        // max t
        if (t > global_t_max) {
            global_t_max = t;
            std::clog << "New global max: " << global_t_max << " at ray " << ray.origin() << " at triangle " << scene[i] << std::endl;
        }
        // min t
        if (t < global_t_min) {
            global_t_min = t;
        }
    }
    std::clog << "Hit at triagle " << triangle_hit << ": [" << scene[triangle_hit] << "]\tat t: " << t_pixel << std::endl;
    return {triangle_hit / 34.0, triangle_hit / 34.0, triangle_hit / 34.0};

    // Compute color from distance
    auto greyscale = 1 - (std::min(t_pixel, 1.5 * t_max) / (1.5 * t_max));
    // auto greyscale = raytracer::clamp(greyscale_unclamped, 0.0, 1.0);
    raytracer::color final_color(greyscale, greyscale, greyscale);
    // std::clog << "ray from: " << ray.origin() << "    \tdir: " << ray.direction() << "   \tt_max: " << t_max << " \tt_count: " << scene.size()
    //   << " \tt_pixel: " << t_pixel << " \tgreyscale: " << greyscale << std::endl;

    // std::clog << final_color << std::endl;

    return final_color;
}

static void SaveImageToPmm(int img_width, int img_height, std::vector<raytracer::color>& img) {
    std::clog << "Saving image to output.pmm" << std::endl;
    std::ofstream output("output.ppm");
    output << "P3\n" << img_width << ' ' << img_height << "\n255\n";
    for (const auto& pixel_color : img) {
        raytracer::write_color(output, pixel_color);
    }
    std::clog << "\rImage saving done       \n";
    output.close();
}

void RenderScene(const std::vector<raytracer::Triangle>& scene) {
    double t = 1.0;
    raytracer::vec3 b = raytracer::cross(d, u);
    double gw = 2 * t * std::tan(fov / 2);
    double gh = gw * (h / w);
    raytracer::vec3 qw = b * (gw / (w - 1));
    raytracer::vec3 qh = u * (gh / (h - 1));
    raytracer::vec3 p00_loc = d * t - (b * (gw / 2)) + (u * (gh / 2));
    raytracer::vec3 r00 = p00_loc / p00_loc.length();
    std::clog << "t: " << t << "\n"
              << "b: " << b << "\n"
              << "gw: " << gw << "\n"
              << "gh: " << gh << "\n"
              << "qw: " << qw << "\n"
              << "qh: " << qh << "\n"
              << "p00_loc: " << p00_loc << "\n"
              << "r00: " << r00 << std::endl;

    std::vector<raytracer::color> img;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            auto pxy = p00_loc + (qw * x) - (qh * y);
            auto rxy = pxy / pxy.length();
            raytracer::ray r(pxy, rxy);

            // std::clog << "Pos of (" << x << ", " << y << ") = (" << pxy << ") and dir: (" << rxy << ")\n";

            raytracer::color pixel_color = ray_color(scene, r);
            img.emplace_back(pixel_color);
        }
    }

    SaveImageToPmm(w, h, img);

    std::clog << "global_t_min: " << global_t_min << " global_t_max: " << global_t_max << std::endl;
}

int main(int argc, char const* argv[]) {
    const char* basepath = "scenes/";
    auto res = raytracer::LoadScene("scenes/CornellBox-Original.obj", basepath);

    RenderScene(res);
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


/*
    SaveImageToPmm(img_width, img_height, img);

    // Image rendering
    float* image = new float[img_width * img_height * 3];

    // Creating a gradient texture
    std::clog << "Generating testing output.pmm" << std::endl;
    std::ofstream output("output.ppm");
    output << "P3\n" << img_width << ' ' << img_height << "\n255\n";
    for (int j = 0; j < img_height; ++j) {
        std::clog << "\rScanlines remaining: " << (img_height - j) << ' ' << std::flush;
        for (int i = 0; i < img_width; ++i) {
            auto pixel_color = raytracer::color(double(i) / (img_width - 1), double(j) / (img_height - 1), 0.0);
            raytracer::write_color(output, pixel_color);
        }
    }
    std::clog << "\rImage saving done       \n";
    output.close();
*/
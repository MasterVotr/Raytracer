#define TINYOBJLOADER_IMPLEMENTATION
#include "color.h"
#include "obj_loader.h"
#include "ray.h"
#include "util.h"
#include "vec3.h"

#include <fstream>
#include <iostream>

// Image
const int img_width = 800;
const int img_height = 450;
const auto aspect_ratio = 16.0 / 9.0;

// Viewport
const auto w = 680.0;
const auto h = 360.0;

// Camera
raytracer::point3 o(278.0, 273.0, -1000.0);
raytracer::vec3 u(0.0, 1.0, 0.0);
raytracer::vec3 d(0.0, 0.0, 1.0);
float fov = 0.6;

raytracer::color ray_color(const std::vector<raytracer::Triangle>& scene, const raytracer::ray& r) {
    // Color computation
    return raytracer::color(0, 0, 0);
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
    auto t = 1.0;
    auto b = d * u;
    auto gw = 2 * t * tan(fov / 2);
    auto gh = gw * h / w;
    auto qw = b * gw / (w - 1);
    auto qh = u * gh / (h - 1);
    auto p00_loc = d * t - b * (gw / 2) + u * (gh / 2);

    std::vector<raytracer::color> img;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            auto pxy = p00_loc + (qw * x) - (qh * y);
            auto rxy = pxy / pxy.length();
            raytracer::ray r(pxy, rxy);

            raytracer::color pixel_color = ray_color(scene, r);
            img.emplace_back(pixel_color);
        }
    }

    SaveImageToPmm(w, h, img);
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

int main(int argc, char const* argv[]) {
    const char* basepath = "scenes/";
    auto res = raytracer::LoadScene("scenes/CornellBox-Original.obj", basepath);

    RenderScene(res);
}
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
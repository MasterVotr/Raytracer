#pragma once

#include "../include/json.hpp"
#include "vec3.h"

namespace raytracer {

struct Camera {
    raytracer::point3 pos;
    raytracer::vec3 up;
    raytracer::vec3 dir;
    float fov;
    unsigned int width;
    unsigned int height;
    unsigned int samples_per_pixel;

    Camera() = default;
    Camera(const nlohmann::json& config) { config_setup(config); }

   private:
    void config_setup(const nlohmann::json& config) {
        std::clog << "Configuring camera..." << std::flush;
        pos = point3(config.at("pos")[0], config.at("pos")[1], config.at("pos")[2]);
        up = vec3(config.at("up")[0], config.at("up")[1], config.at("up")[2]);
        dir = vec3(config.at("dir")[0], config.at("dir")[1], config.at("dir")[2]);
        fov = config.at("fov");
        width = config.at("width");
        height = config.at("height");
        samples_per_pixel = config.at("samples_per_pixel");
        std::clog << "\rCamera configured     " << std::endl;
    }
};

}  // namespace raytracer

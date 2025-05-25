#pragma once

#include <iostream>

#include "include/json.hpp"
#include "src/vec3.h"

namespace raytracer {

class Camera {
   public:
    Camera() = default;
    Camera(const nlohmann::json& config) { config_setup(config); }

    Point3 pos;
    Vec3 up;
    Vec3 dir;
    float fov;
    int width;
    int height;
    int samples_per_pixel;

   private:
    void config_setup(const nlohmann::json& config) {
        std::clog << "Configuring camera..." << std::flush;
        pos = Point3(config.at("pos")[0], config.at("pos")[1], config.at("pos")[2]);
        up = Vec3(config.at("up")[0], config.at("up")[1], config.at("up")[2]);
        dir = Vec3(config.at("dir")[0], config.at("dir")[1], config.at("dir")[2]);
        fov = config.at("fov");
        width = config.at("width");
        height = config.at("height");
        samples_per_pixel = config.at("samples_per_pixel");
        std::clog << "\rCamera configured     " << std::endl;
    }
};

}  // namespace raytracer

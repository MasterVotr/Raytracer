#pragma once

#include "../include/json.hpp"
#include "camera.h"
#include "material.h"
#include "triangle.h"

#include <vector>

namespace raytracer {

class Scene {
   public:
    Scene(const nlohmann::json& config) : config_(config) { config_setup(config_.at("scene")); }

    inline void AddTriangle(const Triangle& t) { triangles_.emplace_back(t); }
    inline const std::vector<Triangle>& GetTriangles() const { return triangles_; }
    inline void AddMaterial(const Material& m) { materials_.emplace_back(m); }
    inline const std::vector<Material>& GetMaterials() const { return materials_; }
    inline void AddLight(const Triangle& t) { lights_.emplace_back(t); }
    inline const std::vector<Triangle>& GetLights() const { return lights_; }
    inline void SetCamera(const Camera& camera) { camera_ = camera; }
    inline const Camera& GetCamera() const { return camera_; }

   private:
    struct PointLight {
        point3 pos;
        vec3 color;
    };

    const nlohmann::json& config_;
    std::vector<Triangle> triangles_;
    std::vector<Material> materials_;
    std::vector<Triangle> lights_;
    Camera camera_;
    std::vector<PointLight> point_lights_;

   private:
    void config_setup(const nlohmann::json& config) {
        // Load Camera config
        camera_ = Camera(config.at("camera"));

        std::clog << "Configuring scene..." << std::flush;
        // Load point lights
        for (const auto& light : config.at("point_lights")) {
            PointLight pl;
            pl.pos = point3(light.at("pos")[0], light.at("pos")[1], light.at("pos")[2]);
            pl.color = vec3(light.at("color")[0], light.at("color")[1], light.at("color")[2]);
            point_lights_.emplace_back(pl);
        }
        std::clog << "\rScene configured     " << std::endl;
    }
};

}  // namespace raytracer
#pragma once

#include <memory>
#include <vector>

#include "include/json.hpp"

#include "src/camera.h"
#include "src/color.h"
#include "src/material.h"
#include "src/triangle.h"
#include "src/vec3.h"

namespace raytracer {

class Scene {
   public:
    Scene(const nlohmann::json& config) : config_(config) { config_setup(config); }

    void AddTriangle(std::shared_ptr<const Triangle> t) { triangles_.emplace_back(t); }
    void AddMaterial(const Material& m) { materials_.emplace_back(m); }
    void AddLight(std::shared_ptr<const Triangle> t) { lights_.emplace_back(t); }
    const std::vector<std::shared_ptr<const Triangle>>& GetTriangles() const { return triangles_; }
    const std::vector<Material>& GetMaterials() const { return materials_; }
    const std::vector<std::shared_ptr<const Triangle>>& GetLights() const { return lights_; }
    void SetCamera(const Camera& camera) { camera_ = camera; }
    const Camera& GetCamera() const { return camera_; }

   private:
    struct PointLight {
        Point3 pos;
        Color color;
    };

    void config_setup(const nlohmann::json& config) {
        // Load Camera config
        camera_ = Camera(config.at("camera"));

        std::clog << "Configuring scene..." << std::flush;
        // Load point lights
        for (const auto& light : config.at("point_lights")) {
            PointLight pl;
            pl.pos = Point3(light.at("pos")[0], light.at("pos")[1], light.at("pos")[2]);
            pl.color = Vec3(light.at("color")[0], light.at("color")[1], light.at("color")[2]);
            point_lights_.emplace_back(pl);
        }
        std::clog << "\rScene configured     " << std::endl;
    }

    const nlohmann::json& config_;
    std::vector<std::shared_ptr<const Triangle>> triangles_;
    std::vector<Material> materials_;
    std::vector<std::shared_ptr<const Triangle>> lights_;
    Camera camera_;
    std::vector<PointLight> point_lights_;
};

}  // namespace raytracer
#pragma once

#include "include/json.hpp"
#include "include/tiny_obj_loader.h"

#include "src/scene.h"

namespace raytracer {

Scene LoadScene(const nlohmann::json& loader_config, const nlohmann::json& scene_config);

}  // namespace raytracer
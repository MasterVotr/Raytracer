#pragma once

#include "vec3.h"

namespace raytracer {

struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    vec3 transmittance;
    vec3 emission;
    float shininess;
    float ior;
    float dissolve;
};

}  // namespace raytracer
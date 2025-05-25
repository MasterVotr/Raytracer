#include <ctime>
#include <fstream>
#include <iostream>

#include "include/json.hpp"

#include "src/ObjLoader/obj_loader.h"
#include "src/Renderer/renderer.h"
#include "src/scene.h"

int main(int argc, char const* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file>" << std::endl;
        return 1;
    }
    std::clog << "Loading config..." << std::flush;
    std::ifstream ifs(argv[1]);
    nlohmann::json config = nlohmann::json::parse(ifs);
    ifs.close();
    std::clog << "\rConfig loaded     " << std::endl;

    try {
        if (config.at("seed") != -1) {
            srand(config.at("seed"));
        } else {
            srand(time(0));
        }
        raytracer::Renderer renderer(config.at("renderer"));
        raytracer::Scene scene = raytracer::LoadScene(config.at("obj_loader"), config.at("scene"));
        renderer.RenderScene(scene);
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

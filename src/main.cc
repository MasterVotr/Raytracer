#include <ctime>
#include <fstream>
#include <iostream>

#include "include/json.hpp"
#include "src/ObjLoader/obj_loader.h"
#include "src/Renderer/renderer.h"
#include "src/color.h"
#include "src/scene.h"

void save_image_to_pmm(std::string filename, int img_width, int img_height, std::vector<float>& img) {
    std::clog << "Saving image..." << std::flush;
    std::ofstream output(filename);
    output << "P3\n" << img_width << ' ' << img_height << "\n255\n";

    for (size_t i = 0; i < img.size(); i += 3) {
        output << static_cast<int>(255.999 * (img[i + 0])) << ' ' << static_cast<int>(255.999 * (img[i + 1])) << ' '
               << static_cast<int>(255.999 * (img[i + 2])) << '\n';
    }

    std::clog << "\rImage saved to " << filename << "        \n";
    output.close();
}

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
        std::vector<float> rendered_img = renderer.RenderScene(scene);
        save_image_to_pmm(config.at("output").at("filename"), scene.GetCamera().width, scene.GetCamera().height,
                          rendered_img);
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

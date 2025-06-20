#include "src/ObjLoader/obj_loader.h"
#define TINYOBJLOADER_IMPLEMENTATION

#include <cassert>
#include <exception>
#include <iostream>

#include "include/json.hpp"
#include "include/tiny_obj_loader.h"

#include "src/material.h"
#include "src/scene.h"
#include "src/triangle.h"

namespace raytracer {

namespace {

void PrintInfo(const tinyobj::attrib_t& attrib, const std::vector<tinyobj::shape_t>& shapes, const std::vector<tinyobj::material_t>& materials) {
    std::cout << "# of vertices  : " << (attrib.vertices.size() / 3) << std::endl;
    std::cout << "# of normals   : " << (attrib.normals.size() / 3) << std::endl;
    std::cout << "# of texcoords : " << (attrib.texcoords.size() / 2) << std::endl;

    std::cout << "# of shapes    : " << shapes.size() << std::endl;
    std::cout << "# of materials : " << materials.size() << std::endl;

    for (size_t v = 0; v < attrib.vertices.size() / 3; v++) {
        printf("  v[%ld] = (%f, %f, %f)\n", static_cast<long>(v), static_cast<const double>(attrib.vertices[3 * v + 0]),
               static_cast<const double>(attrib.vertices[3 * v + 1]), static_cast<const double>(attrib.vertices[3 * v + 2]));
    }

    for (size_t v = 0; v < attrib.normals.size() / 3; v++) {
        printf("  n[%ld] = (%f, %f, %f)\n", static_cast<long>(v), static_cast<const double>(attrib.normals[3 * v + 0]),
               static_cast<const double>(attrib.normals[3 * v + 1]), static_cast<const double>(attrib.normals[3 * v + 2]));
    }

    for (size_t v = 0; v < attrib.texcoords.size() / 2; v++) {
        printf("  uv[%ld] = (%f, %f)\n", static_cast<long>(v), static_cast<const double>(attrib.texcoords[2 * v + 0]),
               static_cast<const double>(attrib.texcoords[2 * v + 1]));
    }

    // For each shape
    for (size_t i = 0; i < shapes.size(); i++) {
        printf("shape[%ld].name = %s\n", static_cast<long>(i), shapes[i].name.c_str());
        printf("Size of shape[%ld].mesh.indices: %lu\n", static_cast<long>(i), static_cast<unsigned long>(shapes[i].mesh.indices.size()));
        printf("Size of shape[%ld].lines.indices: %lu\n", static_cast<long>(i), static_cast<unsigned long>(shapes[i].lines.indices.size()));
        printf("Size of shape[%ld].points.indices: %lu\n", static_cast<long>(i), static_cast<unsigned long>(shapes[i].points.indices.size()));

        size_t index_offset = 0;

        assert(shapes[i].mesh.num_face_vertices.size() == shapes[i].mesh.material_ids.size());

        assert(shapes[i].mesh.num_face_vertices.size() == shapes[i].mesh.smoothing_group_ids.size());

        printf("shape[%ld].num_faces: %lu\n", static_cast<long>(i), static_cast<unsigned long>(shapes[i].mesh.num_face_vertices.size()));

        // For each face
        for (size_t f = 0; f < shapes[i].mesh.num_face_vertices.size(); f++) {
            size_t fnum = shapes[i].mesh.num_face_vertices[f];

            printf("  face[%ld].fnum = %ld\n", static_cast<long>(f), static_cast<unsigned long>(fnum));

            // For each vertex in the face
            for (size_t v = 0; v < fnum; v++) {
                tinyobj::index_t idx = shapes[i].mesh.indices[index_offset + v];
                printf("    face[%ld].v[%ld].idx = %d/%d/%d\n", static_cast<long>(f), static_cast<long>(v), idx.vertex_index, idx.normal_index,
                       idx.texcoord_index);
            }

            printf("  face[%ld].material_id = %d\n", static_cast<long>(f), shapes[i].mesh.material_ids[f]);
            printf("  face[%ld].smoothing_group_id = %d\n", static_cast<long>(f), shapes[i].mesh.smoothing_group_ids[f]);

            index_offset += fnum;
        }

        printf("shape[%ld].num_tags: %lu\n", static_cast<long>(i), static_cast<unsigned long>(shapes[i].mesh.tags.size()));
        for (size_t t = 0; t < shapes[i].mesh.tags.size(); t++) {
            printf("  tag[%ld] = %s ", static_cast<long>(t), shapes[i].mesh.tags[t].name.c_str());
            printf(" ints: [");
            for (size_t j = 0; j < shapes[i].mesh.tags[t].intValues.size(); ++j) {
                printf("%ld", static_cast<long>(shapes[i].mesh.tags[t].intValues[j]));
                if (j < (shapes[i].mesh.tags[t].intValues.size() - 1)) {
                    printf(", ");
                }
            }
            printf("]");

            printf(" floats: [");
            for (size_t j = 0; j < shapes[i].mesh.tags[t].floatValues.size(); ++j) {
                printf("%f", static_cast<const double>(shapes[i].mesh.tags[t].floatValues[j]));
                if (j < (shapes[i].mesh.tags[t].floatValues.size() - 1)) {
                    printf(", ");
                }
            }
            printf("]");

            printf(" strings: [");
            for (size_t j = 0; j < shapes[i].mesh.tags[t].stringValues.size(); ++j) {
                printf("%s", shapes[i].mesh.tags[t].stringValues[j].c_str());
                if (j < (shapes[i].mesh.tags[t].stringValues.size() - 1)) {
                    printf(", ");
                }
            }
            printf("]");
            printf("\n");
        }

        // Print per-face material
        for (size_t i = 0; i < materials.size(); i++) {
            printf("material[%ld].name = %s\n", static_cast<long>(i), materials[i].name.c_str());
            printf("  material[%ld].ambient = (%f, %f, %f)\n", static_cast<long>(i), static_cast<const double>(materials[i].ambient[0]),
                   static_cast<const double>(materials[i].ambient[1]), static_cast<const double>(materials[i].ambient[2]));
            printf("  material[%ld].diffuse = (%f, %f, %f)\n", static_cast<long>(i), static_cast<const double>(materials[i].diffuse[0]),
                   static_cast<const double>(materials[i].diffuse[1]), static_cast<const double>(materials[i].diffuse[2]));
            printf("  material[%ld].specular = (%f, %f, %f)\n", static_cast<long>(i), static_cast<const double>(materials[i].specular[0]),
                   static_cast<const double>(materials[i].specular[1]), static_cast<const double>(materials[i].specular[2]));
            printf("  material[%ld].transmittance = (%f, %f, %f)\n", static_cast<long>(i), static_cast<const double>(materials[i].transmittance[0]),
                   static_cast<const double>(materials[i].transmittance[1]), static_cast<const double>(materials[i].transmittance[2]));
            printf("  material[%ld].emission = (%f, %f, %f)\n", static_cast<long>(i), static_cast<const double>(materials[i].emission[0]),
                   static_cast<const double>(materials[i].emission[1]), static_cast<const double>(materials[i].emission[2]));
            printf("  material[%ld].shininess = %f\n", static_cast<long>(i), static_cast<const double>(materials[i].shininess));
            printf("  material[%ld].ior = %f\n", static_cast<long>(i), static_cast<const double>(materials[i].ior));
            printf("  material[%ld].dissolve = %f\n", static_cast<long>(i), static_cast<const double>(materials[i].dissolve));
        }
    }
}

}  // namespace

Scene LoadScene(const nlohmann::json& loader_config, const nlohmann::json& scene_config) {
    std::string basepath = loader_config.at("basepath");
    std::string filename = basepath + std::string(scene_config.at("source_file"));
    bool triangulate = loader_config.at("triangulate");
    Scene scene(scene_config);
    std::clog << "Loading " << filename << std::flush;

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename.c_str(), basepath.c_str(), triangulate);

    if (!warn.empty()) {
        std::cout << "WARN: " << warn << std::endl;
    }

    if (!err.empty()) {
        std::cerr << "ERR: " << err << std::endl;
    }

    if (!ret) {
        throw std::runtime_error("Failed to load/parse .obj.");
    }
    std::clog << "\rLoading " << filename << " done." << std::endl;

    // obj_loader_internal::PrintInfo(attrib, shapes, materials);

    if (attrib.normals.size() == 0) {
        std::cerr << "No normals found in the model, only flat shading available" << std::endl;
    }
    std::clog << "Translating triangles to structures..." << std::flush;

    for (size_t i = 0; i < shapes.size(); i++) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[i].mesh.num_face_vertices.size(); f++) {
            size_t fnum = shapes[i].mesh.num_face_vertices[f];
            Triangle triangle;
            for (size_t v = 0; v < fnum; v++) {
                tinyobj::index_t idx = shapes[i].mesh.indices[index_offset + v];
                auto vidx = idx.vertex_index;
                triangle.vertices[v].pos =
                    Vec3(static_cast<const float>(attrib.vertices[3 * vidx + 0]), static_cast<const float>(attrib.vertices[3 * vidx + 1]),
                         static_cast<const float>(attrib.vertices[3 * vidx + 2]));
                auto nidx = idx.normal_index;
                if (nidx >= 0) {
                    triangle.vertices[v].norm =
                        Vec3(static_cast<const float>(attrib.normals[3 * nidx + 0]), static_cast<const float>(attrib.normals[3 * nidx + 1]),
                             static_cast<const float>(attrib.normals[3 * nidx + 2]));
                }
            }
            triangle.material_id = shapes[i].mesh.material_ids[f];
            triangle.normal = calculate_triangle_normal(triangle);
            scene.AddTriangle(std::make_shared<const Triangle>(triangle));
            // Lights
            if (materials[triangle.material_id].emission[0] != 0.0 || materials[triangle.material_id].emission[1] != 0.0 ||
                materials[triangle.material_id].emission[2] != 0.0) {
                scene.AddLight(std::make_shared<const Triangle>(triangle));
            }
            index_offset += fnum;
        }
    }

    std::clog << "\rTranslating materials to structures..." << std::flush;
    for (size_t i = 0; i < materials.size(); i++) {
        Material material;
        material.ambient = Vec3(materials[i].ambient[0], materials[i].ambient[1], materials[i].ambient[2]);
        material.diffuse = Vec3(materials[i].diffuse[0], materials[i].diffuse[1], materials[i].diffuse[2]);
        material.specular = Vec3(materials[i].specular[0], materials[i].specular[1], materials[i].specular[2]);
        material.transmittance = Vec3(materials[i].transmittance[0], materials[i].transmittance[1], materials[i].transmittance[2]);
        material.emission = Vec3(materials[i].emission[0], materials[i].emission[1], materials[i].emission[2]);
        material.shininess = materials[i].shininess;
        material.ior = materials[i].ior;
        material.dissolve = materials[i].dissolve;
        scene.AddMaterial(material);
    }

    std::clog << "\rTranslating everything to structures done.          " << std::endl;
    return scene;
}

}  // namespace raytracer
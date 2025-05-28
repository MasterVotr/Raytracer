#pragma once

#include "src/util.h"
#include "src/vec3.h"

namespace raytracer {

struct Vertex {
    Point3 pos;
    Vec3 norm;

    __host__ __device__ bool operator==(const Vertex& other) const { return pos == other.pos && norm == other.norm; }
    __device__ void print() const {
        printf("(");
        pos.print();
        printf(")");
    }
};

inline std::ostream& operator<<(std::ostream& os, const Vertex& vertex) {
    os << "pos:" << vertex.pos << " norm:" << vertex.norm;
    return os;
}

}  // namespace raytracer
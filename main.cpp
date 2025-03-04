#include "vec3.h"
#include "util.h"

#include <fstream>
#include <iostream>

#define WIDTH 800
#define HEIGHT 800


/*struct Sphere {
    Sphere(float rad_, vec3 p_):
        rad(rad_), p(p_) {}

    float intersect(const Ray &r) const { // returns distance, 0 if nohit
        vec3 op = p - r.o; // Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0
        float t, eps=1e-4, b=op.dot(r.d), det=b*b-op.dot(op)+rad*rad;
        if (det<0) return 0; else det=sqrt(det);
        return (t=b-det)>eps ? t : ((t=b+det)>eps ? t : 0);
    }

    float rad;  // radius
    vec3 p;     // position
}; */

int main(int argc, char const* argv[]) {
    float* image = new float[WIDTH * HEIGHT * 3];

    // creating checker texture
    for (int i = 0; i < WIDTH; ++i)
        for (int j = 0; j < HEIGHT; ++j)
            for (int k = 0; k < 3; ++k)
                image[3 * (j * WIDTH + i) + k] = (i + j) % 2 * 255;

    // image saving
    std::ofstream output("output.ppm");
    output << "P3\n" << WIDTH << " " << HEIGHT << "\n" << 255 << std::endl;
    for (int i = 0; i < WIDTH * HEIGHT * 3; ++i)
        output << (int)image[i] << " ";
    output.close();
}

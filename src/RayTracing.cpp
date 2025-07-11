#include "PFM.h"
#include <iostream>
#include <random>
#include "Vec3.h"
#include "Ray.h"
#include "Camera.h"
#include "Scene.h"
#include "WhittedRender.h"

#include <sstream>
#include <filesystem>

#pragma omp declare reduction( vec3_plus : Vec3 : omp_out += omp_in ) initializer( omp_priv = Vec3(0.0f, 0.0f, 0.0f) )

int main() {
    const int image_width = 800;
    const int image_height = 400;
    std::vector<std::shared_ptr<Object>> scene;
    Camera camera = controll_scene(image_width, image_height, scene);

    int sample_num = 10;

    std::vector<float> pixels(image_width * image_height * 3);

    #pragma omp parallel for collapse(2) schedule(static)
    for(int j = image_height - 1; j >= 0; --j) {
        for(int i = 0; i < image_width; ++i) {
            float u = (i + random_float()) / (image_width - 1);
            float v = (j + random_float()) / (image_height - 1);

            Vec3 color = Vec3(0.0f, 0.0f, 0.0f);

            #pragma omp simd reduction(vec3_plus:color)
            for(int s = 0; s < sample_num; s++){

                Ray r = camera.get_ray(u, v);
                color += whitted_render(r, scene);

            }
            color /= sample_num;

            size_t idx = (j * image_width + i) * 3; 
            pixels[idx + 0] = color.x; 
            pixels[idx + 1] = color.y; 
            pixels[idx + 2] = color.z; 
        }
    }

    std::filesystem::create_directories("results");
    std::ostringstream filename;
    filename << "results/rt.pfm";
    write_pfm(filename.str(), pixels, image_width, image_height);

    return 0;

}

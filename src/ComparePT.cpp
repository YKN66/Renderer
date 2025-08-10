#include "PFM.h"
#include <iostream>
#include <random>
#include "Vec3.h"
#include "Ray.h"
#include "Camera.h"
#include "Scene.h"
#include "WhittedRender.h"
#include "CosWeightRender.h"
#include "NEERender.h"
#include "BDPTRender.h"

#include <sstream>
#include <filesystem>

#pragma omp declare reduction( vec3_plus : Vec3 : omp_out += omp_in ) initializer( omp_priv = Vec3(0.0f, 0.0f, 0.0f) )


int main() {
    const int image_width = 800;
    const int image_height = 400;
    std::vector<std::shared_ptr<Object>> scene;
    Camera camera = controll_scene(image_width, image_height, scene);

    int bdpt_sample_num = 5;
    int cos_sample_num = 200;
    int max_path_len = 10;

    std::filesystem::create_directories("compare");

    for (int path_length = 1; path_length <= max_path_len; ++path_length) {
        std::cout << "Rendering path length " << path_length << std::endl;
        
        std::ostringstream path_dir;
        path_dir << "compare/length_" << path_length;
        std::filesystem::create_directories(path_dir.str());


        std::cout << "  Rendering PT (CosWeight)" << std::endl;
        std::vector<float> pixels2(image_width * image_height * 3);

        #pragma omp parallel for collapse(2) schedule(static)
        for(int j = image_height - 1; j >= 0; --j) {
            for(int i = 0; i < image_width; ++i) {
                Vec3 color(0.0f, 0.0f, 0.0f);

                #pragma omp simd reduction(vec3_plus:color)
                for(int sample = 0; sample < cos_sample_num; sample++) {
                    float u = (i + random_float()) / (image_width - 1);
                    float v = (j + random_float()) / (image_height - 1);
                    Ray ray = camera.get_ray(u, v);
                    
                    color += cos_weight_render_n(ray, scene, path_length);
                }
                color /= cos_sample_num;

                size_t idx = (j * image_width + i) * 3; 
                pixels2[idx + 0] = color.x; 
                pixels2[idx + 1] = color.y; 
                pixels2[idx + 2] = color.z; 
            }
        }

        std::ostringstream pt_filename;
        pt_filename << path_dir.str() << "/cos.pfm";
        write_pfm(pt_filename.str(), pixels2, image_width, image_height);

        std::cout << "  Path length " << path_length << " complete!" << std::endl;
    }

    return 0;
}
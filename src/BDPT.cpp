#include "PFM.h"
#include <iostream>
#include <fstream>  
#include <random>
#include "Vec3.h"
#include "Ray.h"
#include "Camera.h"
#include "Scene.h"
#include "BDPTRender.h"

#include <sstream>
#include <filesystem>

#pragma omp declare reduction( vec3_plus : Vec3 : omp_out += omp_in ) initializer( omp_priv = Vec3(0.0f, 0.0f, 0.0f) )

int main() {
    const int image_width = 800;
    const int image_height = 400;
    const int sample_num = 3;

    std::vector<std::shared_ptr<Object>> scene;
    Camera camera = controll_scene(image_width, image_height, scene);

    std::filesystem::create_directories("bdpt_results");

    std::vector<Vec3> framebuffer(image_width * image_height, Vec3(0.0f, 0.0f, 0.0f));
    std::vector<float> mis_err(image_width * image_height, 0.0f);
    
    #pragma omp parallel for collapse(2) schedule(static)
    for(int j = image_height - 1; j >= 0; --j) {
        for(int i = 0; i < image_width; ++i) {
            Vec3 color(0.0f, 0.0f, 0.0f);
            float mis_error = 0.0f;

            #pragma omp simd reduction(vec3_plus:color)
            for(int sample = 0; sample < sample_num; sample++) {
                float u = (i + random_float()) / (image_width - 1);
                float v = (j + random_float()) / (image_height - 1);
                
                color += bdpt_render(camera, scene, u, v, mis_error);
            }
            color /= float(sample_num);
            mis_error /= float(sample_num);
            
            framebuffer[j * image_width + i] += color;
            mis_err[j * image_width + i] = mis_error;
        }
    }

    std::vector<float> pixels(image_width * image_height * 3);
    for (int y = 0; y < image_height; ++y) {
        for (int x = 0; x < image_width; ++x) {
            Vec3 color = framebuffer[y * image_width + x];
            // Vec3 color = framebuffer[y * image_width + x] / float(sample_num);
            size_t idx = (y * image_width + x) * 3;
            pixels[idx + 0] = color.x; 
            pixels[idx + 1] = color.y; 
            pixels[idx + 2] = color.z;
        }
    }
    std::ostringstream filename;
    filename << "bdpt_results/bdpt.pfm";
    write_pfm(filename.str(), pixels, image_width, image_height);


    // ── MIS Σw 誤差を CSV へ
    {
        std::ofstream csv("bdpt_results/mis_error.csv");
        if(!csv){ std::cerr << "[Error] cannot open mis_error.csv\n"; return 1; }
        csv.setf(std::ios::fixed);
        csv.precision(8);

        for(int y = 0; y < image_height; ++y){
            for(int x = 0; x < image_width; ++x){
                csv << mis_err[y * image_width + x];
                if(x < image_width - 1) csv << ',';
            }
            csv << '\n';
        }
    }

    std::cout << "Render finished.\n";

    return 0;
}

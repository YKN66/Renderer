#include "stb_image_write.h"
#include <iostream>
#include <random>
#include "Vec3.h"
#include "Ray.h"
#include "Camera.h"
#include "Scene.h"
#include "WhittedRender.h"
#include "CosWeightRender.h"
#include "NEERender.h"
#include "BDTPRender.h"

#include <sstream>
#include <filesystem>


int main() {
    const int image_width = 800;
    const int image_height = 400;
    std::vector<std::shared_ptr<Object>> scene;
    Camera camera = controll_scene(image_width, image_height, scene);

    int sample_num = 10;
    int max_path_length = 5;

    std::filesystem::create_directories("bdpt_results");

    for (int path_length = 1; path_length <= max_path_length; ++path_length) {
        std::cout << "Rendering path length " << path_length << std::endl;
        
        std::ostringstream path_dir;
        path_dir << "bdpt_results/length_" << path_length;
        std::filesystem::create_directories(path_dir.str());

        for (int s = 0; s <= path_length + 1; ++s) {
            int t = path_length + 1 - s;
            if (t < 0 || s < 0) continue;
            if (s == 0 && t == 0) continue; // Invalid case
            
            std::cout << "  Rendering (s=" << s << ", t=" << t << ")..." << std::endl;
            
            std::vector<unsigned char> pixels(image_width * image_height * 3);

            for(int j = image_height - 1; j >= 0; --j) {
                for(int i = 0; i < image_width; ++i) {
                    Vec3 color(0.0f, 0.0f, 0.0f);

                    for(int sample = 0; sample < sample_num; sample++) {
                        float u = (i + random_float()) / (image_width - 1);
                        float v = (j + random_float()) / (image_height - 1);
                        
                        color += bdpt_render_n(camera, scene, u, v, s, t);
                    }
                    color /= sample_num;

                    color.x = std::pow(std::min(1.0f, std::max(0.0f, color.x)), 1.0f/2.2f);
                    color.y = std::pow(std::min(1.0f, std::max(0.0f, color.y)), 1.0f/2.2f);
                    color.z = std::pow(std::min(1.0f, std::max(0.0f, color.z)), 1.0f/2.2f);

                    int ir = static_cast<int>(255.99 * color.x);
                    int ig = static_cast<int>(255.99 * color.y);
                    int ib = static_cast<int>(255.99 * color.z);

                    size_t idx = ((image_height - 1 - j) * image_width + i) * 3;
                    pixels[idx + 0] = static_cast<unsigned char>(ir);
                    pixels[idx + 1] = static_cast<unsigned char>(ig);
                    pixels[idx + 2] = static_cast<unsigned char>(ib);
                }
            }

            std::ostringstream filename;
            filename << path_dir.str() << "/s" << s << "_t" << t << ".png";
            stbi_write_png(filename.str().c_str(), image_width, image_height, 3, pixels.data(), image_width * 3);
        }
    }

    std::cout << "\nRendering combined BDPT" << std::endl;
    std::vector<unsigned char> pixels(image_width * image_height * 3);

    for(int j = image_height - 1; j >= 0; --j) {
        for(int i = 0; i < image_width; ++i) {
            Vec3 color(0.0f, 0.0f, 0.0f);

            for(int sample = 0; sample < sample_num; sample++) {
                float u = (i + random_float()) / (image_width - 1);
                float v = (j + random_float()) / (image_height - 1);
                
                color += bdpt_render(camera, scene, u, v, max_path_length);
            }
            color /= sample_num;

            color.x = std::pow(std::min(1.0f, std::max(0.0f, color.x)), 1.0f/2.2f);
            color.y = std::pow(std::min(1.0f, std::max(0.0f, color.y)), 1.0f/2.2f);
            color.z = std::pow(std::min(1.0f, std::max(0.0f, color.z)), 1.0f/2.2f);

            int ir = static_cast<int>(255.99 * color.x);
            int ig = static_cast<int>(255.99 * color.y);
            int ib = static_cast<int>(255.99 * color.z);

            size_t idx = ((image_height - 1 - j) * image_width + i) * 3;
            pixels[idx + 0] = static_cast<unsigned char>(ir);
            pixels[idx + 1] = static_cast<unsigned char>(ig);
            pixels[idx + 2] = static_cast<unsigned char>(ib);
        }
    }

    stbi_write_png("bdpt_results/bdpt_combined.png", image_width, image_height, 3, pixels.data(), image_width * 3);
    
    return 0;
}

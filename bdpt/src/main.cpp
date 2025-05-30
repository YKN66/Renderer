#include "stb_image_write.h"
#include <iostream>
#include <random>
#include "Vec3.h"
#include "Ray.h"
#include "Camera.h"
#include "Scene.h"
#include "CosWeightRender.h"
#include "NEERender.h"
#include "BDTPRender.h"
#include <sstream>


int main() {
    const int image_width = 800;
    const int image_height = 400;
    std::vector<std::shared_ptr<Object>> scene;
    Camera camera = controll_scene(image_width, image_height, scene);

    int sample_num = 2400;

    int n = 1;

    for(int k = 1; k <= n; k++){

        std::vector<unsigned char> pixels(image_width * image_height * 3);


        for(int j = image_height - 1; j >= 0; --j) {
            for(int i = 0; i < image_width; ++i) {
                float u = (i + random_float()) / (image_width - 1);
                float v = (j + random_float()) / (image_height - 1);

                Vec3 color = Vec3(0.0f, 0.0f, 0.0f);

                for(int s = 0; s < sample_num; s++){
                    // PT
                    Ray r = camera.get_ray(u, v);
                    color += cos_weight_render(r, scene, 50);
                    // color += nee_render(r, scene, 50);

                    // BDPT
                    // color += bdpt_render(camera, scene, u, v, 50);

                    // Ray r = camera.get_ray(u, v);
                    // color += cos_weight_render_n(r, scene, k);
                    // color += bdpt_render(camera, scene, u, v, 2);

                }
                color /= sample_num;

                color.x = std::min(1.0f, std::max(0.0f, color.x));
                color.y = std::min(1.0f, std::max(0.0f, color.y));
                color.z = std::min(1.0f, std::max(0.0f, color.z));

                int ir = static_cast<int>(255.99 * color.x);
                int ig = static_cast<int>(255.99 * color.y);
                int ib = static_cast<int>(255.99 * color.z);

                size_t idx = ((image_height - 1 - j) * image_width + i) * 3;
                pixels[idx + 0] = static_cast<unsigned char>(ir); 
                pixels[idx + 1] = static_cast<unsigned char>(ig); 
                pixels[idx + 2] = static_cast<unsigned char>(ib); 
            }
        }

        stbi_write_png("cos.png", image_width, image_height, 3/*RGB*/, pixels.data(), image_width * 3/*stride*/);
        // stbi_write_png("nee.png", image_width, image_height, 3/*RGB*/, pixels.data(), image_width * 3/*stride*/);
        // stbi_write_png("bdpt.png", image_width, image_height, 3/*RGB*/, pixels.data(), image_width * 3/*stride*/);

        // std::ostringstream filename;
        // filename << "png/" << k << "_bdpt.png";
        // stbi_write_png(filename.str().c_str(), image_width, image_height, 3, pixels.data(), image_width * 3);

    }


    return 0;

}

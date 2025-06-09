#include "PFM.h"
#include <iostream>
#include <random>
#include "Vec3.h"
#include "Ray.h"
#include "Camera.h"
#include "Scene.h"
#include "BDPTRender.h"

#include <sstream>
#include <filesystem>


int main() {
    const int image_width = 800;
    const int image_height = 400;
    const int sample_num = 3;
    const int max_path_len = 5;

    std::vector<std::shared_ptr<Object>> scene;
    Camera camera = controll_scene(image_width, image_height, scene);

    std::filesystem::create_directories("bdpt_results");


    for (int path_len = 1; path_len <= max_path_len; ++path_len) {
        std::cout << "Rendering path length " << path_len << std::endl;
        
        std::ostringstream path_dir;
        path_dir << "bdpt_results/length_" << path_len;
        std::filesystem::create_directories(path_dir.str());

        for (int s = 0; s <= path_len + 1; ++s) {
            int t = path_len + 1 - s;

            std::cout << "  Rendering (s=" << s << ", t=" << t << ")..." << std::endl;
            std::vector<Vec3> framebuffer(image_width * image_height, Vec3(0.0f, 0.0f, 0.0f));

            if(s == 0) {
                for (int k = 0; k < sample_num; ++k) {
                    /* ライトサブパス生成 */
                    auto l_path = generate_light_subpath(scene, t - 1);
                    if (l_path.empty()) continue;

                    /* β_l   を計算（BDPT と同じ） */
                    std::vector<Vec3> beta_l(l_path.size(), Vec3(1.0f, 1.0f, 1.0f));
                    if (l_path.size() == 1)
                        beta_l[0] = l_path[0].brdf->get_emission();
                    else {
                        Vec3 d0   = (l_path[1].x - l_path[0].x).normalize();
                        float cos0 = std::max(0.f, l_path[0].N.dot(d0));
                        beta_l[0] = l_path[0].brdf->get_emission() * cos0 / std::max(l_path[0].pdf_fwd, 1e-6f);
                    }
                    for (size_t i = 1; i < l_path.size(); ++i) {
                        const auto& vi   = l_path[i - 1];
                        const auto& vip1 = l_path[i];
                        Vec3 d = (vip1.x - vi.x).normalize();
                        float cos = std::max(0.f, vi.N.dot(d));
                        Vec3 fs = vi.is_light ? Vec3(1.0f, 1.0f, 1.0f) : vi.brdf->evaluate(vi.N, d, vi.wi);
                        beta_l[i] = beta_l[i - 1] * fs * cos / std::max(vi.pdf_fwd, 1e-6f);
                    }

                    accumulate_light_only(l_path, beta_l, camera, image_width, image_height, framebuffer, t);
                }
            }
            else {                
                for(int j = image_height - 1; j >= 0; --j) {
                    for(int i = 0; i < image_width; ++i) {
                        Vec3 color(0.0f, 0.0f, 0.0f);

                        for(int sample = 0; sample < sample_num; sample++) {
                            float u = (i + random_float()) / (image_width - 1);
                            float v = (j + random_float()) / (image_height - 1);
                            
                            color += bdpt_render_n(camera, scene, u, v, s, t);
                        }
                        color /= float(sample_num);
                        framebuffer[j * image_width + i] += color;
                    }
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
            filename << path_dir.str() << "/s" << s << "_t" << t << ".pfm";
            write_pfm(filename.str().c_str(), pixels, image_width, image_height);

        }
    }

    // std::cout << "\nRendering combined BDPT" << std::endl;
    // std::vector<float> pixels(image_width * image_height * 3);

    // for(int j = image_height - 1; j >= 0; --j) {
    //     for(int i = 0; i < image_width; ++i) {
    //         Vec3 color(0.0f, 0.0f, 0.0f);

    //         for(int sample = 0; sample < sample_num; sample++) {
    //             float u = (i + random_float()) / (image_width - 1);
    //             float v = (j + random_float()) / (image_height - 1);
                
    //             color += bdpt_render(camera, scene, u, v, max_path_len);
    //         }
    //         color /= sample_num;

    //         color.x = std::pow(std::min(1.0f, std::max(0.0f, color.x)), 1.0f/2.2f);
    //         color.y = std::pow(std::min(1.0f, std::max(0.0f, color.y)), 1.0f/2.2f);
    //         color.z = std::pow(std::min(1.0f, std::max(0.0f, color.z)), 1.0f/2.2f);

    //         int ir = static_cast<int>(255.99 * color.x);
    //         int ig = static_cast<int>(255.99 * color.y);
    //         int ib = static_cast<int>(255.99 * color.z);

    //         size_t idx = (j * image_width + i) * 3; 
    //         pixels[idx + 0] = color.x; 
    //         pixels[idx + 1] = color.y; 
    //         pixels[idx + 2] = color.z; 
    //     }
    // }

    // // stbi_write_png("bdpt_results/bdpt_combined.png", image_width, image_height, 3, pixels.data(), image_width * 3);
    // std::ostringstream filename;
    // filename << "results/bdpt.pfm";
    // write_pfm(filename.str(), pixels, image_width, image_height);
    
    return 0;
}

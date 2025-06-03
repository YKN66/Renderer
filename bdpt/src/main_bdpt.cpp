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
    const int sample_num = 10000;
    const int max_path_len = 5;

    std::vector<std::shared_ptr<Object>> scene;
    Camera camera = controll_scene(image_width, image_height, scene);

    std::filesystem::create_directories("bdpt_results");

        /* ---------- (s ≥ 1) 既存 BDPT ---------- */
    // for (int path_len = 1; path_len <= max_path_len; ++path_len) {
    //     std::cout << "Rendering path length " << path_len << std::endl;
        
    //     std::ostringstream path_dir;
    //     path_dir << "bdpt_results/length_" << path_len;
    //     std::filesystem::create_directories(path_dir.str());

    //     for (int s = 1; s <= path_len + 1; ++s) {
    //         int t = path_len + 1 - s;
    //         if (t < 0 || s < 0) continue;
    //         if (s == 0 && t == 0) continue; // Invalid case
            
    //         std::cout << "  Rendering (s=" << s << ", t=" << t << ")..." << std::endl;
            
    //         std::vector<unsigned char> png(image_width * image_height * 3);

    //         for(int j = image_height - 1; j >= 0; --j) {
    //             for(int i = 0; i < image_width; ++i) {
    //                 Vec3 color(0.0f, 0.0f, 0.0f);

    //                 for(int sample = 0; sample < sample_num; sample++) {
    //                     float u = (i + random_float()) / (image_width - 1);
    //                     float v = (j + random_float()) / (image_height - 1);
                        
    //                     color += bdpt_render_n(camera, scene, u, v, s, t);
    //                 }
    //                 color /= float(sample_num);

    //                 color.x = std::pow(std::min(1.0f, std::max(0.0f, color.x)), 1.0f/2.2f);
    //                 color.y = std::pow(std::min(1.0f, std::max(0.0f, color.y)), 1.0f/2.2f);
    //                 color.z = std::pow(std::min(1.0f, std::max(0.0f, color.z)), 1.0f/2.2f);

    //                 int ir = static_cast<int>(255.99 * color.x);
    //                 int ig = static_cast<int>(255.99 * color.y);
    //                 int ib = static_cast<int>(255.99 * color.z);

    //                 size_t idx = ((image_height - 1 - j) * image_width + i) * 3;
    //                 png[idx + 0] = static_cast<unsigned char>(ir);
    //                 png[idx + 1] = static_cast<unsigned char>(ig);
    //                 png[idx + 2] = static_cast<unsigned char>(ib);
    //             }
    //         }

    //         std::ostringstream filename;
    //         filename << path_dir.str() << "/s" << s << "_t" << t << ".png";
    //         stbi_write_png(filename.str().c_str(), image_width, image_height, 3, png.data(), image_width * 3);
    //     }
    // }
        /* ---------- (s = 0) Light Tracing ---------- */
    {
        int s = 0;
        for (int t = 2; t <= max_path_len + 1; ++t) {      // t ≥ 2
            std::vector<Vec3> framebuffer(image_width * image_height, Vec3(0.0f, 0.0f, 0.0f));

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

            /* ガンマ補正して PNG 出力 */
            std::vector<unsigned char> png(image_width * image_height * 3);
            for (int y = 0; y < image_height; ++y) {
                for (int x = 0; x < image_width; ++x) {
                    Vec3 c = framebuffer[y * image_width + x];
                    // Vec3 c = framebuffer[y * image_width + x] / float(sample_num);
                    c.x = std::pow(std::clamp(c.x, 0.f, 1.f), 1.f / 2.2f);
                    c.y = std::pow(std::clamp(c.y, 0.f, 1.f), 1.f / 2.2f);
                    c.z = std::pow(std::clamp(c.z, 0.f, 1.f), 1.f / 2.2f);

                    size_t idx = (image_height - 1 - y) * image_width * 3 + x * 3;
                    png[idx + 0] = (unsigned char)(255.99f * c.x);
                    png[idx + 1] = (unsigned char)(255.99f * c.y);
                    png[idx + 2] = (unsigned char)(255.99f * c.z);
                }
            }
            std::ostringstream dir; dir << "bdpt_results/length_" << (t - 1);
            std::filesystem::create_directories(dir.str());
            std::ostringstream name; name << dir.str() << "/s0_t" << t << ".png";
            stbi_write_png(name.str().c_str(), image_width, image_height, 3, png.data(), image_width * 3);
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
                
                color += bdpt_render(camera, scene, u, v, max_path_len);
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

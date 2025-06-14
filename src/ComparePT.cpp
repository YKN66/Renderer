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


int main() {
    const int image_width = 800;
    const int image_height = 400;
    std::vector<std::shared_ptr<Object>> scene;
    Camera camera = controll_scene(image_width, image_height, scene);

    int bdpt_sample_num = 5;
    int cos_sample_num = 20;
    int max_path_len = 5;

    std::filesystem::create_directories("compare");

    for (int path_length = 1; path_length <= max_path_len; ++path_length) {
        std::cout << "Rendering path length " << path_length << std::endl;
        
        std::ostringstream path_dir;
        path_dir << "compare/length_" << path_length;
        std::filesystem::create_directories(path_dir.str());

        // std::cout << "  Rendering BDPT" << std::endl;

        // std::vector<Vec3> fb(image_width * image_height, Vec3(0.0f, 0.0f, 0.0f));

        // for (int s = 0; s <= path_length + 1; ++s) {
        //     int t = path_length + 1 - s;
        //     if(s == 0) {
        //         for (int k = 0; k < bdpt_sample_num; ++k) {
        //             /* ライトサブパス生成 */
        //             auto l_path = generate_light_subpath(scene, t - 1);
        //             if (l_path.empty()) continue;

        //             /* β_l   を計算（BDPT と同じ） */
        //             std::vector<Vec3> beta_l(l_path.size(), Vec3(1.0f, 1.0f, 1.0f));
        //             if (l_path.size() == 1)
        //                 beta_l[0] = l_path[0].brdf->get_emission();
        //             else {
        //                 Vec3 d0   = (l_path[1].x - l_path[0].x).normalize();
        //                 float cos0 = std::max(0.f, l_path[0].N.dot(d0));
        //                 beta_l[0] = l_path[0].brdf->get_emission() * cos0 / std::max(l_path[0].pdf_fwd, 1e-6f);
        //             }
        //             for (size_t i = 1; i < l_path.size(); ++i) {
        //                 const auto& vi   = l_path[i - 1];
        //                 const auto& vip1 = l_path[i];
        //                 Vec3 d = (vip1.x - vi.x).normalize();
        //                 float cos = std::max(0.f, vi.N.dot(d));
        //                 Vec3 fs = vi.is_light ? Vec3(1.0f, 1.0f, 1.0f) : vi.brdf->evaluate(vi.N, d, vi.wi);
        //                 beta_l[i] = beta_l[i - 1] * fs * cos / std::max(vi.pdf_fwd, 1e-6f);
        //             }

        //             accumulate_light_only(l_path, beta_l, camera, image_width, image_height, fb, t);
        //         }
        //     }
        //     else {
        //         for(int j = image_height - 1; j >= 0; --j) {
        //             for(int i = 0; i < image_width; ++i) {
        //                 Vec3 color(0.0f, 0.0f, 0.0f);

        //                 for(int sample = 0; sample < bdpt_sample_num; sample++) {
        //                     float u = (i + random_float()) / (image_width - 1);
        //                     float v = (j + random_float()) / (image_height - 1);
                                
        //                     color += bdpt_render(camera, scene, u, v, s, t);
        //                 }
        //                 // color /= bdpt_sample_num;
        //                 fb[j * image_width + i] += color;

        //             }
        //         }
        //     }
        // }

        // std::vector<float> pixels(image_width * image_height * 3);
        // for (int j = image_height - 1; j >= 0; --j) {
        //     for (int i = 0; i < image_width; ++i) {
        //         Vec3 color = fb[j * image_width + i] / float(bdpt_sample_num);

        //         size_t idx = (j * image_width + i) * 3; 
        //         pixels[idx + 0] = color.x; 
        //         pixels[idx + 1] = color.y; 
        //         pixels[idx + 2] = color.z; 

        //     }
        // }

        // std::ostringstream bdpt_filename;
        // bdpt_filename << path_dir.str() << "/bdpt.pfm";
        // write_pfm(bdpt_filename.str(), pixels, image_width, image_height);

        std::cout << "  Rendering PT (CosWeight)" << std::endl;
        std::vector<float> pixels2(image_width * image_height * 3);

        for(int j = image_height - 1; j >= 0; --j) {
            for(int i = 0; i < image_width; ++i) {
                Vec3 color(0.0f, 0.0f, 0.0f);

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
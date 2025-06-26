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

#pragma omp declare reduction( vec3_plus : Vec3 : omp_out += omp_in ) initializer( omp_priv = Vec3(0.0f, 0.0f, 0.0f) )


int main() {
    const int image_width = 800;
    const int image_height = 400;
    const int sample_num = 50;
    const int max_path_len = 10;

    std::vector<std::shared_ptr<Object>> scene;
    Camera camera = controll_scene(image_width, image_height, scene);

    std::filesystem::create_directories("bdpt_results");
    std::vector<Vec3> final_fb(image_width * image_height, Vec3(0.0f, 0.0f, 0.0f));


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
                for (int k = 0; k < 5000000; ++k) {

                    auto l_path = generate_light_subpath(scene, t);
                    // std::cout << "size = ("<< l_path.size() <<")\n";
                    if (l_path.empty()) continue;

                    std::vector<Vec3> beta_l(l_path.size(), Vec3(1.0f, 1.0f, 1.0f));
                    beta_l[0] = l_path[0].brdf->get_emission();
                    for (size_t i = 1; i < l_path.size(); ++i) {
                        const auto& v0   = l_path[i - 1];
                        const auto& v1 = l_path[i];
                        Vec3 d = (v1.x - v0.x).normalize();
                        float cos = std::max(0.f, v0.N.dot(d));
                        Vec3 fs = v0.is_light ? Vec3(1.0f, 1.0f, 1.0f) : v0.brdf->evaluate(v0.N, d, v0.wi);
                        beta_l[i] = beta_l[i - 1] * fs * cos / std::max(v0.pdf_W, 1e-6f);
                    }

                    accumulate_light_only(l_path, beta_l, camera, image_width, image_height, framebuffer, t);
                }
            }
            else if(s == 1) {
                for (int k = 0; k < 10000; ++k) {
                    if (t < 2) {
                        // (s,t) = (1,1) などはここでは処理しない
                        // → 必要なら従来のピクセルループにフォールバックしてもよい
                    } else {
                        auto l_path = generate_light_subpath(scene, t - 1);
                        // std::cout << "size = ("<< l_path.size() <<")\n";
                        if (l_path.empty()) continue;

                        std::vector<Vec3> beta_l(l_path.size(), Vec3(1.0f, 1.0f, 1.0f));
                        beta_l[0] = l_path[0].brdf->get_emission();
                        for (size_t i = 1; i < l_path.size(); ++i) {
                            const auto& v0 = l_path[i-1];
                            const auto& v1 = l_path[i];
                            Vec3 d   = (v1.x - v0.x).normalize();
                            float cos = std::max(0.f, v0.N.dot(d));
                            Vec3 fs  = v0.is_light ? Vec3(1.0f, 1.0f ,1.0f) : v0.brdf->evaluate(v0.N, d, v0.wi);
                            beta_l[i] = beta_l[i-1] * fs * cos / std::max(v0.pdf_W, 1e-6f);
                        }

                        accumulate_eye_only(camera, l_path, beta_l, t, image_width, image_height, framebuffer);
                    }
                }
            }
            else {
                #pragma omp parallel for collapse(2) schedule(static)
                for(int j = image_height - 1; j >= 0; --j) {
                    for(int i = 0; i < image_width; ++i) {
                        Vec3 color(0.0f, 0.0f, 0.0f);

                        #pragma omp simd reduction(vec3_plus:color)
                        for(int sample = 0; sample < sample_num; sample++) {
                            float u = (i + random_float()) / (image_width - 1);
                            float v = (j + random_float()) / (image_height - 1);
                            
                            color += bdpt_render_debug(camera, scene, u, v, s, t);
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


            for(size_t idx = 0; idx < framebuffer.size(); ++idx) {
                final_fb[idx] += framebuffer[idx];
            }

        }
    }

    std::cout << "\nWriting combined BDPT" << std::endl;
    std::vector<float> final_pixels(image_width * image_height * 3);
    for (int y = 0; y < image_height; ++y) {
        for (int x = 0; x < image_width; ++x) {
            Vec3 c = final_fb[y * image_width + x];
            size_t id = (y * image_width + x) * 3;
            final_pixels[id + 0] = c.x;
            final_pixels[id + 1] = c.y;
            final_pixels[id + 2] = c.z;
        }
    }
    write_pfm("bdpt_results/bdpt_combined.pfm", final_pixels, image_width, image_height);
    
    return 0;
}

#pragma once
#include "Vec3.h"
#include "Object.h"
#include "Camera.h"
#include "PathGenerator.h"
#include "PathEvaluator.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <memory>


Vec3 bdpt_render(const Camera& camera, const std::vector<std::shared_ptr<Object>>& scene, float u, float v, int max_depth) {
    
    auto c_path = generate_camera_subpath(camera, scene, u, v, max_depth);
    auto l_path = generate_light_subpath(scene, max_depth);

    Vec3 radiance(0.0f, 0.0f, 0.0f);

    std::vector<Vec3> beta_c(c_path.size(), Vec3(1.0f, 1.0f ,1.0f));
    for (size_t i = 1; i < c_path.size(); ++i){
        const auto& vi   = c_path[i-1];
        const auto& vip1 = c_path[i];
        Vec3 d = (vip1.x - vi.x).normalize();
        float cos_theta = std::max(0.f, vi.N.dot(d));
        Vec3 fs = vi.brdf->evaluate(vi.N, d, vi.wi);

        beta_c[i] = beta_c[i-1] * fs * cos_theta / std::max(vi.pdf_fwd, 1e-6f);
    }

    std::vector<Vec3> beta_l(l_path.size(), Vec3(1.0f ,1.0f ,1.0f));
    if(!l_path.empty()) beta_l[0] = l_path[0].brdf->get_emission();
    for (size_t i = 1; i < l_path.size(); ++i){
        const auto& vi = l_path[i-1];
        const auto& vip1 = l_path[i];
        Vec3 d = (vip1.x - vi.x).normalize();
        float cos_theta = std::max(0.0f, vi.N.dot(d));
        Vec3 fs = vi.is_light ? Vec3(1.0f, 1.0f, 1.0f) : vi.brdf->evaluate(vi.N, d, vi.wi);
        
        beta_l[i] = beta_l[i-1] * fs * cos_theta / std::max(vi.pdf_fwd, 1e-6f);
    }

    // Connect all possible path combinations
    for (size_t s = 1; s <= c_path.size(); ++s) {
        for (size_t t = 0; t <= l_path.size(); ++t) {
            if (s + t < 2 || s + t > max_depth + 1) continue;
            
            if (t == 0){
                if (s <= c_path.size() && c_path[s-1].is_light)
                    radiance += beta_c[s-1] * c_path[s-1].brdf->get_emission();
            }
            else if (t <= l_path.size()) {
                Vec3 G = connect_verices(c_path[s-1], l_path[t-1], scene);
                Vec3 contribute = beta_c[s-1] * beta_l[t-1] * G;
                radiance += contribute;
            }
        }
    }

    return radiance;
}

Vec3 bdpt_render_n(const Camera& camera, const std::vector<std::shared_ptr<Object>>& scene, float u, float v, int ss, int tt) {

    int max_depth = std::max(ss, tt) + 2;
    auto c_path = generate_camera_subpath(camera, scene, u, v, max_depth);
    auto l_path = generate_light_subpath(scene, max_depth);

    // std::cout << "c_path : "<< c_path.size() <<"\n";
    // std::cout << "l_path : "<< l_path.size() <<"\n";

    Vec3 radiance(0.0f, 0.0f, 0.0f);

    std::vector<Vec3> beta_c(c_path.size(), Vec3(1.0f, 1.0f, 1.0f));
    for (size_t i = 1; i < c_path.size(); ++i) {
        const auto& vi = c_path[i-1];
        const auto& vip1 = c_path[i];
        Vec3 d = (vip1.x - vi.x).normalize();
        float cos_theta = std::max(0.0f, vi.N.dot(d));
        Vec3 fs = vi.brdf->evaluate(vi.N, d, vi.wi);
        
        beta_c[i] = beta_c[i-1] * fs * cos_theta / std::max(vi.pdf_fwd, 1e-6f);

        // std::cout << "cos : "<< cos_theta <<"\n";
        // std::cout << "fs = ("<< fs.x <<", "<< fs.y <<", "<< fs.z <<")\n";
        // std::cout << "beta_c[i-1] = ("<< beta_c[i-1].x <<", "<< beta_c[i-1].y <<", "<< beta_c[i-1].z <<")\n\n";
    }

    std::vector<Vec3> beta_l(l_path.size(), Vec3(1.0f, 1.0f, 1.0f));
    if (!l_path.empty()) {
        if (l_path.size() == 1) {
            beta_l[0] = l_path[0].brdf->get_emission();
        } else {
            Vec3 d0 = (l_path[1].x - l_path[0].x).normalize();
            float cos0 = std::max(0.0f, l_path[0].N.dot(d0));
            beta_l[0] = l_path[0].brdf->get_emission() * cos0 / std::max(l_path[0].pdf_fwd, 1e-6f);
        }
    }

    for (size_t i = 1; i < l_path.size(); ++i) {
        const auto& vi   = l_path[i-1];
        const auto& vip1 = l_path[i];
        Vec3 d = (vip1.x - vi.x).normalize();
        float cos_theta = std::max(0.0f, vi.N.dot(d));
        Vec3 fs = vi.is_light ? Vec3(1.0f, 1.0f, 1.0f): vi.brdf->evaluate(vi.N, d, vi.wi);

        beta_l[i] = beta_l[i-1] * fs * cos_theta / std::max(vi.pdf_fwd, 1e-6f);
    }


    size_t s = ss;
    size_t t = tt;
    if (s == 0 && t == 0) return radiance;
    else if (s == 0 && t > 0) return radiance;
    else if (t == 0 && s > 0) {
        if (s <= c_path.size() && c_path[s-1].is_light){
            radiance = beta_c[s-1] * c_path[s-1].brdf->get_emission();
        }
    }
    else if (s > 0 && t > 0) {
        if (s <= c_path.size() && t <= l_path.size()) {
            Vec3 G = connect_verices(c_path[s-1], l_path[t-1], scene);
            
            // Power heuristic with β = 2 (proper BDPT MIS)
            std::vector<float> strategy_pdfs;
            size_t path_len = s + t;
            
            // Enumerate all sampling strategies (s', t') such that s' + t' = path_len
            for (size_t s_prime = 1; s_prime < path_len; ++s_prime) {
                size_t t_prime = path_len - s_prime;
                
                if (s_prime <= c_path.size() && t_prime <= l_path.size()) {
                    float pdf = 1.0f;
                    
                    // Probability of generating camera path with s' vertices
                    for (size_t k = 0; k < s_prime - 1; ++k) {
                        pdf *= std::max(c_path[k].pdf_fwd, 1e-8f);
                    }
                    
                    // Probability of generating light path with t' vertices
                    for (size_t k = 0; k < t_prime - 1; ++k) {
                        pdf *= std::max(l_path[k].pdf_fwd, 1e-8f);
                    }
                    
                    strategy_pdfs.push_back(pdf);
                } else {
                    strategy_pdfs.push_back(0.0f);
                }
            }
            
            // Current strategy has index (s-1)
            int current_strategy_idx = s - 1;
            if (current_strategy_idx >= 0 && current_strategy_idx < strategy_pdfs.size()) {
                float mis_weight = mis_power_heuristic(strategy_pdfs, current_strategy_idx, 2.0f);
                radiance = mis_weight * beta_c[s-1] * beta_l[t-1] * G;
            } else {
                radiance = beta_c[s-1] * beta_l[t-1] * G;
            }
        }
    }
    
    // if (s == 0 && t == 0) {
    //     return radiance; // Invalid
    // } else if (s == 0 && t > 0) {// Light path hits camera directly
    //     return radiance;
    // } else if (t == 0 && s > 0) { // Camera path hits light directly
    //     if (s <= c_path.size() && c_path[s-1].is_light)
    //         radiance = beta_c[s-1] * c_path[s-1].brdf->get_emission();
    // } else if (s > 0 && t > 0) {
    //     // Connect camera and light vertices
    //     if (s <= c_path.size() && t <= l_path.size()) {
    //         Vec3 G = connect_verices(c_path[s-1], l_path[t-1], scene);
    //         Vec3 contribute = beta_c[s-1] * beta_l[t-1] * G;
    //         radiance = contribute;
    //     }
    // }

    return radiance;
}

inline void accumulate_light_only(const std::vector<PathVertex>& l_path, const std::vector<Vec3>& beta_l, const Camera& camera, int W, int H, std::vector<Vec3>& framebuffer, int t) {
    if(t < 2 || l_path.size() < 2) return;
    int idx = t - 2;
    if(idx >= l_path.size()) return;
    const auto& v = l_path[idx];

    Vec3 dir = v.x - camera.pos;
    float dist2 = dir.length_squared();
    float dist = std::sqrt(dist2);
    dir /= dist;

    float s_img, t_img;
    if(!camera.project_dir(dir, s_img, t_img)) return;


    int px = int(s_img * W);
    int py = int(t_img *(H - 1));
    if(px < 0 || px >= W || py < 0 || py >= H) return;

    float cos_cam = std::max(0.0f, (-camera.w).dot(dir));
    float cos_l = std::max(0.0f, v.N.dot(-dir));
    if(cos_cam == 0.0f || cos_l == 0.0f) return;

    float G = (cos_cam * cos_l) / dist2;
    Vec3 fs = v.is_light ? Vec3(1.0f, 1.0f, 1.0f) : v.brdf->evaluate(v.N, -dir, v.wi);
    Vec3 contribute = beta_l[idx] * G * fs;
    framebuffer[py * W + px] += contribute;
}
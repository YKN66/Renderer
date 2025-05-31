#pragma once
#include "Vec3.h"
#include "Object.h"
#include "Camera.h"
#include "PathGenerator.h"
#include "PathEvaluator.h"
#include <cmath>
#include <iostream>

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

    // for(size_t s = 0; s < c_path.size(); ++s) {

    //     if(c_path[s].is_light && int(s)==max_depth){
    //         radiance += beta_c[s] * c_path[s].brdf->get_emission();
    //     }

    //     for(size_t t = 0; t < l_path.size(); ++t) {
    //         if (s != 1 || t != 0) continue;

    //         Vec3 G = connect_verices(c_path[s], l_path[t], scene);
    //         Vec3 contribute = beta_c[s] * beta_l[t] * G;
    //         float w = simple_mis(s, t, c_path[s].pdf_fwd, l_path[t].pdf_fwd);
    //         radiance += w * contribute;
    //     }
    // }

    // Connect all possible path combinations
    for (size_t s = 1; s <= c_path.size(); ++s) {
        for (size_t t = 0; t <= l_path.size(); ++t) {
            // Skip invalid connections
            if (s + t < 2 || s + t > max_depth + 1) continue;
            
            // Handle different connection strategies
            if (t == 0) {
                // Camera path hits light directly (s > 0, t = 0)
                if (s <= c_path.size() && c_path[s-1].is_light) {
                    radiance += beta_c[s-1] * c_path[s-1].brdf->get_emission();
                }
            } else if (t <= l_path.size()) {
                // Connect camera and light vertices (s > 0, t > 0)
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

        // std::cout << "cos : "<< cos_theta <<"\n";
        // std::cout << "fs = ("<< fs.x <<", "<< fs.y <<", "<< fs.z <<")\n";
        // std::cout << "beta_l[i-1] = ("<< beta_l[i-1].x <<", "<< beta_l[i-1].y <<", "<< beta_l[i-1].z <<")\n\n";
    }


    // for(size_t s = 0; s < c_path.size(); ++s) {

    //     if(c_path[s].is_light && int(s)==depth){
    //         // std::cout << "s : "<< s <<"\n";
    //         radiance += beta_c[s] * c_path[s].brdf->get_emission();
    //         // std::cout << "b = ("<< beta_c[s].x <<", "<< beta_c[s].y <<", "<< beta_c[s].z <<")\n";
    //         // std::cout << "e = ("<< c_path[s].brdf->get_emission().x <<", "<< c_path[s].brdf->get_emission().y <<", "<< c_path[s].brdf->get_emission().z <<")\n";
    //         // std::cout << "c = ("<< radiance.x <<", "<< radiance.y <<", "<< radiance.z <<")\n\n";
    //     }

    //     for(size_t t = 0; t < l_path.size(); ++t) {
    //         // if(s + t > depth) continue;
    //         if (s != ss || t != tt) continue;
    //         // if(int(s + t) != depth) continue;

    //         // std::cout << "s : "<< s <<"\n";
    //         // std::cout << "t : "<< t <<"\n\n";

    //         Vec3 G = connect_verices(c_path[s], l_path[t], scene);
    //         Vec3 contribute = beta_c[s] * beta_l[t] * G;
    //         // float w = mis_weight(c_path[s], l_path[t]);
    //         float w = simple_mis(s, t, c_path[s].pdf_fwd, l_path[t].pdf_fwd);
    //         radiance += w * contribute;

    //         // std::cout << "G = ("<< G.x <<", "<< G.y <<", "<< G.z <<")\n";
    //         // std::cout << "contribute = ("<< contribute.x <<", "<< contribute.y <<", "<< contribute.z <<")\n\n";
    //     }
    // }

    // Only evaluate the specific (s,t) strategy requested
    size_t s = ss;
    size_t t = tt;
    
    if (s == 0 && t == 0) {
        // Invalid: need at least one vertex
        return radiance;
    } else if (s == 0 && t > 0) {
        // Light tracing only - skip for now
        return radiance;
    } else if (t == 0 && s > 0) {
        // Camera path hits light directly
        if (s <= c_path.size() && c_path[s-1].is_light) {
            radiance = beta_c[s-1] * c_path[s-1].brdf->get_emission();
        }
    } else if (s > 0 && t > 0) {
        // Connect camera and light vertices
        if (s <= c_path.size() && t <= l_path.size()) {
            Vec3 G = connect_verices(c_path[s-1], l_path[t-1], scene);
            Vec3 contribute = beta_c[s-1] * beta_l[t-1] * G;
            radiance = contribute;
        }
    }

    return radiance;
}
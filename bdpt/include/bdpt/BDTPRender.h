#pragma once
#include "Vec3.h"
#include "Object.h"
#include "Camera.h"
#include "PathGenerator.h"
#include "PathEvaluator.h"
#include <cmath>
#include <iostream>

Vec3 bdpt_render(const Camera& camera, const std::vector<std::shared_ptr<Object>>& scene, float u, float v, int depth) {

    auto c_path = generate_camera_subpath(camera, scene, u, v, depth);
    auto l_path = generate_light_subpath(scene, depth);

    // std::cout << "c_path : "<< c_path.size() <<"\n";
    // std::cout << "l_path : "<< l_path.size() <<"\n";

    Vec3 radiance(0.0f, 0.0f, 0.0f);

    std::vector<Vec3> beta_c(c_path.size(), Vec3(1.0f, 1.0f ,1.0f));
    for (size_t i = 1; i < c_path.size(); ++i){
        const auto& vi   = c_path[i-1];
        const auto& vip1 = c_path[i];
        float cos_theta = std::max(0.f, vi.N.dot(vip1.wi));
        Vec3  fs   = vi.brdf->evaluate(vi.N, vip1.wi, vi.wi);
        beta_c[i]  = beta_c[i-1] * fs * cos_theta / std::max(vi.pdf_fwd, 1e-6f);
    }

    std::vector<Vec3> beta_l(l_path.size(), Vec3(1.0f ,1.0f ,1.0f));
    if(!l_path.empty()) beta_l[0] = l_path[0].brdf->get_emission();
    for (size_t i = 1; i < l_path.size(); ++i){
        const auto& vi   = l_path[i-1];
        const auto& vip1 = l_path[i];
        float cos_theta = std::max(0.f, vi.N.dot(vip1.wi));
        Vec3  fs   = vi.brdf->evaluate(vi.N, vip1.wi, vi.wi);
        beta_l[i]  = beta_l[i-1] * fs * cos_theta / std::max(vi.pdf_fwd, 1e-6f);
    }

    for(size_t s = 0; s < c_path.size(); ++s) {

        if(c_path[s].is_light && int(s+1)==depth) radiance += beta_c[s] * c_path[s].brdf->get_emission();

        for(size_t t = 0; t < l_path.size(); ++t) {
            // if(s + t > depth) continue;
            if(int(s + t + 1) != depth) continue;

            // std::cout << "s : "<< s <<"\n";
            // std::cout << "t : "<< t <<"\n\n";

            Vec3 G = connect_verices(c_path[s], l_path[t], scene);
            Vec3 contribute = beta_c[s] * beta_l[t] * G;
            // float w = mis_weight(c_path[s], l_path[t]);
            float w = simple_mis(s, t, c_path[s].pdf_fwd, l_path[t].pdf_fwd);
            radiance += w * G;
        }
    }

    return radiance;
}
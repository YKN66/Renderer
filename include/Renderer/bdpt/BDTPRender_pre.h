#pragma once
#include "Vec3.h"
#include "Object.h"
#include "Camera.h"
#include "PathGenerator.h"
#include "PathEvaluator.h"

Vec3 bdpt_render(const Camera& camera, const std::vector<std::shared_ptr<Object>>& scene, float u, float v, int max_d) {

    auto c_path = generate_camera_subpath(camera, scene, u, v, max_d);
    auto l_path = generate_light_subpath(scene, max_d);

    // std::cout << "size = ("<< c_path.size() <<", "<< l_path.size() <<")\n"; 

    Vec3 L = Vec3(0.0f, 0.0f, 0.0f);
    Vec3 beta_camera = Vec3(1.0f, 1.0f, 1.0f);
    Vec3 beta_light = Vec3(1.0f, 1.0f, 1.0f);


    for(size_t s = 0; s < c_path.size(); ++s) {

        const auto& v = c_path[s];

        if(v.is_light) L += beta_camera * v.brdf->get_emission();

        if (s+1 < c_path.size()){
            const auto& next = c_path[s+1];
            float cosT = std::max(0.f, v.normal.dot(next.wi));
            Vec3  fs   = v.brdf->evaluate(v.normal, next.wi, v.wi);
            beta_camera = beta_camera * fs * cosT / v.pdf_fwd;
        }

        for(size_t t = 0; t < l_path.size(); ++t) {
            if(s + t > max_d) continue;

            if (t+1 < l_path.size()){
                const auto& next = l_path[t+1];
                float cosT = std::max(0.f, v.normal.dot(next.wi));
                Vec3  fs   = v.brdf->evaluate(v.normal, next.wi, v.wi);
                beta_light = beta_light * fs * cosT / v.pdf_fwd;
            }

            Vec3 contribute = beta_camera * beta_light * connect_verices(c_path[s], l_path[t], scene);
            // float w = mis_weight(c_path[s - 1], l_path[t - 1]);
            // L += w * contribute;
            L += contribute;
        }
    }

    return L;
}
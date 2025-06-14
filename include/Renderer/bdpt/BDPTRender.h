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

Vec3 bdpt_render(const Camera& camera, const std::vector<std::shared_ptr<Object>>& scene, float u, float v, int ss, int tt) {

    int max_depth = std::max(ss, tt) + 2;
    auto c_path = generate_camera_subpath(camera, scene, u, v, max_depth);
    auto l_path = generate_light_subpath(scene, max_depth);



    Vec3 radiance(0.0f, 0.0f, 0.0f);

    std::vector<Vec3> beta_c(c_path.size(), Vec3(1.0f, 1.0f, 1.0f));
    for (size_t i = 1; i < c_path.size(); ++i) {
        const auto& v0 = c_path[i-1];
        float cos = std::max(0.0f, v0.N.dot(v0.wi));
        Vec3 fs = v0.brdf->evaluate(v0.N, v0.wi, v0.wo);
        beta_c[i] = beta_c[i-1] * fs * cos / std::max(v0.pdf_fwd, 1e-6f);
    }

    std::vector<Vec3> beta_l(l_path.size(), Vec3(1.0f, 1.0f, 1.0f));
    if (!l_path.empty()) {
        const auto& v0 = l_path[0];
        float cos0 = std::max(0.0f, v0.N.dot(-v0.wi));
        // beta_l[0] = v0.brdf->get_emission() * cos0 / std::max(v0.pdf_area * v0.pdf_fwd, 1e-6f);
        beta_l[0] = v0.brdf->get_emission() / std::max(v0.pdf_area, 1e-6f);
        // beta_l[0] = v0.brdf->get_emission();
    }

    for (size_t i = 1; i < l_path.size(); ++i) {
        const auto& v0 = l_path[i-1];
        const auto& v1 = l_path[i];
        Vec3 d = (v1.x - v0.x).normalize();
        float cos = std::max(0.0f, v0.N.dot(d));
        Vec3 fs = v0.is_light ? Vec3(1.0f, 1.0f, 1.0f): v0.brdf->evaluate(v0.N, d, v0.wi);

        beta_l[i] = beta_l[i-1] * fs * cos / std::max(v0.pdf_fwd, 1e-6f);

    }


    size_t s = ss;
    size_t t = tt;
    if (s == 0 && t == 0) return radiance;
    else if (s == 0 && t > 0) return radiance;
    else if (s == 1 && t > 0) return radiance;
    else if (t == 0 && s > 0) {
        const auto& vl = c_path[s - 1];
        if (s <= c_path.size() && vl.is_light){
            const auto& vl = c_path[s - 1];
            const auto& vc = c_path[s - 2];
            radiance = beta_c[s-1] * vl.brdf->get_emission();
        }
    }
    else if (s > 0 && t > 0) {
        if (s <= c_path.size() && t <= l_path.size()) {
            const auto& vc = c_path[s - 1];
            const auto& vl = l_path[t - 1];
            Vec3 G = connect_verices(vc, vl, scene);
        
            Vec3 contribute = beta_c[s - 1] * beta_l[t - 1] * G;

            // /* ---------- MIS ウェイト ---------- */
            // size_t k = s + t;
            // std::vector<float> pdfs;
            // for (size_t sp = 1; sp < k; ++sp) {
            //     size_t tp = k - sp;
            //     if (sp <= c_path.size() && tp <= l_path.size())
            //         pdfs.push_back(path_pdf_area(c_path, l_path, sp, tp));
            //     else
            //         pdfs.push_back(0.0f);
            // }
            // float w = mis_power_heuristic(pdfs, s - 1, 2.0f);   // β = 2

            radiance = contribute;
        }

    }

    return radiance;
}

inline void accumulate_light_only(const std::vector<PathVertex>& l_path, const std::vector<Vec3>& beta_l, const Camera& camera, int W, int H, std::vector<Vec3>& fb, int t) {
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
    Vec3 fs = v.is_light ? v.brdf->get_emission() : v.brdf->evaluate(v.N, -dir, v.wi);
    Vec3 contribute = beta_l[idx] * G * fs;
    fb[py * W + px] += contribute;
}


inline void accumulate_eye_only(const Camera& camera, const std::vector<PathVertex>& l_path, const std::vector<Vec3>& beta_l, int t, int W, int H, std::vector<Vec3>& fb) {
    if (t < 2 || l_path.size() < (size_t)t) return;

    const auto& v = l_path[t - 1];
    Vec3 dir = (v.x - camera.pos).normalize();

    float s_img, t_img;
    if (!camera.project_dir(dir, s_img, t_img)) return;

    int px = int(s_img * W);
    int py = int(t_img * (H - 1));
    if (px < 0 || px >= W || py < 0 || py >= H) return;

    // Geometry term G(ys−1, z0)
    float dist2   = (v.x - camera.pos).length_squared();
    float cos_cam = std::max(0.0f, (-camera.w).dot(dir));
    float cos_l   = std::max(0.0f, v.N.dot(-dir));
    if (cos_cam == 0.0f || cos_l == 0.0f) return;
    float G = cos_cam * cos_l / dist2;

    Vec3 fs = v.is_light ? v.brdf->get_emission() : v.brdf->evaluate(v.N, -dir, v.wi);

    Vec3 contrib = beta_l[t - 1] * fs * G;

    fb[py * W + px] += contrib;
}

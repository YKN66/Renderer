#pragma once
#include "Vec3.h"
#include "BRDF.h"
#include "Path.h"
#include <cmath>
#include <vector>

Vec3 connect_verices(const PathVertex& vc, const PathVertex& vl, const std::vector<std::shared_ptr<Object>>& scene) {
    Vec3 dir = vl.x - vc.x;
    float dist2 = dir.length_squared();
    float dist = std::sqrt(dist2);
    Vec3 wi = dir / dist;

    Ray Shadow_ray(vc.x + vc.N * 1e-6f, wi);
    float t_max = dist - 1e-6f;
    for(const auto& obj : scene) {
        float t;
        if(obj->hit(Shadow_ray, 1e-6f, t_max, t)) {
            return Vec3(0.0f, 0.0f, 0.0f);
        }
    }

    //BRDF Geomotry
    float cos_c = std::max(0.0f, vc.N.dot(wi));
    float cos_l = std::max(0.0f, vl.N.dot(-wi));
    float G = cos_c * cos_l / dist2;

    Vec3 fs_cam = vc.brdf->evaluate(vc.N, wi, vc.wi);
    Vec3 fs_lig = vl.is_light ? Vec3(1.0f, 1.0f, 1.0f) : vl.brdf->evaluate(vl.N, -wi, vl.wi);

    Vec3 contribute = fs_cam * fs_lig * G;

    // std::cout << "fs_cam = ("<< fs_cam.x <<", "<< fs_cam.y <<", "<< fs_cam.z <<")\n";
    // std::cout << "fs_lig = ("<< fs_lig.x <<", "<< fs_lig.y <<", "<< fs_lig.z <<")\n";
    // std::cout << "countribute = ("<< contribute.x <<", "<< contribute.y <<", "<< contribute.z <<")\n";

    return contribute;

}

// ω_i = p_i^β / Σ_k p_k^β  (β=2 が一般的＝パワーヒューリスティック)
float mis_power_heuristic(const std::vector<float>& pdfs, int i, float beta = 2.0f)
{
    if(pdfs.empty()) return 0.0f;
    float num   = std::pow(pdfs[i], beta);
    float denom = 0.0f;
    for (float p : pdfs) denom += std::pow(p, beta);
    return (denom > 0.0f) ? num / denom : 0.0f;
}

float simple_mis(int s, int t) {
    // return 1.0f / float(s + t + 1);
    return 1.0f / float(s + 1) * (t + 1);
    // return 1.0f;
}
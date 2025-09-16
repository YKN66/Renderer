#pragma once
#include "Vec3.h"
#include "BRDF.h"
#include "Path.h"
#include <cmath>
#include <vector>

Vec3 connect_verices(const PathVertex& vc, const PathVertex& vl, const std::vector<std::shared_ptr<Object>>& scene) {

    // Vec3 dir = vl.x - vc.x;
    // float dist2 = dir.length_squared();
    // float dist = std::sqrt(dist2);
    // Vec3 wi = dir / dist;

    // Ray Shadow_ray(vc.x + vc.N * 1e-6f, wi);
    // float t_max = dist - 1e-6f;
    // for(const auto& obj : scene) {
    //     float t;
    //     if(obj->hit(Shadow_ray, 1e-6f, t_max, t)) {
    //         return Vec3(0.0f, 0.0f, 0.0f);
    //     }
    // }

    Vec3 shadow_origin = vc.x + vc.N * 1e-6f;
    Vec3 dir = vl.x - shadow_origin;
    float dist2 = dir.length_squared();
    float dist = std::sqrt(dist2);
    Vec3 wi = dir / dist;

    Ray Shadow_ray(shadow_origin, wi);
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

    Vec3 fs_cam = vc.brdf->evaluate(vc.N, wi, vc.wo);
    Vec3 fs_lig = vl.is_light ? Vec3(1.0f, 1.0f, 1.0f) : vl.brdf->evaluate(vl.N, -wi, vl.wo);

    Vec3 contribute = fs_cam * fs_lig * G;

    // std::cout << "fs_cam = ("<< fs_cam.x <<", "<< fs_cam.y <<", "<< fs_cam.z <<")\n";
    // std::cout << "fs_lig = ("<< fs_lig.x <<", "<< fs_lig.y <<", "<< fs_lig.z <<")\n";
    // std::cout << "countribute = ("<< contribute.x <<", "<< contribute.y <<", "<< contribute.z <<")\n";

    return contribute;

}

// // パワーヒューリスティック
// float mis_power_heuristic(const std::vector<float>& pdfs, int i = 0, float beta = 2.0f) {
//     if(pdfs.empty()) return 0.0f;
//     float num   = std::pow(pdfs[i], beta);
//     float denom = 0.0f;
//     for (float p : pdfs) denom += std::pow(p, beta);
//     return (denom > 0.0f) ? num / denom : 0.0f;
// }

float mis_power_heuristic(const std::vector<float>& pdfs, int i, float beta = 2.f){
    if(pdfs.empty()) return 0.f;

    /* ① 有効な最大値を取り出して 1 以下へ正規化 */
    float max_pdf = 0.f;
    for(float p : pdfs)
        if(std::isfinite(p) && p > max_pdf) max_pdf = p;
    if(max_pdf <= 0.f) return 0.f;

    /* ② 正規化 → pow → 加算。これで pow が inf になることはない */
    float num = 0.f, denom = 0.f;
    for(size_t k = 0; k < pdfs.size(); ++k)
    {
        float w = pdfs[k] / max_pdf;                // 0–1 に収まる
        if(!std::isfinite(w) || w <= 0.f) continue; // NaN, inf, 0 は無視
        w = std::pow(w, beta);

        if(int(k) == i) num = w;
        denom += w;
    }
    return (denom > 0.f) ? num / denom : 0.f;
}


float simple_mis(int s, int t) {return 1.0f / float(s + t - 1);}

inline void debug_mis_sum(const std::vector<float>& pdfs, float beta = 2.f, const char* tag = ""){
    float sum = 0.f;
    for(size_t k = 0; k < pdfs.size(); ++k)
        sum += mis_power_heuristic(pdfs, static_cast<int>(k), beta);

    // 許容誤差 1e-4 以上ならログを吐く
    if(std::fabs(sum - 1.f) > 1e-4f)
        std::fprintf(stderr,
            "[MIS-SUM] %s Σw = %.9f (|err| = %.3e, n = %zu)\n",
            tag, sum, std::fabs(sum - 1.f), pdfs.size());
}
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

// ------------------------------------------------------------
// 立体角 → 面積 pdf 変換
// ------------------------------------------------------------
inline float edge_pdf_area(const PathVertex& v0,
                           const PathVertex& v1,
                           float pdf_dir /* = v0.pdf_fwd */)
{
    Vec3 d = v1.x - v0.x;
    float dist2 = d.length_squared();
    float cos1  = std::max(0.0f, v1.N.dot(-d / std::sqrt(dist2)));

    // 立体角密度 (sr-1) から面積密度 (m-2) へ
    return pdf_dir * cos1 / dist2;
}

// ―― 光源頂点 0 だけは拡散面を仮定（コサイン加重） ――
inline float dir_pdf(const PathVertex& v, const Vec3& dir)
{
    if (v.is_light)
        return std::max(0.0f, v.N.dot(dir)) / M_PI;   // cos / π
    return v.brdf->pdf(v.N, dir);
}

// ------------------------------------------------------------
// サブパス (s,t) の生成確率 p(ξ) を面積測度で計算
//   s: カメラ側頂点数
//   t: ライト側頂点数
// ------------------------------------------------------------
inline float path_pdf_area(const std::vector<PathVertex>& c_path,
                           const std::vector<PathVertex>& l_path,
                           size_t s, size_t t)
{
    float pdf = 1.0f;

    // ライト頂点 0 の「位置」確率（面積密度 1 / A）
    if (t > 0)
        pdf *= l_path[0].pdf_fwd;            // generate_light_subpath で格納済み

    // カメラ側エッジ
    for (size_t i = 0; i + 1 < s; ++i)
    {
        const auto& v0 = c_path[i];
        const auto& v1 = c_path[i + 1];
        pdf *= edge_pdf_area(v0, v1, v0.pdf_fwd);     // v0.pdf_fwd は方向 pdf
    }

    // ライト側エッジ
    for (size_t i = 0; i + 1 < t; ++i)
    {
        const auto& v0 = l_path[i];
        const auto& v1 = l_path[i + 1];
        Vec3 dir = (v1.x - v0.x).normalize();
        pdf *= edge_pdf_area(v0, v1, dir_pdf(v0, dir));
    }

    return pdf;
}

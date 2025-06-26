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


inline float dir2area(const float pdf, const PathVertex& v_from, const PathVertex& v_to) {
    Vec3  d      = v_to.x - v_from.x;
    float dist2  = d.length_squared();
    if(dist2 == 0.f) return 0.f;
    float dist   = std::sqrt(dist2);
    Vec3  dir    = d / dist;
    float cos_v  = std::max(0.f, v_from.N.dot(dir));
    return pdf * cos_v / dist2;
}

inline float cal_pdf(const PathVertex& v_from, const PathVertex& v_to){
    Vec3 wi = (v_to.x - v_from.x).normalize();
    float pdf = v_from.brdf->pdf(v_from.N, wi);
    // float pdf = std::max(0.0f, v_from.N.dot(wi)) / M_PI;

    // std::cout << "wi : (" << wi.x << ", " << wi.y << ", " << wi.z << ")\n"; 
    // std::cout << "N : (" << v_from.N.x << ", " << v_from.N.y << ", " << v_from.N.z << ")\n"; 
    // std::cout << "pdf W : " << pdf << "\n";
    // std::cout << "brdf : " << v_from.brdf->pdf(v_from.N, wi) << "\n";

    return pdf;
}

struct StrategyPDF { 
    int s,t; 
    float pdf; 
};

std::vector<StrategyPDF> compute_strategy_pdfs(const std::vector<PathVertex>& cp, const std::vector<PathVertex>& cl, int s, int t) {

    const int k = static_cast<int>(cp.size() + cl.size()) - 1;
    std::vector<StrategyPDF> out;
    out.reserve(s + t);
    // if(s  + t != cp.size() + cl.size()) { 
    // if(s  != (int)cp.size() || t != (int)cl.size()) {
    //     std::cout << "s + t != path.size";
    //     return out;
    // }

    auto camera2light=[&](int a,int b){
        float p = 1.0f;
        for(int i=a;i<=b;++i){
            float t=dir2area(cp[i].pdf_W, cp[i], cp[i+1]);
            // std::cout << "camera : x" << i << " → x" << i+1 << " : " << t << "\n";
            p *= t;
        }
        return p;
    };
    auto light2camera=[&](int a,int b){
        float p = 1.0f;
        for(int i=a;i<=b;++i){
            float t=dir2area(cl[i].pdf_W, cl[i], cl[i+1]);
            // std::cout << "light : y" << i << " → y" << i+1 << " : " << t << "\n";
            // std::cout <<  i << "pdf : (" << cl[i].pdf_W << ")" << "\n";
            // std::cout << "(" << cl[i].x.x << ", " << cl[i].x.y << ", " << cl[i].x.z << ")" << "\n";
            // std::cout << "(" << cl[i+1].x.x << ", " << cl[i+1].x.y << ", " << cl[i+1].x.z << ")" << "\n";
            p *= t;
        }
        return p;
    };

    float Pst = 1.0f;
    if(s>0) Pst *= cp[0].pdf_A;
    if(s-2>=0) Pst *= camera2light(0,s-2);
    if(t>0) Pst *= cl[0].pdf_A;
    if(t-2>=0) Pst *= light2camera(0,t-2);
        
    out.push_back({s,t,Pst}); 

    //  光源側へ 
    int cur_s=s, cur_t=t; float P=Pst;
    while(cur_t>0){
        if(cur_s == s) {
            P   *= dir2area(cal_pdf(cp[cur_s-1], cl[cur_t-1]), cp[cur_s-1], cl[cur_t-1]);
            // std::cout << "light : x" << cur_s-1 << "→ y" << cur_t-1 << " : " << cal_pdf(cp[cur_s-1], cl[cur_t-1]) << "\n";
            // std::cout << "light : x" << cur_s-1 << "→ y" << cur_t-1 << " : " << dir2area(cal_pdf(cp[cur_s-1], cl[cur_t-1]), cp[cur_s-1], cl[cur_t-1]) << "\n";
        }
        else{
            if(cur_t>0) P *= dir2area(cl[cur_t].pdf_rev, cl[cur_t], cl[cur_t-1]);
            // std::cout << "light : y" << cur_t << ", y" << cur_t-1 << " : " << P << "\n";
            // std::cout << "(" << cl[cur_t].x.x << ", " << cl[cur_t].x.y << ", " << cl[cur_t].x.z << ")" << " → ";
            // std::cout << "(" << cl[cur_t-1].x.x << ", " << cl[cur_t-1].x.y << ", " << cl[cur_t-1].x.z << ") : " << cl[cur_t].pdf_rev << " → " << dir2area(cl[cur_t].pdf_rev, cl[cur_t], cl[cur_t-1]) << "\n";
        }
        ++cur_s; --cur_t;
        if(cur_t>0)   P /= dir2area(cl[cur_t-1].pdf_W, cl[cur_t-1], cl[cur_t]);
        else P /= cl[0].pdf_A; //t=0
        out.push_back({cur_s, cur_t, P});
    }

    // カメラ側へ
    cur_s=s, cur_t=t; P=Pst;
    while(cur_s>2){
        if(t == 0) {
            if(cur_t == t){
                P *= cp[s-1].pdf_A;
                // std::cout << "x" << s-1 << " pdfA : " << cp[s-1].pdf_A <<std::endl;
            }
            else {
                if(cur_s>0) P *= dir2area(cp[cur_s].pdf_rev, cp[cur_s], cp[cur_s-1]);
                // std::cout << "light : x" << cur_s << "→ x" << cur_s-1 << " : " << dir2area(cp[cur_s].pdf_rev, cp[cur_s], cp[cur_s-1]) << "\n";
                // std::cout << cp[cur_s].pdf_rev << std::endl;
                // std::cout << "(" << cp[cur_s].x.x << ", " << cp[cur_s].x.y << ", " << cp[cur_s].x.z << ")" << " → ";
                // std::cout << "(" << cp[cur_s-1].x.x << ", " << cp[cur_s-1].x.y << ", " << cp[cur_s-1].x.z << ")" << "\n";
            }
            
            ++cur_t; --cur_s;
            if(cur_s>0)   P /= dir2area(cp[cur_s-1].pdf_W, cp[cur_s-1], cp[cur_s]);
            else P /= cp[0].pdf_A; //s=0
            out.push_back({cur_s, cur_t, P});
        }
        else{
            if(cur_t == t) {
                P   *= dir2area(cal_pdf(cl[cur_t-1], cp[cur_s-1]), cl[cur_t-1], cp[cur_s-1]);
                // std::cout << "light : y" << cur_t-1 << ", x" << cur_s-1 << " : " << cal_pdf(cl[cur_t-1], cp[cur_s-1]) << "\n";
                // std::cout << "light : y" << cur_t-1 << ", x" << cur_s-1 << " : " << dir2area(cal_pdf(cl[cur_t-1], cp[cur_s-1]), cl[cur_t-1], cp[cur_s-1]) << "\n";
            }
            else{
                if(cur_s>0) P *= dir2area(cp[cur_s].pdf_rev, cp[cur_s], cp[cur_s-1]);
                // std::cout << "light : x" << cur_s << ", x" << cur_s-1 << " : " << P << "\n";
                // std::cout << "(" << cp[cur_s].x.x << ", " << cp[cur_s].x.y << ", " << cp[cur_s].x.z << ")" << " → ";
                // std::cout << "(" << cp[cur_s-1].x.x << ", " << cp[cur_s-1].x.y << ", " << cp[cur_s-1].x.z << ")" << "\n";
            }
            ++cur_t; --cur_s;
            if(cur_s>0)   P /= dir2area(cp[cur_s-1].pdf_W, cp[cur_s-1], cp[cur_s]);
            else P /= cp[0].pdf_A; //s=0
            out.push_back({cur_s, cur_t, P});
        }

    }


    return out;
}



Vec3 bdpt_render(const Camera& camera, const std::vector<std::shared_ptr<Object>>& scene, float u, float v) {

    const int MaxDepth = 10;
    auto l_path = generate_light_subpath(scene, MaxDepth - 1);
    auto c_path = generate_camera_subpath(camera, scene, u, v, MaxDepth - 1);

    Vec3 radiance(0.0f, 0.0f, 0.0f);

    int sMax = (int)c_path.size();
    int tMax = (int)l_path.size();
    // std::cout << "sMax " << sMax << std::endl;
    // std::cout << "tMax " << tMax << std::endl << std::endl;

    for(int s=2; s<=sMax; ++s){
        const PathVertex& vc = c_path[s-1];

        for(int t=0; t<=tMax; ++t){
            Vec3 contrib;

            if (t == 0) {
                const auto& vl = c_path[s - 1];
                if (s <= c_path.size() && vl.is_light){
                    const auto& vl = c_path[s - 1];
                    contrib = vl.beta * vl.brdf->get_emission();


                    std::vector<PathVertex> cp_part(c_path.begin(), c_path.begin()+s);
                    std::vector<PathVertex> lp_part(l_path.begin(), l_path.begin()+t);


                    auto pdfs_st = compute_strategy_pdfs(cp_part, lp_part, s, t);
                    std::vector<float> pdfs;  pdfs.reserve(pdfs_st.size());
                    int idx = -1;
                    for(size_t i=0;i<pdfs_st.size();++i){
                        pdfs.push_back(pdfs_st[i].pdf);
                        if(pdfs_st[i].s == s && pdfs_st[i].t == t) idx = int(i);
                    }
                    if(idx < 0) continue;
                    float w = mis_power_heuristic(pdfs, idx, 2.0f);

                    radiance += contrib * w;
                }
            }
            else{
                const PathVertex& vl = l_path[t-1];
                Vec3 G = connect_verices(vc, vl, scene);
                if(G.is_zero()) continue;

                contrib = vc.beta * vl.beta * G;


                std::vector<PathVertex> cp_part(c_path.begin(), c_path.begin()+s);
                std::vector<PathVertex> lp_part(l_path.begin(), l_path.begin()+t);


                auto pdfs_st = compute_strategy_pdfs(cp_part, lp_part, s, t);
                std::vector<float> pdfs;  pdfs.reserve(pdfs_st.size());
                int idx = -1;
                for(size_t i=0;i<pdfs_st.size();++i){
                    pdfs.push_back(pdfs_st[i].pdf);
                    if(pdfs_st[i].s == s && pdfs_st[i].t == t) idx = int(i);
                }
                if(idx < 0) continue;
                float w = mis_power_heuristic(pdfs, idx, 2.0f);

                radiance += contrib * w;
            }
        }

    }

    return radiance;
}



Vec3 bdpt_render_debug(const Camera& camera, const std::vector<std::shared_ptr<Object>>& scene, float u, float v, int ss, int tt) {

    // int max_depth = std::max(ss, tt) + 2;
    auto c_path = generate_camera_subpath(camera, scene, u, v, ss);
    auto l_path = generate_light_subpath(scene, tt);


    Vec3 radiance(0.0f, 0.0f, 0.0f);


    size_t s = ss;
    size_t t = tt;
    if (s == 0 && t == 0) return radiance;
    else if (s == 0 && t > 0) return radiance;
    else if (s == 1 && t > 0){
        if (t <= l_path.size()) {
            const auto& vc = c_path[0];
            const auto& vl = l_path[t-1];
            Vec3 G  = connect_verices(vc, vl, scene);
            radiance = vc.beta * vl.beta * G;
        }
    }
    else if (t == 0 && s > 0) {
        const auto& vl = c_path[s - 1];
        if (s <= c_path.size() && vl.is_light){
            const auto& vl = c_path[s - 1];
            const auto& vc = c_path[s - 2];
            // if(s == 2) radiance = vl.brdf->get_emission();
            radiance = vl.beta * vl.brdf->get_emission();
        }
    }
    else if (s > 0 && t > 0) {
        if (s <= c_path.size() && t <= l_path.size()) {
            const auto& vc = c_path[s - 1];
            const auto& vl = l_path[t - 1];
            Vec3 G = connect_verices(vc, vl, scene);
        
            // Vec3 contribute = beta_c[s - 1] * beta_l[t - 1] * G;
            Vec3 contribute = vc.beta * vl.beta * G;

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

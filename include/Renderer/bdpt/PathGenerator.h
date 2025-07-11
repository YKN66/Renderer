#pragma once
#include "Vec3.h"
#include "BRDF.h"
#include "Path.h"
#include <algorithm>

static bool bounce_walk(std::vector<PathVertex>& path, const std::vector<std::shared_ptr<Object>>& scene, bool isLight) {

    const int  kRRStartDepth  = 3;

    const PathVertex& prev = path.back();
    Ray ray(prev.x + prev.N * 1e-4f, prev.wi);

    float closest_t = 1e30f;
    std::shared_ptr<Object> hit_obj = nullptr;
    for (const auto& obj : scene){
        float t; if(obj->hit(ray, 1e-4f, closest_t, t)){ closest_t = t; hit_obj = obj; }
    }
    if (!hit_obj) return false;

    Vec3 x = ray.at(closest_t);
    Vec3 N = hit_obj->get_normal(x);
    Vec3 wo = -ray.direction.normalize();
    auto  brdf = hit_obj->get_material();

    if (path.size()>1 && hit_obj->is_light() && isLight) return false;

    float pdf_W;
    Vec3  wi   = brdf->sample(N, wo, pdf_W);
    float pdf_rev = brdf->pdf(N, wo);

    PathVertex vn;
    vn.x = x;
    vn.N = N;
    vn.wi = wi;
    vn.wo = wo;
    vn.brdf = brdf;
    vn.pdf_W = pdf_W;
    vn.pdf_rev = pdf_rev;
    if(hit_obj->is_light()) vn.pdf_A = 1.0f / hit_obj->get_area();
    vn.is_light = hit_obj->is_light();

    // ---- β 更新 ----
    Vec3 d = (vn.x - prev.x).normalize();
    float cosPrev = std::max(0.f, prev.N.dot(d));
    Vec3 fs = prev.is_light ? Vec3(1.0f, 1.0f, 1.0f) : prev.brdf->evaluate(prev.N, prev.wi, prev.wo);
    vn.beta = prev.beta * fs * cosPrev / std::max(prev.pdf_W, 1e-6f);

    float ratio = (prev.pdf_W > 1e-6f) ? prev.pdf_rev / prev.pdf_W : 0.f;
    vn.vc  = prev.vcm;
    vn.vcm = prev.vcm * ratio;

    path.push_back(vn);
    int depth = int(path.size()) - 1;

    // if(depth >= kRRStartDepth){
    //     float q = std::clamp(vn.beta.max_component(), 0.05f, 0.95f); // 生存確率
    //     if(random_float() > q) return false;   // 打ち切り
    //     vn.beta /= q;                   // 生存補正
    // }

    if (pdf_W < 1e-6f) return false;
    if (path.size()>1 && vn.is_light && !isLight) return false;
    return true;
}

std::vector<PathVertex> generate_camera_subpath(const Camera& camera, const std::vector<std::shared_ptr<Object>>& scene, float u, float v, int depth) {
    std::vector<PathVertex> path;
    if (depth == 0) return path;

    PathVertex v0;
    Vec3 dir = camera.get_ray(u, v).direction;
    float dist2 = dir.length_squared();
    v0.x = camera.pos;
    v0.N = -camera.w;
    v0.wi = dir.normalize();
    v0.wo = v0.wi;
    v0.brdf = std::make_shared<Sensor>();
    float cos_theta = std::max(0.0f, v0.N.dot(dir / std::sqrt(dist2)));
    float A_img     = camera.viewport_width * camera.viewport_height;
    float pdf_dir   = (cos_theta > 0.0f) ? 1.0f / (cos_theta * cos_theta * cos_theta * A_img) : 0.0f;

    v0.pdf_A = 1.0f / A_img;
    // float pdf_dir   = v0.pdf_A * dist2 / (cos_theta * cos_theta * cos_theta);
    v0.pdf_W = cos_theta;
    v0.pdf_rev = pdf_dir;
    // v0.beta /= v0.pdf_W;
    v0.beta = Vec3(1.0f, 1.0f, 1.0f);
    v0.vc = 0.0f;
    // v0.vcm = 1.0f;
    v0.vcm = A_img / v0.pdf_W;
    Vec3 l = v0.beta * v0.pdf_W;
    // std::cout << "test = ("<< l.x <<", "<< l.y <<", "<< l.z <<")\n";
    v0.is_light = false;
    path.push_back(v0);

    Ray ray(v0.x, v0.wo);

    while(int(path.size())<depth)
        if(!bounce_walk(path,scene,false)) break;

    return path;
}

std::vector<PathVertex> generate_light_subpath(const std::vector<std::shared_ptr<Object>>& scene, int depth) {
    std::vector<PathVertex> path;
    if (depth == 0) return path;

    std::vector<std::shared_ptr<Object>> lights;
    for(const auto& obj : scene) {
        if(obj->is_light()) lights.push_back(obj);
    }
    if(lights.empty()) return path;

    auto rect = std::dynamic_pointer_cast<Rectangle>(lights[0]);

    Vec3 L0 = sample_light_rectangle(rect->get_center(), rect->get_u(), rect->get_v());
    Vec3 nL = rect->get_normal(L0);

    Vec3 dir_local = uniform_hemisphere();
    Vec3 T = tangent_vector(nL);
    Vec3 B = nL.cross(T);
    Vec3 wi = (T * dir_local.x + B * dir_local.y + nL * dir_local.z).normalize();

    float pdf_dir = 1.0f / (2.0f * M_PI);

    PathVertex v0;
    v0.x = L0;
    v0.N = nL;
    v0.wi = wi;
    v0.wo = wi;
    v0.brdf = rect->get_material();
    v0.pdf_A = 1.0f / rect->get_area();
    v0.pdf_W = pdf_dir;
    v0.pdf_rev = pdf_dir;
    v0.beta = Vec3(0.0f, 0.0f, 0.0f);
    v0.vc = 0.0f;
    v0.vcm = 1.0f;
    v0.is_light = true;
    float selPdf = 1.0f; //numLights
    float emissionPdf = v0.pdf_A*pdf_dir*selPdf;
    float cos0 = std::max(0.0f, v0.N.dot(v0.wi));
    v0.beta = rect->get_material()->get_emission() / std::max(v0.pdf_A, 1e-6f);
    path.push_back(v0);

    Ray ray(L0 + nL * 1e-4f, wi);

    while(int(path.size())<depth)
        if(!bounce_walk(path,scene,true)) break;

    return path;
}

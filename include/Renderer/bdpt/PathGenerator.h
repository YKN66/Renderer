#pragma once
#include "Vec3.h"
#include "BRDF.h"
#include "Path.h"

std::vector<PathVertex> generate_camera_subpath(const Camera& camera, const std::vector<std::shared_ptr<Object>>& scene, float u, float v, int depth) {
    std::vector<PathVertex> path;

    PathVertex v0;
    v0.x = camera.pos;
    v0.N = -camera.w;
    v0.wi = camera.get_ray(u, v).direction.normalize();
    v0.brdf = std::make_shared<Sensor>();
    float cos_theta = std::max(0.0f, v0.N.dot(v0.wi));
    float A_img     = camera.viewport_width * camera.viewport_height;
    float pdf_dir   = (cos_theta > 0.0f) ? (1.0f / (A_img * cos_theta)) : 0.0f;
    v0.pdf_fwd = pdf_dir;
    v0.pdf_rev = pdf_dir;
    // v0.pdf_fwd = 1.0f;
    // v0.pdf_rev = 1.0f;
    v0.is_light = false;
    path.push_back(v0);

    Ray ray(v0.x, v0.wi);

    for(int bounce = 0; bounce < depth; ++bounce) {
        float closest_t = 1e30f;
        std::shared_ptr<Object> hit_obj = nullptr;

        for(const auto& obj : scene) {
            float t;
            if(obj->hit(ray, 1e-3f, closest_t, t)) {
                closest_t = t;
                hit_obj = obj;
            }
        }

        if(!hit_obj) break;

        Vec3 x = ray.at(closest_t);
        Vec3 N = hit_obj->get_normal(x);
        Vec3 wo = -ray.direction.normalize();
        auto brdf = hit_obj->get_material();

        //方向サンプリング
        float pdf_fwd;
        Vec3 wi = brdf->sample(N, wo, pdf_fwd);
        float pdf_rev = brdf->pdf(N, wi);

        PathVertex vn;
        vn.x = x;
        vn.N = N;
        vn.wi = wo;
        vn.brdf = brdf;
        vn.pdf_fwd = pdf_fwd;
        vn.pdf_rev = pdf_rev;
        vn.is_light = hit_obj->is_light();
        path.push_back(vn);

        if (vn.is_light) break;// 光源を直接見たら終了
        if(pdf_fwd < 1e-6f) break;

        ray = Ray(x + N * 1e-3f, wi);
    }

    return path;
}

std::vector<PathVertex> generate_light_subpath(const std::vector<std::shared_ptr<Object>>& scene, int depth) {
    std::vector<PathVertex> path;

    std::vector<std::shared_ptr<Object>> lights;
    for(const auto& obj : scene) {
        if(obj->is_light()) lights.push_back(obj);
    }
    if(lights.empty()) return path;

    auto rect = std::dynamic_pointer_cast<Rectangle>(lights[0]);

    Vec3 L0 = sample_light_rectangle(rect->get_center(), rect->get_u(), rect->get_v());
    Vec3 nL = rect->get_normal(L0);

    //サンプリングの方向　local to world
    Vec3 dir_local = cos_weight_sampling();
    Vec3 T = tangent_vector(nL);
    Vec3 B = nL.cross(T);
    Vec3 wi = (T * dir_local.x + B * dir_local.y + nL * dir_local.z).normalize();

    float pdf_dir = std::max(0.0f, nL.dot(wi)) / M_PI;

    PathVertex v0;
    v0.x = L0;
    v0.N = nL;
    v0.wi = -wi;
    v0.brdf = rect->get_material();
    v0.pdf_fwd = 1.0f / rect->get_area();
    v0.pdf_rev = pdf_dir;
    v0.is_light = true;
    path.push_back(v0);

    Ray ray(L0 + nL * 1e-3f, wi);

    for(int bounce = 0; bounce < depth; ++bounce) {
        float closest_t = 1e30f;
        std::shared_ptr<Object> hit_obj = nullptr;

        for(const auto& obj : scene) { 
            float t;
            if(obj->hit(ray, 1e-3f, closest_t, t)) {
                closest_t = t;
                hit_obj = obj;
            }
        }

        if(!hit_obj) break;

        Vec3 x = ray.at(closest_t);
        Vec3 N = hit_obj->get_normal(x);
        Vec3 wo = -ray.direction.normalize();
        auto brdf = hit_obj->get_material();

        // float pdf_rev = brdf->pdf(N, wi);
        float pdf_fwd;
        Vec3 next_dir = brdf->sample(N, wo, pdf_fwd);
        float pdf_rev = brdf->pdf(N, next_dir);

        PathVertex vn;
        vn.x = x;
        vn.N = N;
        vn.wi = wo;
        vn.brdf = brdf;
        vn.pdf_fwd = pdf_fwd;
        vn.pdf_rev = pdf_rev;
        vn.is_light = hit_obj->is_light();
        path.push_back(vn);

        if(pdf_fwd < 1e-6f) break;

        ray = Ray(x + N * 1e-6f, next_dir);
    }

    return path;
}
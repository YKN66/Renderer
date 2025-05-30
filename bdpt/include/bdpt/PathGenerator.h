#pragma once
#include "Vec3.h"
#include "BRDF.h"
#include "Path.h"

std::vector<PathVertex> generate_camera_subpath(const Camera& camera, const std::vector<std::shared_ptr<Object>>& scene, float u, float v, int depth) {
    std::vector<PathVertex> path;

    Vec3 origin = camera.pos;
    Vec3 dir = camera.get_ray(u, v).direction;

    Ray ray(origin, dir);

    for(int bounce = 0; bounce < depth; ++bounce) {
        float closest_t = 1e30f;
        std::shared_ptr<Object> hit_obj = nullptr;

        for(const auto& obj : scene) {
            float t;
            if(obj->hit(ray, 0.001f, closest_t, t)) {
                closest_t = t;
                hit_obj = obj;
            }
        }

        if(!hit_obj) break;

        Vec3 x = ray.at(closest_t);
        Vec3 N = hit_obj->get_normal(x);
        Vec3 wo = -ray.direction.normalize();
        std::shared_ptr<BRDF> brdf = hit_obj->get_material();

        //方向サンプリング
        float pdf;
        Vec3 next_dir = brdf->sample(N, wo, pdf);

        PathVertex vertex;
        vertex.x = x;
        vertex.N = N;
        vertex.wi = wo;
        vertex.brdf = brdf;
        vertex.pdf_fwd = pdf;
        vertex.pdf_rev = pdf;
        vertex.is_light = hit_obj->is_light();
        path.push_back(vertex);

        if (hit_obj->is_light()) break;// 光源を直接見たら終了

        if(pdf < 1e-6f) break;

        ray = Ray(x + N * 1e-6f, next_dir);
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

    auto light = lights[0];
    auto rect = std::dynamic_pointer_cast<Rectangle>(light);

    Vec3 L0 = sample_light_rectangle(rect->get_center(), rect->get_u(), rect->get_v());
    Vec3 nL = rect->get_normal(L0);

    //サンプリングの方向　local to world
    Vec3 dir_local = cos_weight_sampling();
    Vec3 T = tangent_vector(nL);
    Vec3 B = nL.cross(T);
    Vec3 wi_world = (T * dir_local.x + B * dir_local.y + nL * dir_local.z).normalize();

    float pdf = std::max(0.0f, nL.dot(wi_world));

    PathVertex vertex;
    vertex.x = L0;
    vertex.N = nL;
    vertex.wi = -wi_world;
    vertex.brdf = rect->get_material();
    vertex.pdf_fwd = 1.0f / rect->get_area();
    vertex.pdf_rev = pdf;
    vertex.is_light = true;
    path.push_back(vertex);

    Ray ray(L0 + nL * 1e-6f, wi_world);

    for(int bounce = 0; bounce < depth; ++bounce) {
        float closest_t = 1e30f;
        std::shared_ptr<Object> hit_obj = nullptr;

        for(const auto& obj : scene) { 
            float t;
            if(obj->hit(ray, 0.001f, closest_t, t)) {
                closest_t = t;
                hit_obj = obj;
            }
        }

        if(!hit_obj) break;

        Vec3 x = ray.at(closest_t);
        Vec3 N = hit_obj->get_normal(x);
        Vec3 wi = -ray.direction.normalize();
        auto brdf = hit_obj->get_material();
        float pdf = brdf->pdf(N, wi);

        vertex.x = x;
        vertex.N = N;
        vertex.wi = wi;
        vertex.brdf = brdf;
        vertex.pdf_fwd = pdf;
        vertex.pdf_rev = pdf;
        vertex.is_light = hit_obj->is_light();
        path.push_back(vertex);

        Vec3 next_dir = brdf->sample(N, wi, pdf);
        if(pdf < 1e-6f) break;

        ray = Ray(x + N * 1e-6f, next_dir);
    }

    return path;
}
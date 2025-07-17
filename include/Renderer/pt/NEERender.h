#pragma once
#include "Ray.h"
#include "Object.h"
#include "Rectangle.h"
#include "Vec3.h"
#include <memory>
#include <vector>
#include <algorithm>


inline float power_heuristic(float f, float g) {
    float f2 = f * f;
    float g2 = g * g;
    return f2 / (f2 + g2);
}


Vec3 nee_render(const Ray& r, const std::vector<std::shared_ptr<Object>>& scene, int depth) {
    const float epsilon = 1e-4f;
    const int RR = 3;
    Vec3 throughput(1.0f, 1.0f, 1.0f);
    Vec3 radiance(0.0f, 0.0f, 0.0f);
    Ray ray = r;

    float prev_pdf_bsdf = 1.0f;

    for(int bounce = 0; bounce < depth; ++bounce){

        if(bounce >= RR){
            float p = std::clamp(std::max({throughput.x, throughput.y, throughput.z}), 0.0f, 0.99f);
            // float p = std::min(std::max({throughput.x, throughput.y, throughput.z}), 1.0f);
            if(random_float() > p) break;
            throughput /= p;
        }

        float closest_t = 1e30f;
        std::shared_ptr<Object> hit_object = nullptr;

        for(const auto& obj : scene){
            float t;
            if(obj->hit(ray, epsilon, closest_t, t)){
                closest_t = t;
                hit_object = obj;
            }
        }

        if(!hit_object) break;

        Vec3 x = ray.at(closest_t);
        Vec3 N = hit_object->get_normal(x);
        Vec3 wo = -ray.direction.normalize();

        if(hit_object->is_light()){
            if(bounce == 0){
                radiance += throughput * hit_object->get_material()->get_emission();
                break;
            }

            // float pdf_bsdf = prev_pdf_bsdf;
            // auto rect = std::dynamic_pointer_cast<Rectangle>(hit_object);

            // if (rect) {
            //     Vec3  y = x;
            //     float cos_alpha = std::max(0.0f, (-wo).dot(rect->get_normal(y)));
            //     float dist2     = (y - ray.origin).length_squared();
            //     float pdf_bsdf_A   = pdf_bsdf * cos_alpha / dist2;
            //     float pdf_light_A  = 1.0f / rect->get_area();
            //     float pdf_light = pdf_light_A * dist2 / cos_alpha;

            //     float w_bsdf = power_heuristic(pdf_bsdf_A, pdf_light_A);
            //     // float w_bsdf = power_heuristic(pdf_bsdf, pdf_light);

            //     radiance += throughput * hit_object->get_material()->get_emission() * w_bsdf;
            // }
            // break;
        }

        for(const auto& light : scene){
            auto rect = std::dynamic_pointer_cast<Rectangle>(light);
            // if(!rect) continue;
            if(!rect || !rect->is_light()) continue;


            Vec3 light_pos = sample_light_rectangle(rect->get_center(), rect->get_u(), rect->get_v());
            Vec3 shadow_origin = x + N * epsilon;
            Vec3 wi_vec       = light_pos - shadow_origin;
            float dist2       = wi_vec.length_squared();
            float dist        = std::sqrt(dist2);
            Vec3 wi           = wi_vec / dist;

            Ray shadow_ray(shadow_origin, wi);
            bool in_shadow = false;
            for (auto& obj : scene) {
                if(obj->is_light()) continue;
                float t;
                if (obj->hit(shadow_ray, epsilon, dist - epsilon, t)) {
                    in_shadow = true;
                    break;
                }
            }
            if(in_shadow) continue;
            

            Vec3 fr = hit_object->get_material()->evaluate(N, wi, wo);
            Vec3 Le = light->get_material()->get_emission();
            float cos_theta = std::max(0.0f, N.dot(wi));
            float cos_alpha = std::max(0.0f, (-wi).dot(rect->get_normal(light_pos)));
            float area = rect->get_area();

            float pdf_light_A = 1.0f / rect->get_area();
            float G = (cos_theta * cos_alpha) / dist2;

            float pdf_light = dist2 / (cos_alpha * area);
            float pdf_bsdf = hit_object->get_material()->pdf(N, wi);
            float pdf_bsdf_A  = pdf_bsdf * cos_alpha / dist2;

            float w_light = power_heuristic(pdf_light_A, pdf_bsdf_A);
            // float w_light = power_heuristic(pdf_light, pdf_bsdf);



            radiance += throughput * fr * Le * G / pdf_light_A;
            // radiance += throughput * fr * Le * G / pdf_light_A * w_light;

        }

        float pdf;
        Vec3 wi = hit_object->get_material()->sample(N, wo, pdf);
        if(pdf <= 0.0f) break;

        Vec3 fr = hit_object->get_material()->evaluate(N, wi, wo);
        float cos_out = std::max(0.0f, N.dot(wi));
        throughput *= fr * cos_out / pdf;

        ray = Ray(x + N * epsilon, wi);
        prev_pdf_bsdf = pdf;
            
    }

    return radiance;
}
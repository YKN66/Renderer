#pragma once
#include "Ray.h"
#include "Object.h"
#include "Rectangle.h"
#include "Vec3.h"


Vec3 nee_render(const Ray& r, const std::vector<std::shared_ptr<Object>>& scene, int depth) {
    const float epsilon = 1e-3f;
    const int RR = 3;
    Vec3 throughput(1.0f, 1.0f, 1.0f);
    Vec3 radiance(0.0f, 0.0f, 0.0f);
    Ray ray = r;

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

        if(hit_object->is_light()){
            if(bounce == 0){
                radiance += throughput * hit_object->get_material()->get_emission();
            }
            break;
        }

        Vec3 x = ray.at(closest_t);
        Vec3 N = hit_object->get_normal(x);
        Vec3 wo = -ray.direction.normalize();

        for(const auto& light : scene){
            auto rect = std::dynamic_pointer_cast<Rectangle>(light);
            if(!rect) continue;
            
            Vec3 light_pos = sample_light_rectangle(rect->get_center(), rect->get_u(), rect->get_v());
            Vec3 wi = (light_pos - x).normalize();
            float dist = (light_pos - x).length();
            float dist2 = dist * dist;

            Ray shadow_ray(x + N * epsilon, wi);
            bool in_shadow = false;

            for(const auto& obj : scene){
                float t;
                if(obj->hit(shadow_ray, epsilon, dist - epsilon, t) && !obj->is_light()){
                    in_shadow = true;
                    break;
                }
            }
            if(in_shadow) continue;

            Vec3 brdf = hit_object->get_material()->evaluate(N, wi, wo);
            Vec3 Le = light->get_material()->get_emission();
            float cos_theta = std::max(0.0f, N.dot(wi));
            float cos_alpha = std::max(0.0f, (-wi).dot(rect->get_normal(light_pos)));
            float G = cos_theta * cos_alpha / dist2;
            float area = rect->get_area();

            radiance += throughput * (brdf * Le * G * area);

        }

        float pdf;
        Vec3 wi = hit_object->get_material()->sample(N, wo, pdf);
        if(pdf <= 0) break;

        Vec3 brdf_val = hit_object->get_material()->evaluate(N, wi, wo);
        float cos_out = std::max(0.0f, N.dot(wi));
        throughput *= brdf_val * cos_out / pdf;

        ray = Ray(x + N * epsilon, wi);
            
    }

    return radiance;
}
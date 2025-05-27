#pragma once
#include "Ray.h"
#include "Object.h"
#include "Rectangle.h"
#include "Vec3.h"

Vec3 cos_weight_render(const Ray& r, const std::vector<std::shared_ptr<Object>>& scene, int depth) {
    const float epsilon = 1e-3f;
    const int RR = 3;
    Vec3 throughput(1.0f, 1.0f, 1.0f);
    Vec3 radiance(0.0f, 0.0f, 0.0f);
    Ray ray = r;

    for(int bounce = 0; bounce < depth; bounce++) {

        if(bounce >= RR) {
            float p = std::clamp(std::max({throughput.x, throughput.y, throughput.z}), 0.0f, 0.99f);
            // float p = std::min(std::max({throughput.x, throughput.y, throughput.z}), 0.99f);
            if(random_float() > p) break;
            throughput /= p;
        }

        float closest_t = 1e30f;
        std::shared_ptr<Object> hit_object = nullptr;
        for(const auto& obj : scene) {
            float t;
            if(obj->hit(ray, epsilon, closest_t, t)) {
                closest_t = t;
                hit_object = obj;
            }
        }
        if(!hit_object) break;

        Vec3 x = ray.at(closest_t);
        Vec3 N = hit_object->get_normal(x);
        Vec3 wo = -ray.direction.normalize();

        if(hit_object->is_light()) {
            radiance += throughput * hit_object->get_material()->get_emission();
            break;
        }

        float pdf;
        Vec3 wi = hit_object->get_material()->sample(N, wo, pdf);
        if(pdf <= 0) break;

        Vec3 brdf = hit_object->get_material()->evaluate(N, wi, wo);
        float cos_theta = std::max(0.0f, N.dot(wi));

        throughput *= brdf * cos_theta / pdf;

        ray = Ray(x + N * epsilon, wi);
    }

    return radiance;
}
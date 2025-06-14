#pragma once
#include "Ray.h"
#include "Object.h"
#include "Rectangle.h"
#include "Vec3.h"
#include <memory>
#include <vector>


Vec3 whitted_render(const Ray& r, const std::vector<std::shared_ptr<Object>>& scene) {
    const float epsilon = 1e-3f;
    Vec3 radiance(0.0f, 0.0f, 0.0f);

    float closest_t = 1e30f;
    std::shared_ptr<Object> hit_object = nullptr;

    for(const auto& obj : scene) {
        float t;
        if(obj->hit(r, epsilon, closest_t, t)) {
            closest_t = t;
            hit_object = obj;
        }
    }

    if(!hit_object) return Vec3(0.0f, 0.0f, 0.0f);

    if(hit_object->is_light()) return hit_object->get_material()->get_emission();

    Vec3 x = r.at(closest_t);
    Vec3 N = hit_object->get_normal(x);
    Vec3 wo = -r.direction.normalize();

    for(const auto& light : scene) {
        if(!light->is_light()) continue;
        auto rect = std::dynamic_pointer_cast<Rectangle>(light);
        if(!rect) continue;

        Vec3 light_pos = sample_light_rectangle(rect->get_center(), rect->get_u(), rect->get_v());
        Vec3 shadow_origin = x + N * epsilon;
        Vec3 wi_vec       = light_pos - shadow_origin;
        float dist2       = wi_vec.length_squared();
        float dist        = std::sqrt(dist2);
        Vec3 wi           = wi_vec / dist;

        Ray shadow_ray(shadow_origin, wi);
        bool in_shadow = false;
        for (auto& obj : scene) {
            if (obj->is_light()) continue;
            float t;
            if (obj->hit(shadow_ray, epsilon, dist - epsilon, t)) {
                in_shadow = true;
                break;
            }
        }
        if(in_shadow) continue;

        Vec3 brdf = hit_object->get_material()->evaluate(N, wi, wo);
        Vec3 Le = light->get_material()->get_emission();
        float cos_theta = std::max(0.0f, N.dot(wi));
        float area = rect->get_area();

        radiance += brdf * Le * cos_theta * area / dist2;
    }

    return radiance;
}
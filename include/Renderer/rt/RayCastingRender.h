#pragma once
#include "Ray.h"
#include "Object.h"
#include "Rectangle.h"
#include "Vec3.h"
#include <memory>
#include <vector>


Vec3 casting_render(const Ray& r, const std::vector<std::shared_ptr<Object>>& scene) {
    const float epsilon = 1e-4f;
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

    Vec3 brdf = hit_object->get_material()->evaluate(N, wo, wo);
    radiance = brdf;

    return radiance;
}
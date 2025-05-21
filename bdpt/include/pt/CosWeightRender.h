#pragma once
#include "Ray.h"
#include "Object.h"
#include "Rectangle.h"
#include "Vec3.h"

// CosineWight
Vec3 cos_weight_render(const Ray& r, const std::vector<std::shared_ptr<Object>>& scene, int depth) {
    if (depth <= 0) return Vec3(0, 0, 0);

    float closest_t = 10000.0f;
    std::shared_ptr<Object> hit_object = nullptr;

    for (auto& obj : scene) {
        float t;
        if (obj->hit(r, 0.001f, closest_t, t)) {
            closest_t = t;
            hit_object = obj;
        }
    }

    if (hit_object) {

        if (hit_object->is_light()) {
            return hit_object->get_material()->get_emission();
        }

        Vec3 hit_point = r.at(closest_t);
        Vec3 normal = hit_object->get_normal(hit_point);
        int light_samples = 1;
        Vec3 accumulated(0.0f, 0.0f, 0.0f);

        for(int k = 0; k < light_samples; k++) {
            Vec3 target_direction = normal + cos_weight_sampling();
            Ray scatterd(hit_point, target_direction.normalize());
            // Vec3 incoming_color = ray_color(scatterd, scene, depth - 1);
            accumulated += cos_weight_render(scatterd, scene, depth - 1);
        }
        accumulated /= float(light_samples);

        return hit_object->get_material()->get_albedo() * accumulated;

    } else {
        // Vec3 unit_direction = r.direction.normalize();
        // float t = 0.5f * (unit_direction.y + 1.0f);
        // return (1.0f - t) * Vec3(1.0f, 1.0f, 1.0f) + t * Vec3(0.5f, 0.7f, 1.0f);
        return Vec3(0.1f, 0.1f, 0.1f);
    }
}
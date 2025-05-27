#pragma once
#include "Ray.h"
#include "Object.h"
#include "Rectangle.h"
#include "Vec3.h"


Vec3 nee_render2(const Ray& r, const std::vector<std::shared_ptr<Object>>& scene, int depth) {
    if (depth <= 0) return Vec3(0.0f, 0.0f, 0.0f);

    float closest_t = 10000.0f;
    std::shared_ptr<Object> hit_object = nullptr;

    for (auto& obj : scene) {
        float t;
        if (obj->hit(r, 0.001f, closest_t, t)) {
            closest_t = t;
            hit_object = obj;
        }
    }

    if(!hit_object) return Vec3(0.0f, 0.0f, 0.0f);


    if (hit_object->is_light()) {
        return hit_object->get_material()->get_emission();
        // return Vec3(0.0f, 0.0f, 0.0f);
    }

    Vec3 hit_point = r.at(closest_t);
    Vec3 normal = hit_object->get_normal(hit_point);
    Vec3 wo = -r.direction.normalize();

    Vec3 direct_light = Vec3(0.0f, 0.0f, 0.0f);


    for(const auto& light : scene) {
        if(!light->is_light()) continue;
        if(auto rect = std::dynamic_pointer_cast<Rectangle>(light)) {

            // Vec3 light_point = sample_light_sphere(light->get_center(), light->get_radius());
            Vec3 light_point = sample_light_rectangle(rect->get_center(), rect->get_u(), rect->get_v());
            Vec3 wi = (light_point - hit_point).normalize();
            float dist = (light_point - hit_point).length();
            
            Ray shadow_ray(hit_point + normal * 1e-4f, wi);
            bool in_shadow = false;

            for (auto& obj : scene) {
                float t;
                if (obj->hit(shadow_ray, 0.001f, dist - 0.001f, t)) {
                    if(!obj->is_light()) {
                        in_shadow = true;
                        break;
                    }
                }
            }

            if(!in_shadow) {
                Vec3 brdf = hit_object->get_material()->evaluate(normal, wi, wo);
                Vec3 Le = light->get_material()->get_emission();
                float cos_theta = std::max(0.0f, normal.dot(wi));
                float cos_alpha = std::max(0.0f, (-wi).dot(rect->get_normal(light_point)));
                float dist2 = dist * dist;
                float area = rect->get_area();

                direct_light += brdf * Le * cos_alpha * area / dist2;
            }
        }
    }

    Vec3 indirect_light(0.0f, 0.0f, 0.0f);
    Vec3 sample_dir = (normal + cos_weight_sampling()).normalize();
    Ray next_r(hit_point + normal * 1e-3, sample_dir);
    Vec3 Li = nee_render(next_r, scene, depth - 1);

    indirect_light = hit_object->get_material()->get_albedo() * Li;

    return direct_light + indirect_light;


}
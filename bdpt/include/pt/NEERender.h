#pragma once
#include "Ray.h"
#include "Object.h"
#include "Rectangle.h"
#include "Vec3.h"


//NEE
Vec3 nee_render(const Ray& r, const std::vector<std::shared_ptr<Object>>& scene, int depth) {
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

        Vec3 direct_light = Vec3(0, 0, 0);
        int light_samples = 10;

        for(const auto& light : scene) {
            if(!light->is_light()) continue;
            if(auto rect = std::dynamic_pointer_cast<Rectangle>(light)) {

                for(int k = 0; k < light_samples; k++) {
                    // Vec3 light_point = sample_light_sphere(light->get_center(), light->get_radius());
                    Vec3 light_point = sample_light_rectangle(rect->get_center(), rect->get_u(), rect->get_v());
                    Vec3 light_dir = (light_point - hit_point).normalize();
                    float OtoL_dist = (light_point - hit_point).length();
                    
                    Ray next_r(hit_point, light_dir);

                    bool in_shadow = false;
                    for (auto& obj : scene) {
                        float t;
                        if (obj->hit(next_r, 0.001f, OtoL_dist - 0.001f, t)) {
                            if(!obj->is_light()) {
                                in_shadow = true;
                                break;
                            }
                        }
                    }

                    if(!in_shadow) {
                        Vec3 brdf_val = hit_object->get_material()->evaluate(normal, light_dir, -r.direction.normalize());
                        float cos_alpha = std::max(0.0f, (-light_dir).dot(rect->get_normal(light_point)));
                        float dist2 = OtoL_dist * OtoL_dist;
                        float area = rect->get_area();

                        direct_light += brdf_val *light->get_material()->get_emission() * cos_alpha * area / dist2;
                    }

                }
            }
        }

        direct_light /= float(light_samples);

        return direct_light;

    } else {
        return Vec3(0.1f, 0.1f, 0.1f);
    }
}
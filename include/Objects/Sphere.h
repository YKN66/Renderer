#pragma once
#include "Object.h"

class Sphere : public Object {
public:
    Vec3 center;
    float radius;
    std::shared_ptr<BRDF> material;
    bool light_flag;

    Sphere(const Vec3& c, float r, std::shared_ptr<BRDF> m, bool is_light = false)
        : center(c), radius(r), material(m), light_flag(is_light) {}

    bool hit(const Ray& r, float t_min, float t_max, float& t_out) const override {
        Vec3 oc = r.origin - center;
        float a = r.direction.dot(r.direction);
        float b = oc.dot(r.direction);
        float c = oc.dot(oc) - radius * radius;
        float discriminant = b * b - a * c;

        if (discriminant > 0) {
            float root = sqrt(discriminant);
            float temp = (-b - root) / a;
            if (temp < t_max && temp > t_min) {
                t_out = temp;
                return true;
            }
            temp = (-b + root) / a;
            if (temp < t_max && temp > t_min) {
                t_out = temp;
                return true;
            }
        }
        return false;
    }

    Vec3 get_normal(const Vec3& hit_point) const override {
        return (hit_point - center).normalize();
    }

    Vec3 get_center() const override {
        return center;
    }

    float get_radius() const override {
        return radius;
    }

    std::shared_ptr<BRDF> get_material() const override {
        return material;
    }

    bool is_light() const override {
        return light_flag;
    }
};

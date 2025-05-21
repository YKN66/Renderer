#pragma once
#include "Object.h"

class Rectangle : public Object {
public:
    Vec3 center;        // 矩形の中心位置
    Vec3 u, v;          // u軸, v軸ベクトル（矩形の辺方向）
    Vec3 normal;        // 法線ベクトル（u×v）
    float width, height;
    std::shared_ptr<BRDF> material;
    bool light_flag;

    Rectangle(
        const Vec3& center,
        const Vec3& u_vec, const Vec3& v_vec,
        std::shared_ptr<BRDF> material,
        bool is_light = false
    )
        : center(center), u(u_vec), v(v_vec),
          width(u_vec.length() * 2.0f), height(v_vec.length() * 2.0f),
          material(material), light_flag(is_light)
    {
        normal = u.cross(v).normalize();
    }

    bool hit(const Ray& ray, float t_min, float t_max, float& t_out) const override {
        float denom = normal.dot(ray.direction);
        if (std::fabs(denom) < 1e-6f) return false; // 平行なので交差しない

        float t = (center - ray.origin).dot(normal) / denom;
        if (t < t_min || t > t_max) return false;

        Vec3 p = ray.at(t);
        Vec3 d = p - center;

        float du = d.dot(u.normalize());
        float dv = d.dot(v.normalize());

        if (std::fabs(du) <= width / 2 && std::fabs(dv) <= height / 2) {
            t_out = t;
            return true;
        }

        return false;
    }

    Vec3 get_normal(const Vec3& hit_point) const override {
        return normal;
    }

    std::shared_ptr<BRDF> get_material() const override {
        return material;
    }

    bool is_light() const override {
        return light_flag;
    }

    Vec3 get_center() const override {
        return center;
    }

    Vec3 get_u() const override {
        return u;
    }

    Vec3 get_v() const override {
        return v;
    }

    float get_area() const override {
        return u.length() * 2.0f * v.length() * 2.0f;
    }
};
#pragma once
#include "Ray.h"
#include "BRDF.h"
#include "Vec3.h"

class Object {
public:
    virtual ~Object() = default;

    // Rayとの交差判定
    virtual bool hit(const Ray& r, float t_min, float t_max, float& t_out) const = 0;

    // 表面上の法線ベクトル
    virtual Vec3 get_normal(const Vec3& hit_point) const = 0;

    virtual Vec3 get_center() const {return Vec3();}

    virtual float get_radius() const {return 0.0f;}

    virtual Vec3 get_u() const {return Vec3();}

    virtual Vec3 get_v() const {return Vec3();}

    virtual float get_area() const {return 1.0f;}

    // マテリアルへのアクセス
    virtual std::shared_ptr<BRDF> get_material() const = 0;

    // 発光体かどうか
    virtual bool is_light() const { return false; }
};

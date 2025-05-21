#pragma once
#include "Vec3.h"

class BRDF {
public:
    virtual ~BRDF() = default;

    // 光の反射強度を返す
    // N: 法線ベクトル
    // L: ライト方向ベクトル（物体→光源）
    // V: 視線ベクトル（物体→カメラ）
    virtual Vec3 evaluate(const Vec3& N, const Vec3& L, const Vec3& V) const = 0;

    virtual float max_value() const {return 1.0f;}

    virtual Vec3 get_albedo() const { return Vec3(1.0f, 1.0f, 1.0f); }

    virtual Vec3 get_emission() const { return Vec3(0.0f, 0.0f, 0.0f); }

    virtual Vec3 sample(const Vec3& N, const Vec3& wo, float& pdf) const { return Vec3(1.0f, 1.0f, 1.0f); }

    virtual float pdf(const Vec3& N, const Vec3& wi) const {return 1.0f;}
};
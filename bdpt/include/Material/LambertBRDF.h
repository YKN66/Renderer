#pragma once
#include "BRDF.h"
#include <cmath>

class LambertBRDF : public BRDF{
public:
    Vec3 cd; // 物体の拡散反射率

    LambertBRDF(const Vec3& color) : cd(color) {}

    Vec3 evaluate(const Vec3& N, const Vec3& L, const Vec3& V) const override {
        float dotNL = std::max(N.dot(L), 0.0f);

        return (cd / M_PI) * dotNL;
    }

    Vec3 get_albedo() const override {
        return cd;
    }

    Vec3 sample(const Vec3& N, const Vec3& wo, float& pdf) const override {
        Vec3 dri_local = cos_weight_sampling();
        Vec3 T = tangent_vector(N);
        Vec3 B = N.cross(T);
        Vec3 wi = (T * dri_local.x + B * dri_local.y + N * dri_local.z).normalize();

        pdf = std::max(0.0f, N.dot(wi)) / M_PI;

        return wi;
    }

    float pdf(const Vec3& N, const Vec3& wi) const override {
        return std::max(0.0f, N.dot(wi)) / M_PI;
    }
};
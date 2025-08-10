#pragma once
#include "BRDF.h"
#include <cmath>

class Emission : public BRDF{
public:
    Vec3 e;

    Emission(const Vec3& color)
        : e(color) {}

    Vec3 evaluate(const Vec3& N, const Vec3& L, const Vec3& V) const override {
        return Vec3(0, 0, 0);
    }

    Vec3 get_albedo() const override {
        return Vec3(0, 0, 0);
    }

    Vec3 get_emission() const override {
        return e;
    }

    Vec3 sample(const Vec3& N, const Vec3& wo, float& pdf) const override {
        return Vec3(0.0f, 0.0f, 0.0f);
    }

    float pdf(const Vec3& N, const Vec3& wi) const override {
        // return 0.0f;
        return std::max(0.0f, N.dot(wi)) / M_PI;
    }
};
#pragma once
#include "BRDF.h"
#include <cmath>

class Sensor : public BRDF{
public:

    Vec3 evaluate(const Vec3& N, const Vec3& L, const Vec3& V) const override {
        constexpr float EPS_DIR = 0.999f;
        return (N.dot(L) > 0.0f && L.dot(V) > EPS_DIR)
                 ? Vec3(1.0f, 1.0f, 1.0f) : Vec3(0.0f, 0.0f, 0.0f);

        // return (N.dot(L) > 0.0f) ? Vec3(1.0f, 1.0f, 1.0f) : Vec3(0.0f, 0.0f, 0.0f);
    }

    Vec3 get_albedo() const override {
        return Vec3(0.0f, 0.0f, 0.0f);
    }

    Vec3 get_emission() const override {
        return Vec3(0.0f, 0.0f, 0.0f);;
    }

    Vec3 sample(const Vec3& N, const Vec3& wo, float& pdf) const override {
        pdf = 0.0f;
        return Vec3(0.0f, 0.0f, 0.0f);
    }

    float pdf(const Vec3& N, const Vec3& wi) const override {
        // return 0.0f;
        return std::max(0.0f, N.dot(wi)) / M_PI;
    }
};
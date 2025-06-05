#pragma once
#include "BRDF.h"
#include <cmath>

class CookTorranceBRDF : public BRDF{
public:
    Vec3 cd;
    Vec3 F0; // 物体の鏡面反射率
    float alpha; //粗さ(0~1)

    CookTorranceBRDF(const Vec3& cd, const Vec3& cs, float roughness)
        : cd(cd), F0(cs), alpha(roughness) {}

    Vec3 evaluate(const Vec3& N, const Vec3& L, const Vec3& V) const override {
        Vec3 H = half_vector(L, V);

        float dotNL = std::max(N.dot(L), 0.0f);
        float dotNV = std::max(N.dot(V), 0.0f);
        float dotNH = std::max(N.dot(H), 0.0f);
        float dotVH = std::max(V.dot(H), 0.0f);


        float alpha2 = alpha * alpha;
        // float D = alpha / pow((M_PI * (pow(dotNH, 2) * (alpha - 1) + 1)), 2);
        float term = (dotNH * dotNH * (alpha2 - 1.0f) + 1.0f);
        float D = alpha2 / (M_PI * term * term);

        Vec3 F = F0 + (1 - F0) * pow((1 - dotVH), 5);

        float G1_L = (2 * dotNL) / (dotNL + std::sqrt(alpha + (1 - alpha2) * pow(dotNL, 2)));
        float G1_V = (2 * dotNV) / (dotNV + std::sqrt(alpha + (1 - alpha2) * pow(dotNV, 2)));
        float G = G1_L * G1_V;

        Vec3 numerator = F * D * G;
        float denominator = 4.0f * dotNL * dotNV + 0.001f;

        Vec3 kd = Vec3(1.0f, 1.0f, 1.0f) - F0;
        Vec3 diffuse = (cd / M_PI) * dotNL;
        Vec3 specular = numerator / denominator;

        return diffuse + specular;
    }

    Vec3 get_albedo() const override {
        return cd;
    }
};
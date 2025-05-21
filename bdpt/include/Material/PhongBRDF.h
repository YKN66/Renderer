#pragma once
#include "BRDF.h"
#include <cmath>

class PhongBRDF : public BRDF{
public:
    Vec3 cd; // 物体の拡散反射率
    Vec3 cs; // 鏡面のハイライトの色
    float shininess; // 鏡面の鋭さ

    PhongBRDF(const Vec3& diffuse, const Vec3& specular, float s)
        : cd(diffuse), cs(specular), shininess(s) {}

    Vec3 evaluate(const Vec3& N, const Vec3& L, const Vec3& V) const override {
        float dotNL = std::max(N.dot(L), 0.0f);

        Vec3 R = reflect(-L, N);

        float spec_angle = std::max(R.dot(V), 0.0f);
        float spec_term = pow(spec_angle, shininess);

        Vec3 diffuse = (cd / M_PI) * dotNL;
        Vec3 specular = (cs * (shininess + 2) * spec_term) / (2 * M_PI);

        return diffuse + specular;
    }

    Vec3 get_albedo() const override {
        return cd;
    }
};
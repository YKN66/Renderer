#pragma once
#include "Vec3.h"
#include "BRDF.h"

struct PathVertex {
    Vec3 x;
    Vec3 N;
    Vec3 wi; //入射方向
    std::shared_ptr<BRDF> brdf;
    float pdf_fwd = 1.0f;
    float pdf_rev = 1.0f;
    bool is_light = false;
};
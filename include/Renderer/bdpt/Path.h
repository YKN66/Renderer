#pragma once
#include "Vec3.h"
#include "BRDF.h"

struct PathVertex {
    Vec3 x;
    Vec3 N;
    Vec3 wi; //光源からきた方向
    Vec3 wo; // カメラへ向かう方向
    std::shared_ptr<BRDF> brdf;
    float pdf_A = 1.0f;
    float pdf_W = 1.0f;
    float pdf_rev = 1.0f;
    Vec3 beta = Vec3(1.0f, 1.0f, 1.0f);
    float vc = 0.0f; // 1次 MIS weight 補助係数 (non-delta)
    float vcm = 0.0f; // 0次 MIS weight 補助係数
    bool   isDelta;   // サンプリングに使った BSDF が Dirac か
    bool is_light = false;
};
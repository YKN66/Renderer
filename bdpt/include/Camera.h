#ifndef CAMERA_H
#define CAMERA_H

#include "Vec3.h"
#include "Ray.h"

class Camera {
public:
    Vec3 pos;
    Vec3 upper_right_corner, horizontal, vertical;
    Vec3 u, v, w;

    // lookFrom … カメラ位置
    // lookAt … 視線が向かう 1 点
    // vup … 画面の「上」方向（lookDir と直交させる）

    Camera(
        Vec3 lookFrom,
        Vec3 lookAt,
        Vec3 vup,
        float fov_dg,
        float aspect
    );

    Ray get_ray(float s, float t) const;

};

#endif
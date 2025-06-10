#ifndef CAMERA_H
#define CAMERA_H

#include "Vec3.h"
#include "Ray.h"

class Camera {
public:
    Vec3 pos;
    Vec3 upper_right_corner, horizontal, vertical;
    Vec3 u, v, w;
    int width, height;

    float viewport_width = 1.0f;
    float viewport_height = 1.0f;

    // lookFrom … カメラ位置
    // lookAt … 視線が向かう 1 点
    // vup … 画面の「上」方向（lookDir と直交させる）

    Camera(Vec3 lookFrom, Vec3 lookAt, Vec3 vup, float fov_dg, float aspect, int width, int height);

    Ray get_ray(float s, float t) const;

    bool project_dir(const Vec3& dir_world, float& s, float& t) const;

    int image_width() const {return width;}
    int image_height() const {return height;}

};

#endif
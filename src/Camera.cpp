#include "Camera.h"
#include "Vec3.h"
#include "Ray.h"
#include <cmath>

Camera::Camera(Vec3 lookFrom, Vec3 lookAt, Vec3 vup, float fov_dg, float aspect, int width, int height) 
       : width(width), height(height){
    float theta = fov_dg * M_PI / 180.0f;
    float h = std::tan(theta / 2);
    viewport_height = 2.0f * h;
    viewport_width = aspect * viewport_height;

    w = (lookFrom - lookAt).normalize();
    u = vup.cross(w).normalize();
    v = w.cross(u);

    pos = lookFrom;
    horizontal = u * viewport_width;
    vertical = v * viewport_height;
    upper_right_corner = pos - horizontal * 0.5f - vertical * 0.5f - w;
}

Ray Camera::get_ray(float s, float t) const {
    Vec3 dir = upper_right_corner + horizontal * s + vertical * t - pos;
    return Ray(pos, dir.normalize());
}

bool Camera::project_dir(const Vec3& dir_world, float& s, float& t) const {
    float cos_theta = -dir_world.dot(w);
    if(cos_theta <= 0.0f) return false;

    Vec3 p = dir_world / cos_theta;
    s = 0.5f + p.dot(u) / viewport_width;
    t = 0.5f + p.dot(v) / viewport_height;

    return (s >= 0.0f && s < 1.0f && t >= 0.0f && t < 1.0f);
}


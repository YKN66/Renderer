#include "Camera.h"
#include "Vec3.h"
#include "Ray.h"
#include <cmath>

Camera::Camera(
    Vec3 lookFrom,
    Vec3 lookAt,
    Vec3 vup,
    float fov_dg,
    float aspect
) {
    float theta = fov_dg * M_PI / 180.0f;
    float h = std::tan(theta / 2);
    float viewport_height = 2.0f * h;
    float viewport_width = aspect * viewport_height;

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


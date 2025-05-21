#pragma once
#include "Vec3.h"
#include "BRDF.h"
#include "Path.h"

Vec3 connect_verices(const PathVertex& camera_vertex, const PathVertex& light_vertex, const std::vector<std::shared_ptr<Object>>& scene) {
    Vec3 dir = light_vertex.pos - camera_vertex.pos;
    float dist2 = dir.length_squared();
    Vec3 wi = dir.normalize();

    Ray Shadow_ray(camera_vertex.pos, wi);

    for(const auto& obj : scene) {
        float t;
        if(obj->hit(Shadow_ray, 0.001f, sqrt(dist2) - 0.001f, t)) {
            return Vec3(0.0f, 0.0f, 0.0f);
        }
    }

    //BRDF Geomotry
    float cos_theta_cam = std::max(0.0f, camera_vertex.normal.dot(wi));
    float cos_theta_lig = std::max(0.0f, light_vertex.normal.dot(-wi));

    Vec3 fs = camera_vertex.brdf->evaluate(camera_vertex.normal, wi, camera_vertex.wi);
    // Vec3 fl = Vec3(1.0f, 1.0f, 1.0f);

    Vec3 Le = light_vertex.brdf->get_emission();
    Vec3 contribute = fs * Le * (cos_theta_cam * cos_theta_lig / dist2);

    return contribute;

}

float mis_weight(const PathVertex& camera, const PathVertex& light) {
    float pdf_a = camera.pdf_fwd;
    float pdf_b = light.pdf_fwd;

    float w_a = pdf_a * pdf_a;
    float w_b = pdf_b * pdf_b;

    return w_a / (w_a + w_b + 1e-6f);
}
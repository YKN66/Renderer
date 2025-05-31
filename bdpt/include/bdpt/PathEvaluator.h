#pragma once
#include "Vec3.h"
#include "BRDF.h"
#include "Path.h"
#include <cmath>

Vec3 connect_verices(const PathVertex& vertex_c, const PathVertex& vertex_l, const std::vector<std::shared_ptr<Object>>& scene) {
    Vec3 dir = vertex_l.x - vertex_c.x;
    float dist2 = dir.length_squared();
    Vec3 wi = dir.normalize();

    Ray Shadow_ray(vertex_c.x + vertex_c.N * 1e-6f, wi);
    float t_max = std::sqrt(dist2) - 1e-6f;
    for(const auto& obj : scene) {
        float t;
        if(obj->hit(Shadow_ray, 1e-6f, t_max, t)) {
            // std::cout << "shadow \n";
            return Vec3(0.0f, 0.0f, 0.0f);
        }
    }

    //BRDF Geomotry
    float cos_c = std::max(0.0f, vertex_c.N.dot(wi));
    float cos_l = std::max(0.0f, vertex_l.N.dot(-wi));

    // Vec3 fs = vertex_c.brdf->evaluate(vertex_c.N, wi, vertex_c.wi);
    // // Vec3 fl = Vec3(1.0f, 1.0f, 1.0f);

    // Vec3 Le = vertex_l.brdf->get_emission();
    // Vec3 contribute = fs * Le * (cos_c * cos_l / dist2);

    Vec3 fs_cam = vertex_c.brdf->evaluate(vertex_c.N, wi, vertex_c.wi);
    Vec3 fs_lig = vertex_l.is_light ? Vec3(1.0f, 1.0f, 1.0f) : vertex_l.brdf->evaluate(vertex_l.N, -wi, vertex_l.wi);
    // Vec3 fs_cam(1.0f, 1.0f, 1.0f);
    // Vec3 fs_lig(1.0f, 1.0f, 1.0f);
    float G = cos_c * cos_l / dist2;
    Vec3 contribute = fs_cam * fs_lig * Vec3(G, G, G);
    // Vec3 contribute = fs_cam * fs_lig;

    // std::cout << "countribute = ("<< contribute.x <<", "<< contribute.y <<", "<< contribute.z <<")\n";

    return contribute;

}

float mis_weight(const PathVertex& camera, const PathVertex& light) {
    float pdf_a = camera.pdf_fwd;
    float pdf_b = light.pdf_fwd;

    float w_a = pdf_a * pdf_a;
    float w_b = pdf_b * pdf_b;

    return w_a / (w_a + w_b + 1e-6f);
}

float simple_mis(int s, int t, float pdf_cam, float pdf_lig) {
    // if(pdf_cam == 0.0f || pdf_lig == 0.0f) return 0.0f;
    // return 1.0f / float(s + t + 1);
    return 1.0f;
}
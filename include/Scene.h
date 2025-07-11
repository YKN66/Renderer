#pragma once

#include "Object.h"
#include "Camera.h"
// #include "Light.h"
#include "Sphere.h"
#include "Rectangle.h"
#include "LambertBRDF.h"
#include "PhongBRDF.h"
#include "CookTorranceBRDF.h"
#include "Emission.h"
#include "Sensor.h"
#include <memory>
#include <vector>

Camera controll_scene(const int w, const int h, std::vector<std::shared_ptr<Object>>& scene) {
    float aspect = float(w) / (h);
    Camera camera(Vec3(0, 0.2, -3), Vec3(0, -0.2, 1), Vec3(0, 1, 0), 45.0f, aspect, w, h);
    // Camera camera(Vec3(0, 0, -2), Vec3(0, 1, 0), Vec3(0, 0, 1), 45.0f, aspect, w, h);
    // Camera camera(Vec3(0, 2, 0),    // y = +2 → 床 (y = -1) の 3 ユニット上空
    //           Vec3(0,-1, 0),    // 床の中心を注視
    //           Vec3(0, 0, 1),    // 画面上方向を +z に
    //           40.0f, aspect);
              


    auto red = std::make_shared<LambertBRDF>(Vec3(0.9f, 0.2f, 0.2f));
    auto blue = std::make_shared<LambertBRDF>(Vec3(0.2f, 0.2f, 0.9f));
    auto green = std::make_shared<LambertBRDF>(Vec3(0.2f, 0.9f, 0.2f));
    auto white = std::make_shared<LambertBRDF>(Vec3(0.8f, 0.8f, 0.8f));
    auto yellow = std::make_shared<LambertBRDF>(Vec3(0.9f, 0.9f, 0.3f));
    auto light_obj = std::make_shared<Emission>(Vec3(3.0f, 3.0f, 3.0f));
    // auto light_obj = std::make_shared<Emission>(Vec3(3.0f, 0.0f, 0.0f));


    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 1 - 1e-2f, 0), Vec3(0.5, 0, 0), Vec3(0, 0, 0.5), light_obj, true));
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 0, 1), Vec3(0, 1, 0), Vec3(1.25, 0, 0), white)); // 前
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 0, -3), Vec3(1.25, 0, 0), Vec3(0, 1, 0), white)); // 後
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 1, -1), Vec3(1.25, 0, 0), Vec3(0, 0, 2), white)); // 上
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, -1, -1), Vec3(0, 0, 2), Vec3(1.25, 0, 0), white)); // 下
    scene.push_back(std::make_shared<Rectangle>(Vec3(-1.25, 0, -1), Vec3(0, 1, 0), Vec3(0, 0, 2), red)); // 右
    scene.push_back(std::make_shared<Rectangle>(Vec3(1.25, 0, -1), Vec3(0, 0, 2), Vec3(0, 1, 0), blue)); // 左


    scene.push_back(std::make_shared<Sphere>(Vec3(-0.5, -0.80, 0), 0.2, yellow));


    auto cube_mat = green;
    Vec3 cube_center(0.3f, -0.80f, 0.30f);
    float half = 0.20f;
    float theta = 45.0f * M_PI / 180.0f;
    float c = std::cos(theta);
    float s = std::sin(theta);
    Vec3 ex(c, 0, -s);
    Vec3 ez(s, 0,  c);
    Vec3 ey(0, 1,  0);

    scene.push_back(std::make_shared<Rectangle>(cube_center + ez * half, ex * half,  ey * half, cube_mat));
    scene.push_back(std::make_shared<Rectangle>(cube_center - ez * half, -ex * half, ey * half, cube_mat));
    scene.push_back(std::make_shared<Rectangle>(cube_center + ex * half, -ez * half, ey * half, cube_mat));
    scene.push_back(std::make_shared<Rectangle>(cube_center - ex * half, ez * half, ey * half, cube_mat));
    scene.push_back(std::make_shared<Rectangle>( cube_center + ey * half, ex * half, -ez * half, cube_mat));
    scene.push_back(std::make_shared<Rectangle>(cube_center - ey * half, ex * half,  ez * half, cube_mat));


    return camera;
}
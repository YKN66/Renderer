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
#include "CameraBRDF.h"

Camera controll_scene(const int w, const int h, std::vector<std::shared_ptr<Object>>& scene) {
    float aspect = float(w) / (h);
    // Camera camera(Vec3(0, 0, 4), Vec3(0, 0, 0), Vec3(0, 1, 0), 40.0f, aspect);
    Camera camera(Vec3(0, 0, -1), Vec3(0, 0, 1), Vec3(0, 1, 0), 90.0f, aspect);
    // Camera camera(Vec3(0, 0, -1), Vec3(0, 1, 0), Vec3(0, 0, 1), 90.0f, aspect);
    // Camera camera(Vec3(0, 2, 0),    // y = +2 → 床 (y = -1) の 3 ユニット上空
    //           Vec3(0,-1, 0),    // 床の中心を注視
    //           Vec3(0, 0, 1),    // 画面上方向を +z に
    //           40.0f, aspect);
              

    // // Light light1(Vec3(0.5, 0.5, 1), 0.05f, Vec3(1.0f, 1.0f, 1.0f), 0.5f);
    // // Light light2(Vec3(-1, -1, 0), 0.05f, Vec3(1.0f, 0.0f, 0.0f), 16.0f);

    // auto lambert = std::make_shared<LambertBRDF>(Vec3(0.9f, 0.7f, 0.1f));
    // auto phong = std::make_shared<PhongBRDF>(Vec3(0.9f, 0.7f, 0.1f), Vec3(1.0f, 1.0f, 1.0f), 16.0f);
    // auto cook = std::make_shared<CookTorranceBRDF>(Vec3(0.9f, 0.7f, 0.1f), Vec3(0.4f, 0.4f, 0.4f), 0.5f);
    // // auto light_obj = std::make_shared<Emission>(light1.color);
    // auto light_obj = std::make_shared<Emission>(Vec3(1.0f, 1.0f, 1.0f));

    // scene.push_back(std::make_shared<Sphere>(Vec3(-1.2, 0, 0), 0.5, cook));
    // scene.push_back(std::make_shared<Sphere>(Vec3(0, 0, 0), 0.5, cook));
    // scene.push_back(std::make_shared<Sphere>(Vec3(1.2, 0, 0), 0.5, cook));
    // // scene.push_back(std::make_shared<Sphere>(light1.pos, light1.r, light_obj, true));
    // scene.push_back(std::make_shared<Sphere>(Vec3(0.5, 0.5, 1), 0.05f, light_obj, true));

    // scene.push_back(std::make_shared<Rectangle>(Vec3(0, -1, 0), Vec3(0, 0, 2), Vec3(2, 0, 0), lambert));
    // scene.push_back(std::make_shared<Rectangle>(Vec3(0, 1, 0), Vec3(2, 0, 0), Vec3(0, 0, 2), lambert));

    // scene.push_back(std::make_shared<Rectangle>(Vec3(0, 0, -2), Vec3(2, 0, 0), Vec3(0, 1, 0), lambert));

    // scene.push_back(std::make_shared<Rectangle>(Vec3(-2, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 2), lambert));
    // scene.push_back(std::make_shared<Rectangle>(Vec3(2, 0, 0), Vec3(0, 0, 2), Vec3(0, 1, 0), lambert));


    // auto lambert = std::make_shared<LambertBRDF>(Vec3(0.7f, 0.4f, 0.1f));
    auto red = std::make_shared<LambertBRDF>(Vec3(0.9f, 0.1f, 0.1f));
    auto blue = std::make_shared<LambertBRDF>(Vec3(0.1f, 0.9f, 0.1f));
    auto green = std::make_shared<LambertBRDF>(Vec3(0.1f, 0.1f, 0.9f));
    auto white = std::make_shared<LambertBRDF>(Vec3(1.0f, 1.0f, 1.0f));
    auto light_obj = std::make_shared<Emission>(Vec3(3.0f, 3.0f, 3.0f));
    // auto light_obj = std::make_shared<Emission>(Vec3(3.0f, 0.0f, 0.0f));


    scene.push_back(std::make_shared<Rectangle>(Vec3(0, -1, 0), Vec3(0, 0, 1), Vec3(1, 0, 0), red));
    // scene.push_back(std::make_shared<Rectangle>(Vec3(0, 1, 0), Vec3(1, 0, 0), Vec3(0, 0, 1), light_obj, true));
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 1, 0), Vec3(0.5, 0, 0), Vec3(0, 0, 0.5), light_obj, true));

    scene.push_back(std::make_shared<Rectangle>(Vec3(0.75, 1, 0), Vec3(0.25, 0, 0), Vec3(0, 0, 1), white));
    scene.push_back(std::make_shared<Rectangle>(Vec3(-0.75, 1, 0), Vec3(0.25, 0, 0), Vec3(0, 0, 1), white));
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 1, 0.75), Vec3(0.5, 0, 0), Vec3(0, 0, 0.25), white));
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 1, -0.75), Vec3(0.5, 0, 0), Vec3(0, 0, 0.25), white));

    scene.push_back(std::make_shared<Rectangle>(Vec3(-1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1), blue));
    scene.push_back(std::make_shared<Rectangle>(Vec3(1, 0, 0), Vec3(0, 0, 1), Vec3(0, 1, 0), green));
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 0, -1), Vec3(1, 0, 0), Vec3(0, 1, 0), white));
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 0, 1), Vec3(0, 1, 0), Vec3(1, 0, 0), white));


    return camera;
}
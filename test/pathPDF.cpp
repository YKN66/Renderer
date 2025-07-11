#include "PFM.h"
#include <iostream>
#include <random>
#include "Vec3.h"
#include "Ray.h"
#include "Camera.h"
#include "Scene.h"
#include "BDPTRender.h"


int main() {
    std::vector<PathVertex> cp;
    std::vector<PathVertex> cl;
    PathVertex p;

    // 3, 3
    p.x = Vec3(0.0f,0.0f,0.0f); p.N = Vec3(0.0f,1.0f,0.0f); p.pdf_A = 1.0f; p.pdf_W = 1.0f; p.pdf_rev = 1.0f; p.brdf = std::make_shared<Sensor>(); cp.push_back(p);
    p.x = Vec3(1.0f,2.0f,0.0f); p.N = Vec3(0.0f,-1.0f,0.0f); p.pdf_A = 1.0f; p.pdf_W = 1.0f; p.pdf_rev = 1.0f; p.brdf = std::make_shared<LambertBRDF>(Vec3(0.8f,0.0f,0.0f));cp.push_back(p);
    p.x = Vec3(2.0f,1.0f,0.0f); p.N = Vec3(0.0f,1.0f,0.0f); p.pdf_A = 1.0f; p.pdf_W = 1.0f; p.pdf_rev = 1.0f; p.brdf = std::make_shared<LambertBRDF>(Vec3(0.8f,0.0f,0.0f));cp.push_back(p);

    p.x = Vec3(5.0f,5.0f,0.0f); p.N = Vec3(0.0f,-1.0f,0.0f); p.pdf_A = 1.0f; p.pdf_W = 1.0f; p.pdf_rev = 1.0f; p.brdf = std::make_shared<Emission>(Vec3(1.f,1.f,1.f));cl.push_back(p);
    p.x = Vec3(4.0f,2.0f,0.0f); p.N = Vec3(0.0f,1.0f,0.0f); p.pdf_A = 1.0f; p.pdf_W = 1.0f; p.pdf_rev = 1.0f; p.brdf = std::make_shared<LambertBRDF>(Vec3(0.8f,0.0f,0.0f));cl.push_back(p);
    p.x = Vec3(3.0f,5.0f,0.0f); p.N = Vec3(0.0f,-1.0f,0.0f); p.pdf_A = 1.0f; p.pdf_W = 1.0f; p.pdf_rev = 1.0f; p.brdf = std::make_shared<LambertBRDF>(Vec3(0.8f,0.0f,0.0f));cl.push_back(p);

    // // 6, 0
    // p.x = Vec3(0.0f,0.0f,0.0f); p.N = Vec3(0.0f,1.0f,0.0f); p.pdf_A = 1.0f; p.pdf_W = 1.0f; p.pdf_rev = 1.0f; p.brdf = std::make_shared<Sensor>(); cp.push_back(p);
    // p.x = Vec3(1.0f,2.0f,0.0f); p.N = Vec3(0.0f,-1.0f,0.0f); p.pdf_A = 1.0f; p.pdf_W = 1.0f; p.pdf_rev = 1.0f; p.brdf = std::make_shared<LambertBRDF>(Vec3(0.8f,0.0f,0.0f));cp.push_back(p);
    // p.x = Vec3(2.0f,1.0f,0.0f); p.N = Vec3(0.0f,1.0f,0.0f); p.pdf_A = 1.0f; p.pdf_W = 1.0f; p.pdf_rev = 1.0f; p.brdf = std::make_shared<LambertBRDF>(Vec3(0.8f,0.0f,0.0f));cp.push_back(p);
    // p.x = Vec3(3.0f,5.0f,0.0f); p.N = Vec3(0.0f,-1.0f,0.0f); p.pdf_A = 1.0f; p.pdf_W = 1.0f; p.pdf_rev = 1.0f; p.brdf = std::make_shared<LambertBRDF>(Vec3(0.8f,0.0f,0.0f));cp.push_back(p);
    // p.x = Vec3(4.0f,2.0f,0.0f); p.N = Vec3(0.0f,1.0f,0.0f); p.pdf_A = 1.0f; p.pdf_W = 1.0f; p.pdf_rev = 1.0f; p.brdf = std::make_shared<LambertBRDF>(Vec3(0.8f,0.0f,0.0f));cp.push_back(p);
    // p.x = Vec3(5.0f,5.0f,0.0f); p.N = Vec3(0.0f,-1.0f,0.0f); p.pdf_A = 1.0f; p.pdf_W = 1.0f; p.pdf_rev = 1.0f; p.brdf = std::make_shared<Emission>(Vec3(1.f,1.f,1.f));cp.push_back(p);

    auto pdfs = compute_strategy_pdfs(cp, cl, 3, 3);
    // auto pdfs = compute_strategy_pdfs(cp, cl, 6, 0);
    std::cout << "strategy  pdf\n";
    for (const auto& p : pdfs) {
        std::cout << "(" << p.s << ',' << p.t << ")  "
        << std::fixed << std::setprecision(15)   // ← 任意
        << p.pdf << '\n';
    }

    return 0;
}

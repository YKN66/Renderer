#include <cstdlib>
#include "Renderer.h"
#include "random.h"
#include "Image.h"

#define __FAR__ 1.0e33
#define __PI__ 	3.14159265358979323846
#define MAX_RAY_DEPTH INT_MAX

Renderer::Renderer() {
    g_FilmBuffer = (float *) malloc(sizeof(float) * g_FilmWidth * g_FilmHeight * 3);
    g_AccumulationBuffer = (float *) malloc(sizeof(float) * g_FilmWidth * g_FilmHeight * 3);
    g_CountBuffer = (int *) malloc(sizeof(int) * g_FilmWidth * g_FilmHeight);

    resetFilm();
    clearRayTracedResult();
}
Renderer::Renderer(Camera camera, Object obj, std::vector<AreaLight> lights) {
    g_Camera = std::move(camera);
    g_Obj = std::move(obj);
    g_AreaLights = std::move(lights);
}

void Renderer::set3Dscene(Camera camera, Object obj, std::vector<AreaLight> lights, std::vector<ParticipatingMedia> media) {
    g_Camera = std::move(camera);
    g_Obj = std::move(obj);
    g_AreaLights = std::move(lights);
    g_ParticipatingMedia = std::move(media);

    g_FilmBuffer = (float *) malloc(sizeof(float) * g_FilmWidth * g_FilmHeight * 3);
    g_AccumulationBuffer = (float *) malloc(sizeof(float) * g_FilmWidth * g_FilmHeight * 3);
    g_CountBuffer = (int *) malloc(sizeof(int) * g_FilmWidth * g_FilmHeight);
}

void Renderer::setNsamples(const unsigned int nSample, const unsigned int samples) {
    if(nSample <= 0){
        nSamplesPerPixel = 1;
    }
    else{
        if(nSample > samples){
            nSamplesPerPixel = samples;
        }
        else{
            nSamplesPerPixel = nSample;
        }
    }
}

void Renderer::resetFilm() {
    memset(g_AccumulationBuffer, 0, sizeof(float) * g_FilmWidth * g_FilmHeight * 3);
    memset(g_CountBuffer, 0, sizeof(int) * g_FilmWidth * g_FilmHeight);
}

void Renderer::updateFilm() {
    for (int i = 0; i < g_FilmWidth * g_FilmHeight; i++) {
        if (g_CountBuffer[i] > 0) {
            g_FilmBuffer[i * 3] = g_AccumulationBuffer[i * 3] / g_CountBuffer[i];
            g_FilmBuffer[i * 3 + 1] = g_AccumulationBuffer[i * 3 + 1] / g_CountBuffer[i];
            g_FilmBuffer[i * 3 + 2] = g_AccumulationBuffer[i * 3 + 2] / g_CountBuffer[i];
        }
        else {
            g_FilmBuffer[i * 3] = 0.0;
            g_FilmBuffer[i * 3 + 1] = 0.0;
            g_FilmBuffer[i * 3 + 2] = 0.0;
        }
    }
}

void Renderer::saveImg(const std::string filename) {
    GLubyte *g_ImgBuffer = new GLubyte[g_FilmWidth * g_FilmHeight * 3];

    glReadBuffer(GL_FRONT);
    glReadPixels(0, 0, g_FilmWidth, g_FilmHeight, GL_RGB, GL_UNSIGNED_BYTE, g_ImgBuffer);

    Image image(g_FilmWidth, g_FilmHeight, g_ImgBuffer);
    image.save(filename);
    image.generateCSV(filename);
}

void Renderer::stepToNextPixel(RayTracingInternalData &io_data) {
    io_data.nextPixel_i++;
    if (io_data.nextPixel_i >= g_FilmWidth) {
        io_data.nextPixel_i = 0;
        io_data.nextPixel_j++;

        if (io_data.nextPixel_j >= g_FilmHeight) {
            io_data.nextPixel_j = 0;
        }
    }
}

void Renderer::clearRayTracedResult() {
    g_RayTracingInternalData.nextPixel_i = -1;
    g_RayTracingInternalData.nextPixel_j = 0;

    memset(g_FilmBuffer, 0, sizeof(float) * g_FilmWidth * g_FilmHeight * 3);
}

void Renderer::rayTriangleIntersect(const TriMesh &in_Mesh, const int in_Triangle_idx, const Ray &in_Ray, RayHit &out_Result) {
    out_Result.t = __FAR__;

    const Eigen::Vector3d v1 = in_Mesh.vertices[in_Mesh.triangles[in_Triangle_idx](0)];
    const Eigen::Vector3d v2 = in_Mesh.vertices[in_Mesh.triangles[in_Triangle_idx](1)];
    const Eigen::Vector3d v3 = in_Mesh.vertices[in_Mesh.triangles[in_Triangle_idx](2)];

    Eigen::Vector3d triangle_normal = (v1 - v3).cross(v2 - v3);
    triangle_normal.normalize();

    bool isFront = true;

    const double denominator = triangle_normal.dot(in_Ray.d);
    if (denominator >= 0.0)
        isFront = false;

    const double t = triangle_normal.dot(v3 - in_Ray.o) / denominator;

    if (t <= 0.0)
        return;

    const Eigen::Vector3d x = in_Ray.o + t * in_Ray.d;

    Eigen::Matrix<double, 3, 2> A;
    A.col(0) = v1 - v3;
    A.col(1) = v2 - v3;

    Eigen::Matrix2d ATA = A.transpose() * A;
    const Eigen::Vector2d b = A.transpose() * (x - v3);

    const Eigen::Vector2d alpha_beta = ATA.inverse() * b;

    if (alpha_beta.x() < 0.0 || 1.0 < alpha_beta.x() || alpha_beta.y() < 0.0 || 1.0 < alpha_beta.y() ||
        1.0 - alpha_beta.x() - alpha_beta.y() < 0.0 || 1.0 < 1.0 - alpha_beta.x() - alpha_beta.y())
        return;

    out_Result.t = t;
    out_Result.alpha = alpha_beta.x();
    out_Result.beta = alpha_beta.y();
    out_Result.isFront = isFront;
}

void Renderer::rayAreaLightIntersect(const std::vector<AreaLight> &in_AreaLights, const int in_Light_idx, const Ray &in_Ray,
                                     RayHit &out_Result) {
    out_Result.t = __FAR__;

    const Eigen::Vector3d pos = in_AreaLights[in_Light_idx].pos;
    const Eigen::Vector3d arm_u = in_AreaLights[in_Light_idx].arm_u;
    const Eigen::Vector3d arm_v = in_AreaLights[in_Light_idx].arm_v;

    Eigen::Vector3d light_normal = arm_u.cross(arm_v);
    light_normal.normalize();

    bool isFront = true;

    const double denominator = light_normal.dot(in_Ray.d);
    if (denominator >= 0.0)
        isFront = false;

    const double t = light_normal.dot(pos - in_Ray.o) / denominator;

    if (t <= 0.0)
        return;

    const Eigen::Vector3d x = in_Ray.o + t * in_Ray.d;

    // rescale uv coordinates such that they reside in [-1, 1], when the hit point is inside of the light.
    const double u = (x - pos).dot(arm_u) / arm_u.squaredNorm();
    const double v = (x - pos).dot(arm_v) / arm_v.squaredNorm();

    if (u < -1.0 || 1.0 < u || v < -1.0 || 1.0 < v) return;

    out_Result.t = t;
    out_Result.alpha = u;
    out_Result.beta = v;
    out_Result.isFront = isFront;
}

void Renderer::rayTracing(const Object &in_Object, const std::vector<AreaLight> &in_AreaLights, const Ray &in_Ray, RayHit &io_Hit) {
    double t_min = __FAR__;
    double alpha_I = 0.0, beta_I = 0.0;
    int mesh_idx = -99;
    int primitive_idx = -1;
    bool isFront = true;

    for (int m = 0; m < in_Object.meshes.size(); m++) {
        for (int k = 0; k < in_Object.meshes[m].triangles.size(); k++) {
            if (m == in_Ray.prev_mesh_idx && k == in_Ray.prev_primitive_idx) continue;

            RayHit temp_hit;
            rayTriangleIntersect(in_Object.meshes[m], k, in_Ray, temp_hit);
            if (temp_hit.t < t_min) {
                t_min = temp_hit.t;
                alpha_I = temp_hit.alpha;
                beta_I = temp_hit.beta;
                mesh_idx = m;
                primitive_idx = k;
                isFront = temp_hit.isFront;
            }
        }
    }

    for (int l = 0; l < in_AreaLights.size(); l++) {
        if (-1 == in_Ray.prev_mesh_idx && l == in_Ray.prev_primitive_idx) continue;

        RayHit temp_hit;
        rayAreaLightIntersect(in_AreaLights, l, in_Ray, temp_hit);
        if (temp_hit.t < t_min) {
            t_min = temp_hit.t;
            alpha_I = temp_hit.alpha;
            beta_I = temp_hit.beta;
            mesh_idx = -1;
            primitive_idx = l;
            isFront = temp_hit.isFront;
        }
    }

    io_Hit.t = t_min;
    io_Hit.alpha = alpha_I;
    io_Hit.beta = beta_I;
    io_Hit.mesh_idx = mesh_idx;
    io_Hit.primitive_idx = primitive_idx;
    io_Hit.isFront = isFront;
}
void Renderer::rayTracing(const Object &in_Object, const std::vector<AreaLight> &in_AreaLights, const std::vector<ParticipatingMedia> &all_medias, const Ray &in_Ray, RayHit &io_Hit) {
    double t_min = __FAR__;
    double alpha_I = 0.0, beta_I = 0.0;
    int mesh_idx = -99;
    int primitive_idx = -1;
    bool isFront = true;

    for (int m = 0; m < in_Object.meshes.size(); m++) {
        for (int k = 0; k < in_Object.meshes[m].triangles.size(); k++) {
            if (m == in_Ray.prev_mesh_idx && k == in_Ray.prev_primitive_idx) continue;

            RayHit temp_hit;
            rayTriangleIntersect(in_Object.meshes[m], k, in_Ray, temp_hit);
            if (temp_hit.t < t_min) {
                t_min = temp_hit.t;
                alpha_I = temp_hit.alpha;
                beta_I = temp_hit.beta;
                mesh_idx = m;
                primitive_idx = k;
                isFront = temp_hit.isFront;
            }
        }
    }

    for (int l = 0; l < in_AreaLights.size(); l++) {
        if (-1 == in_Ray.prev_mesh_idx && l == in_Ray.prev_primitive_idx) continue;

        RayHit temp_hit;
        rayAreaLightIntersect(in_AreaLights, l, in_Ray, temp_hit);
        if (temp_hit.t < t_min) {
            t_min = temp_hit.t;
            alpha_I = temp_hit.alpha;
            beta_I = temp_hit.beta;
            mesh_idx = -1;
            primitive_idx = l;
            isFront = temp_hit.isFront;
        }
    }

    int p_index;
    const double s = getFreePath(all_medias, in_Ray.o, p_index);
    if(s < t_min){
        t_min = s;
        mesh_idx = -2;
        primitive_idx = p_index;
    }

    io_Hit.t = t_min;
    io_Hit.alpha = alpha_I;
    io_Hit.beta = beta_I;
    io_Hit.mesh_idx = mesh_idx;
    io_Hit.primitive_idx = primitive_idx;
    io_Hit.isFront = isFront;
}

Eigen::Vector3d Renderer::sampleRandomPoint(const AreaLight &in_Light) {
    const double r1 = 2.0 * randomMT() - 1.0;
    const double r2 = 2.0 * randomMT() - 1.0;
    return in_Light.pos + r1 * in_Light.arm_u + r2 * in_Light.arm_v;
}

Eigen::Vector3d Renderer::computeRayHitNormal(const Object &in_Object, const RayHit &in_Hit) {
    const int v1_idx = in_Object.meshes[in_Hit.mesh_idx].triangles[in_Hit.primitive_idx](0);
    const int v2_idx = in_Object.meshes[in_Hit.mesh_idx].triangles[in_Hit.primitive_idx](1);
    const int v3_idx = in_Object.meshes[in_Hit.mesh_idx].triangles[in_Hit.primitive_idx](2);

    const Eigen::Vector3d n1 = in_Object.meshes[in_Hit.mesh_idx].vertex_normals[v1_idx];
    const Eigen::Vector3d n2 = in_Object.meshes[in_Hit.mesh_idx].vertex_normals[v2_idx];
    const Eigen::Vector3d n3 = in_Object.meshes[in_Hit.mesh_idx].vertex_normals[v3_idx];

    const double gamma = 1.0 - in_Hit.alpha - in_Hit.beta;
    Eigen::Vector3d n = in_Hit.alpha * n1 + in_Hit.beta * n2 + gamma * n3;
    n.normalize();

    if (!in_Hit.isFront) n = -n;

    return n;
}

//original function for main.
void Renderer::rendering(const int mode) {
#pragma omp parallel for
    for(int i = 0; i < g_FilmWidth * g_FilmHeight; i++){
//        stepToNextPixel(g_RayTracingInternalData);
//        const int pixel_flat_idx = g_RayTracingInternalData.nextPixel_j * g_FilmWidth + g_RayTracingInternalData.nextPixel_i;

        int pixel_flat_idx = i;
        int next_pixel_i = i % g_FilmWidth;
        int next_pixel_j = i / g_FilmWidth;
        Eigen::Vector3d I = Eigen::Vector3d::Zero();

        for (int k = 0; k < nSamplesPerPixel; k++) {
            double p_x = (next_pixel_i + randomMT()) / g_FilmWidth;
            double p_y = (next_pixel_j + randomMT()) / g_FilmHeight;

            Ray ray;
            g_Camera.screenView(p_x, p_y, ray);
            ray.prev_mesh_idx = -99;
            ray.prev_primitive_idx = -1;
            ray.pdf = 1.0;
            ray.depth = 0;

            std::vector<LightSubPath> light_SubPath;
            std::vector<ViewSubPath> view_SubPath;

            switch(mode){
                case 1: {
                    I += computePathTrace(ray, g_Obj, g_AreaLights);
                    break;
                }
                case 2: {
                    I += computeNEE(ray, g_Obj, g_AreaLights);
                    break;
                }
                case 3: {
                    I += computeMIS(ray, g_Obj, g_AreaLights);
                    break;
                }
                case 4: {
                    I += computePathTrace(ray, g_Obj, g_AreaLights, g_ParticipatingMedia);
                    break;
                }
                case 5: {
                    I += computeMIS(ray, g_Obj, g_AreaLights, g_ParticipatingMedia);
                    break;
                }
                case 6: {
                    int light_index = static_cast<int>(randomMT() * g_AreaLights.size());
//                    if(light_index == g_AreaLights.size()){
//                        std::cerr << "light_index gets over g_AreaLights.size()" << std::endl;
//                        light_index--;
//                    }

                    const Eigen::Vector3d light_normal = g_AreaLights[light_index].arm_u.cross(g_AreaLights[light_index].arm_v).normalized();
                    const Eigen::Vector3d light_point = sampleRandomPoint(g_AreaLights[light_index]);
                    Ray light_ray; RayHit light_rayHit;

                    double light_area = HemisphericSample(light_point, light_ray, g_AreaLights[light_index]);
                    light_ray.prev_mesh_idx = light_index;
                    double cosine = light_ray.d.dot(light_normal);

                    LightTracing(light_ray, g_Obj, cosine, g_AreaLights, light_SubPath);

                    light_rayHit.mesh_idx = -1;
                    light_rayHit.primitive_idx = light_index;
                    light_rayHit.isFront = true;
                    light_SubPath[0].x = light_point;
                    light_SubPath[0].contribute = Eigen::Vector3d::Ones() * 2.0f * __PI__ * cosine;
                    light_SubPath[0].rh = light_rayHit;
                    light_SubPath[0].cosine_o = cosine;
                    light_SubPath[0].materialMode = 0;
                    light_SubPath[0].pdf_forward = 1.0 / light_area;

                    setRadianceAndCurrentPDF(g_AreaLights, light_SubPath, light_index);

                    double current_pdf = 1.0;
                    I += computeBPT(ray, g_Obj, g_AreaLights, light_SubPath, view_SubPath, 1.0, 1.0, current_pdf);
                    break;
                }
            }
//            std::cout << I.transpose() << std::endl;
        }

        g_AccumulationBuffer[pixel_flat_idx * 3] += I.x();
        g_AccumulationBuffer[pixel_flat_idx * 3 + 1] += I.y();
        g_AccumulationBuffer[pixel_flat_idx * 3 + 2] += I.z();
        g_CountBuffer[pixel_flat_idx] += nSamplesPerPixel;
    }
}

Eigen::Vector3d Renderer::computePathTrace(const Ray &in_Ray, const Object &in_Object, const std::vector<AreaLight> &in_AreaLights) {
    if(in_Ray.depth > MAX_RAY_DEPTH)
        return Eigen::Vector3d::Zero();

    Ray new_ray; RayHit in_RayHit;
    rayTracing(in_Object, in_AreaLights, in_Ray, in_RayHit);

    if(in_RayHit.primitive_idx < 0)
        return Eigen::Vector3d::Zero();

    if (in_RayHit.mesh_idx == -1) // the ray has hit an area light
    {
        if (!in_RayHit.isFront)
            return Eigen::Vector3d::Zero();

        return in_AreaLights[in_RayHit.primitive_idx].intensity * in_AreaLights[in_RayHit.primitive_idx].color;
    }

    Eigen::Vector3d x = in_Ray.o + in_RayHit.t * in_Ray.d;
    Eigen::Vector3d normal = computeRayHitNormal(in_Object, in_RayHit);
    Eigen::Vector3d I = Eigen::Vector3d::Zero();

    double kd = in_Object.meshes[in_RayHit.mesh_idx].material.kd;
    double ks = in_Object.meshes[in_RayHit.mesh_idx].material.ks;
    double kt = in_Object.meshes[in_RayHit.mesh_idx].material.kt;
    double r = randomMT();

    if(r < kd){
        diffuseSample(in_Ray.depth, x, normal, in_RayHit, in_Object, new_ray);
        I += computePathTrace(new_ray, in_Object, in_AreaLights).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.getKd()) / kd;
    }
    else if(r < kd + ks){
        const double m = in_Object.meshes[in_RayHit.mesh_idx].material.m;
        blinnPhongSample(m, in_Ray.depth, x, normal, in_Ray.d, in_RayHit, in_Object, new_ray);
        if(new_ray.pdf < 0.0f)
            return I;

        Eigen::Vector3d halfVector = (new_ray.d - in_Ray.d).normalized();
        double cosine_v = std::max<double>(0.0f, -in_Ray.d.dot(halfVector));
        const double cosine_o = std::max<double>(0.0f, normal.dot(new_ray.d));
        I += computePathTrace(new_ray, in_Object, in_AreaLights).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.getKs() * 4.0 * cosine_v * cosine_o * (m + 2.0f)) / (ks * (m + 1.0f));
    }
    else if(r < kd + ks + kt){
        double eta = in_Object.meshes[in_RayHit.mesh_idx].material.eta;
        double cosine_i = std::max<double>(0.0f, -in_Ray.d.dot(normal));
        double fresnel_reflection = Dielectricfresnel(cosine_i, eta);
        double r_fresnel = randomMT();
        if(r_fresnel <= fresnel_reflection){
            //全反射
            reflectSample(in_Ray.depth, x, normal, in_Ray.d, in_RayHit, in_Object, new_ray);
            I += computePathTrace(new_ray, in_Object, in_AreaLights).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.color) / kt;
        }
        else{
            refractSample(eta, in_Ray.depth, x, normal, in_Ray.d, in_RayHit, in_Object, new_ray);
            I += computePathTrace(new_ray, in_Object, in_AreaLights).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.color * eta * eta) / kt;
        }
    }

    return I;
}

Eigen::Vector3d Renderer::computeNEE(const Ray &in_Ray, const Object &in_Object, const std::vector<AreaLight> &in_AreaLights) {
    if(in_Ray.depth >= MAX_RAY_DEPTH)
        return Eigen::Vector3d::Zero();

    Ray new_ray; RayHit in_RayHit;
    rayTracing(in_Object, in_AreaLights, in_Ray, in_RayHit);

    if(in_RayHit.primitive_idx < 0)
        return Eigen::Vector3d::Zero();

    if (in_RayHit.mesh_idx == -1) // the ray has hit an area light
    {
        if (in_RayHit.isFront && in_Ray.depth == 0)
            return in_AreaLights[in_RayHit.primitive_idx].intensity * in_AreaLights[in_RayHit.primitive_idx].color;

        return Eigen::Vector3d::Zero();
    }

    const Eigen::Vector3d x = in_Ray.o + in_RayHit.t * in_Ray.d;
    const Eigen::Vector3d normal = computeRayHitNormal(in_Object, in_RayHit);

    Eigen::Vector3d I = Eigen::Vector3d::Zero();

    const double kd = in_Object.meshes[in_RayHit.mesh_idx].material.kd;
    const double ks = in_Object.meshes[in_RayHit.mesh_idx].material.ks;
    const double kt = in_Object.meshes[in_RayHit.mesh_idx].material.kt;

    double k_sum = kd + ks + kt;
    double r = randomMT() * k_sum;

    //ここのkdを修正!!
    if(r < kd)
        I += computeDirectLighting(in_Ray, in_RayHit, in_AreaLights, in_Object, 1) * k_sum / kd;
    else if(r < kd + ks)
        I += computeDirectLighting(in_Ray, in_RayHit, in_AreaLights, in_Object, 2) * k_sum / ks;

    r = randomMT();
    if(r < kd){
        diffuseSample(in_Ray.depth, x, normal, in_RayHit, in_Object, new_ray);
        I += computeNEE(new_ray, in_Object, in_AreaLights).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.getKd()) / kd;
    }
    else if(r < kd + ks){
        const double m = in_Object.meshes[in_RayHit.mesh_idx].material.m;
        blinnPhongSample(m, in_Ray.depth, x, normal, in_Ray.d, in_RayHit, in_Object, new_ray);
        if(new_ray.pdf < 0.0f)
            return I;

        Eigen::Vector3d halfVector = (new_ray.d - in_Ray.d).normalized();
        double cosine = std::max<double>(0.0f, normal.dot(new_ray.d));
        double cosine_v = std::max<double>(0.0f, -in_Ray.d.dot(halfVector));
        I += computeNEE(new_ray, in_Object, in_AreaLights).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.getKs() * 4.0 * cosine_v * cosine * (m + 2.0f)) / (ks * (m + 1.0f));
    }
    else if(r < kd + ks + kt){
        double eta = in_Object.meshes[in_RayHit.mesh_idx].material.eta;
        double cosine_i = std::max<double>(0.0f, -in_Ray.d.dot(normal));
        double fresnel_reflection = Dielectricfresnel(cosine_i, eta);
        double r_fresnel = randomMT();
        if(r_fresnel <= fresnel_reflection){
            //全反射
            reflectSample(in_Ray.depth, x, normal, in_Ray.d, in_RayHit, in_Object, new_ray);
            I += computeNEE(new_ray, in_Object, in_AreaLights).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.color) / kt;
        }
        else{
            refractSample(eta, in_Ray.depth, x, normal, in_Ray.d, in_RayHit, in_Object, new_ray);
            I += computeNEE(new_ray, in_Object, in_AreaLights).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.color * eta * eta) / kt;
        }
    }

    return I;
}

Eigen::Vector3d Renderer::computeMIS(const Ray &in_Ray, const Object &in_Object, const std::vector<AreaLight> &in_AreaLights) {
    if(in_Ray.depth > MAX_RAY_DEPTH)
        return Eigen::Vector3d::Zero();

    Ray new_ray; RayHit in_RayHit;
    rayTracing(in_Object, in_AreaLights, in_Ray, in_RayHit);

    if(in_RayHit.primitive_idx < 0)
        return Eigen::Vector3d::Zero();

    const Eigen::Vector3d x = in_Ray.o + in_RayHit.t * in_Ray.d;

    if (in_RayHit.mesh_idx == -1) // the ray has hit an area light
    {
        if (in_RayHit.isFront) {
            if (in_Ray.depth == 0) {
                return in_AreaLights[in_RayHit.primitive_idx].intensity * in_AreaLights[in_RayHit.primitive_idx].color;
            }
            else{
                //pathTrace
                Eigen::Vector3d n_light = in_AreaLights[in_RayHit.primitive_idx].arm_u.cross(in_AreaLights[in_RayHit.primitive_idx].arm_v).normalized();
                double cosine = std::max<double>(0.0f, n_light.dot(- in_Ray.d));
                double dist2 = (x - in_Ray.o).squaredNorm();
                double path_pdf = in_Ray.pdf;
                double MIS_weight = 0.0;
                if(path_pdf < 0.0){
                    MIS_weight = 1.0;
                }
                else{
                    double nee_pdf = getLightProbability(in_AreaLights) * dist2 / cosine;
                    MIS_weight = (path_pdf * path_pdf) / (nee_pdf * nee_pdf + path_pdf * path_pdf);
                }
//                MIS_weight = 0.5f;

                return MIS_weight * in_AreaLights[in_RayHit.primitive_idx].intensity * in_AreaLights[in_RayHit.primitive_idx].color;
            }
        }
        return Eigen::Vector3d::Zero();
    }

    const Eigen::Vector3d normal = computeRayHitNormal(in_Object, in_RayHit);
    Eigen::Vector3d I = Eigen::Vector3d::Zero();

    const double kd = in_Object.meshes[in_RayHit.mesh_idx].material.kd;
    const double ks = in_Object.meshes[in_RayHit.mesh_idx].material.ks;
    const double kt = in_Object.meshes[in_RayHit.mesh_idx].material.kt;

    double k_sum = kd + ks;
    double r = randomMT() * k_sum;

    if(r < kd)
        I += computeDirectLighting_MIS(in_Ray, in_RayHit, in_AreaLights, in_Object, 1) * k_sum / kd;
    else if(r < kd + ks)
        I += computeDirectLighting_MIS(in_Ray, in_RayHit, in_AreaLights, in_Object, 2) * k_sum / ks;

    r = randomMT();

    if(r < kd){
        diffuseSample(in_Ray.depth, x, normal, in_RayHit, in_Object, new_ray);
        I += computeMIS(new_ray, in_Object, in_AreaLights).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.getKd()) / kd;
    }
    else if(r < kd + ks){
        const double m = in_Object.meshes[in_RayHit.mesh_idx].material.m;
        blinnPhongSample(m, in_Ray.depth, x, normal, in_Ray.d, in_RayHit, in_Object, new_ray);
        if(new_ray.pdf < 0.0f)
            return I;

        Eigen::Vector3d halfVector = (new_ray.d - in_Ray.d).normalized();
        double cosine_v = std::max<double>(0.0f, -in_Ray.d.dot(halfVector));
        double cosine_o = std::max<double>(0.0f, normal.dot(new_ray.d));
        I += computeMIS(new_ray, in_Object, in_AreaLights).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.getKs() * 4.0 * cosine_v * cosine_o * (m + 2.0f)) / (ks * (m + 1.0f));
    }
    else if(r < kd + ks + kt){
        double eta = in_Object.meshes[in_RayHit.mesh_idx].material.eta;
        double cosine_i = std::max<double>(0.0f, -in_Ray.d.dot(normal));
        double fresnel_reflection = Dielectricfresnel(cosine_i, eta);
        double r_fresnel = randomMT();
        if(r_fresnel <= fresnel_reflection){
            //全反射
            reflectSample(in_Ray.depth, x, normal, in_Ray.d, in_RayHit, in_Object, new_ray);
            I += computeMIS(new_ray, in_Object, in_AreaLights).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.color) / kt;
        }
        else{
            refractSample(eta, in_Ray.depth, x, normal, in_Ray.d, in_RayHit, in_Object, new_ray);
            I += computeMIS(new_ray, in_Object, in_AreaLights).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.color * eta * eta) / kt;
        }
    }

    return I;
}

Eigen::Vector3d Renderer::computeBPT(const Ray &in_Ray, const Object &in_Object, const std::vector<AreaLight> &in_AreaLights, const std::vector<LightSubPath> &light_SubPath, std::vector<ViewSubPath> &view_SubPath, const double prev_cosine, const double prev_russian, double &current_probability) {
    Ray new_ray; RayHit in_RayHit;
    rayTracing(in_Object, in_AreaLights, in_Ray, in_RayHit);

    if(in_RayHit.primitive_idx < 0)
        return Eigen::Vector3d::Zero();

    const Eigen::Vector3d x = in_Ray.o + in_RayHit.t * in_Ray.d;

    //いったん重みは考えない
    if (in_RayHit.mesh_idx == -1) // the ray has hit an area light
    {
        if (in_RayHit.isFront && in_Ray.depth == 0)
            return in_AreaLights[in_RayHit.primitive_idx].intensity * in_AreaLights[in_RayHit.primitive_idx].color;
        else if(in_RayHit.isFront) {
            double dist2 = (in_Ray.o - x).squaredNorm();
            Eigen::Vector3d light_normal = in_AreaLights[in_RayHit.primitive_idx].arm_u.cross(in_AreaLights[in_RayHit.primitive_idx].arm_v);
            double light_area = light_normal.norm() * 4.0;
            light_normal.normalize();

            double cosine_i = std::max<double>(0.0f, light_normal.dot(-in_Ray.d));

            double MIS_weight = calcMISWeight(in_Ray.depth, current_probability, in_Ray.pdf * prev_russian, dist2, prev_cosine, cosine_i, light_area, view_SubPath);
//            MIS_weight = 1.0 / (in_Ray.depth + 1);
            return in_AreaLights[in_RayHit.primitive_idx].intensity * in_AreaLights[in_RayHit.primitive_idx].color * MIS_weight;
        }

        return Eigen::Vector3d::Zero();
    }

    const Eigen::Vector3d normal = computeRayHitNormal(in_Object, in_RayHit);
    Eigen::Vector3d I = Eigen::Vector3d::Zero();

    //prev_russianは持たずに、russianのタイミングでP_rrをsubpathに追加しておいて、ここで書き換えるでもいいかも
    double pdf_forward = 1.0, pdf_reverse = 1.0;
    if(in_Ray.depth > 0){
        double dist2 = (in_Ray.o - x).squaredNorm();
        double cosine_i = std::max<double>(0.0f, normal.dot(-in_Ray.d));
        pdf_forward = prev_russian * in_Ray.pdf * cosine_i / dist2;
        pdf_reverse = prev_cosine / dist2;
        view_SubPath.push_back(ViewSubPath(pdf_forward, pdf_reverse));
    }

    current_probability *= pdf_forward;

    double kd = in_Object.meshes[in_RayHit.mesh_idx].material.kd;
    double ks = in_Object.meshes[in_RayHit.mesh_idx].material.ks;
    double k_sum = kd + ks;
    double r = randomMT() * k_sum;

    if(r < kd)
        I += BidirectionalPathTrace(in_Ray, in_RayHit, in_AreaLights, in_Object, light_SubPath, view_SubPath, current_probability, pdf_reverse, kd, 1) * k_sum / kd;
    else if(r < kd + ks)
        I += BidirectionalPathTrace(in_Ray, in_RayHit, in_AreaLights, in_Object, light_SubPath, view_SubPath, current_probability, pdf_reverse, ks, 2) * k_sum / ks;


    r = randomMT();
    if(r < kd){
        diffuseSample(in_Ray.depth, x, normal, in_RayHit, in_Object, new_ray);

        if(in_Ray.depth > 0){
            view_SubPath.back().pdf_reverse *= kd * getDiffuseProbability(normal, -in_Ray.d);
        }

        double cosine_o = std::max<double>(0.0, normal.dot(new_ray.d));
        I += computeBPT(new_ray, in_Object, in_AreaLights, light_SubPath, view_SubPath, cosine_o, kd, current_probability).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.getKd()) / kd;
    }
    else if(r < kd + ks){
        const double m = in_Object.meshes[in_RayHit.mesh_idx].material.m;
        blinnPhongSample(m, in_Ray.depth, x, normal, in_Ray.d, in_RayHit, in_Object, new_ray);
        if(new_ray.pdf < 0.0f)
            return I;

        if(in_Ray.depth > 0){
            view_SubPath.back().pdf_reverse *= ks * getBlinnPhongProbability(-new_ray.d, normal, -in_Ray.d, m);
        }

        Eigen::Vector3d halfVector = (new_ray.d - in_Ray.d).normalized();
        double cosine_v = std::max<double>(0.0f, -in_Ray.d.dot(halfVector));
        const double cosine_o = std::max<double>(0.0f, normal.dot(new_ray.d));
        I += computeBPT(new_ray, in_Object, in_AreaLights, light_SubPath, view_SubPath, cosine_o, ks, current_probability).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.getKs() * 4.0 * cosine_v * cosine_o * (m + 2.0f)) / (ks * (m + 1.0f));
    }

    return I;
}

Eigen::Vector3d Renderer::computePathTrace(const Ray &in_Ray, const Object &in_Object, const std::vector<AreaLight> &in_AreaLights, std::vector<ParticipatingMedia> &all_media) {
    if(in_Ray.depth > MAX_RAY_DEPTH)
        return Eigen::Vector3d::Zero();

    Ray new_ray; RayHit in_RayHit;
    rayTracing(in_Object, in_AreaLights, all_media, in_Ray, in_RayHit);

    if(in_RayHit.primitive_idx < 0)
        return Eigen::Vector3d::Zero();

    if (in_RayHit.mesh_idx == -1) // the ray has hit an area light
    {
        if (!in_RayHit.isFront)
            return Eigen::Vector3d::Zero();

        return in_AreaLights[in_RayHit.primitive_idx].intensity * in_AreaLights[in_RayHit.primitive_idx].color;
    }

    Eigen::Vector3d I = Eigen::Vector3d::Zero();
    const Eigen::Vector3d x = in_Ray.o + in_RayHit.t * in_Ray.d;

    if(in_RayHit.mesh_idx == -2){

        const int p_index = in_RayHit.primitive_idx;
        ParticipatingMedia *p = &all_media[p_index];
        const double albedo = p->albedo;
        const double r = randomMT();
        if(r >= albedo)
            return Eigen::Vector3d::Zero();

        scatteringSample(p->hg_g, in_Ray.depth, x, in_Ray.d, p_index, new_ray);
        I += computePathTrace(new_ray, in_Object, in_AreaLights, all_media).cwiseProduct(p->color) / albedo;
    }
    else{
        const Eigen::Vector3d normal = computeRayHitNormal(in_Object, in_RayHit);
        const double kd = in_Object.meshes[in_RayHit.mesh_idx].material.kd;
        const double ks = in_Object.meshes[in_RayHit.mesh_idx].material.ks;
        const double r = randomMT();

        if(r < kd){
            diffuseSample(in_Ray.depth, x, normal, in_RayHit, in_Object, new_ray);
            I += computePathTrace(new_ray, in_Object, in_AreaLights, all_media).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.getKd()) / kd;
        }
        else if(r < kd + ks){
            const double m = in_Object.meshes[in_RayHit.mesh_idx].material.m;
            blinnPhongSample(m, in_Ray.depth, x, normal, in_Ray.d, in_RayHit, in_Object, new_ray);
            if(new_ray.pdf < 0.0f)
                return I;

            Eigen::Vector3d halfVector = (new_ray.d - in_Ray.d).normalized();
            double cosine_v = std::max<double>(0.0f, -in_Ray.d.dot(halfVector));
            const double cosine_o = std::max<double>(0.0f, normal.dot(new_ray.d));
            I += computePathTrace(new_ray, in_Object, in_AreaLights, all_media).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.getKs() * 4.0 * cosine_v * cosine_o * (m + 2.0f)) / (ks * (m + 1.0f));
        }
    }

    return I;
}

Eigen::Vector3d Renderer::computeMIS(const Ray &in_Ray, const Object &in_Object, const std::vector<AreaLight> &in_AreaLights, std::vector<ParticipatingMedia> &all_media) {
    Ray new_ray; RayHit in_RayHit;
    rayTracing(in_Object, in_AreaLights, all_media, in_Ray, in_RayHit);

    if(in_RayHit.primitive_idx < 0)
        return Eigen::Vector3d::Zero();

    const Eigen::Vector3d x = in_Ray.o + in_RayHit.t * in_Ray.d;

    if (in_RayHit.mesh_idx == -1) // the ray has hit an area light
    {
        if (in_RayHit.isFront) {
            if (in_Ray.depth == 0) {
                return in_AreaLights[in_RayHit.primitive_idx].intensity * in_AreaLights[in_RayHit.primitive_idx].color;
            }
            else{
                //pathTrace
                Eigen::Vector3d n_light = in_AreaLights[in_RayHit.primitive_idx].arm_u.cross(in_AreaLights[in_RayHit.primitive_idx].arm_v).normalized();
                double cosine = std::max<double>(0.0f, n_light.dot(- in_Ray.d));
                double distance = (x - in_Ray.o).norm();
                double path_pdf = in_Ray.pdf;
                double nee_pdf = getLightProbability(in_AreaLights) * distance * distance / cosine;
                double MIS_weight = (path_pdf * path_pdf) / (nee_pdf * nee_pdf + path_pdf * path_pdf);
//                const double MIS_weight = 0.0f;

                return MIS_weight * in_AreaLights[in_RayHit.primitive_idx].intensity * in_AreaLights[in_RayHit.primitive_idx].color;
            }
        }
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d I = Eigen::Vector3d::Zero();

    if(in_RayHit.mesh_idx == -2){
        //participating media
        I += computeDirectLighting_MIS(in_Ray, in_RayHit, in_AreaLights, all_media, in_Object, 0);

        const int p_index = in_RayHit.primitive_idx;
        ParticipatingMedia *p = &all_media[p_index];
        const double albedo = p->albedo;
        const double r = randomMT();
        if(r >= albedo)
            return Eigen::Vector3d::Zero();

        scatteringSample(p->hg_g, in_Ray.depth, x, in_Ray.d, p_index, new_ray);
        I += computeMIS(new_ray, in_Object, in_AreaLights, all_media).cwiseProduct(p->color) / albedo;
    }
    else{
        const Eigen::Vector3d normal = computeRayHitNormal(in_Object, in_RayHit);

        const double kd = in_Object.meshes[in_RayHit.mesh_idx].material.kd;
        const double ks = in_Object.meshes[in_RayHit.mesh_idx].material.ks;
        double k_sum = kd + ks;
        double r = randomMT() * k_sum;

        if(r < kd)
            I += computeDirectLighting_MIS(in_Ray, in_RayHit, in_AreaLights, all_media, in_Object, 1) * k_sum / kd;
        else if(r < kd + ks)
            I += computeDirectLighting_MIS(in_Ray, in_RayHit, in_AreaLights, all_media, in_Object, 2) * k_sum / ks;

        r = randomMT();

        if(r < kd){
            diffuseSample(in_Ray.depth, x, normal, in_RayHit, in_Object, new_ray);
            I += computeMIS(new_ray, in_Object, in_AreaLights, all_media).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.getKd()) / kd;
        }
        else if(r < kd + ks){
            const double m = in_Object.meshes[in_RayHit.mesh_idx].material.m;
            blinnPhongSample(m, in_Ray.depth, x, normal, in_Ray.d, in_RayHit, in_Object, new_ray);
            if(new_ray.pdf < 0.0f)
                return I;

            Eigen::Vector3d halfVector = (new_ray.d - in_Ray.d).normalized();
            double cosine_v = std::max<double>(0.0f, -in_Ray.d.dot(halfVector));
            const double cosine_o = std::max<double>(0.0f, normal.dot(new_ray.d));
            I += computeMIS(new_ray, in_Object, in_AreaLights, all_media).cwiseProduct(in_Object.meshes[in_RayHit.mesh_idx].material.getKs() * 4.0 * cosine_v * cosine_o * (m + 2.0f)) / (ks * (m + 1.0f));
        }
    }

    return I;
}

Eigen::Vector3d Renderer::computeDirectLighting(const Ray &in_Ray, const RayHit &in_RayHit, const std::vector<AreaLight> &in_AreaLights, const Object &in_Object, const int mode) {
    Eigen::Vector3d I = Eigen::Vector3d::Zero();
    const Eigen::Vector3d x = in_Ray.o + in_RayHit.t * in_Ray.d;
    const Eigen::Vector3d n = computeRayHitNormal(in_Object, in_RayHit);

    for(int i = 0; i < in_AreaLights.size(); i++) {
        const Eigen::Vector3d p_light = sampleRandomPoint(in_AreaLights[i]);
        Eigen::Vector3d x_L = p_light - x;
        const double dist = x_L.norm();
        x_L.normalize();

        Eigen::Vector3d n_light = in_AreaLights[i].arm_u.cross(in_AreaLights[i].arm_v);
        const double area = n_light.norm() * 4.0;
        n_light.normalize();
        const double cos_light = n_light.dot(-x_L);
        if(cos_light <= 0.0) continue;

        double cos_x =  x_L.dot(n);
        if(cos_x <= 0.0) continue;

        // shadow test
        Ray ray;
        ray.o = x;
        ray.d = x_L;
        ray.prev_mesh_idx = in_RayHit.mesh_idx;
        ray.prev_primitive_idx = in_RayHit.primitive_idx;
        RayHit rh;
        rayTracing(in_Object, in_AreaLights, ray, rh);
        if (rh.mesh_idx == -1 && rh.primitive_idx == i) {
            const double G = (cos_x * cos_light) / (dist * dist);

            switch(mode){
                case 1: {
                    const Eigen::Vector3d BSDF = in_Object.meshes[in_RayHit.mesh_idx].material.getKd() / __PI__;
                    I += area * in_AreaLights[i].intensity * in_AreaLights[i].color.cwiseProduct(BSDF * G);
                    break;
                }
                case 2: {
                    const double m = in_Object.meshes[in_RayHit.mesh_idx].material.m;
                    const Eigen::Vector3d halfVector = ((-1 * in_Ray.d) + x_L).normalized();
                    const double cosine = std::max<double>(0.0f, n.dot(halfVector));
                    const Eigen::Vector3d BSDF = in_Object.meshes[in_RayHit.mesh_idx].material.getKs() * (m + 2.0f) * pow(cosine, m) / (2.0f * __PI__);
                    I += area * in_AreaLights[i].intensity * in_AreaLights[i].color.cwiseProduct(BSDF * G);
                    break;
                }
            }
        }
    }

    return I;
}
Eigen::Vector3d Renderer::computeDirectLighting_MIS(const Ray &in_Ray, const RayHit &in_RayHit, const std::vector<AreaLight> &in_AreaLights, const Object &in_Object, const int mode) {
    Eigen::Vector3d I = Eigen::Vector3d::Zero();
    const Eigen::Vector3d x = in_Ray.o + in_RayHit.t * in_Ray.d;
    const Eigen::Vector3d n = computeRayHitNormal(in_Object, in_RayHit);

    for(int i = 0; i < in_AreaLights.size(); i++) {
        const Eigen::Vector3d p_light = sampleRandomPoint(in_AreaLights[i]);
        Eigen::Vector3d x_L = p_light - x;
        const double dist2 = x_L.squaredNorm();
        x_L.normalize();

        Eigen::Vector3d n_light = in_AreaLights[i].arm_u.cross(in_AreaLights[i].arm_v);
        const double area = n_light.norm() * 4.0;
        n_light.normalize();
        const double cos_light = n_light.dot(-x_L);
        if (cos_light <= 0.0) continue;

        // shadow test
        Ray ray;
        ray.o = x;
        ray.d = x_L;
        ray.prev_mesh_idx = in_RayHit.mesh_idx;
        ray.prev_primitive_idx = in_RayHit.primitive_idx;
        RayHit rh;
        rayTracing(in_Object, in_AreaLights, ray, rh);
        if (rh.mesh_idx == -1 && rh.primitive_idx == i) {
            const double cos_x = n.dot(x_L);
            if (cos_x <= 0.0) continue;
            const double G = (cos_x * cos_light) / dist2;

            switch(mode){
                case 1: {
                    const Eigen::Vector3d BSDF = in_Object.meshes[in_RayHit.mesh_idx].material.getKd() / __PI__;

                    double path_pdf = getDiffuseProbability(n, x_L);
                    double nee_pdf = dist2 / (area * cos_light);
                    double MIS_weight = (nee_pdf * nee_pdf) / (nee_pdf * nee_pdf + path_pdf * path_pdf);
//                    MIS_weight = 0.5f;

                    I += MIS_weight * area * in_AreaLights[i].intensity * in_AreaLights[i].color.cwiseProduct(BSDF * G);
                    break;
                }
                case 2: {
                    double m = in_Object.meshes[in_RayHit.mesh_idx].material.m;

                    Eigen::Vector3d halfVector = ((-1 * in_Ray.d) + x_L).normalized();
                    double cosine = std::max<double>(0.0f, n.dot(halfVector));
                    Eigen::Vector3d BSDF = in_Object.meshes[in_RayHit.mesh_idx].material.getKs() * (m + 2.0f) * pow(cosine, m) / (2.0f * __PI__);

                    double path_pdf = getBlinnPhongProbability(in_Ray.d, n, x_L, m);
                    double nee_pdf = dist2 / (area * cos_light);
                    double MIS_weight = (nee_pdf * nee_pdf) / (nee_pdf * nee_pdf + path_pdf * path_pdf);
//                    MIS_weight = 0.5f;

                    I += MIS_weight * area * in_AreaLights[i].intensity * in_AreaLights[i].color.cwiseProduct(BSDF * G);
                    break;
                }
            }
        }
    }

    return I;
}
Eigen::Vector3d Renderer::computeDirectLighting_MIS(const Ray &in_Ray, const RayHit &in_RayHit, const std::vector<AreaLight> &in_AreaLights, const std::vector<ParticipatingMedia> &all_medias, const Object &in_Object, const int mode) {
    Eigen::Vector3d I = Eigen::Vector3d::Zero();
    const Eigen::Vector3d x = in_Ray.o + in_RayHit.t * in_Ray.d;
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    if(mode != 0)
        n = computeRayHitNormal(in_Object, in_RayHit);

    for(int i = 0; i < in_AreaLights.size(); i++) {
        const Eigen::Vector3d p_light = sampleRandomPoint(in_AreaLights[i]);
        Eigen::Vector3d x_L = p_light - x;
        const double distance = x_L.norm();

        int shadow_media_index = 0;
        double s = getFreePath(all_medias, x, shadow_media_index);

        if(s < distance)
            continue;

        x_L.normalize();

        Eigen::Vector3d n_light = in_AreaLights[i].arm_u.cross(in_AreaLights[i].arm_v);
        const double area = n_light.norm() * 4.0;
        n_light.normalize();
        const double cos_light = n_light.dot(-x_L);
        if (cos_light <= 0.0) continue;

        // shadow test
        Ray ray;
        ray.o = x;
        ray.d = x_L;
        ray.prev_mesh_idx = in_RayHit.mesh_idx;
        ray.prev_primitive_idx = in_RayHit.primitive_idx;
        RayHit rh;
        rayTracing(in_Object, in_AreaLights, ray, rh);
        if (rh.mesh_idx == -1 && rh.primitive_idx == i) {
            double cos_x = 1.0;
            if(mode != 0)
                cos_x = n.dot(x_L);

            if (cos_x <= 0.0) continue;
            const double G = (cos_x * cos_light) / (distance * distance);
            const double nee_pdf = (distance * distance) / (area * cos_light);

//            double extinction = all_medias[shadow_media_index].extinction;
//            double freepath_pdf = std::exp(- extinction * distance);
//            freepath_pdf = 1.0;
//            extinction_pdf = extinction * std::exp(-extinction * s);

            switch(mode){
                case 0:{
                    const double path_pdf = getPhaseProbability(in_Ray.d, x_L, all_medias[in_RayHit.primitive_idx].hg_g);
                    const Eigen::Vector3d BSDF = all_medias[in_RayHit.primitive_idx].color * path_pdf;  //pdfがBSDFに相当

                    const double MIS_weight = (nee_pdf * nee_pdf) / (nee_pdf * nee_pdf + path_pdf * path_pdf);
//                    const double MIS_weight = 1.0f;
                    I += MIS_weight * area * in_AreaLights[i].intensity * in_AreaLights[i].color.cwiseProduct(BSDF * G);
                    break;

                }
                case 1: {
                    const Eigen::Vector3d BSDF = in_Object.meshes[in_RayHit.mesh_idx].material.getKd() / __PI__;
                    const double path_pdf = getDiffuseProbability(n, x_L);

                    const double MIS_weight = (nee_pdf * nee_pdf) / (nee_pdf * nee_pdf + path_pdf * path_pdf);
//                    const double MIS_weight = 1.0f;
                    I += MIS_weight * area * in_AreaLights[i].intensity * in_AreaLights[i].color.cwiseProduct(BSDF * G);
                    break;
                }
                case 2: {
                    const double m = in_Object.meshes[in_RayHit.mesh_idx].material.m;

                    const Eigen::Vector3d halfVector = ((-1 * in_Ray.d) + x_L).normalized();
                    const double cosine = std::max<double>(0.0f, n.dot(halfVector));
                    const Eigen::Vector3d BSDF = in_Object.meshes[in_RayHit.mesh_idx].material.getKs() * (m + 2.0f) * pow(cosine, m) / (2.0f * __PI__);

                    const double path_pdf = getBlinnPhongProbability(in_Ray.d, n, x_L, m);
                    const double MIS_weight = (nee_pdf * nee_pdf) / (nee_pdf * nee_pdf + path_pdf * path_pdf);
//                    const double MIS_weight = 1.0f;
                    I += MIS_weight * area * in_AreaLights[i].intensity * in_AreaLights[i].color.cwiseProduct(BSDF * G);
                    break;
                }
            }
        }
    }
    return I;
}

double Renderer::getLightProbability(const std::vector<AreaLight> &in_AreaLights) {
    double pdf = 0.0f;

    for(int i = 0; i < in_AreaLights.size(); i++){
        const Eigen::Vector3d light_cross = in_AreaLights[i].arm_u.cross(in_AreaLights[i].arm_v);
        const double area = light_cross.norm() * 4.0;

        pdf += area;
    }

    return 1.0f / pdf;
}
double Renderer::getDiffuseProbability(const Eigen::Vector3d normal, const Eigen::Vector3d out_dir) {
    const double cosine = std::max<double>(0.0f, normal.dot(out_dir));

    return cosine / __PI__;
}
double Renderer::getBlinnPhongProbability(const Eigen::Vector3d in_dir, const Eigen::Vector3d normal,
                                          const Eigen::Vector3d out_dir, const double m) {
    Eigen::Vector3d halfVector = (-in_dir + out_dir).normalized();
    double cosine_h = std::max<double>(0.0f, normal.dot(halfVector));
    double cosine_v = std::max<double>(0.0f, -in_dir.dot(halfVector));
    double pdf_h = (m + 1) * pow(cosine_h, m) / (2.0 * __PI__);

    return pdf_h / (4.0 * cosine_v);
}
double Renderer::getPhaseProbability(const Eigen::Vector3d in_dir, const Eigen::Vector3d out_dir, const double hg_g) {
    const double cosine = in_dir.dot(out_dir);
//    return (1 - hg_g * hg_g) / (4.0f * __PI__ * pow((1 + hg_g * hg_g + 2.0f * hg_g * cosine), 1.5f));
    return (1 - hg_g * hg_g) / (4.0f * __PI__ * pow((1 + hg_g * hg_g + 2.0f * hg_g * cosine), 1.5f));
}

void Renderer::diffuseSample(const int depth, const Eigen::Vector3d &in_x, const Eigen::Vector3d &in_n, const RayHit &rayHit, const Object &in_Object, Ray &out_ray) {
    Eigen::Vector3d bn =
            in_Object.meshes[rayHit.mesh_idx].vertices[in_Object.meshes[rayHit.mesh_idx].triangles[rayHit.primitive_idx].x()] -
            in_Object.meshes[rayHit.mesh_idx].vertices[in_Object.meshes[rayHit.mesh_idx].triangles[rayHit.primitive_idx].z()];

    bn.normalize();

    const Eigen::Vector3d cn = bn.cross(in_n);

//    const double theta = acos(sqrt(randomMT()));
    const double theta = asin(sqrt(randomMT()));
    const double phi = randomMT() * 2.0f * __PI__;

    const double _dx = sin(theta) * cos(phi);
    const double _dy = cos(theta);
    const double _dz = sin(theta) * sin(phi);

    Eigen::Vector3d x_L = _dx * bn + _dy * in_n + _dz * cn;
    x_L.normalize();
    out_ray.o = in_x;
    out_ray.d = x_L;
    out_ray.prev_mesh_idx = rayHit.mesh_idx;
    out_ray.prev_primitive_idx = rayHit.primitive_idx;
    out_ray.depth = depth + 1;
    out_ray.pdf = cos(theta) / __PI__;
}
void Renderer::blinnPhongSample(const double m, const int depth, const Eigen::Vector3d &in_x, const Eigen::Vector3d &in_n, const Eigen::Vector3d &in_direction, const RayHit &rayHit, const Object &in_Object, Ray &out_ray) {
    Eigen::Vector3d bn =
            in_Object.meshes[rayHit.mesh_idx].vertices[in_Object.meshes[rayHit.mesh_idx].triangles[rayHit.primitive_idx].x()] -
            in_Object.meshes[rayHit.mesh_idx].vertices[in_Object.meshes[rayHit.mesh_idx].triangles[rayHit.primitive_idx].z()];

    bn.normalize();

    const Eigen::Vector3d cn = bn.cross(in_n);

    const double theta = acos(pow(randomMT(), 1.0f / (m + 1.0f)));
    const double phi = randomMT() * 2.0f * __PI__;

    const double _dx = sin(theta) * cos(phi);
    const double _dy = cos(theta);
    const double _dz = sin(theta) * sin(phi);

    Eigen::Vector3d halfVector = _dx * bn + _dy * in_n + _dz * cn;
    halfVector.normalize();

    const Eigen::Vector3d o_parallel = (-1 * in_direction).dot(halfVector) * halfVector;
    const Eigen::Vector3d o_vertical = (-1 * in_direction) - o_parallel;

    out_ray.o = in_x;
    out_ray.d = (o_parallel - o_vertical).normalized();
    out_ray.prev_mesh_idx = rayHit.mesh_idx;
    out_ray.prev_primitive_idx = rayHit.primitive_idx;
    out_ray.depth = depth + 1;


    if(out_ray.d.dot(in_n) < 0.0f)
        out_ray.pdf = -1.0f;
    else{
        double cosine_v = -in_direction.dot(halfVector);
        out_ray.pdf = (m + 1) * pow(cos(theta), m) / (8.0f * __PI__ * cosine_v);
    }

}

void Renderer::reflectSample(const int depth, const Eigen::Vector3d &in_x, const Eigen::Vector3d &in_n, const Eigen::Vector3d &in_direction, const RayHit &rayHit, const Object &in_Object, Ray &out_ray) {
    Eigen::Vector3d out_direction = 2.0 * -in_direction.dot(in_n) * in_n + in_direction;
    out_direction.normalize();

    out_ray.o = in_x;
    out_ray.d = out_direction;
    out_ray.prev_mesh_idx = rayHit.mesh_idx;
    out_ray.prev_primitive_idx = rayHit.primitive_idx;
    out_ray.depth = depth + 1;
    out_ray.pdf = -1.0;
}

void Renderer::refractSample(double &eta, const int depth, const Eigen::Vector3d &in_x, const Eigen::Vector3d &in_n, const Eigen::Vector3d &in_direction,  const RayHit &rayHit, const Object &in_Object, Ray &out_ray) {
    eta = rayHit.isFront ? eta : 1.0 / eta;
    double in_dot_normal = -in_direction.dot(in_n);

    double sqrt = 1.0 - eta * eta * (1.0 - in_dot_normal * in_dot_normal);

    if(sqrt < 0.0){
        //全反射
        reflectSample(depth, in_x, in_n, in_direction, rayHit, in_Object, out_ray);
        eta = 1.0;
        return;
    }

    Eigen::Vector3d out_direction = - std::sqrt(sqrt) * in_n - eta * (-in_direction - in_dot_normal * in_n);
    out_direction.normalize();

    out_ray.o = in_x;
    out_ray.d = out_direction;
    out_ray.prev_mesh_idx = rayHit.mesh_idx;
    out_ray.prev_primitive_idx = rayHit.primitive_idx;
    out_ray.depth = depth + 1;
    out_ray.pdf = -1.0;
}

void Renderer::scatteringSample(const double hg_g, const int depth, const Eigen::Vector3d &in_x, const Eigen::Vector3d &in_direction, const int p_index, Ray &out_ray) {
    Eigen::Vector3d bn;
    if(fabs(in_direction.x()) > 1e-4)
        bn = Eigen::Vector3d::UnitY().cross(in_direction).normalized();
    else
        bn = Eigen::Vector3d::UnitX().cross(in_direction).normalized();

    const Eigen::Vector3d cn = bn.cross(in_direction);

    double cosine;
    if(fabs(hg_g) < 1e-4){
        cosine = 1.0 - 2.0 * randomMT();
    }
    else{
//        double f = (1.0 - hg_g * hg_g) / (1.0 + hg_g - 2.0 * hg_g * randomMT());
        double f = (1.0 - hg_g * hg_g) / (1.0 + hg_g - 2.0 * hg_g * randomMT());
        cosine = - (1.0 + hg_g * hg_g - f * f) / (2.0 * hg_g);
    }

    const double cosine_t = (cosine < -1.0) ? -1.0 : ((cosine > 1.0) ? 1.0 : cosine);
    const double theta = acos(cosine_t);
    const double phi = 2.0f * __PI__ * randomMT();

    const double _dx = sin(theta) * cos(phi);
    const double _dy = cos(theta);
    const double _dz = sin(theta) * sin(phi);

    Eigen::Vector3d d = _dx * bn + _dy * in_direction + _dz * cn;
    d.normalize();

    out_ray.o = in_x;
    out_ray.d = d;
    out_ray.prev_mesh_idx = -2;
    out_ray.prev_primitive_idx = p_index;
    out_ray.depth = depth + 1;
    out_ray.pdf = (1 - hg_g * hg_g) / pow((1 + hg_g * hg_g + 2.0f * hg_g * cos(theta)), 1.5f) / 4.0 / __PI__;
}

double Renderer::HemisphericSample(const Eigen::Vector3d &in_x, Ray &out_ray, const AreaLight &in_Light) {
    const Eigen::Vector3d bn = in_Light.arm_u.normalized();
    const Eigen::Vector3d cn = in_Light.arm_v.normalized();

    const Eigen::Vector3d light_cross = in_Light.arm_u.cross(in_Light.arm_v);
    const double norm = light_cross.norm();
    const Eigen::Vector3d light_normal = light_cross / norm;

    const double theta = acos(randomMT());
    const double phi = randomMT() * 2.0f * __PI__;

    const double _dx = sin(theta) * cos(phi);
    const double _dy = cos(theta);
    const double _dz = sin(theta) * sin(phi);

    Eigen::Vector3d x_L = _dx * bn + _dy * light_normal + _dz * cn;
    x_L.normalize();
    out_ray.o = in_x;
    out_ray.d = x_L;
    out_ray.prev_mesh_idx = -1;
    out_ray.prev_primitive_idx = -1;
    out_ray.depth = 1;
    out_ray.pdf = 1.0f / (2.0f * __PI__);

    return norm * 4.0;
}

bool Renderer::isInParticipatingMedia(const ParticipatingMedia &media, const Eigen::Vector3d &in_point) {
    double distance = (media.pos - in_point).norm();

    if(media.radius > distance)
        return true;

    return false;
}

double Renderer::getFreePath(const std::vector<ParticipatingMedia> &all_medias, const Eigen::Vector3d &in_point, int &index) {
    double s_min = DBL_MAX;
    index = -1;
    for(int i = 0; i < all_medias.size(); i++){
        if(!isInParticipatingMedia(all_medias[i], in_point))
            continue;

        double extinction = all_medias[i].extinction;
        if(extinction > 1e-6) {
            const double s = - std::log(1.0 -randomMT()) / extinction;
            if(s_min > s){
                s_min = s;
                index = i;
            }
        }
    }

    return s_min;
}

double Renderer::Dielectricfresnel(double cos_theta, double eta) {
    cos_theta = std::clamp(cos_theta, -1.0, 1.0);
    if(cos_theta < 0.0){
        eta = 1.0 / eta;
        cos_theta = - cos_theta;
    }

    double sin2_thetaI = 1.0 - cos_theta * cos_theta;
    double sin2_thetaT = sin2_thetaI / eta / eta;
    if(sin2_thetaT >= 1.0)
        return 1.0;
    double cos_thetaT = std::sqrt(1.0 - sin2_thetaT);

    double r_parl = (eta * cos_theta - cos_thetaT) / (eta * cos_theta + cos_thetaT);
    double r_perp = (cos_theta - eta * cos_thetaT) / (cos_theta + eta * cos_thetaT);

    return (r_parl * r_parl + r_perp * r_perp) / 2.0;
}

Eigen::Vector3d Renderer::BidirectionalPathTrace(const Ray &in_Ray, const RayHit &in_RayHit, const std::vector<AreaLight> &in_AreaLights, const Object &in_Object, const std::vector<LightSubPath> &light_SubPath, const std::vector<ViewSubPath> &view_SubPath, const double current_pdf, const double prev_geometry, const double russian_probability, const int mode) {
    Eigen::Vector3d I = Eigen::Vector3d::Zero();
    const Eigen::Vector3d x = in_Ray.o + in_RayHit.t * in_Ray.d;
    const Eigen::Vector3d normal = computeRayHitNormal(in_Object, in_RayHit);
    const int depth = in_Ray.depth;

    for(int i = 0; i < light_SubPath.size(); i++){
        Eigen::Vector3d connect_dir = light_SubPath[i].x - x;
        const double dist2 = connect_dir.squaredNorm();
        connect_dir.normalize();

        Eigen::Vector3d connect_normal;
        if(light_SubPath[i].materialMode == 0)
            connect_normal = (in_AreaLights[light_SubPath[i].rh.primitive_idx].arm_u.cross(in_AreaLights[light_SubPath[i].rh.primitive_idx].arm_v)).normalized();
        else if(light_SubPath[i].materialMode == 1 || light_SubPath[i].materialMode == 2)
            connect_normal = computeRayHitNormal(in_Object, light_SubPath[i].rh);

        const double connect_cos = connect_normal.dot(-connect_dir);
        if(connect_cos <= 0.0) continue;

        // shadow test
        Ray _ray;
        _ray.o = x;
        _ray.d = connect_dir;
        _ray.prev_mesh_idx = in_RayHit.mesh_idx;
        _ray.prev_primitive_idx = in_RayHit.primitive_idx;
        RayHit _rh;
        rayTracing(in_Object, in_AreaLights, _ray, _rh);
        if(_rh.mesh_idx == light_SubPath[i].rh.mesh_idx && _rh.primitive_idx == light_SubPath[i].rh.primitive_idx){
            //接続が成功
            const double cos_x = normal.dot(connect_dir);
            if(cos_x <= 0.0) continue;

            double G = (cos_x * connect_cos) / dist2;
            double light_for_pdf, light_rev_pdf;
            std::tie(light_for_pdf, light_rev_pdf) = calcLightPDF(x, -connect_dir, in_Object, light_SubPath, cos_x, i);
            Eigen::Vector3d connect_Geometry = calcGeometry(-connect_dir, connect_normal, in_Object, light_SubPath, i) * G;

            switch(mode){
                case 1: {
                    const Eigen::Vector3d connect_BSDF = (in_Object.meshes[in_RayHit.mesh_idx].material.getKd() / __PI__).cwiseProduct(connect_Geometry);
                    double kd = in_Object.meshes[in_RayHit.mesh_idx].material.kd;

                    double view_for_pdf = kd * getDiffuseProbability(normal, connect_dir) * connect_cos / dist2;
                    double view_rev_pdf = kd * getDiffuseProbability(normal, -in_Ray.d) * prev_geometry;
//                    std::cout << current_pdf << std::endl;
                    double MIS_weight = calcMISWeight(i, depth, current_pdf, light_for_pdf, light_rev_pdf, view_for_pdf, view_rev_pdf, light_SubPath, view_SubPath);
//                    std::cout << MIS_weight << std::endl;
//                    MIS_weight = 1.0 / (depth + i + 2);
                    I += light_SubPath[i].radiance.cwiseProduct(connect_BSDF) * MIS_weight;
                    break;
                }
                case 2: {
                    const double m = in_Object.meshes[in_RayHit.mesh_idx].material.m;
                    const Eigen::Vector3d halfVector = (connect_dir - in_Ray.d).normalized();
                    const double cosine = std::max<double>(0.0f, normal.dot(halfVector));
                    const Eigen::Vector3d connect_BSDF = (in_Object.meshes[in_RayHit.mesh_idx].material.getKs() * (m + 2.0f) * pow(cosine, m) / (2.0f * __PI__)).cwiseProduct(connect_Geometry);
                    double ks = in_Object.meshes[in_RayHit.mesh_idx].material.ks;

                    double view_for_pdf = ks * getBlinnPhongProbability(in_Ray.d, normal, connect_dir, m) * connect_cos / dist2;
                    double view_rev_pdf = ks * getBlinnPhongProbability(-connect_dir, normal, -in_Ray.d, m) * prev_geometry;
                    double MIS_weight = calcMISWeight(i, depth, current_pdf, light_for_pdf, light_rev_pdf, view_for_pdf, view_rev_pdf, light_SubPath, view_SubPath);

//                    MIS_weight = 1.0 / (depth + i + 2);
                    I += light_SubPath[i].radiance.cwiseProduct(connect_BSDF) * MIS_weight;
                    break;
                }
            }
        }
    }
    return I;
}

void Renderer::LightTracing(const Ray &in_Ray, const Object &in_Object, const double prev_cosine, const std::vector<AreaLight> &in_AreaLights, std::vector<LightSubPath> &in_subpath) {
    int depth = in_Ray.depth;
    if(in_Ray.depth > MAX_RAY_DEPTH){
        in_subpath.resize(depth);
        return;
    }

    Ray new_ray; RayHit in_RayHit;
    rayTracing(in_Object, in_AreaLights, in_Ray, in_RayHit);

    if(in_RayHit.primitive_idx < 0){
        in_subpath.resize(depth);
        return;
    }

    if(in_RayHit.mesh_idx == -1){
        in_subpath.resize(depth);
        return;
    }

    //viewPointに衝突した場合の処理はここに書く
    const Eigen::Vector3d x = in_Ray.o + in_RayHit.t * in_Ray.d;
    const Eigen::Vector3d normal = computeRayHitNormal(in_Object, in_RayHit);
    double dist2 = (x - in_Ray.o).squaredNorm();
    double cosine_i = std::max<double>(0.0f, normal.dot(-in_Ray.d));

    const double kd = in_Object.meshes[in_RayHit.mesh_idx].material.kd;
    const double ks = in_Object.meshes[in_RayHit.mesh_idx].material.ks;
    const double r = randomMT();

    if(r < kd){
        diffuseSample(in_Ray.depth, x, normal, in_RayHit, in_Object, new_ray);
        double cosine_o = std::max<double>(0.0f, normal.dot(new_ray.d));

        LightTracing(new_ray, in_Object, cosine_o, in_AreaLights, in_subpath);

        in_subpath[depth].contribute = in_Object.meshes[in_RayHit.mesh_idx].material.getKd() / kd;
        in_subpath[depth].materialMode = 1;
        in_subpath[depth].cosine_o = cosine_o;
        in_subpath[depth].pdf_reverse = kd * getDiffuseProbability(normal, -in_Ray.d) * prev_cosine / dist2;
        in_subpath[depth + 1].pdf_forward *= kd;
    }
    else if(r < kd + ks){
        const double m = in_Object.meshes[in_RayHit.mesh_idx].material.m;
        blinnPhongSample(m, in_Ray.depth, x, normal, in_Ray.d, in_RayHit, in_Object, new_ray);

        if(new_ray.pdf < 0.0f){
            in_subpath.resize(depth + 1);

            in_subpath[depth].contribute = Eigen::Vector3d::Zero();
            in_subpath[depth].materialMode = 2;
        }
        else{
            double cosine_o = std::max<double>(0.0f, normal.dot(new_ray.d));

            LightTracing(new_ray, in_Object, cosine_o, in_AreaLights, in_subpath);

            Eigen::Vector3d halfVector = (new_ray.d - in_Ray.d).normalized();
            double cosine_v = std::max<double>(0.0f, -in_Ray.d.dot(halfVector));
            in_subpath[depth].contribute = in_Object.meshes[in_RayHit.mesh_idx].material.getKs() * 4.0 * cosine_v * cosine_o * (m + 2.0f) / (ks * (m + 1.0f));
            in_subpath[depth].materialMode = 2;
            in_subpath[depth].cosine_o = cosine_o;
            in_subpath[depth].pdf_reverse = getBlinnPhongProbability(-new_ray.d, normal, -in_Ray.d, m) * prev_cosine / dist2;
            in_subpath[depth + 1].pdf_forward *= ks;
        }
    }
    else{
        in_subpath.resize(depth + 1);
        if(kd > ks)
            in_subpath[depth].materialMode = 1;
        else
            in_subpath[depth].materialMode = 2;

        in_subpath[depth].contribute = Eigen::Vector3d::Zero();
    }

    in_subpath[depth].dist2 = dist2;
    in_subpath[depth].pdf_forward = in_Ray.pdf * cosine_i / dist2;
    in_subpath[depth].in_dir = in_Ray.d;
    in_subpath[depth].x = x;
    in_subpath[depth].rh = in_RayHit;

    return;
}

void Renderer::setRadianceAndCurrentPDF(const std::vector<AreaLight> &in_AreaLights, std::vector<LightSubPath> &light_SubPath, const int light_index) {
    double area = in_AreaLights[light_index].arm_u.cross(in_AreaLights[light_index].arm_v).norm() * 4.0;
    light_SubPath[0].radiance = in_AreaLights[light_index].intensity * in_AreaLights[light_index].color * area;
    light_SubPath[0].current_pdf = light_SubPath[0].pdf_forward;

    for(int i = 1; i < light_SubPath.size(); i++){
        light_SubPath[i].radiance = light_SubPath[i - 1].radiance.cwiseProduct(light_SubPath[i - 1].contribute);
        light_SubPath[i].current_pdf = light_SubPath[i - 1].current_pdf * light_SubPath[i].pdf_forward;
    }
}

Eigen::Vector3d Renderer::calcGeometry(const Eigen::Vector3d &dir, const Eigen::Vector3d &normal, const Object &in_Object, const std::vector<LightSubPath> &light_SubPath, const int index) {
    LightSubPath _s = light_SubPath[index];
    RayHit _rh = _s.rh;

    switch(_s.materialMode){
        case 0: {
            return Eigen::Vector3d::Ones();
        }
        case 1: {
            return in_Object.meshes[_rh.mesh_idx].material.getKd() / __PI__;
        }
        case 2: {
            const double m = in_Object.meshes[_rh.mesh_idx].material.m;
            const Eigen::Vector3d halfVector = (dir - _s.in_dir).normalized();
            const double cosine = std::max<double>(0.0f, normal.dot(halfVector));
            return in_Object.meshes[_rh.mesh_idx].material.getKs() * (m + 2.0f) * pow(cosine, m) / (2.0f * __PI__);
        }
    }

    std::cerr << "error happens in calcGeometry()" << std::endl;
    exit(1);
}

double Renderer::calcMISWeight(const int light_index, const int view_index, const double current_view_pdf, const double light_for_pdf, const double light_rev_pdf,
                               const double view_for_pdf, const double view_rev_pdf, const std::vector<LightSubPath> &light_SubPath, const std::vector<ViewSubPath> &view_SubPath) {

    //light_indexはi, view_indexはdepth想定
    double current_pdf = light_SubPath[light_index].current_pdf * current_view_pdf;
    double denom = current_pdf * current_pdf;

    double product = current_pdf * view_for_pdf / light_SubPath[light_index].pdf_forward;;
    denom += product * product;

    if(light_index > 0){
        product *= light_rev_pdf / light_SubPath[light_index - 1].pdf_forward;
        denom += product * product;
    }
//    for(int i = 2; i <= light_index; i++){
//        product *= light_SubPath[light_index - i + 1].pdf_reverse / light_SubPath[light_index - i].pdf_forward;
//        denom += product * product;
//    }
    for(int i = light_index - 2; i >= 0; i--){
        product *= light_SubPath[i + 1].pdf_reverse / light_SubPath[i].pdf_forward;
        denom += product * product;
    }

    product = current_pdf;
    if(view_index > 0){
        product *= light_for_pdf / view_SubPath[view_index - 1].pdf_forward;
        denom += product * product;
    }
    if(view_index > 1){
        product *= view_rev_pdf / view_SubPath[view_index - 2].pdf_forward;
        denom += product * product;
    }
    for(int i = view_index - 3; i >= 0; i--){
        product *= view_SubPath[i + 1].pdf_reverse / view_SubPath[i].pdf_forward;
        denom += product * product;
    }

    return current_pdf * current_pdf / denom;
}

double Renderer::calcMISWeight(const int view_index, const double current_pdf, const double prev_pdf, const double dist2, const double prev_cosine, const double cosine_light, const double area, const std::vector<ViewSubPath> &view_SubPath){
    double prev_forward_pdf = prev_pdf * cosine_light / dist2;
    double mole_sq = current_pdf * prev_forward_pdf;
    double product = mole_sq;
    double denom = product * product;

    product *= 1.0 / (area * prev_forward_pdf);
    denom += product * product;

    if(view_index > 1){
        double rev_pdf = prev_cosine / (2.0 * __PI__ * dist2);
        product *= rev_pdf / (view_SubPath[view_index - 1].pdf_forward);
        denom += product * product;
    }
    for(int i = view_index - 3; i >= 0; i--){
        product *= view_SubPath[i].pdf_reverse / view_SubPath[i].pdf_forward;
        denom += product * product;
    }

    return mole_sq * mole_sq / denom;
}

std::pair<double, double> Renderer::calcLightPDF(const Eigen::Vector3d &x, const Eigen::Vector3d &dir, const Object &in_Object,
                                                 const std::vector<LightSubPath> &light_SubPath, const double cosine_view_connect, const int index) {
    //光源側の接続点から、視点側の接続点に向かう方向ベクトルがdir
    LightSubPath _s = light_SubPath[index];
    RayHit _rh = _s.rh;

    double dist2_connection = (x - _s.x).squaredNorm();
    double forward, reverse;
    switch (_s.materialMode) {
        case 0: {
            forward = 1.0 / (2.0 * __PI__);
            reverse = 0.0;
            break;
        }
        case 1: {
            Eigen::Vector3d normal = computeRayHitNormal(in_Object, _rh);
            double kd = in_Object.meshes[_rh.mesh_idx].material.kd;
            forward = kd * getDiffuseProbability(normal, dir);
            reverse = kd * getDiffuseProbability(normal, -_s.in_dir) * light_SubPath[index - 1].cosine_o / _s.dist2;
            break;
        }
        case 2: {
            Eigen::Vector3d normal = computeRayHitNormal(in_Object, _rh);
            double m = in_Object.meshes[_rh.mesh_idx].material.m;
            double ks = in_Object.meshes[_rh.mesh_idx].material.ks;
            forward = ks * getBlinnPhongProbability(_s.in_dir, normal, dir, m);
            reverse = ks * getBlinnPhongProbability(-dir, normal, -_s.in_dir, m) * light_SubPath[index - 1].cosine_o / _s.dist2;
            break;
        }
    }

    forward *= cosine_view_connect / dist2_connection;
    return std::make_pair(forward, reverse);
}

#include <iostream>
#include <fstream>  
#include <sstream>
#include <filesystem>
#include <cstdio>
#include <random>
#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>
#include <numbers>

inline constexpr int image_width = 800;
inline constexpr int image_height = 400;
inline constexpr int sample_num = 100;
inline const std::string filename = "results/bptPower" + std::to_string(sample_num) + ".pfm";


inline constexpr float kPi = std::numbers::pi_v<float>;
inline constexpr float k2Pi = 2.0f * kPi;
inline constexpr float kEps = 1e-6f;
inline constexpr float kEpsShadow = 1e-4f;

inline float deg2rad(float d){ return d * (kPi / 180.0f);}
inline float rad2deg(float f){ return f * (180.0f / kPi);}

template<class T>
inline T clamp01(T x){ return std::min<T>(T(1), std::max<T>(T(0), x));}


// === 1. Vec3 & Math ===
struct Vec3 {
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    float length() const {return std::sqrt(x * x + y * y + z * z);}
    float length_squared() const {return x * x + y * y + z * z;}
    Vec3 normalize() const{
        float len = length();
        if(len == 0) return Vec3(0, 0, 0);
        return Vec3(x / len, y / len, z / len);
    }
    Vec3 operator+(const Vec3& other) const {return Vec3(x + other.x, y + other.y, z + other.z);}
    Vec3 operator-(const Vec3& other) const {return Vec3(x - other.x, y - other.y, z - other.z);}
    Vec3 operator-() const {return Vec3(-x, -y, -z);}
    Vec3 operator*(const float c) const {return Vec3(x * c, y * c, z * c);}
    Vec3 operator*(const Vec3& other) const {return Vec3(x * other.x, y * other.y, z * other.z);}
    Vec3 operator/(const float c) const {return Vec3(x / c, y / c, z / c);}
    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    Vec3& operator*=(const Vec3& other) {
        x *= other.x;
        y *= other.y;
        z *= other.z;
        return *this;
    }
    Vec3& operator/=(const float c) {
        x /= c;
        y /= c;
        z /= c;
        return *this;
    }
    bool operator==(const Vec3& other) const {return x == other.x && y == other.y && z == other.z;}

    Vec3 cross(const Vec3& other) const {
        return Vec3(y * other.z - z * other.y,
                    z * other.x - x * other.z,
                    x * other.y - y * other.x);
    }

    float dot(const Vec3& other) const {return float(x * other.x + y * other.y + z * other.z);}
    void print(const std::string s) const{std::cout << ""<< s <<" =(" << x << ", " << y << ", " << z << ")\n";}
    float max_component() const {return std::max({x, y, z});}
    bool is_zero(float eps = 1e-7f) const {return std::fabs(x) < eps && std::fabs(y) < eps && std::fabs(z) < eps;}

};

Vec3 reflect(const Vec3& V, const Vec3& N) {
    Vec3 R = V - N * 2.0f * V.dot(N);
    return R;
}

float angle_between(const Vec3& a, const Vec3& b) {
    float c = a.dot(b) / (a.length() * b.length());
    c = std::max(-1.0f, std::min(1.0f, c));
    float radians = std::acos(c);
    float degree = radians * (180.0f / kPi);
    return degree;
}

Vec3 half_vector(const Vec3& L, const Vec3& V) {return Vec3((L + V).normalize());}

Vec3 tangent_vector(const Vec3& N) {
    Vec3 A;

    if (std::abs(N.x) < 0.999f)
        A = Vec3(1.0, 0.0, 0.0);
    else
        A = Vec3(0.0, 1.0, 0.0);

    Vec3 T = N.cross(A).normalize();
    return T;
}

Vec3 operator-(float c, const Vec3& v) {return Vec3(c - v.x, c - v.y, c - v.z);}
Vec3 operator*(float c, const Vec3& v) {return Vec3(c * v.x, c * v.y, c * v.z);}

float random_float() {
    thread_local static std::mt19937 rng(
        std::random_device{}()
        // 123456789u /* でばっく用の固定値 */
    );
    thread_local static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    return dist(rng);
}

Vec3 random_in_unit_sphere() {
    while(true) {
        Vec3 p(
            random_float() * 2.0f - 1.0f,
            random_float() * 2.0f - 1.0f,
            random_float() * 2.0f - 1.0f
        );
        if (p.length_squared() < 1.0f) return p;
    }
}

Vec3 cos_weight_sampling() {
    float xi1 = random_float();
    float xi2 = random_float();

    float phi = k2Pi * xi1;
    float r = sqrt(xi2);

    float x = r * cos(phi);
    float y = r * sin(phi);
    float z = sqrt(1 - r * r);

    return Vec3(x, y, z);
}

// 上向き一様半球サンプリング
Vec3 uniform_hemisphere() {
    float xi1 = random_float();          // [0,1)
    float xi2 = random_float();
    float z   = xi1;                     // cosθ
    float r   = std::sqrt(std::max(0.f, 1.f - z*z));
    float phi = k2Pi * xi2;
    return Vec3(r * std::cos(phi), r * std::sin(phi), z);   // +z 方向が法線
}

Vec3 sample_light_sphere(const Vec3& center, float radians) {
    Vec3 dir = random_in_unit_sphere().normalize();
    return center + radians * dir;
}

Vec3 sample_light_rectangle(const Vec3& center, const Vec3& u, const Vec3& v) {
    float a = random_float() - 0.5f;
    float b = random_float() - 0.5f;

    return center + a * 2.0f * u + b * 2.0f * v;
}


// === 2. Ray & Camera ===
struct Ray {
    Vec3 origin;
    Vec3 direction;

    Ray(const Vec3& origin, const Vec3& direction)
    : origin(origin), direction(direction) {}

    Vec3 at(float t) const {return origin + direction * t;}
    
};
struct Camera {
    Vec3 pos;
    Vec3 lower_left_corner, horizontal, vertical;
    Vec3 u, v, w;
    int width, height;

    float viewport_width = 1.0f;
    float viewport_height = 1.0f;

    // lookFrom … カメラ位置
    // lookAt … 視線が向かう 1 点
    // vup … 画面の「上」方向（lookDir と直交させる）

    Camera(Vec3 lookFrom, Vec3 lookAt, Vec3 vup, float fov_dg, float aspect, int width, int height) 
       : width(width), height(height){
        float theta = deg2rad(fov_dg);
        float h = std::tan(theta / 2);
        viewport_height = 2.0f * h;
        viewport_width = aspect * viewport_height;

        w = (lookFrom - lookAt).normalize();
        u = vup.cross(w).normalize();
        v = w.cross(u);

        pos = lookFrom;
        horizontal = u * viewport_width;
        vertical = v * viewport_height;
        lower_left_corner = pos - horizontal * 0.5f - vertical * 0.5f - w;
    }

    [[nodiscard]] Ray get_ray(float s, float t) const {
        Vec3 dir = lower_left_corner + horizontal * s + vertical * t - pos;
        return Ray(pos, dir);
    }

    [[nodiscard]] bool project_dir(const Vec3& dir_world, float& s, float& t) const {
        float cos_theta = -dir_world.dot(w);
        if(cos_theta <= 0.0f) return false;

        Vec3 p = dir_world / cos_theta;
        s = 0.5f + p.dot(u) / viewport_width;
        t = 0.5f + p.dot(v) / viewport_height;

        return (s >= 0.0f && s < 1.0f && t >= 0.0f && t < 1.0f);
    }

    int image_width() const {return width;}
    int image_height() const {return height;}

};

// === 3. BRDFs ===
class BRDF {
public:
    virtual ~BRDF() = default;

    // 光の反射強度を返す
    // N: 法線ベクトル
    // L: ライト方向ベクトル（物体→光源）
    // V: 視線ベクトル（物体→カメラ）
    virtual Vec3 evaluate(const Vec3& N, const Vec3& L, const Vec3& V) const = 0;

    virtual float max_value() const {return 1.0f;}

    virtual Vec3 get_albedo() const { return Vec3(1.0f, 1.0f, 1.0f); }

    virtual Vec3 get_emission() const { return Vec3(0.0f, 0.0f, 0.0f); }

    virtual Vec3 sample(const Vec3& N, const Vec3& wo, float& pdf) const { return Vec3(1.0f, 1.0f, 1.0f); }

    virtual float pdf(const Vec3& N, const Vec3& wi) const {return 1.0f;}
};
class Emission : public BRDF{
public:
    Vec3 e;

    Emission(const Vec3& color)
        : e(color) {}

    Vec3 evaluate(const Vec3& N, const Vec3& L, const Vec3& V) const override {
        return Vec3(0, 0, 0);
    }

    Vec3 get_albedo() const override {
        return Vec3(0, 0, 0);
    }

    Vec3 get_emission() const override {
        return e;
    }

    Vec3 sample(const Vec3& N, const Vec3& wo, float& pdf) const override {
        return Vec3(0.0f, 0.0f, 0.0f);
    }

    float pdf(const Vec3& N, const Vec3& wi) const override {
        // return 0.0f;
        return std::max(0.0f, N.dot(wi)) / kPi;
    }
};
class LambertBRDF : public BRDF{
public:
    Vec3 cd; // 物体の拡散反射率

    LambertBRDF(const Vec3& color) : cd(color) {}

    Vec3 evaluate(const Vec3& N, const Vec3& L, const Vec3& V) const override {
        // float dotNL = std::max(N.dot(L), 0.0f);
        // return (cd / kPi) * dotNL;

        // LambertBRDF::evaluate: 反射率/πのみを返す（cos項はパス拡張側で掛ける方針）
        return cd / kPi;
    }

    Vec3 get_albedo() const override {
        return cd;
    }

    Vec3 sample(const Vec3& N, const Vec3& wo, float& pdf) const override {
        Vec3 dri_local = cos_weight_sampling();
        Vec3 T = tangent_vector(N);
        Vec3 B = N.cross(T);
        Vec3 wi = (T * dri_local.x + B * dri_local.y + N * dri_local.z).normalize();

        pdf = std::max(0.0f, N.dot(wi)) / kPi;

        return wi;
    }

    float pdf(const Vec3& N, const Vec3& wi) const override {
        return std::max(0.0f, N.dot(wi)) / kPi;
    }
};
class Sensor : public BRDF{
public:

    Vec3 evaluate(const Vec3& N, const Vec3& L, const Vec3& V) const override {
        // constexpr float EPS_DIR = 0.999f;
        // return (N.dot(L) > 0.0f && L.dot(V) > EPS_DIR)
        //          ? Vec3(1.0f, 1.0f, 1.0f) : Vec3(0.0f, 0.0f, 0.0f);

        return (N.dot(L) > 0.0f) ? Vec3(1.0f, 1.0f, 1.0f) : Vec3(0.0f, 0.0f, 0.0f);
    }

    Vec3 get_albedo() const override {
        return Vec3(0.0f, 0.0f, 0.0f);
    }

    Vec3 get_emission() const override {
        return Vec3(0.0f, 0.0f, 0.0f);;
    }

    Vec3 sample(const Vec3& N, const Vec3& wo, float& pdf) const override {
        pdf = 0.0f;
        return Vec3(0.0f, 0.0f, 0.0f);
    }

    float pdf(const Vec3& N, const Vec3& wi) const override {
        // return 0.0f;
        return std::max(0.0f, N.dot(wi)) / kPi;
    }
};

// === 4. Objects ===
class Object {
public:
    virtual ~Object() = default;

    // Rayとの交差判定
    virtual bool hit(const Ray& r, float t_min, float t_max, float& t_out) const = 0;

    // 表面上の法線ベクトル
    virtual Vec3 get_normal(const Vec3& hit_point) const = 0;

    virtual Vec3 get_center() const {return Vec3();}

    virtual float get_radius() const {return 0.0f;}

    virtual Vec3 get_u() const {return Vec3();}

    virtual Vec3 get_v() const {return Vec3();}

    virtual float get_area() const {return 1.0f;}

    // マテリアルへのアクセス
    virtual std::shared_ptr<BRDF> get_material() const = 0;

    // 発光体かどうか
    virtual bool is_light() const { return false; }
};
class Rectangle : public Object {
public:
    Vec3 center;        // 矩形の中心位置
    Vec3 u, v;          // u軸, v軸ベクトル（矩形の辺方向）
    Vec3 nu, nv;
    float half_u, half_v;
    Vec3 normal;        // 法線ベクトル（u×v）
    float width, height;
    std::shared_ptr<BRDF> material;
    bool light_flag;

    Rectangle(const Vec3& center, const Vec3& u_vec, const Vec3& v_vec,
        std::shared_ptr<BRDF> material, bool is_light = false)
        : center(center), u(u_vec), v(v_vec),
          width(u_vec.length() * 2.0f), height(v_vec.length() * 2.0f),
          material(material), light_flag(is_light)
    {
        nu = u.normalize();
        nv = v.normalize();
        half_u = u.length();
        half_v = v.length();
        normal = u.cross(v).normalize();
    }

    bool hit(const Ray& ray, float t_min, float t_max, float& t_out) const override {
        float denom = normal.dot(ray.direction);
        if (std::fabs(denom) < kEps) return false; // 平行なので交差しない

        float t = (center - ray.origin).dot(normal) / denom;
        if (t < t_min || t > t_max) return false;

        Vec3 p = ray.at(t);
        Vec3 d = p - center;

        float du = d.dot(nu);
        float dv = d.dot(nv);

        if (std::fabs(du) <= half_u && std::fabs(dv) <= half_v) {
            t_out = t;
            return true;
        }

        return false;
    }

    Vec3 get_normal(const Vec3& hit_point) const override {
        return normal;
    }

    std::shared_ptr<BRDF> get_material() const override {
        return material;
    }

    bool is_light() const override {
        return light_flag;
    }

    Vec3 get_center() const override {
        return center;
    }

    Vec3 get_u() const override {
        return u;
    }

    Vec3 get_v() const override {
        return v;
    }

    float get_area() const override {
        return 4.0f * half_u * half_v;
    }
};
class Sphere : public Object {
public:
    Vec3 center;
    float radius;
    std::shared_ptr<BRDF> material;
    bool light_flag;

    Sphere(const Vec3& c, float r, std::shared_ptr<BRDF> m, bool is_light = false)
        : center(c), radius(r), material(m), light_flag(is_light) {}

    bool hit(const Ray& r, float t_min, float t_max, float& t_out) const override {
        Vec3 oc = r.origin - center;
        float a = r.direction.dot(r.direction);
        float b = oc.dot(r.direction);
        float c = oc.dot(oc) - radius * radius;
        float discriminant = b * b - a * c;

        if (discriminant > 0) {
            float root = sqrt(discriminant);
            float temp = (-b - root) / a;
            if (temp < t_max && temp > t_min) {
                t_out = temp;
                return true;
            }
            temp = (-b + root) / a;
            if (temp < t_max && temp > t_min) {
                t_out = temp;
                return true;
            }
        }
        return false;
    }

    Vec3 get_normal(const Vec3& hit_point) const override {
        return (hit_point - center).normalize();
    }

    Vec3 get_center() const override {
        return center;
    }

    float get_radius() const override {
        return radius;
    }

    std::shared_ptr<BRDF> get_material() const override {
        return material;
    }

    bool is_light() const override {
        return light_flag;
    }
};

// === 5. Scene ===
Camera build_scene(const int w, const int h, std::vector<std::shared_ptr<Object>>& scene) {
    scene.reserve(20);
    float aspect = float(w) / (h);
    Camera camera(Vec3(0, 0, -3), Vec3(0, 0, 1), Vec3(0, 1, 0), 45.0f, aspect, w, h);
    // Camera camera(Vec3(0, 0, -2), Vec3(0, 1, 0), Vec3(0, 0, 1), 45.0f, aspect, w, h);
    // Camera camera(Vec3(0, 2, 0),    // y = +2 → 床 (y = -1) の 3 ユニット上空
    //           Vec3(0,-1, 0),    // 床の中心を注視
    //           Vec3(0, 0, 1),    // 画面上方向を +z に
    //           40.0f, aspect);
              


    auto red = std::make_shared<LambertBRDF>(Vec3(0.9f, 0.2f, 0.2f));
    auto blue = std::make_shared<LambertBRDF>(Vec3(0.2f, 0.2f, 0.9f));
    auto green = std::make_shared<LambertBRDF>(Vec3(0.3f, 0.9f, 0.3f));
    auto white = std::make_shared<LambertBRDF>(Vec3(0.8f, 0.8f, 0.8f));
    auto yellow = std::make_shared<LambertBRDF>(Vec3(0.9f, 0.9f, 0.3f));
    auto light_obj = std::make_shared<Emission>(Vec3(3.0f, 3.0f, 3.0f));
    // auto light_obj = std::make_shared<Emission>(Vec3(3.0f, 0.0f, 0.0f));


    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 1 - kEps, 0), Vec3(0.5, 0, 0), Vec3(0, 0, 0.5), light_obj, true));
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 0, 1), Vec3(0, 1, 0), Vec3(1.25, 0, 0), white)); // 前
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 0, -3), Vec3(1.25, 0, 0), Vec3(0, 1, 0), white)); // 後
    // scene.push_back(std::make_shared<Rectangle>(Vec3(0, 1, -1), Vec3(1.25, 0, 0), Vec3(0, 0, 2), white)); // 上
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, -1, -1), Vec3(0, 0, 2), Vec3(1.25, 0, 0), white)); // 下
    scene.push_back(std::make_shared<Rectangle>(Vec3(-1.25, 0, -1), Vec3(0, 1, 0), Vec3(0, 0, 2), red)); // 右
    scene.push_back(std::make_shared<Rectangle>(Vec3(1.25, 0, -1), Vec3(0, 0, 2), Vec3(0, 1, 0), blue)); // 左


    scene.push_back(std::make_shared<Rectangle>(Vec3(0.875, 1, 0), Vec3(0.375, 0, 0), Vec3(0, 0, 1.25), white));
    scene.push_back(std::make_shared<Rectangle>(Vec3(-0.875, 1, 0), Vec3(0.375, 0, 0), Vec3(0, 0, 1.25), white));
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 1, 0.75), Vec3(0.5, 0, 0), Vec3(0, 0, 0.25), white));
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 1, -0.75), Vec3(0.5, 0, 0), Vec3(0, 0, 0.5), white));
    scene.push_back(std::make_shared<Rectangle>(Vec3(0, 1, -2), Vec3(1.25, 0, 0), Vec3(0, 0, 1), white));



    scene.push_back(std::make_shared<Sphere>(Vec3(-0.5, -0.80, 0), 0.2, yellow));


    auto cube_mat = green;
    Vec3 cube_center(0.3f, -0.80f, 0.30f);
    float half = 0.20f;
    float theta = deg2rad(45.0f);
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
// === 6. Path Generators ===
struct PathVertex {
    Vec3 x; // 位置
    Vec3 N; // 法線
    Vec3 wi; //次へ進む方向
    Vec3 wo; // 来た方向
    std::shared_ptr<BRDF> brdf;
    float pdf_A = 1.0f; // 面積測度pdf
    float pdf_W = 1.0f; // 方向測度pdf(サンプリングに使った側)
    float pdf_rev = 1.0f; // 逆遷移の方向測度pdf
    Vec3 beta = Vec3(1.0f, 1.0f, 1.0f); // 通過スループット
    float vc = 0.0f, vcm = 0.0f; // MIS補助(Veachの定式化に対応)
    bool isDelta = false;   // サンプリングに使った BSDF が Dirac か
    bool is_light = false;
};

static bool bounce_walk(std::vector<PathVertex>& path, const std::vector<std::shared_ptr<Object>>& scene, bool isLight) {

    const int  kRRStartDepth  = 3;

    PathVertex& prev = path.back();

    float q = 1.0f;
    if(int(path.size()) - 1 >= kRRStartDepth){
        q = std::clamp(prev.beta.max_component(), 0.05f, 0.95f);
        if(random_float() > q) return false;
        // prev.beta /= q;
        prev.pdf_W *= q;
        // prev.pdf_rev *= q;
    }

    Ray ray(prev.x + prev.N * kEpsShadow, prev.wi);

    float closest_t = 1e30f;
    std::shared_ptr<Object> hit_obj = nullptr;
    for (const auto& obj : scene){
        float t; if(obj->hit(ray, kEpsShadow, closest_t, t)){ closest_t = t; hit_obj = obj; }
    }
    if (!hit_obj) return false;

    Vec3 x = ray.at(closest_t);
    Vec3 N = hit_obj->get_normal(x);
    Vec3 wo = -ray.direction.normalize();
    auto  brdf = hit_obj->get_material();

    if (path.size()>1 && hit_obj->is_light() && isLight) return false;

    float pdf_W;
    Vec3  wi   = brdf->sample(N, wo, pdf_W);
    float pdf_rev = brdf->pdf(N, wo);

    PathVertex vn;
    vn.x = x;
    vn.N = N;
    vn.wi = wi;
    vn.wo = wo;
    vn.brdf = brdf;
    vn.pdf_W = pdf_W;
    vn.pdf_rev = pdf_rev;
    if(hit_obj->is_light()) vn.pdf_A = 1.0f / hit_obj->get_area();
    vn.is_light = hit_obj->is_light();

    // ---- β 更新 ----
    Vec3 d = (vn.x - prev.x).normalize();
    float cosPrev = std::max(0.f, prev.N.dot(d));
    Vec3 fs = prev.is_light ? Vec3(1.0f, 1.0f, 1.0f) : prev.brdf->evaluate(prev.N, prev.wi, prev.wo);
    vn.beta = prev.beta * fs * cosPrev / std::max(prev.pdf_W, kEps);


    float p = 1.0f;
    if(int(path.size()) >= kRRStartDepth){
        p = std::clamp(vn.beta.max_component(), 0.05f, 0.95f);
        vn.pdf_rev *= p;
    }

    float ratio = (prev.pdf_W > kEps) ? prev.pdf_rev / prev.pdf_W : 0.f;
    vn.vc  = prev.vcm;
    vn.vcm = prev.vcm * ratio;

    path.push_back(vn);
    // int depth = int(path.size()) - 1;

    // if(depth >= kRRStartDepth){
    //     float q = std::clamp(vn.beta.max_component(), 0.0f, 1.0f); // 生存確率
    //     if(random_float() > q) return false;
    //     vn.beta /= q;
    //     prev.pdf_W *= q;
    //     // vn.pdf_rev *= q;
    // }

    if (pdf_W < kEps) return false;
    if (path.size()>1 && vn.is_light && !isLight) return false;
    return true;
}

std::vector<PathVertex> generate_camera_subpath(const Camera& camera, const std::vector<std::shared_ptr<Object>>& scene, float u, float v, int depth) {
    std::vector<PathVertex> path;
    if (depth == 0) return path;

    PathVertex v0;
    Vec3 dir = camera.get_ray(u, v).direction;
    float dist2 = dir.length_squared();
    v0.x = camera.pos;
    v0.N = -camera.w;
    v0.wi = dir.normalize();
    v0.wo = v0.wi;
    v0.brdf = std::make_shared<Sensor>();
    float cos_theta = std::max(0.0f, v0.N.dot(dir / std::sqrt(dist2)));
    float A_img     = camera.viewport_width * camera.viewport_height;
    float pdf_dir   = (cos_theta > 0.0f) ? 1.0f / (cos_theta * cos_theta * cos_theta * A_img) : 0.0f;

    v0.pdf_A = 1.0f / A_img;
    // float pdf_dir   = v0.pdf_A * dist2 / (cos_theta * cos_theta * cos_theta);
    v0.pdf_W = cos_theta;
    v0.pdf_rev = pdf_dir;
    // v0.beta /= v0.pdf_W;
    v0.beta = Vec3(1.0f, 1.0f, 1.0f);
    v0.vc = 0.0f;
    // v0.vcm = 1.0f;
    v0.vcm = A_img / v0.pdf_W;
    Vec3 l = v0.beta * v0.pdf_W;
    // std::cout << "test = ("<< l.x <<", "<< l.y <<", "<< l.z <<")\n";
    v0.is_light = false;
    path.push_back(v0);

    Ray ray(v0.x, v0.wo);

    while(int(path.size())<depth)
        if(!bounce_walk(path,scene,false)) break;

    return path;
}

std::vector<PathVertex> generate_light_subpath(const std::vector<std::shared_ptr<Object>>& scene, int depth) {
    std::vector<PathVertex> path;
    if (depth == 0) return path;

    std::vector<std::shared_ptr<Object>> lights;
    for(const auto& obj : scene) {
        if(obj->is_light()) lights.push_back(obj);
    }
    if(lights.empty()) return path;

    auto rect = std::dynamic_pointer_cast<Rectangle>(lights[0]);

    Vec3 L0 = sample_light_rectangle(rect->get_center(), rect->get_u(), rect->get_v());
    Vec3 nL = rect->get_normal(L0);

    // Vec3 dir_local = uniform_hemisphere();
    Vec3 dir_local = cos_weight_sampling();
    Vec3 T = tangent_vector(nL);
    Vec3 B = nL.cross(T);
    Vec3 wi = (T * dir_local.x + B * dir_local.y + nL * dir_local.z).normalize();

    // float pdf_dir = 1.0f / k2Pi;
    float pdf_dir = std::max(0.0f, nL.dot(wi)) / kPi;

    PathVertex v0;
    v0.x = L0;
    v0.N = nL;
    v0.wi = wi;
    v0.wo = wi;
    v0.brdf = rect->get_material();
    v0.pdf_A = 1.0f / rect->get_area();
    v0.pdf_W = pdf_dir;
    v0.pdf_rev = pdf_dir;
    v0.beta = Vec3(0.0f, 0.0f, 0.0f);
    v0.vc = 0.0f;
    v0.vcm = 1.0f;
    v0.is_light = true;
    float selPdf = 1.0f; // numLights（将来拡張用）
    float emissionPdf = v0.pdf_A*pdf_dir*selPdf; // 未使用
    float cos0 = std::max(0.0f, v0.N.dot(v0.wi));
    v0.beta = rect->get_material()->get_emission() / std::max(v0.pdf_A, kEps);
    path.push_back(v0);

    Ray ray(L0 + nL * kEpsShadow, wi);

    while(int(path.size())<depth)
        if(!bounce_walk(path,scene,true)) break;

    return path;
}

// === 7. Connect & MIS ===
Vec3 connect_vertices(const PathVertex& vc, const PathVertex& vl, const std::vector<std::shared_ptr<Object>>& scene) {

    Vec3 shadow_origin = vc.x + vc.N * kEpsShadow;
    Vec3 dir = vl.x - shadow_origin;
    float dist2 = dir.length_squared();
    float dist = std::sqrt(dist2);
    Vec3 wi = dir / dist;

    Ray Shadow_ray(shadow_origin, wi);
    float t_max = dist - kEpsShadow;
    for(const auto& obj : scene) {
        float t;
        if(obj->hit(Shadow_ray, kEpsShadow, t_max, t)) {
            return Vec3(0.0f, 0.0f, 0.0f);
        }
    }

    //BRDF Geomotry
    float cos_c = std::max(0.0f, vc.N.dot(wi));
    float cos_l = std::max(0.0f, vl.N.dot(-wi));
    float G = cos_c * cos_l / dist2;

    Vec3 fs_cam = vc.brdf->evaluate(vc.N, wi, vc.wo);
    Vec3 fs_lig = vl.is_light ? Vec3(1.0f, 1.0f, 1.0f) : vl.brdf->evaluate(vl.N, -wi, vl.wo);

    Vec3 contribute = fs_cam * fs_lig * G;

    // std::cout << "fs_cam = ("<< fs_cam.x <<", "<< fs_cam.y <<", "<< fs_cam.z <<")\n";
    // std::cout << "fs_lig = ("<< fs_lig.x <<", "<< fs_lig.y <<", "<< fs_lig.z <<")\n";
    // std::cout << "countribute = ("<< contribute.x <<", "<< contribute.y <<", "<< contribute.z <<")\n";

    return contribute;

}

float mis_power_heuristic(const std::vector<float>& pdfs, int i, float beta = 2.f){
    if(pdfs.empty()) return 0.f;

    /* ① 有効な最大値を取り出して 1 以下へ正規化 */
    float max_pdf = 0.f;
    for(float p : pdfs)
        if(std::isfinite(p) && p > max_pdf) max_pdf = p;
    if(max_pdf <= 0.f) return 0.f;

    /* ② 正規化 → pow → 加算。これで pow が inf になることはない */
    float num = 0.f, denom = 0.f;
    for(size_t k = 0; k < pdfs.size(); ++k)
    {
        float w = pdfs[k] / max_pdf;                // 0–1 に収まる
        if(!std::isfinite(w) || w <= 0.f) continue; // NaN, inf, 0 は無視
        w = std::pow(w, beta);

        if(int(k) == i) num = w;
        denom += w;
    }
    return (denom > 0.f) ? num / denom : 0.f;
}

float simple_mis(int s, int t) {return 1.0f / float(s + t - 1);}

inline void debug_mis_sum(const std::vector<float>& pdfs, float beta = 2.f, const char* tag = ""){
    float sum = 0.f;
    for(size_t k = 0; k < pdfs.size(); ++k)
        sum += mis_power_heuristic(pdfs, static_cast<int>(k), beta);

    if(std::fabs(sum - 1.f) > kEpsShadow)
        std::fprintf(stderr,
            "[MIS-SUM] %s Σw = %.9f (|err| = %.3e, n = %zu)\n",
            tag, sum, std::fabs(sum - 1.f), pdfs.size());
}

inline std::vector<PathVertex> make_full_path(const std::vector<PathVertex>& c_path, const std::vector<PathVertex>& l_path) {
    std::vector<PathVertex> full;
    full.reserve(c_path.size() + l_path.size());

    /* 1) カメラ側をそのまま先頭へ  x0, x1, …, xs-1 */
    full.insert(full.end(), c_path.begin(), c_path.end());

    /* 2) 光源側は後ろに逆順で追加  yt-1, …, y1, y0  */
    for(auto it = l_path.rbegin(); it != l_path.rend(); ++it)
        full.push_back(*it);

    // /* 3) 各頂点の wi / wo を“つなぎ目”に合わせて再設定
    //       ─ wi: 次頂点へ進む方向
    //       ─ wo: 前頂点（=カメラ側）へ向かう方向 */
    // for(std::size_t i = 0; i + 1 < full.size(); ++i){
    //     Vec3 dir = (full[i+1].x - full[i].x).normalize(); // i → i+1
    //     full[i].wi     = dir;
    //     full[i+1].wo   = -dir;
    // }
    return full;
}

inline float dir2area(const float pdf, const PathVertex& v_from, const PathVertex& v_to) {
    Vec3  d      = v_to.x - v_from.x;
    float dist2  = d.length_squared();
    if(dist2 == 0.0f) return 0.0f;
    float dist   = std::sqrt(dist2);
    Vec3  dir    = d / dist;
    // float cos_v  = std::max(0.f, v_from.N.dot(dir));
    float cos_v  = std::max(0.f, v_to.N.dot(-dir));
    return pdf * cos_v / dist2;
}

inline float cal_pdf(const PathVertex& v_from, const PathVertex& v_to){
    Vec3 wi = (v_to.x - v_from.x).normalize();
    float pdf = v_from.brdf->pdf(v_from.N, wi);
    // float pdf = std::max(0.0f, v_from.N.dot(wi)) / kPi;

    // std::cout << "wi : (" << wi.x << ", " << wi.y << ", " << wi.z << ")\n"; 
    // std::cout << "N : (" << v_from.N.x << ", " << v_from.N.y << ", " << v_from.N.z << ")\n"; 
    // std::cout << "pdf W : " << pdf << "\n";
    // std::cout << "brdf : " << v_from.brdf->pdf(v_from.N, wi) << "\n";

    return pdf;
}

struct StrategyPDF { 
    int s,t; 
    float pdf; 
};

std::vector<StrategyPDF> compute_strategy_pdfs(const std::vector<PathVertex>& cp, const std::vector<PathVertex>& cl, int s, int t) {

    const int k = static_cast<int>(cp.size() + cl.size()) - 1;
    std::vector<StrategyPDF> out;
    out.reserve(s + t);
    // if(s  + t != cp.size() + cl.size()) { 
    // if(s  != (int)cp.size() || t != (int)cl.size()) {
    //     std::cout << "s + t != path.size";
    //     return out;
    // }

    auto camera2light=[&](int a,int b){
        float p = 1.0f;
        for(int i=a;i<=b;++i){
            // float t = dir2area(cp[i].pdf_W, cp[i], cp[i+1]);
            float t = 1.0f;
            if(i==0) t = dir2area(cp[0].pdf_rev, cp[i], cp[i+1]);
            else t = dir2area(cp[i].pdf_W, cp[i], cp[i+1]);
            // std::cout << "camera : x" << i << " → x" << i+1 << " : " << t << "\n";
            p *= t;
        }
        return p;
    };
    auto light2camera=[&](int a,int b){
        float p = 1.0f;
        for(int i=a;i<=b;++i){
            float t=dir2area(cl[i].pdf_W, cl[i], cl[i+1]);
            // std::cout << "light : y" << i << " → y" << i+1 << " : " << t << "\n";
            // std::cout <<  i << "pdf : (" << cl[i].pdf_W << ")" << "\n";
            // std::cout << "(" << cl[i].x.x << ", " << cl[i].x.y << ", " << cl[i].x.z << ")" << "\n";
            // std::cout << "(" << cl[i+1].x.x << ", " << cl[i+1].x.y << ", " << cl[i+1].x.z << ")" << "\n";
            p *= t;
        }
        return p;
    };

    float Pst = 1.0f;
    if(s>0) Pst *= cp[0].pdf_A;
    if(s-2>=0) Pst *= camera2light(0,s-2);
    if(t>0) Pst *= cl[0].pdf_A;
    if(t-2>=0) Pst *= light2camera(0,t-2);
        
    out.push_back({s,t,Pst}); 

    //  光源側へ 
    int cur_s=s, cur_t=t; float P=Pst;
    while(cur_t>0){
        if(cur_s == s) {
            P   *= dir2area(cal_pdf(cp[cur_s-1], cl[cur_t-1]), cp[cur_s-1], cl[cur_t-1]);
            // std::cout << "light : x" << cur_s-1 << "→ y" << cur_t-1 << " : " << cal_pdf(cp[cur_s-1], cl[cur_t-1]) << "\n";
            // std::cout << "light : x" << cur_s-1 << "→ y" << cur_t-1 << " : " << dir2area(cal_pdf(cp[cur_s-1], cl[cur_t-1]), cp[cur_s-1], cl[cur_t-1]) << "\n";
        }
        else{
            if(cur_t>0) P *= dir2area(cl[cur_t].pdf_rev, cl[cur_t], cl[cur_t-1]);
            // std::cout << "light : y" << cur_t << ", y" << cur_t-1 << " : " << P << "\n";
            // std::cout << "(" << cl[cur_t].x.x << ", " << cl[cur_t].x.y << ", " << cl[cur_t].x.z << ")" << " → ";
            // std::cout << "(" << cl[cur_t-1].x.x << ", " << cl[cur_t-1].x.y << ", " << cl[cur_t-1].x.z << ") : " << cl[cur_t].pdf_rev << " → " << dir2area(cl[cur_t].pdf_rev, cl[cur_t], cl[cur_t-1]) << "\n";
        }
        ++cur_s; --cur_t;
        if(cur_t>0)   P /= dir2area(cl[cur_t-1].pdf_W, cl[cur_t-1], cl[cur_t]);
        else P /= cl[0].pdf_A; //t=0
        out.push_back({cur_s, cur_t, P});
    }

    // カメラ側へ
    cur_s=s, cur_t=t; P=Pst;
    while(cur_s>2){
        if(t == 0) {
            if(cur_t == t){
                P *= cp[s-1].pdf_A;
                // std::cout << "x" << s-1 << " pdfA : " << cp[s-1].pdf_A <<std::endl;
            }
            else {
                if(cur_s>0) P *= dir2area(cp[cur_s].pdf_rev, cp[cur_s], cp[cur_s-1]);
                // std::cout << "light : x" << cur_s << "→ x" << cur_s-1 << " : " << dir2area(cp[cur_s].pdf_rev, cp[cur_s], cp[cur_s-1]) << "\n";
                // std::cout << cp[cur_s].pdf_rev << std::endl;
                // std::cout << "(" << cp[cur_s].x.x << ", " << cp[cur_s].x.y << ", " << cp[cur_s].x.z << ")" << " → ";
                // std::cout << "(" << cp[cur_s-1].x.x << ", " << cp[cur_s-1].x.y << ", " << cp[cur_s-1].x.z << ")" << "\n";
            }
            
            ++cur_t; --cur_s;
            if(cur_s>0){
                if(cur_s-1 == 0) P /= dir2area(cp[0].pdf_rev, cp[cur_s-1], cp[cur_s]);
                else P /= dir2area(cp[cur_s-1].pdf_W, cp[cur_s-1], cp[cur_s]);
            }
            else P /= cp[0].pdf_A; //s=0
            out.push_back({cur_s, cur_t, P});
        }
        else{
            if(cur_t == t) {
                P   *= dir2area(cal_pdf(cl[cur_t-1], cp[cur_s-1]), cl[cur_t-1], cp[cur_s-1]);
                // std::cout << "light : y" << cur_t-1 << ", x" << cur_s-1 << " : " << cal_pdf(cl[cur_t-1], cp[cur_s-1]) << "\n";
                // std::cout << "light : y" << cur_t-1 << ", x" << cur_s-1 << " : " << dir2area(cal_pdf(cl[cur_t-1], cp[cur_s-1]), cl[cur_t-1], cp[cur_s-1]) << "\n";
            }
            else{
                if(cur_s>0) P *= dir2area(cp[cur_s].pdf_rev, cp[cur_s], cp[cur_s-1]);
                // std::cout << "light : x" << cur_s << ", x" << cur_s-1 << " : " << P << "\n";
                // std::cout << "(" << cp[cur_s].x.x << ", " << cp[cur_s].x.y << ", " << cp[cur_s].x.z << ")" << " → ";
                // std::cout << "(" << cp[cur_s-1].x.x << ", " << cp[cur_s-1].x.y << ", " << cp[cur_s-1].x.z << ")" << "\n";
            }
            ++cur_t; --cur_s;
            if(cur_s>0)   P /= dir2area(cp[cur_s-1].pdf_W, cp[cur_s-1], cp[cur_s]);
            else P /= cp[0].pdf_A; //s=0
            out.push_back({cur_s, cur_t, P});
        }

    }


    return out;
}

Vec3 bdpt_render(const Camera& camera, const std::vector<std::shared_ptr<Object>>& scene, float u, float v, float& mis_error) {

    const int MaxDepth = 1000;
    auto l_path = generate_light_subpath(scene, MaxDepth - 1);

    auto c_path = generate_camera_subpath(camera, scene, u, v, MaxDepth - 1);

    Vec3 radiance(0.0f, 0.0f, 0.0f);

    int sMax = (int)c_path.size();
    int tMax = (int)l_path.size();
    // std::cout << "sMax " << sMax << std::endl;
    // std::cout << "tMax " << tMax << std::endl << std::endl;

    for(int s=2; s<=sMax; ++s){
        const PathVertex& vc = c_path[s-1];

        for(int t=0; t<=tMax; ++t){
            Vec3 contrib;

            if (t == 0) {
                const auto& vl = c_path[s - 1];
                if (s <= c_path.size() && vl.is_light){
                    contrib = vl.beta * vl.brdf->get_emission();
                }
            }
            else{
                const PathVertex& vl = l_path[t-1];
                Vec3 G = connect_vertices(vc, vl, scene);
                if(G.is_zero()) continue;

                contrib = vc.beta * vl.beta * G;
            }

            std::vector<PathVertex> cp_part(c_path.begin(), c_path.begin()+s);
            std::vector<PathVertex> lp_part(l_path.begin(), l_path.begin()+t);


            auto pdfs_st = compute_strategy_pdfs(cp_part, lp_part, s, t);


            std::vector<float> pdfs;  pdfs.reserve(pdfs_st.size());
            int idx = -1;
            for(size_t i=0;i<pdfs_st.size();++i){
                pdfs.push_back(pdfs_st[i].pdf);
                if(pdfs_st[i].s == s && pdfs_st[i].t == t) idx = int(i);
            }
            if(idx < 0) continue;
            float w = mis_power_heuristic(pdfs, idx, 2.0f);
            // float w = simple_mis(s, t);


            // float sum_w = 0.0f;
            // for(size_t k=0; k<pdfs.size(); ++k)
            //     sum_w += mis_power_heuristic(pdfs, int(k), 2.0f);   // 全ストラテジ分

            // mis_error += std::fabs(sum_w - 1.0f);  // ★ ここで誤差だけ蓄積

            radiance += contrib * w;
        }

    }

    return radiance;
}

Vec3 bdpt_render_debug(const Camera& camera, const std::vector<std::shared_ptr<Object>>& scene, float u, float v, int ss, int tt) {

    // int max_depth = std::max(ss, tt) + 2;
    auto c_path = generate_camera_subpath(camera, scene, u, v, ss);
    auto l_path = generate_light_subpath(scene, tt);


    Vec3 radiance(0.0f, 0.0f, 0.0f);


    size_t s = ss;
    size_t t = tt;
    if (s == 0 && t == 0) return radiance;
    else if (s == 0 && t > 0) return radiance;
    else if (s == 1 && t > 0){
        if (t <= l_path.size()) {
            const auto& vc = c_path[0];
            const auto& vl = l_path[t-1];
            Vec3 G  = connect_vertices(vc, vl, scene);
            radiance = vc.beta * vl.beta * G;
        }
    }
    else if (t == 0 && s > 0) {
        const auto& vl = c_path[s - 1];
        if (s <= c_path.size() && vl.is_light){
            const auto& vl = c_path[s - 1];
            const auto& vc = c_path[s - 2];
            // if(s == 2) radiance = vl.brdf->get_emission();
            radiance = vl.beta * vl.brdf->get_emission();
        }
    }
    else if (s > 0 && t > 0) {
        if (s <= c_path.size() && t <= l_path.size()) {
            const auto& vc = c_path[s - 1];
            const auto& vl = l_path[t - 1];
            Vec3 G = connect_vertices(vc, vl, scene);
        
            // Vec3 contribute = beta_c[s - 1] * beta_l[t - 1] * G;
            Vec3 contribute = vc.beta * vl.beta * G;

            // /* ---------- MIS ウェイト ---------- */
            // size_t k = s + t;
            // std::vector<float> pdfs;
            // for (size_t sp = 1; sp < k; ++sp) {
            //     size_t tp = k - sp;
            //     if (sp <= c_path.size() && tp <= l_path.size())
            //         pdfs.push_back(path_pdf_area(c_path, l_path, sp, tp));
            //     else
            //         pdfs.push_back(0.0f);
            // }
            // float w = mis_power_heuristic(pdfs, s - 1, 2.0f);   // β = 2

            radiance = contribute;
        }

    }

    return radiance;
}

inline void accumulate_light_only(const std::vector<PathVertex>& l_path, const std::vector<Vec3>& beta_l, const Camera& camera, int W, int H, std::vector<Vec3>& fb, int t) {
    if(t < 2 || l_path.size() < 2) return;
    int idx = t - 2;
    if(idx >= l_path.size()) return;
    const auto& v = l_path[idx];

    Vec3 dir = v.x - camera.pos;
    float dist2 = dir.length_squared();
    float dist = std::sqrt(dist2);
    dir /= dist;

    float s_img, t_img;
    if(!camera.project_dir(dir, s_img, t_img)) return;


    int px = int(s_img * W);
    int py = int(t_img *(H - 1));
    if(px < 0 || px >= W || py < 0 || py >= H) return;

    float cos_cam = std::max(0.0f, (-camera.w).dot(dir));
    float cos_l = std::max(0.0f, v.N.dot(-dir));
    if(cos_cam == 0.0f || cos_l == 0.0f) return;

    float G = (cos_cam * cos_l) / dist2;
    Vec3 fs = v.is_light ? v.brdf->get_emission() : v.brdf->evaluate(v.N, -dir, v.wi);
    Vec3 contribute = beta_l[idx] * G * fs;
    fb[py * W + px] += contribute;
}

inline void accumulate_eye_only(const Camera& camera, const std::vector<PathVertex>& l_path, const std::vector<Vec3>& beta_l, int t, int W, int H, std::vector<Vec3>& fb) {
    if (t < 2 || l_path.size() < (size_t)t) return;

    const auto& v = l_path[t - 1];
    Vec3 dir = (v.x - camera.pos).normalize();

    float s_img, t_img;
    if (!camera.project_dir(dir, s_img, t_img)) return;

    int px = int(s_img * W);
    int py = int(t_img * (H - 1));
    if (px < 0 || px >= W || py < 0 || py >= H) return;

    // Geometry term G(ys−1, z0)
    float dist2   = (v.x - camera.pos).length_squared();
    float cos_cam = std::max(0.0f, (-camera.w).dot(dir));
    float cos_l   = std::max(0.0f, v.N.dot(-dir));
    if (cos_cam == 0.0f || cos_l == 0.0f) return;
    float G = cos_cam * cos_l / dist2;

    Vec3 fs = v.is_light ? v.brdf->get_emission() : v.brdf->evaluate(v.N, -dir, v.wi);

    Vec3 contrib = beta_l[t - 1] * fs * G;

    fb[py * W + px] += contrib;
}


// === 8. Utility ===
inline void write_pfm(const std::string& filename, const std::vector<float>& data, int width, int height) {

    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs) throw std::runtime_error("cannot open " + filename);
    ofs << "PF\n" << width << ' ' << height << "\n-1.0\n";

    ofs.write(reinterpret_cast<const char*>(data.data()), sizeof(float) * data.size());
}



// === 9. main() ===
#ifdef _OPENMP
  #include <omp.h>
  #pragma omp declare reduction( vec3_plus : Vec3 : omp_out += omp_in ) initializer( omp_priv = Vec3(0.0f, 0.0f, 0.0f) )
#else
  // 非OpenMP環境向けのダミー　pragmaは無視されるので何もしない
#endif
int main() {

    std::vector<std::shared_ptr<Object>> scene;
    Camera camera = build_scene(image_width, image_height, scene);

    std::filesystem::create_directories("results");

    std::vector<Vec3> framebuffer(image_width * image_height, Vec3(0.0f, 0.0f, 0.0f));
    std::vector<float> mis_err(image_width * image_height, 0.0f);
    
    #pragma omp parallel for collapse(2) schedule(static)
    for(int j = image_height - 1; j >= 0; --j) {
        for(int i = 0; i < image_width; ++i) {
            Vec3 color(0.0f, 0.0f, 0.0f);
            float mis_error = 0.0f;

            #pragma omp simd reduction(vec3_plus:color)
            for(int sample = 0; sample < sample_num; sample++) {
                float u = (i + random_float()) / (image_width - 1);
                float v = (j + random_float()) / (image_height - 1);
                
                color += bdpt_render(camera, scene, u, v, mis_error);
            }
            color /= float(sample_num);
            mis_error /= float(sample_num);
            
            framebuffer[j * image_width + i] += color;
            mis_err[j * image_width + i] = mis_error;
        }
    }

    std::vector<float> pixels(image_width * image_height * 3);
    for (int y = 0; y < image_height; ++y) {
        for (int x = 0; x < image_width; ++x) {
            Vec3 color = framebuffer[y * image_width + x];
            // Vec3 color = framebuffer[y * image_width + x] / float(sample_num);
            size_t idx = (y * image_width + x) * 3;
            pixels[idx + 0] = color.x; 
            pixels[idx + 1] = color.y; 
            pixels[idx + 2] = color.z;
        }
    }

    write_pfm(filename, pixels, image_width, image_height);

    std::cout << "Render finished.\n";

    return 0;
}

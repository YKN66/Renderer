#include "Vec3.h"
#include <cmath>

Vec3::Vec3() : x(0), y(0), z(0) {}

Vec3::Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

float Vec3::length() const {
    return std::sqrt(x * x + y * y + z * z);
}

float Vec3::length_squared() const {
    return x * x + y * y + z * z;
}

Vec3 Vec3::normalize() const{
    float len = length();
    if(len == 0) return Vec3(0, 0, 0);
    return Vec3(x / len, y / len, z / len);
}

bool Vec3::operator==(const Vec3& other) const {
    return x == other.x && y == other.y && z == other.z;
}

Vec3 Vec3::operator+(const Vec3& other) const {
    return Vec3(x + other.x, y + other.y, z + other.z);
}

Vec3 Vec3::operator-(const Vec3& other) const {
    return Vec3(x - other.x, y - other.y, z - other.z);
}

Vec3 Vec3::operator-() const {
    return Vec3(-x, -y, -z);
}

Vec3 Vec3::operator*(const float c) const {
    return Vec3(x * c, y * c, z * c);
}

Vec3 Vec3::operator*(const Vec3& other) const {
    return Vec3(x * other.x, y * other.y, z * other.z);
}

Vec3 Vec3::operator/(const float c) const {
    return Vec3(x / c, y / c, z / c);
}

Vec3& Vec3::operator+=(const Vec3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Vec3& Vec3::operator*=(const Vec3& other) {
    x *= other.x;
    y *= other.y;
    z *= other.z;
    return *this;
}

Vec3& Vec3::operator/=(const float c) {
    x /= c;
    y /= c;
    z /= c;
    return *this;
}

Vec3 Vec3::cross(const Vec3& other) const {
    return Vec3(y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x);
}

float Vec3::dot(const Vec3& other) const {
    return float(x * other.x + y * other.y + z * other.z);
}

void Vec3::print(const std::string s) const{
    std::cout << ""<< s <<" =(" << x << ", " << y << ", " << z << ")\n";
}

Vec3 reflect(const Vec3& V, const Vec3& N) {
    Vec3 R = V - N * 2.0f * V.dot(N);
    return R;
}

float angle_between(const Vec3& a, const Vec3& b) {
    float radians = acos(a.dot(b) / (a.length() * b.length()));
    float degree = radians * (180.0f / static_cast<float>(M_PI));
    return degree;
}

Vec3 half_vector(const Vec3& L, const Vec3& V) {
    return Vec3((L + V).normalize());
}

Vec3 tangent_vector(const Vec3& N) {
    Vec3 A;

    if (std::abs(N.x) < 0.999f)
        A = Vec3(1.0, 0.0, 0.0);
    else
        A = Vec3(0.0, 1.0, 0.0);

    Vec3 T = N.cross(A).normalize();
    return T;
}

Vec3 operator-(float c, const Vec3& v) {
    return Vec3(c - v.x, c - v.y, c - v.z);
}

Vec3 operator*(float c, const Vec3& v) {
    return Vec3(c * v.x, c * v.y, c * v.z);
}

std::mt19937 rng(std::random_device{}());
std::uniform_real_distribution<float> dist(0.0f, 1.0f);

float random_float() {
    return dist(rng);
}

Vec3 random_in_unit_sphere() {
    while(true) {
        Vec3 p(
            random_float() * 2.0f - 1.0f,
            random_float() * 2.0f - 1.0f,
            random_float() * 2.0f - 1.0f
        );
        if (p.length_squared() < 1.0f)
            return p;
    }
}

Vec3 cos_weight_sampling() {
    float xi1 = random_float();
    float xi2 = random_float();

    float phi = 2 * M_PI * xi1;
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
    float phi = 2.0f * M_PI * xi2;
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
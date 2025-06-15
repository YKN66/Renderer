#include <iostream>
#include <random>
#ifndef VEC3_H
#define VEC3_H

class Vec3 {
public:
    float x, y, z;

    Vec3();
    Vec3(float x, float y, float z);

    float length() const;
    float length_squared() const;
    Vec3 normalize() const;
    Vec3 operator+(const Vec3& other) const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator-() const;
    Vec3 operator*(const float c) const;
    Vec3 operator*(const Vec3& other) const;
    Vec3 operator/(const float c) const;
    Vec3& operator+=(const Vec3& other);
    Vec3& operator*=(const Vec3& other);
    Vec3& operator/=(const float c);
    Vec3 cross(const Vec3& other) const;
    float dot(const Vec3& other) const;

    void print(const std::string s) const;
    bool operator==(const Vec3& other) const;

};

Vec3 reflect(const Vec3& V, const Vec3& N);
float angle_between(const Vec3& a, const Vec3& b);
Vec3 half_vector(const Vec3& L, const Vec3& V);
Vec3 tangent_vector(const Vec3& N);

Vec3 operator-(float c, const Vec3& v);
Vec3 operator*(float c, const Vec3& v);

float random_float();
Vec3 random_in_unit_sphere();
Vec3 cos_weight_sampling();
Vec3 uniform_hemisphere();

Vec3 sample_light_sphere(const Vec3& center, float radians);
Vec3 sample_light_rectangle(const Vec3& center, const Vec3& u, const Vec3& v);

#endif
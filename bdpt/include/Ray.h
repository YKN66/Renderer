#ifndef RAY_H
#define RAY_H

#include "Vec3.h"

class Ray {
public:
    Vec3 origin;
    Vec3 direction;

    Ray(const Vec3& origin, const Vec3& direction);

    Vec3 at(float t) const;
    
};

#endif
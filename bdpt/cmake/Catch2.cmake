#include <iostream>
#include "Vec3.h"

int main() {
    Vec3 a(3, 3, 3);
    Vec3 b(1, 1, 1);

    Vec3 c = a.operator+(b);
    Vec3 d = c.normalize();
    Vec3 e = a.operator-(b);
    Vec3 f = a.operator*(3);

    std::cout << "c = ("<< c.x <<", "<< c.y <<", "<< c.z <<")\n";
    std::cout << "normalize = ("<< d.x <<", "<< d.y <<", "<< d.z <<")\n";
    std::cout << "N = "<<d.x * d.x + d.y * d.y + d.z * d.z<<"\n";
    std::cout << "e = ("<< e.x <<", "<< e.y <<", "<< e.z <<")\n";
    std::cout << "f = ("<< f.x <<", "<< f.y <<", "<< f.z <<")\n";
}

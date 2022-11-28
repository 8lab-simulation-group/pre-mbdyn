#include "Vec3d.h"

int main() {
    Vec3d x(1.,2.,3.);
    std::cout<<x<<std::endl;
    std::cout<<x/10.0<<std::endl;

    const Vec3d a(1,2,3);
    double n = a.norm();
    std::cout<<n<<std::endl;
    Vec3d b;
    b=a;
    return 0;
}
#include "Ray.h"

Ray::Ray(Vec3 orig, Vec3 dirc) : rayOrig(orig), rayDirc(dirc) {}

Ray::Ray(Vec3 dirc) : rayOrig({ 0, 0, 0 }), rayDirc(dirc) {}




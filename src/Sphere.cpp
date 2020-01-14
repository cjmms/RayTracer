#include "Sphere.h"

Sphere::Sphere(Color color, Vec3 origin, float radius)
    :radius(radius)
{
    this->color = color;
    this->origin = origin;
}


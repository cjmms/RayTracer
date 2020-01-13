#include "Sphere.h"

Sphere::Sphere(Color const& color, float x, float y, float z, float radius)
    :x(x), y(y), z(z), radius(radius)
{
    this->color = { color.r, color.g, color.b };
    if (radius <= 0) this->radius = 1.0f;   
}
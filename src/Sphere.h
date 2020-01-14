#pragma once

#include <vector>
#include "Ray.h"


#if defined __linux__ || defined __APPLE__ 
#else 
// Windows doesn't define these values by default, Linux does
#define M_PI 3.141592653589793 
#define INFINITY 1e8 
#endif 

struct Vec3;

struct Color {
	float r;
	float g;
	float b;
};

class Sphere {

public:	
	Color color;
	Vec3 origin;
	float radius;

	Sphere(Color color, Vec3 origin, float radius);
};
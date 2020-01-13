#pragma once

#include <vector>

#if defined __linux__ || defined __APPLE__ 
#else 
// Windows doesn't define these values by default, Linux does
#define M_PI 3.141592653589793 
#define INFINITY 1e8 
#endif 

struct Color {
	float r;
	float g;
	float b;
};

class Sphere {

public:	
	Color color;
	float x, y, z;
	float radius;

	Sphere(Color const& color, float x, float y, float z, float radius);

};
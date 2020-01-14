#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include "Sphere.h"
#include "Ray.h"



unsigned int height = 480, width = 640;
float invWidth = 1 / float(width), invHeight = 1 / float(height);
float fov = 30, aspectratio = width / float(height);
float angle = tan(M_PI * 0.5 * fov / 180.);


std::vector<Color> image;

static float dot(Vec3 const& a, Vec3 const& b)
{
	return a.x * b.x +
		a.y + b.y +
		a.z + b.z;
}

// a points to b
static Vec3 direction(Vec3 const& a, Vec3 const& b)
{
	return { b.x - a.x, b.y - a.y, b.z - a.z };
}


bool checkIntersection(Ray const& ray, Sphere const& s, Vec3& point)
{
	Vec3 m = direction(ray.rayOrig, s.origin);
	float b = dot(m, ray.rayDirc);
	float c = dot(m, m) - s.radius * s.radius;

	// the ray points away from the sphere
	if (b > 0.0f && c > 0.0f) return false;

	float discrinant = b * b - c;

	// no intersection
	if (discrinant < 0.0f) return false;

	float t = -b - sqrt(discrinant);

	// If t is negative, ray started inside sphere so clamp t to zero 
	if (t < 0.0f) t = 0.0f;
	point = { ray.rayOrig.x + ray.rayDirc.x * t,
			  ray.rayOrig.y + ray.rayDirc.y * t,
			  ray.rayOrig.z + ray.rayDirc.z * t };

	return true;
}

void gen_ppm_image() {
	std::ofstream ofs("./test.ppm", std::ios::out, std::ios::binary);
	ofs << "P6\n"
		<< width
		<< " "
		<< height 
		<< "\n255\n";

	for (unsigned int i = 0; i < image.size(); ++i) {
		ofs << (unsigned char) (std::min(1.0f, image[i].r) * 255)
			<< (unsigned char) (std::min(1.0f, image[i].g) * 255)
			<< (unsigned char) (std::min(1.0f, image[i].b) * 255);
	}	

	ofs.close();
}

Ray computePrimRay(unsigned int x, unsigned int y) 
{
	float xx = (2 * ((x + 0.5) * invWidth) - 1) * angle * aspectratio;
	float yy = (1 - 2 * ((y + 0.5) * invHeight)) * angle;

	std::cout << "x: " << xx
		<< "y: " << yy << std::endl;

	return Ray({ 0.0, 0.0, 0.0 }, {xx, yy, -1.0f});
}


int main() {
	image = std::vector<Color>(width * height, { 0.5f, 0.5f, 0.5f });
	Sphere sphere({ 1.00, 0.32, 0.36 }, { 0.0, 0, -20 }, 4.0f);

	Ray primRay = computePrimRay(1, 1);		

	gen_ppm_image();
}



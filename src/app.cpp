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

static float distance(Vec3 const& a, Vec3 const& b)
{
	return  sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2) + pow(b.z - a.z, 2));
}


bool checkIntersection(Ray const& ray, Sphere const& s, Vec3& point)
{
	//solve for tc
	float L = distance(s.origin , ray.rayOrig);

	float tc = ray.rayDirc.x * L + ray.rayDirc.y * L + ray.rayDirc.z * -L;
	if (tc < 0.0) return false;


	float d = sqrt((L * L) - (tc * tc));

	if (d > s.radius) return false;

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
	int pixel = 0;
	Sphere sphere({ 1.00, 0.32, 0.36 }, { 0.0, 0, -20 }, 1.0f);
	//Vec3 p;
	//std::cout << checkIntersection(computePrimRay(width / 2, height / 2), sphere, p) << std::endl;

	for (unsigned int i = 0; i < width; ++i) 
	{
		for (unsigned int j = 0; j < height; ++j, ++pixel) 
		{
			Ray primRay = computePrimRay(i, j);
			Vec3 p;
			if (checkIntersection(primRay, sphere, p)) image[pixel] = { 1.0, 1.0, 1.0 };


		}
	
	}

	gen_ppm_image();

	//std::cout << sqrt(4.1) << std::endl;
}



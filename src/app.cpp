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

	Ray primRay = computePrimRay(1, 1);		

	gen_ppm_image();
}



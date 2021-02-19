
#include "Camera.h"
#include "Obj.h"
#include <random>


// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::random_device device;
std::mt19937_64 RNGen(device());
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].



Ray Camera::generateRay(int pixelX, int pixelY, int W, int H)
{
	// calculate X, Y, Z
	const Vector3f X = (ry * W / H) * orient._transformVector(Vector3f::UnitX());
	const Vector3f Y = ry * orient._transformVector(Vector3f::UnitY());
	const Vector3f Z = -1 * orient._transformVector(Vector3f::UnitZ());

	// center of pixel
	//const float dx = 2 * (0.5f + pixelX) / W - 1;
	//const float dy = 2 * (0.5f + pixelY) / H - 1;

	// simple trick that generate random ray from region of one pixel
	const float dx = 2 * (myrandom(RNGen) + pixelX) / W - 1;
	const float dy = 2 * (myrandom(RNGen) + pixelY) / H - 1;


	return Ray(eye, Vector3f(dx * X + dy * Y + Z));
}


#include "Camera.h"
#include "Obj.h"
#include <random>


// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::random_device device;
std::mt19937_64 RNGen(device());
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].


Vector2f RandomPointInCircle(float radius)
{
	float r = radius * sqrtf(myrandom(RNGen));
	float angle = 2.0f * 3.1415f * r * myrandom(RNGen);

	return Vector2f(r * cosf(angle), r * sin(angle));
}



Ray Camera::generateRay(int pixelX, int pixelY, int W, int H)
{
	// calculate X, Y, Z
	const Vector3f X = (ry * W / H) * orient._transformVector(Vector3f::UnitX());
	const Vector3f Y = ry * orient._transformVector(Vector3f::UnitY());
	const Vector3f Z = -1 * orient._transformVector(Vector3f::UnitZ());

	// simple trick that generate random ray from region of one pixel
	const float dx = 2 * (myrandom(RNGen) + pixelX) / W - 1;
	const float dy = 2 * (myrandom(RNGen) + pixelY) / H - 1;


	if (!DOV) return Ray(eye, Vector3f(dx * X + dy * Y + Z));
	return DepthOfViewRay(eye, X, Y, Z, dx, dy);
}


Ray Camera::DepthOfViewRay(Vector3f eye, Vector3f X, Vector3f Y, Vector3f Z, float dx, float dy)
{
	Vector2f offset = RandomPointInCircle(radius);
	const float rx = offset.x();
	const float ry = offset.y();

	Vector3f Eye = eye + rx * X + ry * Y;
	Vector3f Dir = (D * dx - rx) * X + (D * dy - ry) * Y + D * Z;

	return Ray(Eye, Dir.normalized());
}

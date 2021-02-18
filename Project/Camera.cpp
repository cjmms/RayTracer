#include "Camera.h"
#include "Obj.h"

Ray Camera::generateRay(int pixelX, int pixelY, int W, int H)
{
	// calculate X, Y, Z
	const Vector3f X = (ry * W / H) * orient._transformVector(Vector3f::UnitX());
	const Vector3f Y = ry * orient._transformVector(Vector3f::UnitY());
	const Vector3f Z = -1 * orient._transformVector(Vector3f::UnitZ());

	const float dx = 2 * (0.5f + pixelX) / W - 1;
	const float dy = 2 * (0.5f + pixelY) / H - 1;

	return Ray(eye, Vector3f(dx * X + dy * Y + Z));
}
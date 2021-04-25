#pragma once
#include "geom.h"
class Ray;

class Camera {
public:
	Vector3f eye;
	Quaternionf orient;
	float ry;
	float radius;
	float D;
	bool DepthOfField;

	inline void setCamera(const Vector3f& _eye, const Quaternionf& _o, const float _ry, float D, float r)
	{
		eye = _eye; orient = _o; ry = _ry; this->D = D; radius = r;  DepthOfField = false;
	}

	// x and y are values from 0 to W/H
	Ray generateRay(int pixelX, int pixelY, int W, int H);


private:
	Ray DepthOfViewRay(Vector3f eye, Vector3f X, Vector3f Y, Vector3f Z, float dx, float dy);
};


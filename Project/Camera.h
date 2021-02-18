#pragma once
#include "geom.h"
class Ray;

class Camera {
public:
	Vector3f eye;
	Quaternionf orient;
	float ry;

	inline void setCamera(const Vector3f& _eye, const Quaternionf& _o, const float _ry)
	{
		eye = _eye; orient = _o; ry = _ry;
	}

	// x and y are values from 0 to W/H
	Ray generateRay(int pixelX, int pixelY, int W, int H);

};


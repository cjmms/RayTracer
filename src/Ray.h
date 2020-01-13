#pragma once

struct Vec3 {
	float x, y, z;
};

class Ray {

public:
	Vec3 rayOrig;
	Vec3 rayDirc;
	Ray(Vec3 orig, Vec3 dirc) : rayOrig(orig), rayDirc(dirc) {};
};
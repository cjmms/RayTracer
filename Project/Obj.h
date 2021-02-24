#pragma once


#include <vector>
#include "glm/glm/glm.hpp"
#include "Camera.h"

const float PI = 3.1415f;
#define Epsilon 0.0001f
extern const float Radians;    // Convert degrees to radians

typedef Eigen::AlignedBox<float, 3> Bbox;  // The BV type provided by Eigen


class Shape;
class Material;


class Interval {
public:
	float t0, t1;
	Vector3f N0, N1;

	Interval() : t0(0), t1(INFINITY) {};
	Interval(float t0, Vector3f N0, float t1, Vector3f N1);
	void empty();
	Interval overlap(Interval interval) const;

	void print(std::string str) const;
};







class Ray {
public:
	Vector3f point;
	Vector3f direction;

	Ray(Vector3f p, Vector3f dir);

	inline Vector3f evl(float t) const { return point + direction * t; }

	// positive return value
	float dis(Vector3f p);
};




class Intersection
{
public:
	Shape* shape;

	float t;
	Vector3f position;
	Vector3f normal;

public:
	Intersection() :shape(nullptr), t(-1) {};
	Intersection(Shape *shape, float t);
	Intersection(Shape* shape, Vector3f position, Vector3f normal);

	bool hasIntersection() { return -1.0f != t; }

	// normal is normalized
	void set(Shape* shape, Vector3f position, Vector3f normal, float t);
};




class Shape {
public:
	Material* mat;

	Shape(Material* mat) :mat(mat) {}

	inline virtual ~Shape() {}
	
	inline virtual bool intersect(const Ray ray, Intersection& intersection) { return false; };
	virtual Bbox bbox() const { return Bbox(); };
};


inline Bbox bounding_box(const Shape* obj) { return obj->bbox(); }


// Sphere
class Sphere : public Shape
{
public:
	float radius;
	Vector3f center;

	Sphere(Vector3f c, float r, Material* m) : Shape(m), radius(r), center(c) {}

	inline Vector3f getNormal(Vector3f p) { return p - center; }

	bool intersect(const Ray ray, Intersection& intersection) override;

	// choose a uniform distributed point on the sphere
	Intersection SampleSphere();

	Bbox bbox() const override;
};


// Box
class Box : public Shape
{
public:
	Vector3f base, diag;

	Box(const Vector3f base, const Vector3f diag, Material* m) : Shape(m), base(base), diag(diag) {}

	bool intersect(const Ray ray, Intersection& intersection) override;
	
	Bbox bbox() const override;
};


// Cylinder
class Cylinder : public Shape
{
public:
	Vector3f base, axis;
	float radius;

	Cylinder(const Vector3f base, const Vector3f axis, float radius, Material* m)
		: Shape(m), base(base), axis(axis), radius(radius) {}

	bool intersect(const Ray ray, Intersection& intersection) override;

	Bbox bbox() const override;
};


// Triangle
class Triangle : public Shape
{
public:
	Vector3f v0, v1, v2, n0, n1, n2;

	Triangle(Vector3f v0, Vector3f v1, Vector3f v2, Vector3f n0, Vector3f n1, Vector3f n2, Material* m)
		: Shape(m), v0(v0), v1(v1), v2(v2), n0(n0), n1(n1), n2(n2) {}

	bool intersect(const Ray ray, Intersection& intersection) override;

	Bbox bbox() const override;
};




// ray slab intersection 
// N: slab direction
// d0, d1: plane offsets
Interval intersectSlab(const Ray ray, Vector3f N, float d0, float d1);

// ray default cylinder intersection
Interval intersectCylinder(Vector3f D, Vector3f Q, float radius, Quaternionf q);
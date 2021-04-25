#include "Obj.h"
#include <cmath>




Interval::Interval(float t0, Vector3f N0, float t1, Vector3f N1)
{
		this->t0 = t0;
		this->N0 = N0.normalized();
		this->t1 = t1;
		this->N1 = N1.normalized();
}

Interval Interval::overlap(Interval interval) const
{
	// max of t0
	float t0 = (interval.t0 > this->t0) ? interval.t0 : this->t0;
	Vector3f N0 = (interval.t0 > this->t0) ? interval.N0 : this->N0;

	// min of t1
	float t1 = (interval.t1 < this->t1) ? interval.t1 : this->t1;
	Vector3f N1 = (interval.t1 < this->t1) ? interval.N1 : this->N1;

	return Interval(t0, N0, t1, N1);
}

void Interval::print(std::string str) const
{
	std::cout << str << ": " ;
	std::cout << "t0 " << t0 ;
	std::cout << " t1 " << t1 << std::endl;
}


void Interval::empty()
{
	t0 = 0;
	t1 = -1;
}



Ray::Ray(Vector3f p, Vector3f dir)
	: point(p), direction(dir)
{
	direction.normalize();
}



float Ray::dis(Vector3f p)
{
	Vector3f temp = p - this->point;

	// using glm because I've no idea how to get vector size out of Vector3f
	const glm::vec3 v(temp.x(), temp.y(), temp.z());
	return v.length();
}



Intersection::Intersection(Shape* shape, float t)
	:shape(shape), t(t)
{
}


Intersection::Intersection(Shape* shape, Vector3f position, Vector3f normal)
	:shape(shape), position(position), normal(normal.normalized())
{
}



void Intersection::set(Shape* shape, Vector3f position, Vector3f normal, float t)
{
	this->shape = shape;
	this->position = position;
	this->normal = normal.normalized();
	this->t = t;
}




Vector3f Sphere::BezierCurve(float t) const
{
	return (1 - t)* (1 - t)* A + 2 * t * (1 - t) * B + t * t * C;
}








// Chpater 6.2, Page 144
// Ray:				P(t) = S + tV
// Sphere:			x^2 + y^2 + z^2 = r^2
// Q:				ray origin - center
// a:				V^2
// b:				2(Q * V)
// c:				Q^2 - r^2
// discriminant:	b^2 - 4ac
// roots:			(-b +/- sqrt(dis)) / 2a
bool Sphere::intersect(const Ray ray, Intersection& intersection)
{

	Vector3f Qbar = ray.point - center;
	float QdotD = Qbar.dot(ray.direction);
	float fvar = sqrt(QdotD * QdotD - Qbar.dot(Qbar) + radius * radius);
	if (-QdotD - fvar > 0.0001f) intersection.t = -QdotD - fvar;
	else if (-QdotD + fvar > 0.0001f) intersection.t = -QdotD + fvar;
	else return false;

	intersection.position = ray.evl(intersection.t);
	intersection.normal = (intersection.position - center).normalized();
	intersection.shape = this;

	return true;
}



Intersection Sphere::SampleSphere()
{
	float t1 = myrandom(RNGen); // first unifromly distributed random number
	float t2 = myrandom(RNGen); // first unifromly distributed random number

	float z = 2 * t1 - 1.0f;
	float r = sqrtf(1 - z*z);

	float a = 2 * PI * t2;

	Vector3f N(r * cosf(a), r * sinf(a), z);
	Vector3f P = center + radius * N.normalized();

	return Intersection(this, P, N);
}







Bbox Sphere::bbox() const 
{
	const Vector3f leftBottom(center[0] - radius, center[1] - radius, center[2] - radius);
	const Vector3f rightTop(center[0] + radius, center[1] + radius, center[2] + radius);
	return Bbox(leftBottom, rightTop);
}




bool Box::intersect(const Ray ray, Intersection& intersection)
{
	// 3 slab intersection detection
	const Interval i0 = intersectSlab(ray, Vector3f(1, 0, 0), -base.x(), -base.x() - diag.x());
	if (i0.t1 == -1) return false;	// no intersection, not in between, return false

	const Interval i1 = intersectSlab(ray, Vector3f(0, 1, 0), -base.y(), -base.y() - diag.y());
	if (i1.t1 == -1) return false;	// no intersection, not in between, return false

	const Interval i2 = intersectSlab(ray, Vector3f(0, 0, 1), -base.z(), -base.z() - diag.z());
	if (i2.t1 == -1) return false;	// no intersection, not in between, return false

	// calculate the correct intersection interval
	const Interval temp = i0.overlap(i1);
	const Interval interval = temp.overlap(i2);

	// fill intersection
	if (interval.t0 > interval.t1) return false;	// No intersection
	else
	{		
		if (interval.t0 < Epsilon && interval.t1 < Epsilon) return false;	// intersection behind ray

		// since t0 is smaller, if t0 is positive, t0 is intersection, vice versa
		if (interval.t0 > Epsilon) intersection.set(this, ray.evl(interval.t0), interval.N0, interval.t0);
		else intersection.set(this, ray.evl(interval.t1), interval.N1, interval.t1);

		return true;
	}
}


Bbox Box::bbox() const
{
	return Bbox(base, base + diag);
}


// Ray: Q + tD
// slab: N + d0, N + d1
Interval intersectSlab(const Ray ray, Vector3f N, float d0, float d1) 
{
	const Vector3f Q = ray.point;
	const Vector3f D = ray.direction;

	const float NdotQ = N.dot(Q);
	const float NdotD = N.dot(D);

	if (NdotD != 0)	// Ray is not in parallel with slabs
	{
		// 2 intersections
		const float t0 = -(d0 + NdotQ) / NdotD;
		const float t1 = -(d1 + NdotQ) / NdotD;

		if (t0 < t1) return Interval(t0, N, t1, -N);
		else return Interval(t1, N, t0, -N);
	}
	else 
	{
		// ray is in parallel with slabs
		const float s0 = NdotQ + d0;
		const float s1 = NdotQ + d1;

		if ((s0 > Epsilon && s1 > Epsilon) || (s0 < Epsilon && s1 < Epsilon))	// same sign, not in between of slabs
			return Interval(0, N, -1, N);	// no intersection to Box, N doesn't matter
		else // in between of slabs
			return Interval();	// [0, Infinity]
	}
}


Interval intersectCylinder(Vector3f D, Vector3f Q, float radius, Quaternionf q)
{
	const float a = D[0] * D[0] + D[1] * D[1];
	const float b = 2 * (D[0] * Q[0] + D[1] * Q[1]);
	const float c = Q[0] * Q[0] + Q[1] * Q[1] - radius * radius;

	const float determinant = b * b - 4.0f * a * c;

	Vector3f N;
	if (determinant < Epsilon) return Interval(0, N, -1, N);	// no intersection

	const float t1 = (-b + sqrtf(determinant)) / 2 / a;
	const float t2 = (-b - sqrtf(determinant)) / 2 / a;
	
	Vector3f n1 = Q + t1 * D;
	Vector3f n2 = Q + t2 * D;

	n1[2] = 0;
	n2[2] = 0;

	n1 = q.conjugate()._transformVector(n1);
	n2 = q.conjugate()._transformVector(n2);

	if (t1 < t2) return Interval(t1, n1, t2, n2);
	return Interval(t2, n2, t1, n1);
}




Bbox Cylinder::bbox() const  
{
	Vector3f min, max;

	// find min x, y, z and max x, y, z
	for (unsigned int i = 0; i < 3; ++i)
	{
		if (base[i] < (base[i] + axis[i]))
		{
			min[i] = base[i];
			max[i] = base[i] + axis[i];
		}
		else
		{
			min[i] = base[i] + axis[i];
			max[i] = base[i];
		}
	}

	return Bbox(min - Vector3f(radius, radius, radius), max + Vector3f(radius, radius, radius));
}



bool Cylinder::intersect(const Ray ray, Intersection& intersection)
{
	// coordinate transformation
	Quaternionf q = Quaternionf::FromTwoVectors(axis, Vector3f::UnitZ());
	Vector3f D = q._transformVector(ray.direction);
	Vector3f Q = q._transformVector(ray.point - base);
	const Ray tRay(Q, D);

	const Interval slabInterval = intersectSlab(tRay, Vector3f::UnitZ(), 0, -axis.norm());
	if (slabInterval.t1 == -1) return false;	// no intersection, not in between, return false

	const Interval cylinderInterval = intersectCylinder(D, Q, radius, q);
	if (cylinderInterval.t1 == -1) return false;		// no intersection to Cylinder

	// calculate the correct intersection interval
	Interval interval = slabInterval.overlap(cylinderInterval);

	if (interval.t0 > interval.t1) return false;	// No intersection
	else
	{
		if (interval.t0 < 0 && interval.t1 < 0) return false;	// intersection behind ray

		// since t0 is smaller, if t0 is positive, t0 is intersection, vice versa
		if (interval.t0 > 0) intersection.set(this, ray.evl(interval.t0), interval.N0, interval.t0);
		else intersection.set(this, ray.evl(interval.t1), interval.N1, interval.t1);

		return true;
	}
}


float minIn3(float a, float b, float c)
{
	float temp = a < b ? a : b;
	return temp < c ? temp : c;
}

float maxIn3(float a, float b, float c)
{
	float temp = a > b ? a : b;
	return temp > c ? temp : c;
}


Bbox Triangle::bbox() const
{
	Vector3f min(minIn3(v0[0], v1[0], v2[0]), minIn3(v0[1], v1[1], v2[1]), minIn3(v0[2], v1[2], v2[2]));

	Vector3f max(maxIn3(v0[0], v1[0], v2[0]), maxIn3(v0[1], v1[1], v2[1]), maxIn3(v0[2], v1[2], v2[2]));

	return Bbox(min, max);
}


bool Triangle::intersect(const Ray ray, Intersection& intersection)
{
	/*
	Vector3f Q = ray.point;
	Vector3f D = ray.direction;

	Vector3f E1 = v1 - v0;
	Vector3f E2 = v2 - v0;

	Vector3f p = D.cross(E2);
	float d = p.dot(E1);

	if (d == 0) return false;	// ray parellel to tri

	Vector3f S = Q - v0;
	float u = p.dot(S) / d;
	if (u < 0 || u > 1) return false;	//intersected with plane, outside E2 edge

	Vector3f q = S.cross(E1);
	float v = D.dot(q) / d;
	if (v < 0 || (u + v) > 1) return false;	// intersected with plane, outside other edge

	float t = E2.dot(q) / d;
	if (t < 0) return false;	// triangle behind the ray

	Vector3f Normal = (1.0f - u - v) * n0 + u * n1 + v * n2;
	intersection.set(this, ray.evl(t), Normal, t);

	return true;*/

	Vector3f e1 = v1 - v0;
	Vector3f e2 = v2 - v0;
	Vector3f p = ray.direction.cross(e2);
	float d = p.dot(e1);
	if (abs(d) < 0.00001f) return false;
	Vector3f s = ray.point - v0;
	float u = (p.dot(s)) / d;
	if (u < 0.0f || u >1) return false;
	Vector3f q = s.cross(e1);
	float v = ray.direction.dot(q) / d;
	if (v < 0.0f || u + v > 1.0f) return false;
	float t = e2.dot(q) / d;
	if (t < 0.0f)return false;

	intersection.t = t;
	intersection.position = ray.evl(t);
	intersection.normal = e1.cross(e2);
	intersection.normal.normalize();
	intersection.shape = this;
	return true;


}
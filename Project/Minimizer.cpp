#include "Minimizer.h"
#include "Obj.h"

Minimizer::Minimizer(const Ray& r)
	:ray(r)
{
	intersection = Intersection();

}

// done
float Minimizer::minimumOnObject(Shape* obj)
{
	Intersection temp; 
	if (obj->intersect(ray, temp)) {
		if (intersection.t == -1 || temp.t < intersection.t) intersection = temp;
		return temp.t; 
	}
	else return INF;
}


// Called by BVMinimize to intersect the ray with a Bbox and
// returns the t value.  This should be similar to the already
// written box (3 slab) intersection.  (The difference being: a                     
// distance of zero should be returned if the ray starts within the Bbox.)
// Return INF to indicate no intersection.
float Minimizer::minimumOnVolume(const Bbox& box)
{
	Vector3f L = box.min();  // Box corner
	Vector3f U = box.max();  // Box corner

	// 3 slab intersection detection
	const Interval i0 = intersectSlab(ray, Vector3f(1, 0, 0), -L.x(),  - U.x());
	if (i0.t1 == -1) return INF;

	const Interval i1 = intersectSlab(ray, Vector3f(0, 1, 0), -L.y(),  - U.y());
	if (i1.t1 == -1) return INF;

	const Interval i2 = intersectSlab(ray, Vector3f(0, 0, 1), -L.z(),  - U.z());
	if (i2.t1 == -1) return INF;


	// calculate the correct intersection interval
	const Interval temp = i0.overlap(i1);
	const Interval interval = temp.overlap(i2);

	// fill intersection
	if (interval.t0 > interval.t1) return INF;	// No intersection
	else if (interval.t1 < Epsilon) return INF;	// intersection behind ray
	else if (interval.t0 < Epsilon && interval.t1 > Epsilon) return 0.0f;	// intersection inside box
	else return interval.t0 > interval.t1 ? interval.t1 : interval.t0;	// return smallest intersection
}
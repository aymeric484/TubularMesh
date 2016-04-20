#ifndef MATHTOOLS_H
#define MATHTOOLS_H


#include <gmtl/gmtl.h>
#include "point3d.h"
#include "Headers/myMap.h"

using namespace std;

extern PFP::MAP myMap;

/******************************************************************************/
/*************************      MATHTOOLS CLASS		***************************/
/******************************************************************************/
/**
 * The MathTools class provides some useful mathematical functions for 
 * generating a network of blood vessels
 * @author Cyril Kern
 */
class MathTools
{
public:
	/**
	 * compute the matrix rotation for an arbitrary axis and angle
	 * @param axis arbitrary axis of rotation
	 * @param angle arbitrary angle of rotation
	 * @return the rotation matrix's quaternion
	 */
	static gmtl::Quatf rotationQuat(gmtl::Vec3f axis, float const angle);
	
	/**
	 * compute a vector orthogonal to a given vector
	 * @param tangent input vector
	 * @return orthogonal vector
	 */
	static gmtl::Vec3f computeOrthogonalVector(gmtl::Vec3f tangent);
	
	/**
	 * compute the barycenter of the 3 points
	 * @param p0 first point
	 * @param p1 second point
	 * @param p2 third point
	 * @return the barycenter
	 */
	static gmtl::Vec3f computeBary(Point3D* p0, Point3D* p1, Point3D* p2);
	
	/**
	 * compute the barycenter of n points
	 * @param p vector of points
	 * @return the barycenter
	 */
	static gmtl::Vec3f computeBary(vector<Point3D*> p);
	
	/**
	 * compute the barycenter of the 3 points
	 * @param Pa first point
	 * @param Pb second point
	 * @param Pc third point
	 * @return the barycenter
	 */
	static gmtl::Vec3f computeBary(gmtl::Vec3f Pa, gmtl::Vec3f Pb, gmtl::Vec3f Pc);
	
	/**
	 * compute the barycenter of n points
	 * @param p vector of points
	 * @return the barycenter
	 */
	static gmtl::Vec3f computeBary(vector<gmtl::Vec3f> P);
	
	/**
	 * compute the barycenter of a triangle
	 * @param d dart representing the triangle
	 * @return the barycenter
	 */
	static gmtl::Vec3f computeBary(Dart d);
	
	/**
	 * compute the normal of the faces composed by the 3 points
	 * @param p0 first point
	 * @param p1 second point
	 * @param p2 third point
	 * @return the direct normal
	 */
	static gmtl::Vec3f computeNormal(Point3D* p0, Point3D* p1, Point3D* p2);
	
	/**
	 * compute the normal of n points
	 * @param p vector of points
	 * @return the normal
	 */
	static gmtl::Vec3f computeNormal(vector<Point3D*> p);
	
	/**
	 * compute the normal of the faces composed by the 3 points
	 * @param Pa first point
	 * @param Pb second point
	 * @param Pc third point
	 * @return the direct normal
	 */
	static gmtl::Vec3f computeNormal(gmtl::Vec3f Pa, gmtl::Vec3f Pb, gmtl::Vec3f Pc);
	
	/**
	 * compute the normal of n points
	 * @param P vector of points
	 * @return the normal
	 */
	static gmtl::Vec3f computeNormal(vector<gmtl::Vec3f> P);
	
	/**
	 * compute the normal of a triangle
	 * @param d dart representing the triangle
	 * @return the normal
	 */
	static gmtl::Vec3f computeNormal(Dart d);
	
	/**
	 * compute the orthogonal projection of the point pt on plane based on (p0, p1, p2) relatively to an origin
	 * @param p0 point of the plane
	 * @param p1 point of the plane
	 * @param p2 point of the plane
	 * @param pt the point to project
	 * @param ptOrigine the reference point
	 * @return the projection of the point
	 */
	static gmtl::Vec2f projectOrthogonal(Point3D * p0, Point3D * p1, Point3D* p2, Point3D* pt, Point3D* ptOrigine);
	
	/**
	 * compute the orthogonal projection of the point pt on plane based on (p0, p1, p2) relatively to an origin
	 * @param p0 point of the plane
	 * @param p1 point of the plane
	 * @param p2 point of the plane
	 * @param pt the point to project
	 * @param ptOrigine the reference point
	 * @return the projection of the point
	 */
	static gmtl::Vec2f projectOrthogonal(gmtl::Vec3f p0, gmtl::Vec3f p1, gmtl::Vec3f p2, gmtl::Vec3f pt, gmtl::Vec3f ptOrigine);
	
	/**
	 * compute the orthogonal projection of the point pt on plane made by (a b c d) relatively to an origin
	 * @param a coordinates of the plane (ax + by + cz + d = 0)
	 * @param b coordinates of the plane (ax + by + cz + d = 0)
	 * @param c coordinates of the plane (ax + by + cz + d = 0)
	 * @param pt the point to project
	 * @param ptOrigine the reference point
	 * @return the projection of the point
	 */
	static gmtl::Vec2f projectOrthogonal(float a, float b, float c, gmtl::Vec3f pt, gmtl::Vec3f ptOrigine);
	
	/**
	 * compute the angle between two vectors
	 * @param v0 first vector
	 * @param v1 second vector
	 * @return the angle in radians
	 */
	static float angle(gmtl::Vec3f v0, gmtl::Vec3f v1);
	
	/**
	 * compute the smallest angle in a triangle
	 * @param points the list of the points of the triangle
	 * @return the minimum angle
	 */
	static float angleMinInTri(gmtl::Vec3f * points);
	
	
	/**
	 * compute the smallest distance from a point to a plane
	 * @param a coordinates of the plane (ax + by + cz + d = 0)
	 * @param b coordinates of the plane (ax + by + cz + d = 0)
	 * @param c coordinates of the plane (ax + by + cz + d = 0)
	 * @param d coordinates of the plane (ax + by + cz + d = 0)
	 * @param pt the point
	 * @return distance
	 */
	static float distancePlanPoint(float a, float b, float c, float d, gmtl::Vec3f pt);
	
	/**
	 * return the d coordinates of a plane (ax + by + cz + d = 0)
	 * @param planNormal the normal of the plane
	 * @param pt a point of the plane
	 * @return the distance
	 */
	static float dCoordsOfThePlan(gmtl::Vec3f planNormal, gmtl::Vec3f pt);
	
	/**
	 * return the d coordinates of a plane (ax + by + cz + d = 0)
	 * @param a coordinates of the plane (ax + by + cz + d = 0)
	 * @param b coordinates of the plane (ax + by + cz + d = 0)
	 * @param c coordinates of the plane (ax + by + cz + d = 0)
	 * @param pt a point of the plane
	 * @return the distance
	 */
	static float dCoordsOfThePlan(float a, float b, float c, gmtl::Vec3f pt);
	
	/**
	 * test if it exists an intersection between a disk and a bowl
	 * @param bowlCenter the bowl central point
	 * @param bowlRadius the radius of the bowl
	 * @param diskCenter the disk central point
	 * @param diskRadius the radius of the disk
	 * @param a equation of the plane of the disk (ax + by + cz + d = 0)
	 * @param b equation of the plane of the disk (ax + by + cz + d = 0)
	 * @param c equation of the plane of the disk (ax + by + cz + d = 0)
	 * @param d equation of the plane of the disk (ax + by + cz + d = 0)
	 * @return true if an intersection exists
	 */
	static bool intersectsBowlDisk(gmtl::Vec3f bowlCenter, float bowlRadius,
								   gmtl::Vec3f diskCenter, float diskRadius, float a, float b, float c, float d);
	
	/**
	 * compute the projection of a point onto a sphere
	 * @param origine the point we want to project
	 * @param direction the direction of projection
	 * @param centerSphere the center of the sphere
	 * @param radiusSphere the radius of the sphere
	 * @return the projection of the point
	 */
	static gmtl::Vec3f projectionOnSphere(gmtl::Vec3f origine, gmtl::Vec3f direction, gmtl::Vec3f centerSphere, float radiusSphere);
	
	/**
	 * compute the convex hull given a list of 3D points
	 * @param lstPoints the list of 3D points taken as input for the convex hull
	 * @param newHull a marker to mark the convex hull
	 */
	static void convexHull(std::vector<PFP::EMB*> lstPoints, Marker newHull);
};

#endif

#ifndef MATHTOOLS_H
#define MATHTOOLS_H

#include "Geometry/vector_gen.h"
#include "Geometry/intersection.h"

namespace CGoGN
{

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
	template <typename QUAT, typename AA, typename VEC3>
	static QUAT rotationQuat(VEC3 axis, float const angle)
	{
		AA axisAngle(angle, axis[0], axis[1], axis[2]);

		float half_angle = axisAngle[0] * 0.5f; // angle * 0.5
		float sin_half_angle = std::sin(half_angle);
		QUAT q;
		q[3] = std::cos(half_angle);
		q[0] = sin_half_angle * axisAngle[1]; //first axe
		q[1] = sin_half_angle * axisAngle[2]; //second axe
		q[2] = sin_half_angle * axisAngle[3]; //third axe

		return q;
	}
	

	template <typename QUAT, typename VEC3>
	static VEC3 transform(QUAT rot, VEC3 vector)
	{
		QUAT rot_conj( -rot[0], -rot[1], -rot[2], rot[3] );
		QUAT pure( vector[0], vector[1], vector[2], 0.0 );
		QUAT temp(
			pure[3]*rot_conj[0] + pure[0]*rot_conj[3] + pure[1]*rot_conj[2] - pure[2]*rot_conj[1],
			pure[3]*rot_conj[1] + pure[1]*rot_conj[3] + pure[2]*rot_conj[0] - pure[0]*rot_conj[2],
			pure[3]*rot_conj[2] + pure[2]*rot_conj[3] + pure[0]*rot_conj[1] - pure[1]*rot_conj[0],
			pure[3]*rot_conj[3] - pure[0]*rot_conj[0] - pure[1]*rot_conj[1] - pure[2]*rot_conj[2]
		);

		VEC3 result(
			rot[3]*temp[0] + rot[0]*temp[3] + rot[1]*temp[2] - rot[2]*temp[1],
			rot[3]*temp[1] + rot[1]*temp[3] + rot[2]*temp[0] - rot[0]*temp[2],
			rot[3]*temp[2] + rot[2]*temp[3] + rot[0]*temp[1] - rot[1]*temp[0]
		);

		return result;
	}

	/**
	 * compute a vector orthogonal to a given vector
	 * @param tangent input vector
	 * @return orthogonal vector
	 */
	template <typename VEC3>
	static VEC3 computeOrthogonalVector(VEC3 tangent)
	{
		VEC3 vec;

		if (tangent[2]==0.f)
		{
			vec[0] = tangent[1];
			vec[1] = -tangent[0];
			vec[2] = 1.f;
		}
		else
		{
			vec[0] = 1.f;
			vec[1] = 1.f;
			vec[2] = (-tangent[0] - tangent[1])/tangent[2];
		}

		vec.normalize();
		return vec;
	}
	
	/**
	 * compute the normal of the faces composed by the 3 points
	 * @param Pa first point
	 * @param Pb second point
	 * @param Pc third point
	 * @return the direct normal
	 */
	template <typename VEC3>
	static VEC3 computeNormal(VEC3 Pa, VEC3 Pb, VEC3 Pc)
	{
		VEC3 Vab = Pb - Pa;
		VEC3 Vac = Pc - Pa;

		VEC3 N;

		cross(N,Vab,Vac);

		N.normalize();
		//gmtl::normalize(N);
		return N;
	}
	
	/**
	 * compute the normal of n points
	 * @param P vector of points
	 * @return the normal
	 */
//	static Geom::Vec3f computeNormal(vector<Geom::Vec3f> P);
	
	/**
	 * compute the normal of a triangle
	 * @param d dart representing the triangle
	 * @return the normal
	 */
//	static Geom::Vec3f computeNormal(Dart d);
	
	/**
	 * compute the orthogonal projection of the point pt on plane based on (p0, p1, p2) relatively to an origin
	 * @param p0 point of the plane
	 * @param p1 point of the plane
	 * @param p2 point of the plane
	 * @param pt the point to project
	 * @param ptOrigine the reference point
	 * @return the projection of the point
	 */
//	static Geom::Vec2f projectOrthogonal(Point3D * p0, Point3D * p1, Point3D* p2, Point3D* pt, Point3D* ptOrigine);
	
	/**
	 * compute the orthogonal projection of the point pt on plane based on (p0, p1, p2) relatively to an origin
	 * @param p0 point of the plane
	 * @param p1 point of the plane
	 * @param p2 point of the plane
	 * @param pt the point to project
	 * @param ptOrigine the reference point
	 * @return the projection of the point
	 */
//	static Geom::Vec2f projectOrthogonal(Geom::Vec3f p0, Geom::Vec3f p1, Geom::Vec3f p2, Geom::Vec3f pt, Geom::Vec3f ptOrigine);
	
	/**
	 * compute the orthogonal projection of the point pt on plane made by (a b c d) relatively to an origin
	 * @param a coordinates of the plane (ax + by + cz + d = 0)
	 * @param b coordinates of the plane (ax + by + cz + d = 0)
	 * @param c coordinates of the plane (ax + by + cz + d = 0)
	 * @param pt the point to project
	 * @param ptOrigine the reference point
	 * @return the projection of the point
	 */
	template <typename QUAT, typename AA, typename VEC3>
	static Geom::Vec2f projectOrthogonal(float a, float b, float c,VEC3 pt, VEC3 ptOrigine) {
		// compute the plane normal

		VEC3 planeNormal(a,b,c);

		// if the plane exists
		if (planeNormal.norm() != 0)
		{
			// create a single up vector
			VEC3 upVector(0.0f,0.0f,1.0f);

			// get the vector to turn around
			VEC3 rotationVector;
			rotationVector = upVector^planeNormal;
			rotationVector.normalize();

			// get the angle of rotation
			float angRot=angle(planeNormal, upVector);

			// create the rotation
			AA axisAngle(angRot, rotationVector[0], rotationVector[1], rotationVector[2]);

			//convert AxisAngle to Quaternion
			float half_angle = axisAngle[0] * 0.5f; // angle * 0.5
			float sin_half_angle = std::sin(half_angle);
			QUAT myQuat;
			myQuat[3] = std::cos(half_angle);
			myQuat[0] = sin_half_angle * axisAngle[1]; //first axe
			myQuat[1] = sin_half_angle * axisAngle[2]; //second axe
			myQuat[2] = sin_half_angle * axisAngle[3]; //third axe

			// rotate the point
			VEC3 ptToRote=pt-ptOrigine;
			VEC3 ptRote=MathTools::transform(myQuat,ptToRote);
			// point put it in the plane
			Geom::Vec2f ptProj(ptRote[0],ptRote[1]);

			return ptProj;
		}
		else
		{
			printf("warning, plan inexistant - renvoie 0\n");
			return Geom::Vec2f(0,0);
			exit(-1);
		}
	}
	

	/**
	 * compute the smallest angle in a triangle
	 * @param points the list of the points of the triangle
	 * @return the minimum angle
	 */
//	static float angleMinInTri(Geom::Vec3f * points);
	
	
	/**
	 * compute the smallest distance from a point to a plane
	 * @param a coordinates of the plane (ax + by + cz + d = 0)
	 * @param b coordinates of the plane (ax + by + cz + d = 0)
	 * @param c coordinates of the plane (ax + by + cz + d = 0)
	 * @param d coordinates of the plane (ax + by + cz + d = 0)
	 * @param pt the point
	 * @return distance
	 */
	template <typename VEC3>
	static float distancePlanPoint(float a, float b, float c, float d, VEC3 pt) {
		return (std::fabs(a*pt[0]+b*pt[1]+c*pt[2]+d)/std::sqrt(a*a+b*b+c*c));
	}
	
	/**
	 * return the d coordinates of a plane (ax + by + cz + d = 0)
	 * @param planNormal the normal of the plane
	 * @param pt a point of the plane
	 * @return the distance
	 */
	template <typename VEC3>
	static float dCoordsOfThePlan(VEC3 planNormal, VEC3 pt) {
		return dCoordsOfThePlan(planNormal[0],planNormal[1],planNormal[2],pt);
	}
	
	/**
	 * return the d coordinates of a plane (ax + by + cz + d = 0)
	 * @param a coordinates of the plane (ax + by + cz + d = 0)
	 * @param b coordinates of the plane (ax + by + cz + d = 0)
	 * @param c coordinates of the plane (ax + by + cz + d = 0)
	 * @param pt a point of the plane
	 * @return the distance
	 */
	template <typename VEC3>
	static float dCoordsOfThePlan(float a, float b, float c, VEC3 pt) {
		return -(a*pt[0]+b*pt[1]+c*pt[2]);
	}
	
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
	template <typename QUAT, typename AA, typename VEC3>
	static bool intersectsBowlDisk(VEC3 bowlCenter, float bowlRadius,
								   VEC3 diskCenter, float diskRadius, float a, float b, float c, float d) {

		// compute the distance from the bowlcenter to the plane
		float disPlPt=distancePlanPoint(a,b,c,d,bowlCenter);

		// if an intersection can exist
		if (disPlPt<=bowlRadius)
		{
			// compute the distance between the center of the projection of the central point of the bowl
			// and the intersection with the plane
			disPlPt=disPlPt/bowlRadius;
			float angle = acos(disPlPt);
			float rad = sin(angle);
			rad=rad*bowlRadius;

			// compute the position of the orthogonal projection of bowlCenter on the plane
			// diskCenter is the origin
			Geom::Vec2f positionProjete=projectOrthogonal<QUAT,AA,VEC3>(a,b,c,bowlCenter,diskCenter);

			// test the distances
			if (positionProjete.norm() <= (rad+diskRadius))
			{
				return true;
			}
		}
		return false;
	}

	/**
	 * compute the projection of a point onto a sphere
	 * @param origine the point we want to project
	 * @param direction the direction of projection
	 * @param centerSphere the center of the sphere
	 * @param radiusSphere the radius of the sphere
	 * @return the projection of the point
	 */
	template <typename VEC3>
	static VEC3 projectionOnSphere(VEC3 origine, VEC3 direction, VEC3 centerSphere, float radiusSphere)
	{
		Geom::Vec4f maSphere(centerSphere[0],centerSphere[1], centerSphere[2],radiusSphere);

		int numHits;
		float inter;
		float inter2;
		VEC3 res(0,0,0);

		if(Geom::intersectionRaySphere(maSphere, origine, direction, numHits, inter, inter2))
		{
			res = origine + direction * inter;
		}

		return res;
	}
	
	/**
	 * compute the convex hull given a list of 3D points
	 * @param lstPoints the list of 3D points taken as input for the convex hull
	 * @param newHull a marker to mark the convex hull
	 */
//	static void convexHull(std::vector<PFP::EMB*> lstPoints, Marker newHull);
};

}

#endif

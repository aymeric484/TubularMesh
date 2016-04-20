#ifndef VESSEL_H
#define VESSEL_H

#include <cmath>
#include <vector>
#include <gmtl/gmtl.h>

using namespace std;

/******************************************************************************/
/*************************      VESSELRADIUS CLASS		***********************/
/******************************************************************************/
/**
 * position and radius of a vessel's point
 */
class VesselRadius
{
public:
	/**
	 * position of the point
	 */
	float x;
	float y;
	float z;
	
	/**
	 * radius of the vessel at this point
	 */
	float radius;
	
	/**
	 * default constructor
	 */
	VesselRadius();
	
	/**
	 * constructor
	 */
	VesselRadius(float px, float py, float pz, float rad);
	
	/**
	 * constructor
	 */
	VesselRadius(const VesselRadius& v);
	
	/**
	 * distance function
	 */
	float distance(VesselRadius vr);
};


/******************************************************************************/
/*************************      VESSEL CLASS		***************************/
/******************************************************************************/
/**
 * defines a blood vessel
 */
class Vessel
{
public:
	/**
	 * start and stop index
	 */
	unsigned int idStart;
	unsigned int idStop;
	
	/**
	 * number of passed points after intersection cut from start
	 */
	unsigned int decalFromStart;
	
	/**
	 * number of passed points after intersection cut from end
	 */
	unsigned int decalFromStop;
	
	/**
	 * list of points corresponding to the vessel
	 */
	std::vector<VesselRadius> lstVesselRadius;
	
	/**
	 * coordinates of the elements of the vessel
	 */
	std::vector<gmtl::Vec3f> lstCoordinates;
	
	/**
	 * true if the vessel has been reduced/cut
	 */
	bool reduced;
	
	/**
	 * number of segments corresponding to the vessel's basis (triangle, square, polygon, ...)
	 */
	unsigned int nbFaces;
	
	/**
	 * average radius of the vessel
	 */
	float averageRadius;
	
	/**
	 * default constructor
	 */
	Vessel();
	
	/**
	 * copy constructor
	 */
	Vessel(const Vessel& v);
	
	/**
	 * add a point to the vessel
	 */
	void addVesselRadius(VesselRadius ves);
	
	/**
	 * vessel's length between two points
	 */
	float distanceFromPointToPoint(int id0, int id1);
	
	/**
	 * compute the vessel's average radius
	 */
	void computeAverageRadius();
	
	/**
	 * pre-processing: simplification of the vessel
	 */
	void simplify();
	
	/**
	 * test function
	 */
	void verify();
	
	/**
	 * function that doubles the vessel's radius
	 */
	void dilate();
	
};

#endif

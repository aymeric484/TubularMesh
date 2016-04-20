
#include "Headers/MathTools.h"

/******************************************************************************/
/*************************      MATHTOOLS CLASS	FUNCTIONS	*******************/
/******************************************************************************/
/**
 * The MathTools class provides some useful mathematical functions for 
 * generating a network of blood vessels
 * @author Cyril Kern
 */


// compute the matrix rotation for an arbitrary axis and angle
gmtl::Quatf MathTools::rotationQuat(gmtl::Vec3f axis, float const angle)
{
	gmtl::AxisAnglef aa(angle, axis);
	gmtl::Quatf q=gmtl::make<gmtl::Quatf>(aa);
	
	return q;
}


// compute a vector orthogonal to a given vector
gmtl::Vec3f MathTools::computeOrthogonalVector(gmtl::Vec3f tangent)
{
	gmtl::Vec3f vec;
	
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
	
	gmtl::normalize(vec);
	return vec;
}
	

// compute the barycenter of the 3 points
gmtl::Vec3f MathTools::computeBary(Point3D* p0, Point3D* p1, Point3D* p2)
{
	return computeBary(p0->getPosition(), p1->getPosition(), p2->getPosition());
}


// compute the barycenter of n points
gmtl::Vec3f MathTools::computeBary(vector<Point3D*> p)
{
	vector<gmtl::Vec3f> pos;
	for (unsigned int i=0;i<p.size();i++)
		pos[i] = p[i]->getPosition();
	return computeBary(pos);
}


// compute the barycenter of the 3 points
gmtl::Vec3f MathTools::computeBary(gmtl::Vec3f Pa, gmtl::Vec3f Pb, gmtl::Vec3f Pc)
{
	gmtl::Vec3f bary = Pa+Pb+Pc;
	bary=bary/3.0f;
	return bary;
}


// compute the barycenter of n points
gmtl::Vec3f MathTools::computeBary(vector<gmtl::Vec3f> P)
{
	gmtl::Vec3f bary = gmtl::Vec3f(0.0,0.0,0.0);
	for (unsigned int i=0;i<P.size();i++)
		bary = bary + P[i];
	bary=bary/(float)(P.size());
	return bary;
}
	

// compute the barycenter of a triangle
gmtl::Vec3f MathTools::computeBary(Dart d)
{
	Point3D *pt0 = reinterpret_cast<PFP::EMB*>(myMap.getVertexEmb(d));
	Point3D *pt1 = reinterpret_cast<PFP::EMB*>(myMap.getVertexEmb(myMap.phi1(d)));
	Point3D *pt2 = reinterpret_cast<PFP::EMB*>(myMap.getVertexEmb(myMap.phi_1(d)));
	return MathTools::computeBary(pt0, pt1, pt2);
}

	
// compute the normal of the faces composed by the 3 points
gmtl::Vec3f MathTools::computeNormal(Point3D* p0, Point3D* p1, Point3D* p2)
{
	return computeNormal(p0->getPosition(), p1->getPosition(), p2->getPosition());
}
	

// compute the normal of n points
gmtl::Vec3f MathTools::computeNormal(vector<Point3D*> p)
{
	vector<gmtl::Vec3f> pos;
	for (unsigned int i=0;i<p.size();i++)
		pos[i] = p[i]->getPosition();
	return computeNormal(pos);
}
	

// compute the normal of the faces composed by the 3 points
gmtl::Vec3f MathTools::computeNormal(gmtl::Vec3f Pa, gmtl::Vec3f Pb, gmtl::Vec3f Pc)
{
	gmtl::Vec3f Vab = Pb - Pa;
	gmtl::Vec3f Vac = Pc - Pa;
	
	gmtl::Vec3f N;
	cross(N,Vab,Vac);
	
	//gmtl::normalize(N);
	return N;
}
	

// compute the normal of n points
gmtl::Vec3f MathTools::computeNormal(vector<gmtl::Vec3f> P)
{
	gmtl::Vec3f Vab = P[1] - P[0];
	gmtl::Vec3f Vac = P[2] - P[0];
	
	gmtl::Vec3f N;
	cross(N,Vab,Vac);
	
	gmtl::normalize(N);
	return N;
}
	

// compute the normal of a triangle
gmtl::Vec3f MathTools::computeNormal(Dart d)
{
	Point3D *pt0 = reinterpret_cast<PFP::EMB*>(myMap.getVertexEmb(d));
	Point3D *pt1 = reinterpret_cast<PFP::EMB*>(myMap.getVertexEmb(myMap.phi1(d)));
	Point3D *pt2 = reinterpret_cast<PFP::EMB*>(myMap.getVertexEmb(myMap.phi_1(d)));
	return MathTools::computeNormal(pt0, pt1, pt2);
}
	

// compute the orthogonal projection of the point pt on plane based on (p0, p1, p2) relatively to an origin
gmtl::Vec2f MathTools::projectOrthogonal(Point3D * p0, Point3D * p1, Point3D* p2, Point3D* pt, Point3D* ptOrigine)
{
	return projectOrthogonal(p0->getPosition(),p1->getPosition(),p2->getPosition(),pt->getPosition(), ptOrigine->getPosition());
}
	
	
// compute the orthogonal projection of the point pt on plane based on (p0, p1, p2) relatively to an origin
gmtl::Vec2f MathTools::projectOrthogonal(gmtl::Vec3f p0, gmtl::Vec3f p1, gmtl::Vec3f p2, gmtl::Vec3f pt, gmtl::Vec3f ptOrigine)
{
	// compute the plane normal
	gmtl::Vec3f planeNormal=computeNormal(p0,p1,p2);
	return projectOrthogonal(planeNormal[0], planeNormal[1], planeNormal[2], pt, ptOrigine);
}
	

// compute the orthogonal projection of the point pt on plane made by (a b c d) relatively to an origin
gmtl::Vec2f MathTools::projectOrthogonal(float a, float b, float c, gmtl::Vec3f pt, gmtl::Vec3f ptOrigine)
{
	// compute the plane normal
	gmtl::Vec3f planeNormal(a,b,c);
	
	// if the plane exists
	if (gmtl::length(planeNormal)!=0)
	{
		// create a single up vector
		gmtl::Vec3f upVector(0.0f,0.0f,1.0f);
		
		// get the vector to turn around
		gmtl::Vec3f rotationVector;
		gmtl::cross(rotationVector,planeNormal,upVector);
		gmtl::normalize(rotationVector);
		// get the angle of rotation
		float angRot=angle(planeNormal, upVector);
		
		// create the rotation
		gmtl::AxisAnglef ang(angRot, rotationVector[0], rotationVector[1], rotationVector[2]);
		gmtl::Quatf myQuat=gmtl::make<gmtl::Quatf>(ang);
		
		// rotate the point
		gmtl::Vec3f ptToRote=pt-ptOrigine;
		gmtl::Vec3f ptRote=myQuat*ptToRote;
		// point put it in the plane
		gmtl::Vec2f ptProj(ptRote[0],ptRote[1]);
		
		return ptProj;
	}
	else
	{
		printf("warning, plan inexistant - renvoie 0\n");
		return gmtl::Vec2f(0,0);
		exit(-1);
	}
}
	

// compute the angle between two vectors
float MathTools::angle(gmtl::Vec3f v0, gmtl::Vec3f v1)
{
	
	if (v0==v1)
		return 0;
	
	double rangle=gmtl::dot(v0,v1)/(gmtl::length(v0)*gmtl::length(v1));
	
	if (rangle>1 && rangle<1.005)
	{
		//printf("attention approximation de la virule à 5.10^-3\n");
		rangle=1;
	}
	
	if (rangle<-1 && rangle>-1.005)
	{
		//printf("attention approximation de la virule à -5.10^-3\n");
		rangle=-1;
	}
	
	rangle=acos(rangle);
	return rangle;
}
	

// compute the smallest angle in a triangle
float MathTools::angleMinInTri(gmtl::Vec3f * points)
{
	bool init=true;
	float angleMin=0;
	for (int j=0;j<3;j++)
	{
		float ang=MathTools::angle(points[(j+1)%3]-points[j],points[(j+2)%3]-points[j]);
		if (init || ang<angleMin)
			angleMin=ang;
		
		init=false;
	}
	return angleMin;
}
	
	
// compute the smallest distance from a point to a plane
float MathTools::distancePlanPoint(float a, float b, float c, float d, gmtl::Vec3f pt)
{
	return (fabs(a*pt[0]+b*pt[1]+c*pt[2]+d)/sqrt(a*a+b*b+c*c));
}
	

// return the d coordinates of a plane (ax + by + cz + d = 0)
float MathTools::dCoordsOfThePlan(gmtl::Vec3f planNormal, gmtl::Vec3f pt)
{
	return dCoordsOfThePlan(planNormal[0],planNormal[1],planNormal[2],pt);
}
	

// return the d coordinates of a plane (ax + by + cz + d = 0)
float MathTools::dCoordsOfThePlan(float a, float b, float c, gmtl::Vec3f pt)
{
	return -(a*pt[0]+b*pt[1]+c*pt[2]);
}
	

// test if there exists an intersection between a disk and a bowl
bool MathTools::intersectsBowlDisk(gmtl::Vec3f bowlCenter, float bowlRadius,
							   gmtl::Vec3f diskCenter, float diskRadius, float a, float b, float c, float d)
{
	// compute the distance from the bowlcenter to the plane
	float disPlPt=distancePlanPoint(a,b,c,d,bowlCenter);
	
	// if an intersection can exist
	if (disPlPt<=bowlRadius)
	{
		// compute the distance between the center of the projection of the central point of the bowl
		// and the intersection with the plane
		disPlPt=disPlPt/bowlRadius;
		float angle=acos(disPlPt);
		float rad=sin(angle);
		rad=rad*bowlRadius;
		
		// compute the position of the orthogonal projection of bowlCenter on the plane
		// diskCenter is the origin
		gmtl::Vec2f positionProjete=projectOrthogonal(a,b,c,bowlCenter,diskCenter);
		
		// test the distances
		if (gmtl::length(positionProjete) <= (rad+diskRadius))
		{
			return true;
		}
	}
	return false;
}
	

// compute the projection of a point onto a sphere
gmtl::Vec3f MathTools::projectionOnSphere(gmtl::Vec3f origine, gmtl::Vec3f direction, gmtl::Vec3f centerSphere, float radiusSphere)
{
	gmtl::Spheref maSphere(centerSphere, radiusSphere);
	gmtl::Rayf monRay(origine, direction);
	int numHits;
	float inter;
	float inter2;
	gmtl::Vec3f res(0,0,0);
	if (gmtl::intersect(maSphere,monRay,numHits,inter,inter2))
	{
		res=monRay.getOrigin() + monRay.getDir() * inter;
	}
	return res;
}
	

// compute the convex hull given a list of 3D points
void MathTools::convexHull(std::vector<PFP::EMB*> lstPoints, Marker newHull)
{
	Marker supp=myMap.getNewMarker(); // marker for darts to be deleted
	
	// creation of the first tetraedron
	Dart d = myMap.newOrientedFace(3);
	Dart side1 = myMap.newOrientedFace(3);
	myMap.phi2Sew(d,side1);
	Dart side2 = myMap.newOrientedFace(3);
	myMap.phi2Sew(myMap.phi1(d),side2);
	myMap.phi2Sew(myMap.phi_1(side1), myMap.phi1(side2));
	Dart side3 = myMap.newOrientedFace(3);
	myMap.phi2Sew(myMap.phi_1(d),side3);
	myMap.phi2Sew(myMap.phi_1(side2), myMap.phi1(side3));
	myMap.phi2Sew(myMap.phi_1(side3), myMap.phi1(side1));
	
	// mark the first tet as part of the new hull
	myMap.markCell(2,d,newHull);
	myMap.markCell(2,side1,newHull);
	myMap.markCell(2,side2,newHull);
	myMap.markCell(2,side3,newHull);
	
	// embed the tet
	myMap.embedCell(0,d,0,lstPoints[0]);
	myMap.embedCell(0,myMap.phi1(d),0,lstPoints[1]);
	myMap.embedCell(0,myMap.phi_1(d),0,lstPoints[2]);
	myMap.embedCell(0,myMap.phi<2,-1>(d),0,lstPoints[lstPoints.size()-1]);
	
	if (lstPoints.size() < 6)
		cerr << "pas assez de points pour l'EC" << endl;
	
	for (unsigned int i=3;i<lstPoints.size()-1;i++)
	{		
		PFP::EMB* ptCur=lstPoints[i];
		bool outside=false;
		
		for (Dart dCur=myMap.begin();dCur!=myMap.end();++dCur) // for all the darts of the map
		{
			if (myMap.isMarkedDart(dCur,newHull)) // if the current dart is marked as part of the new hull
			{
				gmtl::Vec3f norm=computeNormal(dCur);
				gmtl::Vec3f bary=computeBary(dCur);
				gmtl::Vec3f norm2=ptCur->getPosition() - bary;
				double dot=gmtl::dot(norm, norm2);
				if (dot>0.0) // test whether the point is outside of the convex hull
				{
					outside=true;
					myMap.markCell(2,dCur,supp); // if yes, mark the cell to be deleted
				}
			}
		}
		
		if (outside) // if we are outside of the CH
		{	
			std::vector<Dart> toDel;
			std::vector<Dart> border;
			
			for (Dart dCur=myMap.begin();dCur!=myMap.end();++dCur) // for all darts
			{
				if (myMap.isMarkedDart(dCur,supp)) // if the current dart is marked to be deleted
				{
					if (!myMap.isMarkedDart(myMap.phi2(dCur),supp)) // and its phi2 neighbor is not
					{
						border.push_back(myMap.phi2(dCur)); // add the dart's phi2 neighbor to the border
					}
					toDel.push_back(dCur);
				}
			}
			
			// delete all darts marked to be deleted
			for (std::vector<Dart>::iterator it=toDel.begin();it!=toDel.end();++it)
			{
				myMap.phiUnsew(1,myMap.phi_1(*it));
				myMap.phiUnsew(2,*it);
				myMap.eraseDart(*it);
			}
			
			Dart dSav;
			Marker newDart=myMap.getNewMarker();
			
			for (std::vector<Dart>::iterator it=border.begin();it!=border.end();++it)
			{
				Dart dNew=myMap.newOrientedFace(3);
				myMap.markDart(dNew,newDart);
				myMap.markCell(2,dNew,newHull);
				dSav=dNew;
				myMap.phi2Sew(dNew,*it);
			}
			
			for (std::vector<Dart>::iterator it=border.begin();it!=border.end();++it)
			{
				Dart cur=*it;
				Dart next=myMap.phi_1(cur);
				while (!myMap.isMarkedDart(myMap.phi2(next),newDart))
				{
					next=myMap.phi<2,-1>(next);
				}
				myMap.phi2Sew(myMap.phi<2,1>(cur),myMap.phi<2,-1>(next));
				
			}
			myMap.embedCell(0,myMap.phi_1(dSav),0,ptCur);
			
			for (Dart dCur=myMap.begin();dCur!=myMap.end();++dCur)
			{
				if (myMap.isMarkedDart(dCur,newDart))
				{
					myMap.embedCell(0,myMap.phi_1(dCur),0,ptCur); 
				}
			}
			
			for (std::vector<Dart>::iterator it=border.begin();it!=border.end();++it)
			{
				Dart cur=*it;
				myMap.embedCell(0,cur,0,myMap.getVertexEmb(cur));
			}
			myMap.clearMarkers(newDart);
			myMap.releaseMarker(newDart);
		}
	}
	myMap.clearMarkers(supp);
	myMap.releaseMarker(supp);
}



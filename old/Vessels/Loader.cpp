
#include "Headers/Loader.h"
#include<list>

/******************************************************************************/
/*************************      LOADER CLASS FUNCTIONS		*******************/
/******************************************************************************/

// constructor
Loader::Loader(string filename)
{
	ifstream file(filename.c_str());
	
	bool init=true;
	float xmin=0;
	float xmax=0;
	float ymin=0;
	float ymax=0;
	float zmin=0;
	float zmax=0;
	
	string line;
	string delim("label[(=)];:> \t\"\n\rGrph}{g-");
	while( getline( file, line ) )
	{
		StringTokenizer stk(line,delim);
		
		if (line.find("((",0)!=string::npos)
		{
		}
		else
		{
			if (stk.hasMoreTokens())
			{
				Vessel ves;
				ves.idStart=atoi((stk.nextToken()).c_str());
				ves.idStop=atoi((stk.nextToken()).c_str());
				
				while (stk.hasMoreTokens())
				{
					VesselRadius vr;
					
					vr.x=atof((stk.nextToken()).c_str());
					vr.y=atof((stk.nextToken()).c_str());
					vr.z=atof((stk.nextToken()).c_str());
					
					stk.nextToken();
					stk.nextToken();
					vr.radius=atof((stk.nextToken()).c_str());
					stk.nextToken();
					stk.nextToken();
					stk.nextToken();
					
					if (init || vr.x<xmin)
						xmin=vr.x;
					if (init || vr.y<ymin)
						ymin=vr.y;
					if (init || vr.z<zmin)
						zmin=vr.z;
					if (init || vr.x>xmax)
						xmax=vr.x;
					if (init || vr.y>ymax)
						ymax=vr.y;
					if (init || vr.z>zmax)
						zmax=vr.z;
					
					init=false;
					ves.addVesselRadius(vr);
				}
				listVessels.push_back(ves);
			}
		}
	}
	
	decX=(xmin+xmax)/2.0f;
	decY=(ymin+ymax)/2.0f;
	decZ=(zmin+zmax)/2.0f;
}


// return the size of a face
int Loader::sizeFace(Dart d)
{
	int taille=0;
	Dart dd=d;
	do
	{
		dd=myMap.phi1(dd);
		taille++;
	}
	while (dd!=d);
	
	return taille;
}


// test if it is a triangle
bool Loader::isTriangle(Dart d)
{
	return (myMap.phi<1,1,1>(d)==d);
}


// test if two triangles are equal (i.e. whether they have the same embedding)
bool Loader::sameTriangleEmbedding(Dart d, Dart e)
{
	return ( isTriangle(d) && isTriangle(e) 
			&& (myMap.getVertexEmb(d)->getPosition() == myMap.getVertexEmb(e)->getPosition()) 
			&& (myMap.getVertexEmb(myMap.phi1(d))->getPosition() == myMap.getVertexEmb(myMap.phi_1(e))->getPosition()) 
			&& (myMap.getVertexEmb(myMap.phi_1(d))->getPosition() == myMap.getVertexEmb(myMap.phi1(e))->getPosition()) );
}


// get the number of intersections of the graph
void Loader::getIntersectionInfo()
{
	nbIntersections = -1;
	bool init=true;
	for (std::vector<Vessel>::iterator itVessel=listVessels.begin();itVessel!=listVessels.end();itVessel++)
	{
		unsigned int id0=(itVessel)->idStart;
		unsigned int id1=(itVessel)->idStop;
		
		if (init || id0>nbIntersections)
			nbIntersections=id0;
		if (init || id1>nbIntersections)
			nbIntersections=id1;
		
		init=false;
	}
	nbIntersections++;
	
	nbConnected = new int[nbIntersections];
	
	for (unsigned int i=0;i<nbIntersections;i++)
		nbConnected[i]=0;
	
	for (unsigned int i=0;i<listVessels.size();i++)
	{
		Vessel ves=listVessels[i];
		nbConnected[ves.idStart]++;
		nbConnected[ves.idStop]++;
	}
}


// cut the vessels to avoid undesired (geometric) intersections
void Loader::computeDecalOfVessels()
{
	// get the number of intersections of the graph
	getIntersectionInfo();
	
	for (unsigned int indexIntersection=0;indexIntersection<nbIntersections;indexIntersection++)
	{
		// find the vessels that are connected to this intersection
		std::vector<int> connected;
		for (unsigned int i=0;i<listVessels.size();i++)
		{
			Vessel ve=listVessels[i];
			if (ve.idStart==indexIntersection || ve.idStop==indexIntersection)
				connected.push_back(i);
		}
		
		// for each of them
		for (unsigned int i=0;i<connected.size();i++)
		{
			Vessel vesselCur=listVessels[connected[i]];
			unsigned int decalage;
			
			if (vesselCur.idStart==indexIntersection)
				decalage=vesselCur.decalFromStart;
			else
				decalage=vesselCur.decalFromStop;
			
			// test with all the others
			for (unsigned int j=0;j<connected.size();j++)
			{
				if (i!=j)
				{
					// determine the extremum points for the iteration
					Vessel vesselTest=listVessels[connected[j]];
					bool intersect=true;
					int delta;
					int idCur;
					int idArr;
					if (vesselTest.idStart==indexIntersection)
					{
						delta=1;
						idCur=0;
						idArr=(vesselTest.lstVesselRadius).size();
					}
					else
					{
						delta=-1;
						idCur=(vesselTest.lstVesselRadius).size()-1;
						idArr=-1;
					}
					
					// while there is no intersection and we haven't tested all the possible intersections
					while (intersect && decalage < vesselCur.lstVesselRadius.size())
					{
						gmtl::Vec3f planNormale;
						gmtl::Vec3f origineDisk;
						float radiusDisk;
						
						// get the equation of the disk that we will test for the intersection
						if (decalage==0)
						{
							VesselRadius v0;
							VesselRadius v1;
							if (vesselCur.idStart==indexIntersection)
							{
								v0=vesselCur.lstVesselRadius[0];
								v1=vesselCur.lstVesselRadius[1];
							}
							else
							{
								v0=vesselCur.lstVesselRadius[vesselCur.lstVesselRadius.size()-1];
								v1=vesselCur.lstVesselRadius[vesselCur.lstVesselRadius.size()-2];
							}
							
							planNormale=gmtl::Vec3f(v0.x-v1.x,v0.y-v1.y,v0.z-v1.z);
							radiusDisk=v0.radius;
							origineDisk=gmtl::Vec3f(v0.x,v0.y,v0.z);
						}
						else
						{
							VesselRadius v0;
							VesselRadius v1;
							VesselRadius v2;
							if (vesselCur.idStart==indexIntersection)
							{
								v0=vesselCur.lstVesselRadius[(decalage)-1];
								v1=vesselCur.lstVesselRadius[(decalage)];
								v2=vesselCur.lstVesselRadius[(decalage)+1];
							}
							else
							{
								v0=vesselCur.lstVesselRadius[(vesselCur.lstVesselRadius.size()-1)-(decalage)-1];
								v1=vesselCur.lstVesselRadius[(vesselCur.lstVesselRadius.size()-1)-(decalage)];
								v2=vesselCur.lstVesselRadius[(vesselCur.lstVesselRadius.size()-1)-(decalage)+1];
							}
							
							gmtl::Vec3f vn0(v0.x-v1.x,v0.y-v1.y,v0.z-v1.z);
							gmtl::Vec3f vn1(v1.x-v2.x,v1.y-v2.y,v1.z-v2.z);
							
							planNormale=(vn0+vn1)/2.0f;
							radiusDisk=v1.radius;
							origineDisk=gmtl::Vec3f(v1.x,v1.y,v1.z);
						}
						
						gmtl::normalize(planNormale);
						float d=MathTools::dCoordsOfThePlan(planNormale,origineDisk);
						
						intersect=false;
						
						// verify the intersection with all the spheres of the possible cutting vessels
						for(int k=idCur;k!=idArr && !intersect;k+=delta)
						{
							VesselRadius vrTest=vesselTest.lstVesselRadius[k];
							gmtl::Vec3f centerBowl(vrTest.x,vrTest.y,vrTest.z);
							intersect=MathTools::intersectsBowlDisk(centerBowl,vrTest.radius,
																	origineDisk,radiusDisk,
																	planNormale[0],planNormale[1],planNormale[2],d);
						}
						// if there is an intersection, try the next point of the current vessel
						if (intersect)
						{
							decalage++;
						}
					}
				}
			}
			
			if (vesselCur.idStart==indexIntersection)
				vesselCur.decalFromStart=decalage;
			else
				vesselCur.decalFromStop=decalage;
			
			listVessels[connected[i]] = vesselCur;
		}
	}
	
	for (unsigned int i=0;i<listVessels.size();i++)
	{
		Vessel ve=listVessels[i];
		if ((ve.decalFromStart+ve.decalFromStop) > (ve.lstVesselRadius.size()-1))
		{
			int taille=ve.lstVesselRadius.size();
			if (taille%2 == 0)
			{
				ve.decalFromStart=(taille/2)-1;
				ve.decalFromStop=(taille/2)-1;
			}
			else
			{
				ve.decalFromStart=((int) (taille/2));
				ve.decalFromStop=((int) (taille/2))-1;
			}
			ve.reduced=true;
			listVessels[i] = ve;
		}
	}
}


// construct the extruded vessels (excluding the intersections)
void Loader::extrudeVessels()
{
	Vessel ve;
	
	for (unsigned int i=0; i<listVessels.size(); i++)
	{
		std::vector<gmtl::Vec3f> objV;
		std::vector<gmtl::Vec3f> pathV;
		std::vector<float> pathRadius;
		gmtl::Vec3f vesselPiece, tangentFirst, tangentLast, centerFirst, centerLast;
		
		ve = listVessels[i]; // ve is a vessel
		ve.simplify(); // simplify each vessel
		
		// compute the first vessel's tangent -> the normal in the extrusion
		VesselRadius vr0 = ve.lstVesselRadius[ve.decalFromStart];
		VesselRadius vr1 = ve.lstVesselRadius[ve.decalFromStart+1];
		tangentFirst = gmtl::Vec3f(vr1.x-vr0.x, vr1.y-vr0.y, vr1.z-vr0.z);
		gmtl::normalize(tangentFirst);
		
		// compute the last vessel's tangent
		VesselRadius vrLast = ve.lstVesselRadius[ve.lstVesselRadius.size()-ve.decalFromStop-1];
		VesselRadius vrLast2 = ve.lstVesselRadius[ve.lstVesselRadius.size()-ve.decalFromStop-2];
		tangentLast = gmtl::Vec3f(vrLast.x-vrLast2.x, vrLast.y-vrLast2.y, vrLast.z-vrLast2.z);
		gmtl::normalize(tangentLast);
		
		// compute the first triangle -> the input object in the extrusion
		gmtl::Vec3f firstTrianglePoint = MathTools::computeOrthogonalVector(tangentFirst);
		const float angleTwoThirdsPi = gmtl::Math::PI * (2./3.f);
		const float angleFourThirdsPi = gmtl::Math::PI * (4./3.f);
		gmtl::Vec3f secondTrianglePoint =  MathTools::rotationQuat(tangentFirst, angleTwoThirdsPi) * firstTrianglePoint;
		gmtl::Vec3f thirdTrianglePoint = MathTools::rotationQuat(tangentFirst, angleFourThirdsPi) * firstTrianglePoint;
		
		// compute the last triangle
		gmtl::Vec3f lastTrianglePoint = MathTools::computeOrthogonalVector(tangentLast);
		gmtl::Vec3f secondLastTrianglePoint = MathTools::rotationQuat(tangentLast, angleTwoThirdsPi) * lastTrianglePoint;
		gmtl::Vec3f thirdLastTrianglePoint = MathTools::rotationQuat(tangentLast, angleFourThirdsPi) * lastTrianglePoint;
		
		// compute the centers (first and last)
		centerFirst = gmtl::Vec3f(vr0.x, vr0.y, vr0.z);
		centerLast = gmtl::Vec3f(vrLast.x, vrLast.y, vrLast.z);
		
		// add the first triangle in the object to be extruded
		objV.push_back(centerFirst + vr0.radius*firstTrianglePoint);
		objV.push_back(centerFirst + vr0.radius*secondTrianglePoint);
		objV.push_back(centerFirst + vr0.radius*thirdTrianglePoint);
		
		
		// for all pieces of a vessel (correctly shifted)
		for (unsigned int j=ve.decalFromStart; j<ve.lstVesselRadius.size()-ve.decalFromStop; j++)
		{
			VesselRadius vr = ve.lstVesselRadius[j];
			vesselPiece = gmtl::Vec3f(vr.x, vr.y, vr.z);
			
			// add the coordinates and radius of the vessel's piece to the path
			pathV.push_back(vesselPiece); 
			pathRadius.push_back(vr.radius/vr0.radius); 
		}
		
		// perform the extrusion of the vessel network
		if (pathV.size() > 1) // won't work if the vessel's width is smaller than 1
		{
			Algo::Modelisation::Primitive<PFP> *extru = Algo::Modelisation::extrusion_scale_prim<PFP>(myMap, objV, centerFirst, tangentFirst, true, pathV, false, pathRadius);
			std::vector<Dart> vecDarts = extru->getVertexDarts();
			
			// add the first triangle in the list of coordinates for the convex hull
			ve.lstCoordinates.push_back(myMap.getVertexEmb(vecDarts[0])->getPosition());
			ve.lstCoordinates.push_back(myMap.getVertexEmb(vecDarts[1])->getPosition());
			ve.lstCoordinates.push_back(myMap.getVertexEmb(vecDarts[2])->getPosition());
			
			// add the last triangle in the list of coordinates for the convex hull
			ve.lstCoordinates.push_back(myMap.getVertexEmb(vecDarts[vecDarts.size()-3])->getPosition());
			ve.lstCoordinates.push_back(myMap.getVertexEmb(vecDarts[vecDarts.size()-2])->getPosition());
			ve.lstCoordinates.push_back(myMap.getVertexEmb(vecDarts[vecDarts.size()-1])->getPosition());
			
			ve.nbFaces = 3;			
			
		}
		else // in the case of an extrusion of thickness 0
		{
			ve.lstCoordinates.push_back(centerFirst + vr0.radius*firstTrianglePoint);
			ve.lstCoordinates.push_back(centerFirst + vr0.radius*secondTrianglePoint);
			ve.lstCoordinates.push_back(centerFirst + vr0.radius*thirdTrianglePoint);
			
			ve.lstCoordinates.push_back(centerFirst + vr0.radius*firstTrianglePoint);
			ve.lstCoordinates.push_back(centerFirst + vr0.radius*secondTrianglePoint);
			ve.lstCoordinates.push_back(centerFirst + vr0.radius*thirdTrianglePoint);
			ve.nbFaces = 3;
			
		}
		listVessels[i] = ve;
	}
	
	// close the map and perform 0-embedding of the new vertices
	// !!! the extrusion functions DO NOT have the option of closing the extruded objects !!!
	myMap.closeMap();
	for (Dart d=myMap.begin();d!=myMap.end();++d)
		if (myMap.getEmbedding(d,0)!=NULL)
			myMap.embedCell(0,d,0,myMap.getEmbedding(d,0));
}


// construct the intersections between the extruded vessels
void Loader::constructIntersection()
{
	// get the number of intersections of the graph
	getIntersectionInfo();
	
	for (unsigned int indexIntersection=0;indexIntersection<nbIntersections;indexIntersection++)
	{
		std::vector<int> connectedFromBegin;
		std::vector<int> connectedFromEnd;
		
		for (unsigned int j=0;j<listVessels.size();j++)
		{
			Vessel ve=listVessels[j];
			
			if (ve.idStart==indexIntersection)
				connectedFromBegin.push_back(j);
			if (ve.idStop==indexIntersection)
				connectedFromEnd.push_back(j);
		}
		
		if (connectedFromBegin.size()+connectedFromEnd.size()>1) // test if it is an intersection and not an end of vessel
		{
			std::vector<gmtl::Vec3f> lstPointsOfHull;
			std::vector<gmtl::Vec3f> lstPointsOfHullSave;
			gmtl::Vec3f positionIntersection(0,0,0);
			bool init=true;
			
			std::vector<int> lstNumSides;
			std::vector<gmtl::Vec3f> initCoords;
			
			// get the coordinates of the points of the vessel that will compose the intersection
			for (unsigned int j=0;j<connectedFromBegin.size();j++)
			{
				Vessel ve=listVessels[connectedFromBegin[j]];
				nbFaces=ve.nbFaces;
				
				if (init) // if it is the first vessel we test, we get the coordinates of the central point
				{
					VesselRadius vr=ve.lstVesselRadius[0];
					positionIntersection=gmtl::Vec3f(vr.x,vr.y,vr.z);
					init=false;
				}
				
				gmtl::Vec3f trans(0,0,0);
				
				if (ve.reduced)
				{
					VesselRadius vr=ve.lstVesselRadius[ve.decalFromStart];
					trans=gmtl::Vec3f(vr.x-positionIntersection[0],vr.y-positionIntersection[1],
									  vr.z-positionIntersection[2]);
					gmtl::normalize(trans);
					trans=trans*100.f;
				}
				
				for (unsigned int k=0;k<nbFaces;k++)
				{
					initCoords.push_back(ve.lstCoordinates[k]);
					lstPointsOfHull.push_back(ve.lstCoordinates[k]+trans);
				}
				lstNumSides.push_back(nbFaces);
			}
			
			// get the coordinates of the points of the vessels that will compose the intersection
			for (unsigned int j=0;j<connectedFromEnd.size();j++)
			{
				Vessel ve=listVessels[connectedFromEnd[j]];
				nbFaces=ve.nbFaces;
				
				if (init) // if it is the last vessel we test, we get the coordinates of the central point
				{
					VesselRadius vr=ve.lstVesselRadius[ve.lstVesselRadius.size()-1];
					positionIntersection=gmtl::Vec3f(vr.x,vr.y,vr.z);
					init=false;
				}
				
				gmtl::Vec3f trans(0,0,0);
				
				if (ve.reduced)
				{
					VesselRadius vr=ve.lstVesselRadius[ve.lstVesselRadius.size()-1-ve.decalFromStop];
					trans=gmtl::Vec3f(vr.x-positionIntersection[0],vr.y-positionIntersection[1]
									  ,vr.z-positionIntersection[2]);
					gmtl::normalize(trans);
					trans=trans*100.f;
				}
				
				for (int k=nbFaces-1;k>=0;k--)
				{
					initCoords.push_back(ve.lstCoordinates[ve.lstCoordinates.size()-1-k]);
					lstPointsOfHull.push_back(ve.lstCoordinates[ve.lstCoordinates.size()-1-k]+trans);
				}
				lstNumSides.push_back(nbFaces);
			}
			
			// ********************* beginning of the sphere intersection scheme *********************
			
			// get the furthest point of the center
			float distanceMax=0;
			for (unsigned int j=0;j<lstPointsOfHull.size();j++)
			{
				gmtl::Vec3f pointsCurrent=lstPointsOfHull[j];
				gmtl::Vec3f dis=positionIntersection-pointsCurrent;
				float distance=gmtl::length(dis);
				if (distance>distanceMax)
					distanceMax=distance;
			}
			
			int idCurSide=0;
			
			for (unsigned int j=0;j<lstPointsOfHull.size();j++)
				lstPointsOfHullSave.push_back(lstPointsOfHull[j]);
			
			// projection of the points on a sphere to be used for convex hull
			for (unsigned int j=0;j<lstPointsOfHull.size();j+=lstNumSides[idCurSide-1])
			{
				nbFaces=lstNumSides[idCurSide];
				idCurSide++;
				
				gmtl::Vec3f bary(0,0,0);
				for (unsigned int k=0;k<nbFaces;k++)
					bary+=lstPointsOfHull[j+k];
				bary=bary/(float) nbFaces;
				
				gmtl::Vec3f decalage=bary-positionIntersection;
				gmtl::normalize(decalage);
				
				for (unsigned int k=0;k<nbFaces;k++)
				{
					gmtl::Vec3f direction=lstPointsOfHull[j+k]-positionIntersection;
					gmtl::normalize(direction);
					gmtl::Vec3f newPos = MathTools::projectionOnSphere(lstPointsOfHull[j+k],decalage,positionIntersection,distanceMax*2.0);
					lstPointsOfHull[j+k]=newPos;
				}
			}
			// ********************* end of the sphere intersection scheme *********************
			
			
			// set the vec3f to point3f for embedding
			std::vector<Point3D *> lstPoints;
			for (unsigned int j=0;j<lstPointsOfHull.size();j++)
			{
				Point3D * tmp=Point3D::create(lstPointsOfHull[j]);
				CGoGN::Emb::RefCountObject::ref(tmp);
				lstPoints.push_back(tmp);
			}
			
			idCurSide=0;
			Marker hullMark=myMap.getNewMarker();
			
			// add all the points to the hull
			MathTools::convexHull(lstPoints,hullMark); 
			
			// set the coordinates at the right position
			for (unsigned int j=0;j<lstPointsOfHull.size();j++) 
				lstPoints[j]->setPosition(initCoords[j]);
			
			// clear markers
			FunctorUnmark<PFP::MAP> fHullUnMark(myMap,hullMark);
			myMap.foreach_dart(fHullUnMark);
			myMap.releaseMarker(hullMark);
		}
	}
}


// merge the vessels into one single object
void Loader::mergeVessels()
{
	Marker markTreated=myMap.getNewMarker();
	FunctorMark<PFP::MAP> fm(myMap,markTreated);
	std::list<Dart> listDarts;
	
	// get the list of triangle darts that have the same embedding
	for (Dart d=myMap.begin();d!=myMap.end();d++)
	{
		if (!myMap.isMarkedDart(d,markTreated))
		{
			myMap.foreach_dart_of_face(d,fm);
			
			for (Dart e=myMap.begin();e!=myMap.end();e++)
			{
				if (!myMap.isMarkedDart(e,markTreated) && e!=d && sameTriangleEmbedding(d,e))
				{
					myMap.foreach_dart_of_face(e,fm);
					listDarts.push_back(d);
					listDarts.push_back(e);
				}
			}
		}
	}
	
	// sew the volumes together
	for(std::list<Dart>::iterator it=listDarts.begin();it!=listDarts.end();++it)
	{
		Dart d0=*it;
		++it;
		Dart d1=*it;
		myMap.sewVolumes(d0,myMap.phi_1(d1));
	}
	
	// clear markers
	myMap.clearMarkers(markTreated);
	myMap.releaseMarker(markTreated);
	
	// verify that all darts are 0-embedded
	for (Dart d=myMap.begin();d!=myMap.end();++d)
		if (myMap.getEmbedding(d,0)!=NULL)
			myMap.embedCell(0,d,0,myMap.getEmbedding(d,0));
}


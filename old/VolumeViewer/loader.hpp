
#include "loader.h"
#include "Algo/Geometry/centroid.h"
#include "Geometry/basic.h"
#include<list>

/******************************************************************************/
/*************************      LOADER CLASS FUNCTIONS		*******************/
/******************************************************************************/

namespace CGoGN
{

// constructor
template <typename PFP>
Loader<PFP>::Loader(typename PFP::MAP& map, VertexAttribute<VEC3, MAP>& positions, std::string filename) : m_map(map), m_positions(positions)
{
	std::ifstream file(filename.c_str());
	
	bool init=true;
	float xmin=0;
	float xmax=0;
	float ymin=0;
	float ymax=0;
	float zmin=0;
	float zmax=0;
	
	std::string line;
	std::string delim("label[(=)];:> \t\"\n\rGrph}{g-");
	while( std::getline( file, line ) )
	{
		StringTokenizer stk(line,delim);
		
		if (line.find("((",0)!=std::string::npos)
		{
		}
		else
		{
			if (stk.hasMoreTokens())
			{
				Vessel<VEC3> ves;
				ves.idStart= std::atoi((stk.nextToken()).c_str());
				ves.idStop=std::atoi((stk.nextToken()).c_str());
				
				while (stk.hasMoreTokens())
				{
					VesselRadius vr;
					
					vr.x=std::atof((stk.nextToken()).c_str());
					vr.y=std::atof((stk.nextToken()).c_str());
					vr.z=std::atof((stk.nextToken()).c_str());
					
					stk.nextToken();
					stk.nextToken();
					vr.radius=std::atof((stk.nextToken()).c_str());
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

					std::cout << vr.x << std::endl;
					std::cout << vr.y << std::endl;
					std::cout << vr.z << std::endl;
					std::cout << vr.radius << std::endl;
					std::cout << std::endl;
				}
				listVessels.push_back(ves);

				std::cout << "end vessel" << std::endl;
			}
		}
	}
	
	decX=(xmin+xmax)/2.0f;
	decY=(ymin+ymax)/2.0f;
	decZ=(zmin+zmax)/2.0f;
}


// return the size of a face
template <typename PFP>
int Loader<PFP>::sizeFace(Dart d)
{
	int taille=0;
	Dart dd=d;
	do
	{
		dd=m_map.phi1(dd);
		taille++;
	}
	while (dd!=d);
	
	return taille;
}


// test if it is a triangle
template <typename PFP>
bool Loader<PFP>::isTriangle(Dart d)
{
	return (m_map.phi1(m_map.phi1(m_map.phi1(d)))) == d;
}


// test if two triangles are equal (i.e. whether they have the same embedding)
template <typename PFP>
bool Loader<PFP>::sameTriangleEmbedding(Dart d, Dart e)
{
	return ( isTriangle(d) && isTriangle(e) 
			 && (m_positions[d] == m_positions[e])
			 && (m_positions[m_map.phi1(d)] == m_positions[m_map.phi_1(e)])
			 && (m_positions[m_map.phi_1(d)] == m_positions[m_map.phi1(e)] ));
//			&& (m_map.template getEmbedding<VERTEX>(d) == m_map.template getEmbedding<VERTEX>(e))
//			&& (m_map.template getEmbedding<VERTEX>(m_map.phi1(d)) == m_map.template getEmbedding<VERTEX>(m_map.phi_1(e)))
//			&& (m_map.template getEmbedding<VERTEX>(m_map.phi_1(d)) == m_map.template getEmbedding<VERTEX>(m_map.phi1(e))) );
}


// get the number of intersections of the graph
template <typename PFP>
void Loader<PFP>::getIntersectionInfo()
{
	nbIntersections = -1;
	bool init=true;
	for(unsigned int i = 0 ; i < listVessels.size() ; i++)
	{
		unsigned int id0=listVessels[i].idStart;
		unsigned int id1=listVessels[i].idStop;
		
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
		Vessel<VEC3> ves=listVessels[i];
		nbConnected[ves.idStart]++;
		nbConnected[ves.idStop]++;
	}
}


// cut the vessels to avoid undesired (geometric) intersections
template <typename PFP>
void Loader<PFP>::computeDecalOfVessels()
{
	// get the number of intersections of the graph
	getIntersectionInfo();
	
	for (unsigned int indexIntersection=0;indexIntersection<nbIntersections;indexIntersection++)
	{
		// find the vessels that are connected to this intersection
		std::vector<int> connected;
		for (unsigned int i=0;i<listVessels.size();i++)
		{
			Vessel<VEC3> ve=listVessels[i];
			if (ve.idStart==indexIntersection || ve.idStop==indexIntersection)
				connected.push_back(i);
		}
		
		// for each of them
		for (unsigned int i=0;i<connected.size();i++)
		{
			Vessel<VEC3> vesselCur=listVessels[connected[i]];
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
					Vessel<VEC3> vesselTest=listVessels[connected[j]];
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
						VEC3 planNormale;
						VEC3 origineDisk;
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
							
							planNormale=VEC3(v0.x-v1.x,v0.y-v1.y,v0.z-v1.z);
							radiusDisk=v0.radius;
							origineDisk=VEC3(v0.x,v0.y,v0.z);
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
							
							VEC3 vn0(v0.x-v1.x,v0.y-v1.y,v0.z-v1.z);
							VEC3 vn1(v1.x-v2.x,v1.y-v2.y,v1.z-v2.z);
							
							planNormale=(vn0+vn1)/2.0f;
							radiusDisk=v1.radius;
							origineDisk=VEC3(v1.x,v1.y,v1.z);
						}
						
						planNormale.normalize();						
						float d=MathTools::dCoordsOfThePlan(planNormale,origineDisk);
						
						intersect=false;
						
						// verify the intersection with all the spheres of the possible cutting vessels
						for(int k=idCur;k!=idArr && !intersect;k+=delta)
						{
							VesselRadius vrTest=vesselTest.lstVesselRadius[k];
							VEC3 centerBowl(vrTest.x,vrTest.y,vrTest.z);							
							intersect=MathTools::intersectsBowlDisk<QUAT,AA,VEC3>(centerBowl,vrTest.radius,
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
		Vessel<VEC3> ve=listVessels[i];
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
template <typename PFP>
void Loader<PFP>::extrudeVessels()
{
	Vessel<VEC3> ve;
	
	for (unsigned int i=0; i<listVessels.size(); i++)
	{
		std::vector<VEC3> objV;
		std::vector<VEC3> pathV;
		std::vector<float> pathRadius;
		VEC3 vesselPiece, tangentFirst, tangentLast, centerFirst, centerLast;
		
		ve = listVessels[i]; // ve is a vessel
		ve.simplify(); // simplify each vessel
		
		// compute the first vessel's tangent -> the normal in the extrusion
		VesselRadius vr0 = ve.lstVesselRadius[ve.decalFromStart];
		VesselRadius vr1 = ve.lstVesselRadius[ve.decalFromStart+1];
		tangentFirst = VEC3(vr1.x-vr0.x, vr1.y-vr0.y, vr1.z-vr0.z);
		tangentFirst.normalize();
		
		// compute the last vessel's tangent
		VesselRadius vrLast = ve.lstVesselRadius[ve.lstVesselRadius.size()-ve.decalFromStop-1];
		VesselRadius vrLast2 = ve.lstVesselRadius[ve.lstVesselRadius.size()-ve.decalFromStop-2];
		tangentLast = VEC3(vrLast.x-vrLast2.x, vrLast.y-vrLast2.y, vrLast.z-vrLast2.z);
		tangentLast.normalize();
		
		// compute the first triangle -> the input object in the extrusion
		VEC3 firstTrianglePoint = MathTools::computeOrthogonalVector(tangentFirst);
		const float angleTwoThirdsPi = M_PI * (2./3.f);
		const float angleFourThirdsPi = M_PI * (4./3.f);		
		VEC3 secondTrianglePoint = MathTools::transform<QUAT, VEC3>(MathTools::rotationQuat<QUAT, AA, VEC3>(tangentFirst, angleTwoThirdsPi), firstTrianglePoint);
		VEC3 thirdTrianglePoint = MathTools::transform<QUAT, VEC3>(MathTools::rotationQuat<QUAT, AA, VEC3>(tangentFirst, angleFourThirdsPi), firstTrianglePoint);
		
		// compute the last triangle
		VEC3 lastTrianglePoint = MathTools::computeOrthogonalVector(tangentLast);
		VEC3 secondLastTrianglePoint = MathTools::transform<QUAT, VEC3>(MathTools::rotationQuat<QUAT, AA, VEC3>(tangentLast, angleTwoThirdsPi), lastTrianglePoint);
		VEC3 thirdLastTrianglePoint = MathTools::transform<QUAT, VEC3>(MathTools::rotationQuat<QUAT, AA, VEC3>(tangentLast, angleFourThirdsPi), lastTrianglePoint);
		
		// compute the centers (first and last)
		centerFirst = VEC3(vr0.x, vr0.y, vr0.z);
		centerLast = VEC3(vrLast.x, vrLast.y, vrLast.z);
		
		// add the first triangle in the object to be extruded
		objV.push_back(centerFirst + vr0.radius*firstTrianglePoint);
		objV.push_back(centerFirst + vr0.radius*secondTrianglePoint);
		objV.push_back(centerFirst + vr0.radius*thirdTrianglePoint);

		
		// for all pieces of a vessel (correctly shifted)
		for (unsigned int j=ve.decalFromStart; j<ve.lstVesselRadius.size()-ve.decalFromStop; j++)
		{
			VesselRadius vr = ve.lstVesselRadius[j];
			vesselPiece = VEC3(vr.x, vr.y, vr.z);
			
			// add the coordinates and radius of the vessel's piece to the path
			pathV.push_back(vesselPiece); 
			pathRadius.push_back(vr.radius/vr0.radius); 
		}
		
		// perform the extrusion of the vessel network
		if (pathV.size() > 1) // won't work if the vessel's width is smaller than 1
		{
			Algo::Surface::Tilings::Tiling<PFP>* extru = Algo::Surface::Modelisation::extrusion_scale_prim<PFP>(m_map, m_positions, objV, centerFirst, tangentFirst, true, pathV, false, pathRadius);

			std::vector<Dart> vecDarts = extru->getVertexDarts();
/*
			if(pathV.size() > 2)
			{
				for(unsigned int i = 0 ; i <  vecDarts.size() - 6; i=i+3)
				{
					std::vector<Dart> vec;
					vec.push_back(m_map.phi1(m_map.phi1(vecDarts[i])));
					vec.push_back(m_map.phi1(m_map.phi2(m_map.phi_1(vecDarts[i]))));
					vec.push_back(m_map.phi_1(m_map.phi2(m_map.phi1(vecDarts[i]))));
					m_map.splitVolume(vec);
				}
			}
*/
			// add the first triangle in the list of coordinates for the convex hull
			ve.lstCoordinates.push_back(m_positions[vecDarts[0]]);
			ve.lstCoordinates.push_back(m_positions[vecDarts[1]]);
			ve.lstCoordinates.push_back(m_positions[vecDarts[2]]);

			// add the last triangle in the list of coordinates for the convex hull
			ve.lstCoordinates.push_back(m_positions[vecDarts[vecDarts.size()-3]]);
			ve.lstCoordinates.push_back(m_positions[vecDarts[vecDarts.size()-2]]);
			ve.lstCoordinates.push_back(m_positions[vecDarts[vecDarts.size()-1]]);


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

}


// construct the intersections between the extruded vessels
template <typename PFP>
void Loader<PFP>::constructIntersection()
{
	// get the number of intersections of the graph
	getIntersectionInfo();
	
	for (unsigned int indexIntersection=0;indexIntersection<nbIntersections;indexIntersection++)
	{
		std::vector<int> connectedFromBegin;
		std::vector<int> connectedFromEnd;
		
		for (unsigned int j=0;j<listVessels.size();j++)
		{
			Vessel<VEC3> ve=listVessels[j];
			
			if (ve.idStart==indexIntersection)
				connectedFromBegin.push_back(j);
			if (ve.idStop==indexIntersection)
				connectedFromEnd.push_back(j);
		}
		
		if (connectedFromBegin.size()+connectedFromEnd.size()>1) // test if it is an intersection and not an end of vessel
		{
			std::vector<VEC3> lstPointsOfHull;
			std::vector<VEC3> lstPointsOfHullSave;
			VEC3 positionIntersection(0,0,0);
			bool init=true;
			
			std::vector<int> lstNumSides;
			std::vector<VEC3> initCoords;
			
			// get the coordinates of the points of the vessel that will compose the intersection
			for (unsigned int j=0;j<connectedFromBegin.size();j++)
			{
				Vessel<VEC3> ve=listVessels[connectedFromBegin[j]];
				nbFaces=ve.nbFaces;
				
				if (init) // if it is the first vessel we test, we get the coordinates of the central point
				{
					VesselRadius vr=ve.lstVesselRadius[0];
					positionIntersection=VEC3(vr.x,vr.y,vr.z);
					init=false;
				}
				
				VEC3 trans(0,0,0);
				
				if (ve.reduced)
				{
					VesselRadius vr=ve.lstVesselRadius[ve.decalFromStart];
					trans=VEC3(vr.x-positionIntersection[0],vr.y-positionIntersection[1],
									  vr.z-positionIntersection[2]);
					trans.normalize();
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
				Vessel<VEC3> ve=listVessels[connectedFromEnd[j]];
				nbFaces=ve.nbFaces;
				
				if (init) // if it is the last vessel we test, we get the coordinates of the central point
				{
					VesselRadius vr=ve.lstVesselRadius[ve.lstVesselRadius.size()-1];
					positionIntersection=VEC3(vr.x,vr.y,vr.z);
					init=false;
				}
				
				VEC3 trans(0,0,0);
				
				if (ve.reduced)
				{
					VesselRadius vr=ve.lstVesselRadius[ve.lstVesselRadius.size()-1-ve.decalFromStop];
					trans=VEC3(vr.x-positionIntersection[0],vr.y-positionIntersection[1]
									  ,vr.z-positionIntersection[2]);
					trans.normalize();
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
				VEC3 pointsCurrent=lstPointsOfHull[j];
				VEC3 dis=positionIntersection-pointsCurrent;
				float distance=dis.norm();
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

				VEC3 bary(0,0,0);
				for (unsigned int k=0;k<nbFaces;k++)
					bary+=lstPointsOfHull[j+k];
				bary=bary/(float) nbFaces;
				
				VEC3 decalage=bary-positionIntersection;
				decalage.normalize();
				
				for (unsigned int k=0;k<nbFaces;k++)
				{
					VEC3 direction=lstPointsOfHull[j+k]-positionIntersection;
					direction.normalize();
					VEC3 newPos = MathTools::projectionOnSphere(lstPointsOfHull[j+k],decalage,positionIntersection,distanceMax*3.0);
					lstPointsOfHull[j+k]=newPos;
				}
			}
			// ********************* end of the sphere intersection scheme *********************
			

			std::vector<unsigned int> lstEmbOfHull;
			convexHull(lstPointsOfHull, lstEmbOfHull);

			assert(lstEmbOfHull.size() == lstPointsOfHull.size());


			for (unsigned int j=0;j<lstEmbOfHull.size();j++)
				m_positions[lstEmbOfHull[j]] = initCoords[j];

		}
	}
}


template <typename PFP>
void Loader<PFP>::convexHull(std::vector<VEC3>& pts_of_hull, std::vector<unsigned int>& emb_of_hull)
{
	assert(pts_of_hull.size() > 4);

	DartMarker<MAP> md_to_supress(m_map);

	//create a first tetrahedron
	Dart d = Algo::Surface::Modelisation::createTetrahedron<PFP>(m_map,true);

	//embed it with the 4 firt points
	m_positions[d] = pts_of_hull[0];
	emb_of_hull.push_back(m_map.template getEmbedding<VERTEX>(d));
	m_positions[m_map.phi1(d)] = pts_of_hull[1];
	emb_of_hull.push_back(m_map.template getEmbedding<VERTEX>(m_map.phi1(d)));
	m_positions[m_map.phi_1(d)] = pts_of_hull[2];
	emb_of_hull.push_back(m_map.template getEmbedding<VERTEX>(m_map.phi_1(d)));
	m_positions[m_map.phi_1(m_map.phi2(d))] = pts_of_hull[3];
	emb_of_hull.push_back(m_map.template getEmbedding<VERTEX>(m_map.phi_1(m_map.phi2(d))));

	//for the remaining points
	for(unsigned int i = 4; i < pts_of_hull.size() ; i++)
	{
		VEC3 cur = pts_of_hull[i];
		bool outside = false;

		// foreach face of the convex hull
		Traversor3WF<MAP> tf(m_map, d);
		for(Dart f = tf.begin() ; f != tf.end() ; f = tf.next())
		{
			//test if the point is outside the convex hull
			double tetv = Geom::tetraSignedVolume<VEC3>(cur, m_positions[f], m_positions[m_map.phi1(f)], m_positions[m_map.phi_1(f)]);

			if(tetv < 0.0) {
				outside = true;
				md_to_supress.markOrbit(Face(f));
			}

//			VEC3 norm = Algo::Surface::Geometry::triangleNormal<PFP>(m_map, f, m_positions);
//			VEC3 bary = Algo::Surface::Geometry::faceCentroid<PFP>(m_map, f, m_positions);
//			VEC3 norm2 = m_positions[f] - bary;
//			double dot = norm * norm2;
//			if (dot>0.0) // test whether the point is outside of the convex hull
//			{
//				outside=true;
//				md_to_supress.markOrbit(Face(f));
//				std::cout << "plop" << std::endl;
//			}
		}

		//the point is outside the convex hull
		if(outside)
		{
//			std::vector<Dart> toDel;

//			DartMarker<MAP> me(m_map);
			//merge marked faces
			// foreach edge of the convex hull
//			Traversor3WE<MAP> te(m_map, d);
//			for(Dart e = te.begin() ; e != te.end() ; e = te.next())
//			{
//				if(!me.isMarked(e))
//				{
//					if(md_to_supress.isMarked(e))
//					{
//						if(md_to_supress.isMarked(m_map.phi2(e)))
//						{
//							toDel.push_back(e);
//							me.markOrbit(Edge(e));
//						}
//					}
//				}
//			}

			Dart dboundary = NIL;
			for (Dart d = m_map.begin() ; d != m_map.end() ; m_map.next(d))
			{
				if(md_to_supress.isMarked(d))
				{
					if(md_to_supress.isMarked(m_map.phi2(d)))
					{
						m_map.mergeFaces(d);
					}
					else
						dboundary = d;
				}
			}

			md_to_supress.unmarkAll();

//			if(toDel.size() > 0)
//			{
//				Dart dboundary;
//				for(unsigned int i = 0 ; i < toDel.size() ; i++)
//				{
//					dboundary = m_map.mergeFaces(toDel[i]);
//				}

//				if(i == 8)
//					break;

				// triangule the face (are the marked faces always contiguous ? )
				Dart dcenter = Algo::Surface::Modelisation::trianguleFace<PFP>(m_map, dboundary);

				//embed the central point to the cur point
				m_positions[dcenter] = cur;
				emb_of_hull.push_back(m_map.template getEmbedding<VERTEX>(dcenter));
//			}
//			else {
//				std::cout << "nothing to delete ? " << std::endl;
//				std::cout << i << std::endl << std::endl;
//			}
		}
		else
		{
			std::cout << "inside the hull ?" << std::endl;
			std::cout << cur << std::endl << std::endl;
		}


	}
}


// merge the vessels into one single object
template <typename PFP>
void Loader<PFP>::mergeVessels()
{
	std::list<Dart> listDarts;

	DartMarker<MAP> mtreated(m_map);
	
	// get the list of triangle darts that have the same embedding
	for (Dart d = m_map.begin(); d != m_map.end(); m_map.next(d))
	{
		if (!m_map.isBoundaryMarked(3,d))
		{
			if(!mtreated.isMarked(d))
			{
				mtreated.markOrbit(Face(d));

				for (Dart e = m_map.begin(); e != m_map.end();  m_map.next(e))
				{
					if (!m_map.isBoundaryMarked(3,e))
					{
						if (!mtreated.isMarked(e) && e!=d && sameTriangleEmbedding(d,e))
						{
							mtreated.markOrbit(Face(e));

							listDarts.push_back(d);
							listDarts.push_back(e);
						}
					}
				}
			}
		}
	}

	std::cout << "list of volumes to sew constructed : " << listDarts.size() << std::endl;
	
	// sew the volumes together
	for(std::list<Dart>::iterator it=listDarts.begin();it!=listDarts.end();++it)
	{
		Dart d0=*it;
		++it;
		Dart d1=*it;
		m_map.sewVolumes(d0,m_map.phi_1(d1));
	}
}

}

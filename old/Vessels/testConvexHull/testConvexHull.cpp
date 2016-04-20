/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* version 0.1                                                                  *
* Copyright (C) 2009, IGG Team, LSIIT, University of Strasbourg                *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Web site: https://iggservis.u-strasbg.fr/CGoGN/                              *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/

#include <iostream>
#include "os_spec.h"

#include "GLSLShader.h"
#include "glutwin.h"

#include "map_dual/emapd2.h"
#include "import/export.h"

#include "Render/map_glRender.h"
#include "point3d.h"

#include "Modelisation/primitives.h"
#include "Modelisation/extrusion.h"
#include "Modelisation/subdivision.h"
#include <gmtl/Output.h>

#include <gmtl/Matrix.h>
#include <gmtl/Quat.h>
#include <gmtl/Vec.h>
#include <gmtl/Generate.h>
#include <gmtl/Math.h>
#include <gmtl/Intersection.h>

#include <cmath>
#include <vector>
#include <tools/StringTokenizer.h>
#include <fstream>
#include <stdlib.h>

using namespace CGoGN;

struct PFP {
	typedef DartObj<DP::MAPD2_V0_mem> DART;
	typedef e0mapd2<DART> MAP;
	typedef Emb::Point3D EMB;
	static const int id0=0;
};

typedef PFP::MAP::Dart Dart;
typedef PFP::EMB Point3D;
PFP::MAP myMap;

bool adaptiveExtrusion;

class myGlutWin: public Utils::SimpleGlutWin
{
public:
	gmtl::Vec4f colDif;
	gmtl::Vec4f colSpec;
	gmtl::Vec4f colClear;
	gmtl::Vec4f colNormal;
	float shininess;
	
	/**
	 * position of object
	 */
	gmtl::Vec3f gPosObj;
	
	/**
	 * width of object
	 */
	float gWidthObj;
	
	/**
	 * factor to apply to normal drawing
	 */
	float normalScaleFactor;
	
	/**
	 * mesh normals
	 */
	std::vector<gmtl::Vec3f> vnormals;
	
	/**
	 * redraw CB
	 */
	void myRedraw();
	
	/**
	 * keyboard CB
	 */
	void myKeyboard(unsigned char keycode, int x, int y);
	
	/**
	 * Display list init
	 */
	void initDL();
	
	/**
	 * Display list init only for normal drawing
	 */
	void initDLNormals(void);
	
	/**
	 * Display list init only for lines drawing
	 */
	void initDLLines(void);
	
	/**
	 * GL initialization
	 */
	void myInitGL();
	
	/**
	 * GL (old school) rendering function (called by init DL)
	 */
	void render(int renderMode);
	
	/**
	 * table of shaders
	 */
	Utils::GLSLShader shaders[8];
	
	/**
	 * inverse the normal when computing normal
	 */
	bool invertedNormals;
	
	/**
	 * inverse object for culling
	 */
	bool invertedObject;
	
	/**
	 * rendering normals ?
	 */
	bool renderNormal;
	
	/**
	 * rendering lines ?
	 */
	bool renderLines;
	
	/**
	 * style of rendering
	 */
	int renderStyle;
	
	/**
	 * Display List for object
	 */
	GLuint dl_obj;
	
	/**
	 * Display List for object (second)
	 */
	GLuint dl_obj2;
	
	/**
	 * Display List for normal
	 */
	GLuint dl_norm;
	
	/**
	 * render mode enum
	 */
	enum { CLEAR=1, LINE, FLAT, GOURAUD, PHONG, NORMAL };
	
	myGlutWin(): 	invertedNormals(true), invertedObject(false),
	renderNormal(false), renderLines(true), renderStyle(FLAT),
	dl_obj(-1), dl_obj2(-1), dl_norm(-1) {}
};

/**
 * The MathTools class provides some usefull mathematical functions for generating a network of blood vessel
 * @author Cyril Kern
 */
class MathTools
{
	public:
		/**
		 * compute the barycentre of the 3 points
		 * @param p0 first point
		 * @param p1 second point
		 * @param p2 third point
		 * @return the barycentre
		 */
		static gmtl::Vec3f computeBary(Point3D* p0, Point3D* p1, Point3D* p2)
		{
				return computeBary(p0->getPosition(), p1->getPosition(), p2->getPosition());
		}
	
		static gmtl::Vec3f computeBary(vector<Point3D*> p)
		{
			vector<gmtl::Vec3f> pos;
			for (unsigned int i=0;i<p.size();i++)
				pos[i] = p[i]->getPosition();
			return computeBary(pos);
		}
	
		/**
		 * compute the barycentre of the 3 points
		 * @param pa first point
		 * @param pb second point
		 * @param pc third point
		 * @return the barycentre
		 */
		static gmtl::Vec3f computeBary(gmtl::Vec3f Pa, gmtl::Vec3f Pb, gmtl::Vec3f Pc)
		{
			gmtl::Vec3f bary = Pa+Pb+Pc;
			bary=bary/3.0f;
			return bary;
		}
	
		static gmtl::Vec3f computeBary(vector<gmtl::Vec3f> P)
		{
			gmtl::Vec3f bary = gmtl::Vec3f(0.0,0.0,0.0);
			for (unsigned int i=0;i<P.size();i++)
				bary = bary + P[i];
			bary=bary/(float)(P.size());
			return bary;
		}
		
		/**
		 * compute the normal of the faces composed by the 3 points
		 * @param p0 first point
		 * @param p1 second point
		 * @param p2 third point
		 * @return the direct normal
		 */
		static gmtl::Vec3f computeNormal(Point3D* p0, Point3D* p1, Point3D* p2)
		{
				return computeNormal(p0->getPosition(), p1->getPosition(), p2->getPosition());
		}
	
		static gmtl::Vec3f computeNormal(vector<Point3D*> p)
		{
			vector<gmtl::Vec3f> pos;
			for (unsigned int i=0;i<p.size();i++)
				pos[i] = p[i]->getPosition();
			return computeNormal(pos);
		}
		
		/**
		 * compute the normal of the faces composed by the 3 points
		 * @param pa first point
		 * @param pb second point
		 * @param pc third point
		 * @return the direct normal
		 */
		static gmtl::Vec3f computeNormal(gmtl::Vec3f Pa, gmtl::Vec3f Pb, gmtl::Vec3f Pc)
		{
			gmtl::Vec3f Vab = Pb - Pa;
			gmtl::Vec3f Vac = Pc - Pa;
			
			gmtl::Vec3f N;
			cross(N,Vab,Vac);
			
			//gmtl::normalize(N);
			return N;
		}
	
		static gmtl::Vec3f computeNormal(vector<gmtl::Vec3f> P)
		{
			gmtl::Vec3f Vab = P[1] - P[0];
			gmtl::Vec3f Vac = P[2] - P[0];
			
			gmtl::Vec3f N;
			cross(N,Vab,Vac);
			
			gmtl::normalize(N);
			return N;
		}
		
		/**
		 * compute the orthogonal projection of the point pt on plane based on (p0, p1, p2) relatively to an origin
		 * @param p0 point of the plane
		 * @param p1 point of the plane
		 * @param p2 point of the plane
		 * @param pt the point to project
		 * @param ptOrigine the reference point
		 * @return the projection of the point
		 */
		static gmtl::Vec2f projectOrthogonal(Point3D * p0, Point3D * p1, Point3D* p2, Point3D* pt, Point3D* ptOrigine)
		{
			return projectOrthogonal(p0->getPosition(),p1->getPosition(),p2->getPosition(),pt->getPosition(), ptOrigine->getPosition());
		}
		
		
		/**
		 * compute the orthogonal projection of the point pt on plane based on (p0, p1, p2) relatively to an origin
		 * @param p0 point of the plane
		 * @param p1 point of the plane
		 * @param p2 point of the plane
		 * @param pt the point to project
		 * @param ptOrigine the reference point
		 * @return the projection of the point
		 */
		static gmtl::Vec2f projectOrthogonal(gmtl::Vec3f p0, gmtl::Vec3f p1, gmtl::Vec3f p2, gmtl::Vec3f pt, gmtl::Vec3f ptOrigine)
		{
			//compute the plane normal
			gmtl::Vec3f planeNormal=computeNormal(p0,p1,p2);
			
			return projectOrthogonal(planeNormal[0], planeNormal[1], planeNormal[2], pt, ptOrigine);
		}
		
		
		/**
		 * compute the orthogonal projection of the point pt on plane made by (a b c d) relatively to an origin
		 * @param a coordinates of the plane (ax + by + cz + d = 0)
		 * @param b coordinates of the plane (ax + by + cz + d = 0)
		 * @param c coordinates of the plane (ax + by + cz + d = 0)
		 * @param pt the point to project
		 * @param ptOrigine the reference point
		 * @return the projection of the point
		 */
		static gmtl::Vec2f projectOrthogonal(float a, float b, float c, gmtl::Vec3f pt, gmtl::Vec3f ptOrigine)
		{
			//compute the plane normal
			gmtl::Vec3f planeNormal(a,b,c);
			
			//if the plane exist
			if (gmtl::length(planeNormal)!=0)
			{
				//create a signle up vector
				gmtl::Vec3f upVector(0.0f,0.0f,1.0f);
				
				//get the vector to turn around
				gmtl::Vec3f rotationVector;
				gmtl::cross(rotationVector,planeNormal,upVector);
				gmtl::normalize(rotationVector);
				//get the angle of rotation
				float angRot=angle(planeNormal, upVector);
				
				//create the rotation
				gmtl::AxisAnglef ang(angRot, rotationVector[0], rotationVector[1], rotationVector[2]);
				gmtl::Quatf myQuat=gmtl::make<gmtl::Quatf>(ang);
				
				//rotate the point
				gmtl::Vec3f ptToRote=pt-ptOrigine;
				gmtl::Vec3f ptRote=myQuat*ptToRote;
				//point put it in the plane
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
		
		/**
		 * compute the angle between two vectors
		 * @param v0 first vector
		 * @param v1 second vector
		 * @return the angle in radians
		 */
		static float angle(gmtl::Vec3f v0, gmtl::Vec3f v1)
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
		
		/**
		 * compute the smallest angle in a triangle
		 * @param points the list of the points of the triangle
		 * @return the minimum angle
		 */
		static float angleMinInTri(gmtl::Vec3f * points)
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
		
		
		/**
		 * compute the smallest distance from a point to a plane
		 * @param a coordinates of the plane (ax + by + cz + d = 0)
		 * @param b coordinates of the plane (ax + by + cz + d = 0)
		 * @param c coordinates of the plane (ax + by + cz + d = 0)
		 * @param d coordinates of the plane (ax + by + cz + d = 0)
		 * @param pt the point
		 * @return distance
		 */
		static float distancePlanPoint(float a, float b, float c, float d, gmtl::Vec3f pt)
		{
			return (abs(a*pt[0]+b*pt[1]+c*pt[2]+d)/sqrt(a*a+b*b+c*c));
		}
		
		/**
		 * return the d coordinates of a plane (ax + by + cz + d = 0)
		 * @param planNormal the normal of the plane
		 * @param pt a point of the plane
		 * @return the distance
		 */
		static float dCoordsOfThePlan(gmtl::Vec3f planNormal, gmtl::Vec3f pt)
		{
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
		static float dCoordsOfThePlan(float a, float b, float c, gmtl::Vec3f pt)
		{
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
		static bool intersectsBowlDisk(gmtl::Vec3f bowlCenter, float bowlRadius,
									   gmtl::Vec3f diskCenter, float diskRadius, float a, float b, float c, float d)
		{
			
			//compute the distance from the bowlcenter to the plane
			float disPlPt=distancePlanPoint(a,b,c,d,bowlCenter);
			
			//if an intersection can exist
			if (disPlPt<=bowlRadius)
			{
				//compute the distance between the center of the projection of the central point of the bowl
				//and the intersection with the plane
				disPlPt=disPlPt/bowlRadius;
				float angle=acos(disPlPt);
				float rad=sin(angle);
				rad=rad*bowlRadius;
				
				//compute the position of the orthogonal projection of bowlCenter on the plane
				//diskCenter is the origin
				gmtl::Vec2f positionProjete=projectOrthogonal(a,b,c,bowlCenter,diskCenter);
				
				//test the distances
				if (gmtl::length(positionProjete) <= (rad+diskRadius))
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
		static gmtl::Vec3f projectionOnSphere(gmtl::Vec3f origine, gmtl::Vec3f direction, gmtl::Vec3f centerSphere, float radiusSphere)
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
		
		// TODO : ajouter fonction de calcul de l'enveloppe convexe de points 3D
		
		/**
		 * print the coordinates of the point on the screen
		 * @param pt the point
		 */
		static void debug(Point3D * pt)
		{
			debug(pt->getPosition());
		}
		/**
		 * print the coordinates of the vertex on the screen
		 * @param v the vertex
		 */
		static void debug(gmtl::Vec3f v)
		{
			std::cout << v[0] << "," << v[1] << "," << v[2] ;
		}
		
};

/**
 * position et rayon d'un point du vaisseau sanguin
 */
class VesselRadius
{
	public:
		/**
		 * position du point
		 */
		float x;
		float y;
		float z;
		
		/**
		 * rayon du vaisseau en ce point
		 */
		float radius;
		
		VesselRadius(float px, float py, float pz, float rad)
		{
			x=px;
			y=py;
			z=pz;
			radius=rad;
		}
		
		VesselRadius()
		{
			x=0;
			y=0;
			z=0;
			radius=0;
		}
		
		VesselRadius(const VesselRadius& v)
		{
			x=v.x;
			y=v.y;
			z=v.z;
			radius=v.radius;
		}
		
		float distance(VesselRadius vr)
		{
			float dis=(x-vr.x)*(x-vr.x)+(y-vr.y)*(y-vr.y)+(z-vr.z)*(z-vr.z);
			dis=sqrt(dis);
			return dis;
		}
};

/**
 * définit un vaisseau sanguin
 */
class Vessel
{
	public:
		/**
		 * indice de départ et d'arrivée
		 */
		unsigned int idStart;
		unsigned int idStop;
		/**
		 * nombre de points qui seront sautés après découpe au niveau des intersections à partir du départ
		 */
		unsigned int decalFromStart;
		/**
		 * nombre de points qui seront sautés après découpe au niveau des intersections à partir de la fin
		 */
		unsigned int decalFromStop;
		/**
		 * la liste des points composants le vaisseau
		 */
		std::vector<VesselRadius> lstVesselRadius;
		/**
		 * les coordonnées des éléments du vaisseau
		 */
		std::vector<gmtl::Vec3f> lstCoordinates;
		
		/**
		 * vrai si le vaisseau a été réduit/découpé
		 */
		bool reduced;
		
		/**
		 * nombre de segments formant la base du vaisseau (triangle, carré, pentagone, ...)
		 */
		unsigned int nbFaces;
		
		/**
		 * rayon moyen du vaisseau
		 */
		float averageRadius;
		
		/**
		 * constructeur par défaut
		 */
		Vessel()
		{
			idStart=-1;
			idStop=-1;
			decalFromStart=0;
			decalFromStop=0;
			reduced=false;
			
			nbFaces=3;
			averageRadius=0;
		}
		
		/**
		 * constructeur par recopie
		 */
		Vessel(const Vessel& v)
		{
			idStart=v.idStart;
			idStop=v.idStop;
			for (unsigned int i=0;i<v.lstVesselRadius.size();i++)
			{
				lstVesselRadius.push_back(VesselRadius(v.lstVesselRadius[i]));
			}
			for (unsigned int i=0;i<v.lstCoordinates.size();i++)
			{
				lstCoordinates.push_back(gmtl::Vec3f(v.lstCoordinates[i]));
			}
			decalFromStart=v.decalFromStart;
			decalFromStop=v.decalFromStop;
			reduced=v.reduced;
			nbFaces=v.nbFaces;
		}
		
		/**
		 * ajouter un point au vaisseau
		 */
		void addVesselRadius(VesselRadius ves)
		{
			lstVesselRadius.push_back(ves);
		}
		
		/**
		 * longueur du vaisseau sanguin entre deux points
		 */
		float distanceFromPointToPoint(int id0, int id1)
		{
			if (id0>id1)
			{
				int tmp=id1;
				id1=id0;
				id0=tmp;
			}
			float res=0;
			for (int i=id0;i<id1;i++)
			{
				VesselRadius v0=lstVesselRadius[i];
				VesselRadius v1=lstVesselRadius[i+1];
				
				res=res+v0.distance(v1);
			}
			return res;
		}
		
		/**
		 * calcule le rayon moyen du vaisseau
		 */
		void computeAverageRadius()
		{
			float res=0;
			for (unsigned int i=0;i<lstVesselRadius.size();i++)
			{
				res+=lstVesselRadius[i].radius;
			}
			res=res/lstVesselRadius.size();
			averageRadius=res;
		}
		
		/**
		 * simplifie le vaisseau sanguin
		 */
		void simplify()
		{
			float rad=0;
			int taille=lstVesselRadius.size();
			
			//recherche du rayon moyen
			for (int i=0;i<taille;i++)
			{
				rad+=lstVesselRadius[i].radius;
			}
			rad/=(float) taille;
			rad*=sqrt(3);
			
			//on va supprimer les points intermédiaires tant qu'il y a au moins (racine de 3) * (rayon moyen) de distance entre deux points
			
			//on se place sur le point de départ
			unsigned int posPre=decalFromStart;
			//tant qu'on le raccourcit
			bool raccourci=true;
			
			while (raccourci)
			{
				raccourci=false;
				bool fait=false;
				//sur toute la longueur du vaisseau
				while (posPre+1<taille-1-decalFromStop && !fait)
				{
					//si la distance est inférieure à la distance minimale à avoir
					if (distanceFromPointToPoint(posPre,posPre+1)<rad)
					{
						//on supprime le point
						lstVesselRadius.erase(lstVesselRadius.begin()+(posPre+1));
						taille=lstVesselRadius.size();
						raccourci=true;
					}
					else
					{
						fait=true;
					}
				}
				posPre++;
			}
		}
		
		void verify()
		{
			std::cout << "verif" << std::endl;
			for (unsigned int i=0;i<lstVesselRadius.size()-1;i++)
			{
				if (distanceFromPointToPoint(i,i+1)<0.01)
				{
					std::cout << "vla un pb " << i << " " << distanceFromPointToPoint(i,i+1) << std::endl;
					std::cout << distanceFromPointToPoint(i,i+1) << std::endl;
				}
			}
		}
		
		//dilate le rayon du vaisseau par 2
		void dilate()
		{
			for (unsigned int i=0;i<lstVesselRadius.size();i++)
			{
				lstVesselRadius[i].radius*=2;
			}
		}
};

class Loader
{
	private:
		unsigned int nbIntersections;
		int *nbConnected;
	
		float decX;
		float decY;
		float decZ;
		
	public:
		unsigned int nbFaces;
	
		/**
		 * return the size of a face
		 * @param d first dart of the face
		 * @return the size of the face
		 */
		int sizeFace(Dart d)
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
	
		vector<Vessel> listVessels;
		void getIntersectionInfo();
		void computeDecalOfVessels();
		void constructIntersection();
	
		Loader(string filename)
		{
			ifstream file(filename.c_str());
			
			bool init=true;
			//nbFaces = 3;
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
};

void myGlutWin::myInitGL()
{
	glClearColor(0.2,0.2,0.2,0.0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);

	// shader init
	if (! shaders[0].simpleInit("phong_vs.txt","phong_ps.txt")) {
		std::cerr<< "No shader !!"<< std::endl;
	}
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
	glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition);
	glEnable(GL_LIGHT0);

	SelectorTrue<PFP::MAP::Dart> allDarts;
	Algo::Render::computeNormalVertices<PFP>(myMap, vnormals, allDarts, invertedNormals);
}

void myGlutWin::render(int renderMode)
{
	SelectorTrue<PFP::MAP::Dart> allDartsOri;
	SelectorTrue<PFP::MAP::Dart> allDarts;

	switch (renderMode) {
	case PHONG:
		Algo::Render::computeNormalVertices<PFP>(myMap, vnormals, allDarts, invertedNormals);
		glEnable(GL_LIGHTING);
		glEnable(GL_CULL_FACE);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		glShadeModel(GL_SMOOTH);

		if (invertedObject)
			glFrontFace(GL_CW);
		else
			glFrontFace(GL_CCW);

		shaders[0].bind();
		Algo::Render::Direct::renderTriQuadPoly<PFP>(myMap, Algo::Render::Direct::SMOOTH, 1.0f, allDartsOri, vnormals, invertedNormals);
//		Algo::Render::Direct::renderTriStrips<PFP>(myMap, Algo::Render::Direct::SMOOTH, 1.0f, allDarts, vnormals, invertedNormals);
		shaders[0].unbind();
		break;

	case GOURAUD:
		Algo::Render::computeNormalVertices<PFP>(myMap, vnormals, allDarts, invertedNormals);
		glEnable(GL_LIGHTING);
		glEnable(GL_CULL_FACE);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		glShadeModel(GL_SMOOTH);

		if (invertedObject)
			glFrontFace(GL_CW);
		else
			glFrontFace(GL_CCW);

		Algo::Render::Direct::renderTriQuadPoly<PFP>(myMap, Algo::Render::Direct::SMOOTH, 1.0f, allDarts, vnormals, invertedNormals);
//		Algo::Render::Direct::renderTriFans<PFP>(myMap, Algo::Render::Direct::SMOOTH, 1.0f, allDartsOri, vnormals, invertedNormals);
		break;

	case FLAT:
		glEnable(GL_LIGHTING);
		glEnable(GL_CULL_FACE);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		glShadeModel(GL_FLAT);
		Algo::Render::computeNormalVertices<PFP>(myMap, vnormals, allDarts, invertedNormals);

		if (invertedObject)
			glFrontFace(GL_CW);
		else
			glFrontFace(GL_CCW);

		Algo::Render::Direct::renderTriQuadPoly<PFP>(myMap, Algo::Render::Direct::FLAT, 1.0f, allDartsOri, vnormals, invertedNormals);
		break;

	case CLEAR:
		glDisable(GL_LIGHTING);
		glEnable(GL_CULL_FACE);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		glShadeModel(GL_SMOOTH);

		if (invertedObject)
			glFrontFace(GL_CW);
		else
			glFrontFace(GL_CCW);

		glColor4fv(colClear.getData());

		Algo::Render::Direct::renderTriQuadPoly<PFP>(myMap, Algo::Render::Direct::FLAT, 1.0f, allDartsOri, vnormals, invertedNormals);
		break;


	case LINE:
		glDisable(GL_LIGHTING);
		glEnable(GL_CULL_FACE);
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);

		if (invertedObject)
			glFrontFace(GL_CW);
		else
			glFrontFace(GL_CCW);

		if (renderStyle!=CLEAR)
			glColor3f(0.0f,0.0f,0.0f);
		else
			glColor4fv(colDif.getData());

		Algo::Render::Direct::renderTriQuadPoly<PFP>(myMap, Algo::Render::Direct::LINE, 1.0f, allDartsOri, vnormals, invertedNormals);
		break;

	case NORMAL:
		glDisable(GL_LIGHTING);
		glColor3fv(colNormal.getData());

		Algo::Render::Direct::renderNormalVertices<PFP>(myMap, allDarts, vnormals, normalScaleFactor);
		break;

	default:
		break;
	}
}

void myGlutWin::initDL(void)
{
	GLint t1 = glutGet(GLUT_ELAPSED_TIME);

	if (glIsList(dl_obj)) glDeleteLists(dl_obj,1);
	dl_obj = glGenLists(1);
	glNewList(dl_obj,GL_COMPILE);
	render(renderStyle);
	glEndList();

	GLint t2 = glutGet(GLUT_ELAPSED_TIME);
	GLfloat seconds = (t2 - t1) / 1000.0f;
	std::cout << "GListisation: "<< seconds << "sec" << std::endl;
}

void myGlutWin::initDLNormals(void)
{
	std::vector<gmtl::Vec3f> normals;
	SelectorTrue<PFP::MAP::Dart> allDarts;

	if (glIsList(dl_norm)) glDeleteLists(dl_norm,1);

	Algo::Render::computeNormalVertices<PFP>(myMap, normals, allDarts, invertedNormals);

	if (renderNormal) {
		dl_norm = glGenLists(1);
		glNewList(dl_norm,GL_COMPILE);
		render(NORMAL);
		glEndList();
	}
}

void myGlutWin::initDLLines(void)
{
	SelectorTrue<PFP::MAP> allDarts;
	std::vector<gmtl::Vec3f> normals;

	if (glIsList(dl_obj2)) glDeleteLists(dl_obj2,1);

	dl_obj2 = glGenLists(1);
	glNewList(dl_obj2,GL_COMPILE);
	render(LINE);
	glEndList();
}

void myGlutWin::myRedraw(void)
{
	GLfloat black[4]= {0.0f,0.0f,0.0f,1.0f};
	GLfloat amb[4]= {0.2f,0.1f,0.1f,1.0f};

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();

	float sc = 50./gWidthObj;
	glScalef(sc,sc,sc);
	glTranslatef(-gPosObj[0],-gPosObj[1],-gPosObj[2]);

	if (glIsList(dl_obj2)) glCallList(dl_obj2);
	if (glIsList(dl_norm)) glCallList(dl_norm);

	glEnable( GL_POLYGON_OFFSET_FILL );
	glPolygonOffset( 1.0f, 1.0f );

	if (glIsList(dl_obj)) {
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, colDif.getData());
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);

		if (renderStyle!=PHONG) {
			glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,black);
			glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, 1.0f );
		}
		else {
			glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,colSpec.getData());
			glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, shininess );
		}
		glCallList(dl_obj);
	}
	glDisable( GL_POLYGON_OFFSET_FILL );

	glPopMatrix();
}

void myGlutWin::myKeyboard(unsigned char keycode, int x, int y)
{
	SelectorTrue<PFP::MAP::Dart> allDarts;
	GLint t1 = glutGet(GLUT_ELAPSED_TIME);
	GLint t2;
	GLfloat seconds;

	switch(keycode) {
	case 'c':
		Algo::Modelisation::CatmullClark<PFP>(myMap,allDarts);
		initDL();
		if (renderLines) initDLLines();
		else {
			if (glIsList(dl_obj2))
				glDeleteLists(dl_obj2,1);
		}
		glutPostRedisplay();
		break;
	case 'L':
		Algo::Modelisation::LoopSubdivision<PFP>(myMap,allDarts);
		initDL();
		if (renderLines) initDLLines();
		else {
			if (glIsList(dl_obj2))
				glDeleteLists(dl_obj2,1);
		}
		glutPostRedisplay();
		break;
	case 'S':
		normalScaleFactor *= 1.1f;
		initDLNormals();
		glutPostRedisplay();
		break;
	case 's':
		normalScaleFactor /= 1.1f;
		initDLNormals();
		glutPostRedisplay();
		break;
	case 'v':
		for(int i=0; i<20; ++i) {
			myRedraw();
			glutSwapBuffers();
		}
		t2 = glutGet(GLUT_ELAPSED_TIME);
		seconds = (t2 - t1) / 1000.0f;
		std::cout << "fps: "<< 20.0/seconds << "sec" << std::endl;
		break;
	case 'u':
		myMap.reverseOrientation();
		std::cout << "reverseOrientation" << std::endl;		
		glutPostRedisplay();
		break;
	case 'd':
		setFoc(getFoc()/1.05f);
		reshape(-1,-1);
		glutPostRedisplay();
		break;
	case 'D':
		setFoc(getFoc()*1.05f);
		reshape(-1,-1);
		glutPostRedisplay();
		break;
	case 'i':
		invertedNormals = !invertedNormals;
		initDL();
		glutPostRedisplay();
		break;
	case 'o':
		invertedObject = !invertedObject;
		initDL();
		glutPostRedisplay();
		break;
	case 'O':
		invertedObject = !invertedObject;
		invertedNormals = !invertedNormals;
		initDL();
		glutPostRedisplay();
		break;
	case 'n':
		renderNormal = ! renderNormal;
		if (renderNormal) initDLNormals();
		else {
			if (glIsList(dl_norm)) glDeleteLists(dl_norm,1);
		}
		glutPostRedisplay();
		break;
	case 'f':
		renderStyle = FLAT;
		initDL();
		glutPostRedisplay();
		break;
	case 'g':
		renderStyle = GOURAUD;
		initDL();
		glutPostRedisplay();
		break;
	case 'p':
		renderStyle = PHONG;
		initDL();
		glutPostRedisplay();
		break;
	case 'b':
		renderStyle = CLEAR;
		initDL();
		glutPostRedisplay();
		break;
	case 'l':
		renderLines = !renderLines;
		if (renderLines) initDLLines();
		else {
			if (glIsList(dl_obj2)) glDeleteLists(dl_obj2,1);
		}
		glutPostRedisplay();
		break;
	case 'z':
		shininess /= 1.1f;
		glutPostRedisplay();
		break;
	case 'Z':
		shininess *= 1.1f;
		glutPostRedisplay();
		break;
	}
}

// compute the matrix rotation for an arbitrary axis and angle
gmtl::Matrix44f rotationMatrix(gmtl::Vec3f axis, const float angle)
{
	// make a rotation quaternion from an axis angle
	gmtl::Quatf q;
	gmtl::setRot(q, gmtl::AxisAnglef(angle, axis));
	
	// set the rotation part of the matrix to the rotation defined by the quaternion
	gmtl::Matrix44f mat;
	gmtl::setRot(mat, q);
	
	return mat;
}

// compute a vector orthogonal to a given vector
gmtl::Vec3f getFirstPolygonPoint(gmtl::Vec3f tangente)
{
	gmtl::Vec3f vec;
	vec[0] = 1.f;
	vec[1] = 1.f;
	vec[2] = (-tangente[0] - tangente[1])/tangente[2];
	gmtl::normalize(vec);
	return vec;
}

// get information on the intersections
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
	//get the number of intersections of the graph
	getIntersectionInfo();
	
	for (unsigned int indiceIntersection=0;indiceIntersection<nbIntersections;indiceIntersection++)
	{
		//find the vessel that are connected to this intersection
		std::vector<int> connected;
		for (unsigned int i=0;i<listVessels.size();i++)
		{
			Vessel ve=listVessels[i];
			if (ve.idStart==indiceIntersection || ve.idStop==indiceIntersection)
				connected.push_back(i);
		}
		
		//for each of them
		for (unsigned int i=0;i<connected.size();i++)
		{
			Vessel vesselCur=listVessels[connected[i]];
			unsigned int decalage;
			
			if (vesselCur.idStart==indiceIntersection)
				decalage=vesselCur.decalFromStart;
			else
				decalage=vesselCur.decalFromStop;
			
			//test with all the others
			for (unsigned int j=0;j<connected.size();j++)
			{
				if (i!=j)
				{
					//determine the extremum points for the iteration
					Vessel vesselTest=listVessels[connected[j]];
					bool intersect=true;
					int delta;
					int idCur;
					int idArr;
					if (vesselTest.idStart==indiceIntersection)
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
					
					//while there is no intersection and we haven't tested all the possible intersections
					while (intersect && decalage < vesselCur.lstVesselRadius.size())
					{
						gmtl::Vec3f planNormale;
						gmtl::Vec3f origineDisk;
						float radiusDisk;
						
						//get the equation of the disk that we will test for the intersection
						if (decalage==0)
						{
							VesselRadius v0;
							VesselRadius v1;
							if (vesselCur.idStart==indiceIntersection)
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
							if (vesselCur.idStart==indiceIntersection)
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
						
						//and verify the intersection with all the spheres of the possible cutting vessel
						for(int k=idCur;k!=idArr && !intersect;k+=delta)
						{
							VesselRadius vrTest=vesselTest.lstVesselRadius[k];
							gmtl::Vec3f centerBowl(vrTest.x,vrTest.y,vrTest.z);
							intersect=MathTools::intersectsBowlDisk(centerBowl,vrTest.radius,
																	origineDisk,radiusDisk,
																	planNormale[0],planNormale[1],planNormale[2],d);
						}
						//if there is an intersection, try the next point of the current vessel
						if (intersect)
						{
							decalage++;
						}
					}
				}
			}
			
			if (vesselCur.idStart==indiceIntersection)
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

// compute the barycenter
gmtl::Vec3f computeBary(Dart d)
{
	Point3D *pt0 = reinterpret_cast<PFP::EMB*>(myMap.getVertexEmb(d));
	Point3D *pt1 = reinterpret_cast<PFP::EMB*>(myMap.getVertexEmb(myMap.phi1(d)));
	Point3D *pt2 = reinterpret_cast<PFP::EMB*>(myMap.getVertexEmb(myMap.phi_1(d)));
	return MathTools::computeBary(pt0, pt1, pt2);
}

// compute the normal
gmtl::Vec3f computeNormal(Dart d)
{
	Point3D *pt0 = reinterpret_cast<PFP::EMB*>(myMap.getVertexEmb(d));
	Point3D *pt1 = reinterpret_cast<PFP::EMB*>(myMap.getVertexEmb(myMap.phi1(d)));
	Point3D *pt2 = reinterpret_cast<PFP::EMB*>(myMap.getVertexEmb(myMap.phi_1(d)));
	return MathTools::computeNormal(pt0, pt1, pt2);
}

// compute the convex hull given a list of 3D points
void convexHull(std::vector<PFP::EMB*> lstPoints, Marker newHull)
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
	myMap.embedCell(0,myMap.phi_1(d),0,lstPoints[1]);
	myMap.embedCell(0,myMap.phi1(d),0,lstPoints[2]);
	myMap.embedCell(0,myMap.phi<2,-1>(d),0,lstPoints[lstPoints.size()-1]);
	
	for (unsigned int i=3;i<lstPoints.size()-1;++i)
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
				if (dot>0) // test whether the point is outside of the convex hull
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
			myMap.embedCell(0,myMap.phi_1(dSav),0,ptCur); // c ici que ca MERDE
			
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

void Loader::constructIntersection()
{
	for (unsigned int indiceIntersection=0;indiceIntersection<nbIntersections;indiceIntersection++)
	{
		std::vector<int> connectedFromBegin;
		std::vector<int> connectedFromEnd;

		for (unsigned int j=0;j<listVessels.size();j++)
		{
			Vessel ve=listVessels[j];
			
			if (ve.idStart==indiceIntersection)
				connectedFromBegin.push_back(j);
			if (ve.idStop==indiceIntersection)
				connectedFromEnd.push_back(j);
		}
		
		if (connectedFromBegin.size()+connectedFromEnd.size()>1) //test if it is an intersection and not an end of vessel
		{
			std::vector<gmtl::Vec3f> lstPointsOfHull;
			std::vector<gmtl::Vec3f> lstPointsOfHullSave;
			gmtl::Vec3f positionIntersection(0,0,0);
			bool init=true;
			
			std::vector<int> lstNumSides;
			
			std::vector<gmtl::Vec3f> initCoords;
			
			//get the coordinates of the points of the vessel that will compose the intersection
			for (unsigned int j=0;j<connectedFromBegin.size();j++)
			{
				Vessel ve=listVessels[connectedFromBegin[j]];
				nbFaces=ve.nbFaces;
				
				//if it is the first vessel we test, we get the coordinates of the central point
				if (init)
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
			
			for (unsigned int j=0;j<connectedFromEnd.size();j++)
			{
				Vessel ve=listVessels[connectedFromEnd[j]];
				nbFaces=ve.nbFaces;

				if (init)
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
			

			// ********************* use the sphere intersection scheme *********************
			
			//get the furthest point of the center
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
			{
				lstPointsOfHullSave.push_back(lstPointsOfHull[j]);
			}
			
			//projection of the points on a sphere to be used for convex hull
			for (unsigned int j=0;j<lstPointsOfHull.size();j+=lstNumSides[idCurSide-1])
			{
				nbFaces=lstNumSides[idCurSide];
				idCurSide++;
				
				gmtl::Vec3f bary(0,0,0);
				for (unsigned int k=0;k<nbFaces;k++)
				{
					bary+=lstPointsOfHull[j+k];
				}
				bary=bary/(float) nbFaces;
				
				gmtl::Vec3f decalage=bary-positionIntersection;
				gmtl::normalize(decalage);
				
				for (unsigned int k=0;k<nbFaces;k++)
				{
					gmtl::Vec3f direction=lstPointsOfHull[j+k]-positionIntersection;
					gmtl::normalize(direction);
					gmtl::Vec3f newPos = MathTools::projectionOnSphere(lstPointsOfHull[j+k],decalage,positionIntersection, distanceMax*2.0f);
					lstPointsOfHull[j+k]=newPos;
				}
			}
			// ********************* end of sphere intersection scheme *********************
			

			//set the vec3f to point3f for embedding
			std::vector<Point3D *> lstPoints;
			for (unsigned int j=0;j<lstPointsOfHull.size();j++)
			{
				Point3D * tmp=Point3D::create(lstPointsOfHull[j]);
				CGoGN::Emb::RefCountObject::ref(tmp);
				lstPoints.push_back(tmp);
			}
			
			//idCurSide=0;
			Marker hullMark=myMap.getNewMarker();
			//add all the points to the hull
			//cout << lstPoints.size() << endl;
			convexHull(lstPoints,hullMark);

			for (unsigned int j=0;j<lstPointsOfHull.size();j++)
			{
				lstPoints[j]->setPosition(initCoords[j]);
			}
			
			FunctorUnmark<PFP::MAP> fHullUnMark(hullMark);
			myMap.foreach_dart(fHullUnMark);
			myMap.releaseMarker(hullMark);
		}
	}
}

// Fixed arc length for the construction of polygons in the extrusion
// Big vessels' circles are approximated by regular polygons (with many edges)
// whereas small vessels' circles are approximated by triangles or quads.
const float arcLength = 1.f;

int main(int argc, char **argv)
{
	adaptiveExtrusion = false;
	
	myGlutWin mgw;
	mgw.init(&argc,argv,800,800);

	gmtl::Vec3f gMax;
	gmtl::Vec3f gMin;

	// create a list of 3D points
	std::vector<gmtl::Vec3f> lstPointsOfHull;
	lstPointsOfHull.push_back(gmtl::Vec3f(0,0,0));
	lstPointsOfHull.push_back(gmtl::Vec3f(0,0,1));
	lstPointsOfHull.push_back(gmtl::Vec3f(0,1,0));
	lstPointsOfHull.push_back(gmtl::Vec3f(0,1,1));
	lstPointsOfHull.push_back(gmtl::Vec3f(1,0,0));
	lstPointsOfHull.push_back(gmtl::Vec3f(1,0,1));
	lstPointsOfHull.push_back(gmtl::Vec3f(1,1,0));
	lstPointsOfHull.push_back(gmtl::Vec3f(1,1,1));
	
	gmtl::Vec3f axis = gmtl::Vec3f(1, 0, 0);
	float angle = gmtl::Math::PI * .34;
	
	for (unsigned int i=0; i<lstPointsOfHull.size();i++)
	{
		lstPointsOfHull[i] = rotationMatrix(axis, angle) * lstPointsOfHull[i];
		cout << lstPointsOfHull[i] << endl;
	}
	
	lstPointsOfHull.push_back(MathTools::computeBary(lstPointsOfHull[0],lstPointsOfHull[2],lstPointsOfHull[4]));
	cout << lstPointsOfHull[lstPointsOfHull.size()-1] << endl;
	
	//set the vec3f to point3f for embedding
	std::vector<Point3D *> lstPoints;
	for (unsigned int j=0;j<lstPointsOfHull.size();j++)
	{
		Point3D * tmp=Point3D::create(lstPointsOfHull[j]);
		CGoGN::Emb::RefCountObject::ref(tmp);
		lstPoints.push_back(tmp);
	}
	
	Marker hullMark=myMap.getNewMarker();

	convexHull(lstPoints,hullMark);
	
	FunctorUnmark<PFP::MAP> fHullUnMark(hullMark);
	myMap.foreach_dart(fHullUnMark);
	myMap.releaseMarker(hullMark);
	
	// compute the bounding box
	Algo::Render::computeBoundingBox<PFP>(myMap,gMin,gMax);
	std::cout <<"BB:"<<gMin<<" / "<<gMax<<std::endl;

	mgw.gPosObj = (gMax+gMin)/2.0f;
	float tailleX = gMax[0] - gMin[0];
	float tailleY = gMax[1] - gMin[1];
	float tailleZ = gMax[2] - gMin[2];

	mgw.gWidthObj = std::max( std::max(tailleX,tailleY),tailleZ);
	mgw.normalScaleFactor = std::min( std::min(tailleX,tailleY),tailleZ) / 50.0f;
	mgw.colClear = gmtl::Vec4f(0.2f,0.2f,0.2f,0.1);
	mgw.colDif = gmtl::Vec4f(0.8f,0.9f,0.7f,1.0f);
	mgw.colSpec = gmtl::Vec4f(0.9f,0.9f,0.9f,1.0f);
	mgw.colNormal = gmtl::Vec4f(1.0f,0.0f,0.0f,1.0f);
	mgw.shininess=80.0f;

	std::cout << "Nb sommets = " << myMap.getNbVertices() << std::endl;
	std::cout << "Nb aretes  = " << myMap.getNbEdges() << std::endl;
	std::cout << "Nb faces   = " << myMap.getNbFaces() << std::endl;

	std::cout << "closing map ... ";
	Marker m = myMap.closeMap(true);
	myMap.setBoundaryMarkerValue(m);
	std::cout << "done" << std::endl;

	std::cout << "Nb marked darts = " << myMap.getNbMarked(m) << std::endl;
	std::cout << "Nb sommets = " << myMap.getNbVertices() << std::endl;
	std::cout << "Nb aretes  = " << myMap.getNbEdges() << std::endl;
	std::cout << "Nb faces   = " << myMap.getNbFaces() << std::endl;
	
	mgw.myInitGL();
	mgw.initDL();
	mgw.initDLLines();
	mgw.mainLoop();
	return 0;
}

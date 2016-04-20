#ifndef _LOADER_H
#define _LOADER_H

#include "mathTools.h"
#include <vector>
#include "stringTokenizer.hpp"
#include <fstream>
#include "vessel.h"
#include <stdlib.h>
#include "Algo/Modelisation/polyhedron.h"
#include "Algo/Modelisation/extrusion.h"

/******************************************************************************/
/*************************      LOADER CLASS		***************************/
/******************************************************************************/


namespace CGoGN
{


template <typename PFP>
class Loader
{
private:
	typedef typename PFP::MAP MAP;
	typedef typename PFP::VEC3 VEC3;
	typedef typename PFP::Quatf QUAT;
	typedef typename PFP::AxisAnglef AA;
	unsigned int nbIntersections;
	int *nbConnected;
	
	float decX;
	float decY;
	float decZ;
	typename PFP::MAP& m_map;
	VertexAttribute<VEC3, MAP>& m_positions;
	
public:
	/**
	 * number of faces
	 */
	unsigned int nbFaces;
	
	/**
	 * return the size of a face
	 * @param d first dart of the face
	 * @return the size of the face
	 */
	int sizeFace(Dart d);
	
	/**
	 * list of vessels
	 */
	std::vector<Vessel<VEC3> > listVessels;
	
	/**
	 * test a dart to determine whether it corresponds to a triangle
	 */
	bool isTriangle(Dart d);
	
	/**
	 * test two darts to determine whether they match the same triangle (embedding)
	 */
	bool sameTriangleEmbedding(Dart d, Dart e);
	
	/**
	 * get the number of intersections of the graph
	 */
	void getIntersectionInfo();
	
	/**
	 * branching area pre-processing step: cut the vessels in the interserction region 
	 * to avoid undesired (geometric) intersections
	 */
	void computeDecalOfVessels();
	
	/**
	 * construct the extruded vessels
	 */
	void extrudeVessels();
	
	/**
	 * construct the intersections
	 */
	void constructIntersection();
	
	/**
	 * merge the branches together with the intersections by sewing the volumes
	 */		
	void mergeVessels();
	
	/**
	 * constructor
	 */
	Loader(MAP& map, VertexAttribute<VEC3, MAP>& positions, std::string filename);

	void convexHull(std::vector<VEC3>& pts_of_hull, std::vector<unsigned int>& emb_of_hull);

};


}

#include "loader.hpp"

#endif

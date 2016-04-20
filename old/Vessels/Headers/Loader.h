#ifndef LOADER_H
#define LOADER_H

#include "Headers/MathTools.h"
#include <vector>
#include "Headers/StringTokenizer.hpp"
#include <fstream>
#include "Headers/Vessel.h"
#include <stdlib.h>
#include "Modelisation/primitives.h"
#include "Modelisation/extrusion.h"
#include "Headers/myMap.h"


/******************************************************************************/
/*************************      LOADER CLASS		***************************/
/******************************************************************************/
class Loader
{
private:
	unsigned int nbIntersections;
	int *nbConnected;
	
	float decX;
	float decY;
	float decZ;
	
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
	vector<Vessel> listVessels;
	
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
	Loader(string filename);
};

#endif

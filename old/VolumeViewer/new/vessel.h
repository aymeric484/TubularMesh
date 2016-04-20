#ifndef _VESSEL_H
#define _VESSEL_H

#include <vector>
#include "Geometry/vector_gen.h"
#include "graph.h"

#include "Topology/map/embeddedMap2.h"
#include "Topology/generic/attributeHandler.h"


namespace CGoGN
{

class Vessel
{

public:
	Graph* m_graph;

	VertexAttribute<Geom::Vec3f, Graph::Type> m_positions;

	VertexAttribute<float, Graph::Type> m_radiuses;

	VertexAttribute<int, Graph::Type> m_ids;

	Vertex start;

	Vertex stop;

	Vessel();

	inline float distance(const GraphTypes::Vertex& v1, const GraphTypes::Vertex& v2) {
		Geom::Vec3f c1 = m_positions[v1.m_emb];
		Geom::Vec3f c2 = m_positions[v2.m_emb];
		return std::sqrt((c1[0]-c2[0])*(c1[0]-c2[0])+(c1[1]-c2[1])*(c1[1]-c2[1])+(c1[2]-c2[2])*(c1[2]-c2[2]));
	}

};

}

/**
 * defines a blood vessel
 */
//class Vessel
//{
//public:
//    //Container::Graph<VesselRadius> g;
//    CGoGN::EmbeddedMap2 map;
//    CGoGN::VertexAttribute<VesselRadius> vesselRadiuses;

//    /**
//         * start and stop index
//         */
//    unsigned int idStart;
//    unsigned int idStop;

//    /**
//         * number of passed points after intersection cut from start
//         */
//    unsigned int decalFromStart;

//    /**
//         * number of passed points after intersection cut from end
//         */
//    unsigned int decalFromStop;

//    /**
//         * list of points corresponding to the vessel
//         */
//    std::vector<VesselRadius> vesselRadiuses;

//    /**
//         * coordinates of the elements of the vessel
//         */
//    std::vector<CGoGN::Geom::Vec3f> vesselCoordinates;

//    /**
//         * true if the vessel has been reduced/cut
//         */
//    bool reduced;

//    /**
//         * number of segments corresponding to the vessel's basis (triangle, square, polygon, ...)
//         */
//    unsigned int nbFaces;

//    /**
//         * average radius of the vessel
//         */
//    float averageRadius;

//    /**
//         * default constructor
//         */
//    Vessel();

//    /**
//         * copy constructor
//         */
//    Vessel(const Vessel& v);

//    /**
//         * add a point to the vessel
//         */
//    void addVesselRadius(VesselRadius ves);

//    /**
//         * vessel's length between two points
//         */
//    float distanceFromPointToPoint(int id0, int id1);

//    /**
//         * compute the vessel's average radius
//         */
//    void computeAverageRadius();

//    /**
//         * pre-processing: simplification of the vessel
//         */
//    void simplify();

//    /**
//         * test function
//         */
//    void verify();

//    /**
//         * function that doubles the vessel's radius
//         */
//    void dilate();

//};

///**
// * \brief Displays a Vessel.
// * \param[out] out the stream where to print the Vessel.
// * \param[in] v the VesselVertex.
// * @return a reference to the stream \p out.
// */
//inline std::ostream& operator<<(std::ostream& out, const Vessel& v) {
//    out << "[" << v.idStart << "," << v.idStop << "]";
//    return out;
//}


#endif

#ifndef VESSEL_BUILDER_H
#define VESSEL_BUILDER_H


//#include "vessel.h"
//#include <string>

//class VesselBuilder
//{

//public:
//    typedef Graph<VesselRadius> Vessel;

//    /**
//     * constructor
//     */
//    VesselBuilder(const std::string& filename);

//    /**
//     * list of vessels
//     */
//    //std::vector<Vessel> vessels;

//    std::vector<Vessel> m_vessels;

//};


#include <string>
#include "vessel.h"

namespace CGoGN {

class VesselBuilder
{

public:

	/**
	 * constructor
	 */
	VesselBuilder(const std::string& filename);

	/**
	 * list of vessels
	 */
//	std::vector<CGoGN::Graph<VesselRadius>* > m_vessels;
	std::vector<Vessel*> m_vessels;

	unsigned int nb_external_nodes;

};

}

#endif

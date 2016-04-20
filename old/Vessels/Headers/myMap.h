#ifndef MYMAP_H
#define MYMAP_H

#include "map_dual/emapd2.h"

using namespace CGoGN;

struct PFP {
	typedef DartObj<DP::MAPD2_V0_mem> DART;
	typedef e0mapd2<DART> MAP;
	typedef Emb::Point3D EMB;
	static const int id0=0;
};

typedef PFP::MAP::Dart Dart;
typedef PFP::EMB Point3D;

#endif

#include "vessel.h"

namespace CGoGN
{

// default constructor
Vessel::Vessel()
{
	m_graph = new Graph();
	m_positions = m_graph->m_map.addAttribute<Geom::Vec3f, VERTEX, Graph::Type>("positions");
	m_radiuses = m_graph->m_map.addAttribute<float, VERTEX, Graph::Type>("radiuses");
	m_ids = m_graph->m_map.addAttribute<int, VERTEX, Graph::Type>("ids");
}

}
//// copy constructor
//Vessel::Vessel(const Vessel& v)
//{
//        idStart=v.idStart;
//        idStop=v.idStop;
//        for (unsigned int i=0;i<v.vesselRadiuses.size();i++)
//        {
//                vesselRadiuses.push_back(VesselRadius(v.vesselRadiuses[i]));
//        }
//        for (unsigned int i=0;i<v.vesselCoordinates.size();i++)
//        {
//                vesselCoordinates.push_back(CGoGN::Geom::Vec3f(v.vesselCoordinates[i]));
//        }
//        decalFromStart=v.decalFromStart;
//        decalFromStop=v.decalFromStop;
//        reduced=v.reduced;
//        nbFaces=v.nbFaces;
//}


//// add a point to the vessel
//void Vessel::addVesselRadius(VesselRadius ves)
//{
//        vesselRadiuses.push_back(ves);
//}


//// vessel's length between two points
//float Vessel::distanceFromPointToPoint(int id0, int id1)
//{
//        if (id0>id1)
//        {
//                int tmp=id1;
//                id1=id0;
//                id0=tmp;
//        }
//        float res=0;
//        for (int i=id0;i<id1;i++)
//        {
//                VesselRadius v0=vesselRadiuses[i];
//                VesselRadius v1=vesselRadiuses[i+1];

//                res=res+v0.distance(v1);
//        }
//        return res;
//}


//// compute the vessel's average radius
//void Vessel::computeAverageRadius()
//{
//        float res=0;
//        for (unsigned int i=0;i<vesselRadiuses.size();i++)
//        {
//                res+=vesselRadiuses[i].radius_;
//        }
//        res=res/vesselRadiuses.size();
//        averageRadius=res;
//}


//// pre-processing: simplification of the vessel
//void Vessel::simplify()
//{
//        float rad=0;
//        int taille=vesselRadiuses.size();

//        // search for the average radius
//        for (int i=0;i<taille;i++)
//        {
//                rad+=vesselRadiuses[i].radius_;
//        }
//        rad/=(float) taille;
//        rad*=sqrt(3);

//        // we will delete the intermediate points while there is at least sqrt(3)*(average radius)
//        // distance between two points

//        // we position at the starting point
//        unsigned int posPre=decalFromStart;
//        bool raccourci=true;
//        // while we shorten it
//        while (raccourci)
//        {
//                raccourci=false;
//                bool fait=false;
//                // over the whole vessel's length
//                while (posPre+1<taille-1-decalFromStop && !fait)
//                {
//                        // if the distance is smaller than the minimal distance
//                        if (distanceFromPointToPoint(posPre,posPre+1)<rad)
//                        {
//                                // delete the point
//                                vesselRadiuses.erase(vesselRadiuses.begin()+(posPre+1));
//                                taille=vesselRadiuses.size();
//                                raccourci=true;
//                        }
//                        else
//                        {
//                                fait=true;
//                        }
//                }
//                posPre++;
//        }
//}

//// test function
//void Vessel::verify()
//{
//        std::cout << "verif" << std::endl;
//        for (unsigned int i=0;i<vesselRadiuses.size()-1;i++)
//        {
//                if (distanceFromPointToPoint(i,i+1)<0.01)
//                {
//                        std::cout << "vla un pb " << i << " " << distanceFromPointToPoint(i,i+1) << std::endl;
//                        std::cout << distanceFromPointToPoint(i,i+1) << std::endl;
//                }
//        }
//}


//// doubles the vessel's radius
//void Vessel::dilate()
//{
//        for (unsigned int i=0;i<vesselRadiuses.size();i++)
//        {
//                vesselRadiuses[i].radius_*=2;
//        }
//}




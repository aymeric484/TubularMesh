
#include "Headers/Vessel.h"

/******************************************************************************/
/********************      VESSELRADIUS CLASS FUNCTIONS	    *******************/
/******************************************************************************/

// default constructor
VesselRadius::VesselRadius()
{
	x=0;
	y=0;
	z=0;
	radius=0;
}	


// constructor
VesselRadius::VesselRadius(float px, float py, float pz, float rad)
{
	x=px;
	y=py;
	z=pz;
	radius=rad;
}


// constructor
VesselRadius::VesselRadius(const VesselRadius& v)
{
	x=v.x;
	y=v.y;
	z=v.z;
	radius=v.radius;
}

// distance function
float VesselRadius::distance(VesselRadius vr)
{
	float dis=(x-vr.x)*(x-vr.x)+(y-vr.y)*(y-vr.y)+(z-vr.z)*(z-vr.z);
	dis=sqrt(dis);
	return dis;
}


/******************************************************************************/
/*************************      VESSEL CLASS FUNCTIONS		*******************/
/******************************************************************************/

// default constructor
Vessel::Vessel()
{
	idStart=-1;
	idStop=-1;
	decalFromStart=0;
	decalFromStop=0;
	reduced=false;
	
	nbFaces=3;
	averageRadius=0;
}


// copy constructor
Vessel::Vessel(const Vessel& v)
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


// add a point to the vessel
void Vessel::addVesselRadius(VesselRadius ves)
{
	lstVesselRadius.push_back(ves);
}


// vessel's length between two points
float Vessel::distanceFromPointToPoint(int id0, int id1)
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


// compute the vessel's average radius
void Vessel::computeAverageRadius()
{
	float res=0;
	for (unsigned int i=0;i<lstVesselRadius.size();i++)
	{
		res+=lstVesselRadius[i].radius;
	}
	res=res/lstVesselRadius.size();
	averageRadius=res;
}


// pre-processing: simplification of the vessel
void Vessel::simplify()
{
	float rad=0;
	int taille=lstVesselRadius.size();
	
	// search for the average radius
	for (int i=0;i<taille;i++)
	{
		rad+=lstVesselRadius[i].radius;
	}
	rad/=(float) taille;
	rad*=sqrt(3);
	
	// we will delete the intermediate points while there is at least sqrt(3)*(average radius) 
	// distance between two points
	
	// we position at the starting point
	unsigned int posPre=decalFromStart;
	bool raccourci=true;
	// while we shorten it
	while (raccourci)
	{
		raccourci=false;
		bool fait=false;
		// over the whole vessel's length
		while (posPre+1<taille-1-decalFromStop && !fait)
		{
			// if the distance is smaller than the minimal distance
			if (distanceFromPointToPoint(posPre,posPre+1)<rad)
			{
				// delete the point
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

// test function
void Vessel::verify()
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


// doubles the vessel's radius
void Vessel::dilate()
{
	for (unsigned int i=0;i<lstVesselRadius.size();i++)
	{
		lstVesselRadius[i].radius*=2;
	}
}



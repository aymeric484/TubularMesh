#include "vesselBuilder.h"
#include <iostream>
#include <fstream>

#include "Topology/generic/cells.h"

namespace CGoGN {

class StringTokenizer
{
private:
	std::string str;
	std::string delim;
	std::string::size_type index;
	bool hasTokens;

	void decalToNextToken()
	{
		bool encore=true;
		while (index<str.length() && encore)
		{
			char cur=str.at(index);
			std::string::size_type pos=delim.find(cur,0);
			if (pos==std::string::npos)
			{
				encore=false;
			}
			else
			{
				index++;
			}
		}
	}

	void decalToEndOfToken()
	{
		bool encore=true;
		while (index<str.length() && encore)
		{
			char cur=str.at(index);
			std::string::size_type pos=delim.find(cur,0);
			if (pos!=std::string::npos)
			{
				encore=false;
			}
			else
			{
				index++;
			}
		}
	}
public:
	StringTokenizer(std::string s, std::string del)
	{
		str=std::string(s);
		delim=std::string(del);
		index=0;
		decalToNextToken();
	}

	bool hasMoreTokens()
	{
		return index<str.length();
	}

	std::string nextToken()
	{
		std::string::size_type idStart=index;
		decalToEndOfToken();

		std::string::size_type idStop=index;
		decalToNextToken();
		return std::string(str,idStart,idStop-idStart);
	}

};


VesselBuilder::VesselBuilder(const std::string& filename)
{
//	std::ifstream file(filename.c_str(), std::ios::in);

//	std::string line;
//	std::string delim("label[(=)];:> \t\"\n\rGrph}{g-");
//	while( std::getline( file, line ) )
//	{
//		StringTokenizer stk(line,delim);

//		if (line.find("((",0)== std::string::npos)
//		{
//			if (stk.hasMoreTokens())
//			{
//				Vessel ves;

//				ves.idStart=atoi((stk.nextToken()).c_str());
//				ves.idStop=atoi((stk.nextToken()).c_str());

//				while (stk.hasMoreTokens())
//				{
//					VesselRadius vr;

//					vr.x_=atof((stk.nextToken()).c_str());
//					vr.y_=atof((stk.nextToken()).c_str());
//					vr.z_=atof((stk.nextToken()).c_str());

//					stk.nextToken();
//					stk.nextToken();
//					vr.radius_=atof((stk.nextToken()).c_str());
//					stk.nextToken();
//					stk.nextToken();
//					stk.nextToken();


//					ves.addVesselRadius(vr);
//				}
//				vessels.push_back(ves);
//			}
//		}
//	}

	std::ifstream file(filename.c_str(), std::ios::in);

	std::string line;
	std::string delim("label[(=)];:> \t\"\n\rGrph}{g-");
	int nb_node = 0;
	while( std::getline( file, line ) )
	{
		StringTokenizer stk(line,delim);

		if (line.find("((",0)== std::string::npos)
		{
			if (stk.hasMoreTokens())
			{
				Vessel* ves = new Vessel();

				int idStart = atoi((stk.nextToken()).c_str());
				int idStop = atoi((stk.nextToken()).c_str());

				GraphTypes::Vertex prev;
				//First node
				if(stk.hasMoreTokens())
				{
					float x = atof((stk.nextToken()).c_str());
					float y = atof((stk.nextToken()).c_str());
					float z = atof((stk.nextToken()).c_str());
					stk.nextToken();
					stk.nextToken();
					float radius = atof((stk.nextToken()).c_str());
					stk.nextToken();
					stk.nextToken();
					stk.nextToken();

					prev = ves->m_graph->new_vertex();
					ves->m_positions[prev.m_emb] = Geom::Vec3f(x, y, z);
					ves->m_radiuses[prev.m_emb] = radius;
					ves->m_ids[prev.m_emb] = idStart;


					std::cout << Geom::Vec3f(x, y, z) << std::endl << std::endl;
				}

				while (stk.hasMoreTokens())
				{
					std::cout << "plop" << std::endl;

					float x = atof((stk.nextToken()).c_str());
					float y = atof((stk.nextToken()).c_str());
					float z = atof((stk.nextToken()).c_str());
					stk.nextToken();
					stk.nextToken();
					float radius = atof((stk.nextToken()).c_str());
					stk.nextToken();
					stk.nextToken();
					stk.nextToken();

					GraphTypes::Vertex nv = ves->m_graph->new_vertex();
					ves->m_positions[nv.m_emb] = Geom::Vec3f(x, y, z);
					ves->m_radiuses[nv.m_emb] = radius;
					ves->m_ids[nv.m_emb] = -1;

					std::cout << Geom::Vec3f(x, y, z) << std::endl;

					ves->m_graph->connect_vertices(prev, nv);

					if(ves->m_ids[prev.m_emb] != -1)
						ves->start = Vertex(prev.m_d);
					prev = nv;
				}

				//last id
				ves->m_ids[prev.m_emb] = idStop;

				ves->stop = Vertex(prev.m_d);

				std::cout << ves->start << " | " << ves->stop << std::endl;

				m_vessels.push_back(ves);
			}
		}
		else
			nb_node++;
	}

	nb_external_nodes = --nb_node;
	std::cout << "nb_external_nodes=" << nb_external_nodes << std::endl;
}

}

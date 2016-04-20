#ifndef _GRAPH_H
#define _GRAPH_H

#include <set>
#include <vector>
#include <iostream>


#include "Topology/map/embeddedMap2.h"
#include "Topology/generic/attributeHandler.h"

namespace CGoGN {

namespace GraphTypes {

	struct Vertex {

		unsigned int m_emb;
		Dart m_d;

		Vertex() : m_emb(EMBNULL), m_d(NIL) {}

		Vertex(unsigned int emb) : m_emb(emb), m_d(NIL) {}


	};

	struct Edge {

		Dart m_d;

		Edge() : m_d(NIL) {}

		Edge(Dart d) : m_d(d) {}
	};

}


class Graph {

public:
	typedef EmbeddedMap2 Type;

	EmbeddedMap2 m_map;

public:
	Graph() {  }

	~Graph() { m_map.clear(true); }

	/**
	 * \brief new_vertex
	 * \param[in] data
	 * \return
	 */
	GraphTypes::Vertex new_vertex() {
		AttributeContainer& m_container = m_map.getAttributeContainer<VERTEX>() ;
		unsigned int id = m_container.insertLine();
		return GraphTypes::Vertex(id);
	}

	GraphTypes::Edge new_edge() {
		Dart d = m_map.newPolyLine(1);
		return GraphTypes::Edge(d);
	}


	/**
	 * \brief Connects the vertices v1 and v2 by
	 * introducing an edge between them
	 * \param[in] v1
	 * \param[in] v2
	 * \return
	 */
	GraphTypes::Edge connect_vertices(GraphTypes::Vertex v1, GraphTypes::Vertex v2) {

		GraphTypes::Edge d = new_edge();

		if(v1.m_d != NIL)
			m_map.connectVertices(v1.m_d, d.m_d);

		if(v2.m_d != NIL)
			m_map.connectVertices(v2.m_d, m_map.phi2(d.m_d));

		m_map.setDartEmbedding<VERTEX>(d.m_d, v1.m_emb);
		m_map.setDartEmbedding<VERTEX>(m_map.phi1(d.m_d), v2.m_emb);
//		v1.m_d = d.m_d;
//		v2.m_d = m_map.phi1(d.m_d);
	}

	/**
	 * \brief Disconnects the vertices v1 and v2 by
	 * removing their common edge
	 * \param[in] v1
	 * \param[in] v2
	 */
	void disconnect_edges(Vertex v1, Vertex v2) {

	}

	/**
	 * \brief Returns in l all vertices connected to v, directly
	 * or indirectly
	 * \param[in] v
	 * \param[out] l
	 */
	void get_connected_component(Vertex v, std::vector<Vertex>& l) {

	}

};

//template <typename T>
//inline std::ostream& operator<<(std::ostream& out, const Graph<typename T>& g)
//{
//	out << "[";
//	for(unsigned int i = 0 ; i < g.vertices.nbElements() ; ++i)
//	{
//		out << "(" << g.vertices[i] << ")";

//	}
//	out << "]" << std::endl;
//	return out;
//}


}





//namespace Container {



//template <typename T>
//struct Vertex {
//private:
//    typedef std::set<int> Neighbours;
//    T data_;
//    int id_;
//    bool extremity;

//public:
//    Neighbours neighbours;

//    Vertex( int id, T& data ): id_(id), data_(data) {}

//    bool operator<( const Vertex<T>& ref ) const {
//        return ( ref.id_ < id_ );
//    }
//    bool operator==( const Vertex<T>& ref ) const {
//        return ( ref.id_ == id_ );
//    }
//};

//template<typename T>
//inline std::ostream& operator<<(std::ostream& out, const Vertex<typename T>& v) {
//    out << "[" << v.id_ << " ; " << "]" << " -- ";

//    return out;
//}



//template <typename T>
//class Graph
//{
//private :
//    std::vector<Container::Vertex<T> > vertices;


//    void addEdgeIndices ( int index1, int index2 ) {
//        vertices[ index1 ].neighbours.insert( index2 );
//    }

//    typename std::vector<Container::Vertex<T> >::iterator findVertexIndex( int val, bool& res )
//    {
//        typename std::vector<Container::Vertex<T> >::iterator it;
//        Container::Vertex<T> v(val);
//        it = std::find( vertices.begin(), vertices.end(), v );
//        if (it != vertices.end()){
//            res = true;
//            return it;
//        } else {
//            res = false;
//            return vertices.end();
//        }
//    }

//public:
//    int addVertex(Container::Vertex<T> v) {
//        vertices.push_back(v);
//        return vertices.size() - 1;
//    }

//    Container::Vertex<T>& at(int i)
//    {
//        return vertices[i];
//    }

//    void addEdge ( int n1, int n2 ) {

//        bool foundNet1 = false, foundNet2 = false;
//        typename std::vector<Container::Vertex<T> >::iterator vit1 = findVertexIndex( n1, foundNet1 );
//        int node1Index = -1, node2Index = -1;
//        if ( !foundNet1 ) {
//            Container::Vertex<T> v1( n1 );
//            vertices.push_back( v1 );
//            node1Index = vertices.size() - 1;
//        } else {
//            node1Index = vit1 - vertices.begin();
//        }
//        typename std::vector<Container::Vertex<T> >::iterator vit2 = findVertexIndex( n2, foundNet2);
//        if ( !foundNet2 ) {
//            Container::Vertex<T> v2( n2 );
//            vertices.push_back( v2 );
//            node2Index = vertices.size() - 1;
//        } else {
//            node2Index = vit2 - vertices.begin();
//        }

//        assert( ( node1Index > -1 ) && ( node1Index <  vertices.size()));
//        assert( ( node2Index > -1 ) && ( node2Index <  vertices.size()));

//        addEdgeIndices( node1Index, node2Index );
//    }

//};

///**
// * \brief Displays a Vessel.
// * \param[out] out the stream where to print the Vessel.
// * \param[in] v the VesselVertex.
// * @return a reference to the stream \p out.
// */
//template<typename T>
//inline std::ostream& operator<<(std::ostream& out, const Graph<typename T>& v) {
//    for(unsigned int i = 0 ; i < v.vertices.size() ; i++)
//    {
//        out << v.vertices[i] << " -- ";
//    }
//    return out;
//}


//}


#endif // GRAPH_H


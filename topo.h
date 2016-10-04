#ifndef TOPO_H
#define TOPO_H

#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/core/basic/dart.h>
#include <cgogn/core/utils/unique_ptr.h>

#include <cgogn/io/map_import.h> // peut etre pas besoin
#include <cgogn/io/tetgen_io.h>
#include <cgogn/io/volume_import.h>

#include <cgogn/geometry/algos/normal.h>


#include "tetgen.h"
#include "squelette.h"
#include "tetgen_structure_io.h"
#include "branchtopo.h"


using Map3 = cgogn::CMap3<cgogn::DefaultMapTraits>;
using Map2 = cgogn::CMap2<cgogn::DefaultMapTraits>;

using Vertex = typename Map3::Vertex;
using Vertex32 = typename Map3::Vertex2;
using Vertex2 = typename Map2::Vertex;
using Edge = typename Map3::Edge;
using Face = typename Map3::Face;
using Face2 = typename Map2::Face;
using Volume = typename Map3::Volume;

using Dart = cgogn::Dart;

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

using tetgenio = tetgen::tetgenio;
using TetgenReal = REAL;


template <typename T1>
using VertexAttribute = Map3::VertexAttribute<T1>;
template <typename T2>
using VertexAttribute2 = Map2::VertexAttribute<T2>;
template <typename T3>
using Vertex2Attribute = Map3::Attribute<T3, Map3::Vertex2::ORBIT>;

using MapBuilder = cgogn::CMap3Builder_T<Map3::MapTraits>;
using MapBuilder2 = cgogn::CMap2Builder_T<Map2::MapTraits>;


class topo
{

public:

    topo(const Squelette&, const unsigned int&);

    void SubdivisionConcentrique();
    void InterpolationConcentrique();
    void UpdateCoordinates();
    void GetCouchesConcentriques();
    void SubdivisionCouche(const unsigned int&);

    Map3 map_;
    VertexAttribute<Vec3> vertex_position_;
    VertexAttribute<Vec3> vertex_position_bis_;
    VertexAttribute<Vec3> vertex_position_ter_;
    VertexAttribute<Vec3> vertex_inter_position_;

    VertexAttribute<Vec3> vertex_normal_;
    VertexAttribute<Vec3> vertex_normal_bis_;
    VertexAttribute<int> vertex_appartenance_;
    VertexAttribute<int> vertex_appartenance_bis_;


    //VertexAttribute<int> vertex_appartenance_;

    int nb_appuis_;
    int indice_repartition_;
    VertexAttribute2<Vec3> vertex2_position_;



private:

    void MakeFromSkeleton(const Squelette&, const unsigned int&);
    void Generate_tetgen(const std::string&, int);
    void MakeIntersection(std::vector<TriangleGeo>, std::vector<Vec3> sommets_intersection);
    void MakeBranch(const std::vector<Vec3>&, const unsigned int&);
    void TestMergeSew();
    void TestSimpleSew();
    //void Sewbranches()

    std::unique_ptr<tetgenio> export_tetgen();


    Map3 map3inter_;
    Map3 map3branch_;
    Map2 map2_;

    std::vector<BranchTopo> controls_;
    std::vector<int> save_pos_;


    //VertexAttribute<Vec3> vertex_position2_;

    cgogn::CellCache<Map3> cell_cache_prec_;



};

#endif // TOPO_H

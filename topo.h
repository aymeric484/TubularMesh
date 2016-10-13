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
using Edge2 = typename Map2::Edge;
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

    // Subdivise toute les branches en ajoutant une couche concentrique
    void SubdivisionConcentrique();

    // Nous permet de déplacer plusieurs couches concentriques ensemble => reduire ou augmenter leur épaisseur
    void InterpolationConcentrique();

    // Subdivise toute les couches concentriques
    void SubdivisionCouche(const unsigned int&);

    // Permet de recaler les points obtenus par SubdivisionCouche si l'on a modifier l'épaisseur des couches concentrique
    void UpdateCoordinates();

    // Remplis la variable controls en ajoutant les dart de chaque couche concentrique
    void GetCouchesConcentriques();


    Map3 map_; // La carte combinatoire principale que l'on affichera

    VertexAttribute<Vec3> vertex_position_; // Positions que l'on utilisera

    VertexAttribute<Vec3> vertex_position_bis_; // Pas utilisé
    VertexAttribute<Vec3> vertex_position_ter_; // Pas utilisé
    VertexAttribute<Vec3> vertex_inter_position_; // Pas utilisé

    VertexAttribute<Vec3> vertex_normal_; // S'utilisera pour l'affichage

    VertexAttribute<Vec3> vertex_normal_bis_; // Pas utilisé

    VertexAttribute<int> vertex_appartenance_;

    VertexAttribute<int> vertex_appartenance_bis_; // Pas utilisé

    int nb_appuis_; // Compte le nb d'appui sur "C" => le nombre de couche concentrique que l'on créer

    int indice_repartition_; // Compte le nb d'appuis sur "+" et décompte le nb d'appui sur "-". Sert à décaler les couches concentriques

    VertexAttribute2<Vec3> vertex2_position_; // Position de la map2 contenant l'intersection, généré par MakeIntersection()

    Dart dart_to_color_red_;
    Dart dart_to_color_blue_;
    Dart dart_to_color_black_;



private:

    // Créer toute les branches de la map_
    // Appel à MakeIntersection, Generate_tetgen pour créer une map3 intermédiaire
    // Fusion + couture des intersections avec les branches
    void MakeFromSkeleton(const Squelette&, const unsigned int&);

    // Génère une map3 contenant l'intersection à partir d'une struct tetgenio
    void Generate_tetgen(const std::string&, int);

    // Génère une tetgenio à partir d'une map2 contenant la surface de l'intersection
    std::unique_ptr<tetgenio> export_tetgen();

    // Génère une map2 contenant la surface de l'intersection à partir d'un tableau de triangelGeo contenant la connectivité de l'enveloppe convexe
    void MakeIntersection(std::vector<TriangleGeo>, std::vector<Vec3> sommets_intersection);

    // Pas utilisé
    void MakeBranch(const std::vector<Vec3>&, const unsigned int&);

    // Test de couture après une fusion
    void TestMergeSew();

    // Test de couture simple
    void TestSimpleSew();




    Map3 map3inter_; // map3 contenant une intersection subdivisé
    Map3 map3branch_; // Pas utilisé
    Map2 map2_; // map2 contenant la surface de l'intersection

    std::vector<BranchTopo> controls_; // Tableau contenant les darts sauvegardé au cours de l'extrusion et des subdivision concentrique, pour chaque branche
    std::vector<int> save_pos_; // Pas utilisé

    cgogn::CellCache<Map3> cell_cache_prec_;

};

#endif // TOPO_H

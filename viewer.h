#ifndef VIEWER_H
#define VIEWER_H

#include <QApplication>

#include <QMatrix4x4>

#include <math.h>

#include <qoglviewer.h>
#include <QKeyEvent>

#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/core/basic/dart.h>
#include <cgogn/core/utils/unique_ptr.h>

#include <cgogn/io/map_import.h> // peut etre pas besoin
#include <cgogn/io/tetgen_io.h>
#include <cgogn/io/volume_import.h>

#include <cgogn/rendering/map_render.h>
#include <cgogn/rendering/drawer.h>
#include <cgogn/rendering/volume_drawer.h>
#include <cgogn/rendering/topo_drawer.h>

#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/shaders/shader_simple_color.h>
#include <cgogn/rendering/shaders/shader_flat.h>
#include <cgogn/rendering/shaders/shader_phong.h>
#include <cgogn/rendering/shaders/shader_vector_per_vertex.h>
#include <cgogn/rendering/shaders/shader_bold_line.h>
#include <cgogn/rendering/shaders/shader_point_sprite.h>
#include <cgogn/rendering/shaders/shader_round_point.h>

#include <cgogn/geometry/algos/ear_triangulation.h>
#include <cgogn/geometry/algos/bounding_box.h>
#include <cgogn/geometry/algos/normal.h>
#include <cgogn/geometry/algos/filtering.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include "tetgen.h"
//#include "branch.h"
#include "window.h"
#include "coucheconcentrique.h"
#include "squelette.h"
#include "tetgen_structure_io.h"

/*

using Inherit = cgogn::io::VolumeImport<cgogn::CMap3<cgogn::DefaultMapTraits>::MapTraits>;
template <typename T>
using ChunkArray = typename Inherit::template ChunkArray<T>;*/

using Map3 = cgogn::CMap3<cgogn::DefaultMapTraits>;
using Map2 = cgogn::CMap2<cgogn::DefaultMapTraits>;

using Vertex = typename Map3::Vertex;
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
using MapBuilder = cgogn::CMap3Builder_T<Map3::MapTraits>;
using MapBuilder2 = cgogn::CMap2Builder_T<Map2::MapTraits>;


class Viewer : public QOGLViewer
{

public:

    Viewer();
    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;

    virtual void draw();
    virtual void init();
    void update_bb();

    virtual void keyPressEvent(QKeyEvent *);
    void mousePressEvent(QMouseEvent*);
    void MakeFromSkeleton(const std::vector<Vec3>&, const unsigned int&);
    void generate_tetgen(const std::string&);

    // Méthode temporaire de test
    //void MakeIntersection(std::vector<TriangleGeo>, Vec4); // Elle sera plus à inclure plus tard dans une boucle d'intersection de MakeFromSkeleton qui contiendra aussi une boucle de branche
    void MakeIntersection(std::vector<TriangleGeo> triangles, std::vector<Vec3> sommets_intersection);
    //void OrientationFromSkel
    virtual ~Viewer();
    virtual void closeEvent(QCloseEvent *e);



private:

    void InterpolationConcentrique();
    void UpdateCoordinates();
    void GetCouchesConcentriques();
    void SubdivisionCouche(const unsigned int&);
    std::unique_ptr<tetgenio> export_tetgen();

    Map3 map_;
    Map3 map3bis_;
    Map2 map2_;

    std::vector<CoucheConcentrique> subdivised_volume_control_;
    std::vector<Dart> volume_control_;

    VertexAttribute<Vec3> vertex_position_;
    VertexAttribute2<Vec3> vertex2_position_;


    VertexAttribute<Vec3> vertex_position2_;
    VertexAttribute<Vec3> vertex_normal_;


    cgogn::CellCache<Map3> cell_cache_prec_;

    cgogn::geometry::AABB<Vec3> bb_;

    cgogn::rendering::VBO* vbo_pos_;

    cgogn::rendering::MapRender* render_;

    cgogn::rendering::TopoDrawer* topo_drawer_;
    std::unique_ptr<cgogn::rendering::TopoDrawer::Renderer> topo_drawer_rend_;

    cgogn::rendering::VolumeDrawer* volume_drawer_;
    std::unique_ptr<cgogn::rendering::VolumeDrawer::Renderer> volume_drawer_rend_;

    cgogn::rendering::DisplayListDrawer* drawer_;
    std::unique_ptr<cgogn::rendering::DisplayListDrawer::Renderer> drawer_rend_;

    std::unique_ptr<cgogn::rendering::ShaderPointSprite::Param> param_point_sprite_;

    bool volume_rendering_;
    bool topo_rendering_;
    bool vertices_rendering_;
    bool edge_rendering_;
    bool bb_rendering_;

    float volume_expl_;
    int nb_appuis_;
    int indice_repartition_;
};
#endif // VIEWER_H

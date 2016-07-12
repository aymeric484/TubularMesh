#ifndef VIEWER_H
#define VIEWER_H

#include <QApplication>

#include <QMatrix4x4>

#include <math.h>

#include <qoglviewer.h>
#include <QKeyEvent>

#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/basic/dart.h>

#include <cgogn/io/map_import.h> // peut etre pas besoin

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

//#include "branch.h"
#include "window.h"
#include "coucheconcentrique.h"
#include "squelette.h"



using Map3 = cgogn::CMap3<cgogn::DefaultMapTraits>;
using Vertex = typename Map3::Vertex;
using Edge = typename Map3::Edge;
using Face = typename Map3::Face;
using Volume = typename Map3::Volume;

using Dart = cgogn::Dart;

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;


template <typename T>
using VertexAttribute = Map3::VertexAttribute<T>;
using MapBuilder = cgogn::CMap3Builder_T<Map3::MapTraits>;


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
    void MakeFromBranch(const std::vector<Vec3>&, const unsigned int&);

    //void OrientationFromSkel
    virtual ~Viewer();
    virtual void closeEvent(QCloseEvent *e);



private:

    void InterpolationConcentrique();
    void UpdateCoordinates();
    void GetCouchesConcentriques();
    void SubdivisionCouche(const unsigned int&);

    Map3 map_;

    std::vector<CoucheConcentrique> subdivised_volume_control_;
    std::vector<Dart> volume_control_;

    VertexAttribute<Vec3> vertex_position_;
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

#ifndef VIEWER2_H
#define VIEWER2_H


#include <QApplication>
#include <QMatrix4x4>
#include <QKeyEvent>

#include <qoglviewer.h>

#include <cgogn/core/cmap/cmap2.h>
//#include <cgogn/core/cmap/cmap2_tri.h>
//#include <cgogn/core/cmap/cmap2_quad.h>

#include <cgogn/io/map_import.h>

#include <cgogn/geometry/algos/bounding_box.h>
#include <cgogn/geometry/algos/normal.h>

#include <cgogn/rendering/map_render.h>
#include <cgogn/rendering/shaders/shader_simple_color.h>
#include <cgogn/rendering/shaders/shader_flat.h>
#include <cgogn/rendering/shaders/shader_phong.h>
#include <cgogn/rendering/shaders/shader_vector_per_vertex.h>
#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/shaders/shader_bold_line.h>
#include <cgogn/rendering/shaders/shader_point_sprite.h>

#include <cgogn/rendering/shaders/shader_round_point.h>

#include <cgogn/geometry/algos/ear_triangulation.h>

#include <cgogn/rendering/drawer.h>

#include "window.h"
#include "coucheconcentrique.h"
#include "squelette.h"

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using Map2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
//using Map2 = cgogn::CMap2Tri<cgogn::DefaultMapTraits>;
//using Map2 = cgogn::CMap2Quad<cgogn::DefaultMapTraits>;


using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;


using Vertex2 = typename Map2::Vertex;
using Edge2 = typename Map2::Edge;
using Face2 = typename Map2::Face;
using Dart = cgogn::Dart;
using MapBuilder2 = cgogn::CMap2Builder_T<Map2::MapTraits>;


template <typename T>
using VertexAttribute2 = Map2::VertexAttribute<T>;


class Viewer2 : public QOGLViewer
{
public:

    Viewer2();

    CGOGN_NOT_COPYABLE_NOR_MOVABLE(Viewer2);

    virtual void draw();
    virtual void init();
    void import(const std::string& surface_mesh);
    virtual ~Viewer2();
    virtual void closeEvent(QCloseEvent *e);
    void MakeIntersection(std::vector<TriangleGeo>, Vec4, std::vector<Vec3>);

private:

    Map2 map_;
    VertexAttribute2<Vec3> vertex_position_;
    VertexAttribute2<Vec3> vertex_normal_;

    cgogn::geometry::AABB<Vec3> bb_;

    std::unique_ptr<cgogn::rendering::MapRender> render_;

    std::unique_ptr<cgogn::rendering::VBO> vbo_pos_;
    std::unique_ptr<cgogn::rendering::VBO> vbo_norm_;
    std::unique_ptr<cgogn::rendering::VBO> vbo_color_;
    std::unique_ptr<cgogn::rendering::VBO> vbo_sphere_sz_;

    std::unique_ptr<cgogn::rendering::ShaderBoldLine::Param> param_edge_;
    std::unique_ptr<cgogn::rendering::ShaderFlat::Param> param_flat_;
    std::unique_ptr<cgogn::rendering::ShaderVectorPerVertex::Param> param_normal_;
    std::unique_ptr<cgogn::rendering::ShaderPhongColor::Param> param_phong_;
    std::unique_ptr<cgogn::rendering::ShaderPointSpriteColorSize::Param> param_point_sprite_;


    std::unique_ptr<cgogn::rendering::DisplayListDrawer> drawer_;
    std::unique_ptr<cgogn::rendering::DisplayListDrawer::Renderer> drawer_rend_;

    bool phong_rendering_;
    bool flat_rendering_;
    bool vertices_rendering_;
    bool edge_rendering_;
    bool normal_rendering_;
    bool bb_rendering_;
};

#endif


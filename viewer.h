#ifndef VIEWER_H
#define VIEWER_H

#include <QApplication>

#include <QMatrix4x4>

#include <math.h>

#include <qoglviewer.h>
#include <QKeyEvent>

#include <cgogn/core/utils/unique_ptr.h>

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

#include "topo.h"

class Viewer : public QOGLViewer
{

public:

    Viewer(Squelette);
    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;

    virtual void draw();
    virtual void init();
    void update_bb();

    virtual void keyPressEvent(QKeyEvent *);
    void mousePressEvent(QMouseEvent*);

    virtual ~Viewer();
    virtual void closeEvent(QCloseEvent *e);
    topo GetTopo(topo&);


private:

    topo ma_topo_;

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

};
#endif // VIEWER_H

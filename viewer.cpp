#include "viewer.h"

Viewer::~Viewer()
{}

void Viewer::closeEvent(QCloseEvent*)
{
    delete drawer_;
    delete volume_drawer_;
    delete topo_drawer_;
    delete render_;
    delete vbo_pos_;
}

Viewer::Viewer(Squelette mon_squelette) :
    ma_topo_(mon_squelette, TYPE_PRIMITIVE),
    bb_(),
    vbo_pos_(nullptr),
    render_(nullptr),
    topo_drawer_(nullptr),
    topo_drawer_rend_(nullptr),
    volume_drawer_(nullptr),
    volume_drawer_rend_(nullptr),
    drawer_(nullptr),
    drawer_rend_(nullptr),
    volume_rendering_(true),
    topo_rendering_(false),
    vertices_rendering_(false),
    edge_rendering_(false),
    bb_rendering_(true),
    volume_expl_(0.8f)
{
    cgogn::geometry::compute_AABB(ma_topo_.vertex_position_, bb_);
    setSceneRadius(bb_.diag_size()/2.0);
    Vec3 center = bb_.center();
    setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
    showEntireScene();
}

void Viewer::keyPressEvent(QKeyEvent *ev)
{
    switch (ev->key()) {
        case Qt::Key_Z:
        {
            ma_topo_.UpdateCoordinates();
            std::cout << "coucou" << std::endl;
            break;
        }
        case Qt::Key_L:
        {
            ma_topo_.SubdivisionCouche(3);
            break;
        }
        case Qt::Key_Minus:
        {
            ma_topo_.indice_repartition_--;
            ma_topo_.InterpolationConcentrique(); // Calcul des coordonées de chaque points ajouté par "C"
            break;
        }
        case Qt::Key_Plus:
        {
            ma_topo_.indice_repartition_++;
            ma_topo_.InterpolationConcentrique(); // Calcul des coordonées de chaque points ajouté par "C"
            break;
        }
        case Qt::Key_E:
            edge_rendering_ = !edge_rendering_;
            break;
        case Qt::Key_V:
            vertices_rendering_ = !vertices_rendering_;
            break;
        case Qt::Key_T:
            topo_rendering_ = !topo_rendering_;
            break;
        case Qt::Key_B:
            bb_rendering_ = !bb_rendering_;
            break;
        case Qt::Key_C:
        {
            ma_topo_.nb_appuis_++;
            ma_topo_.SubdivisionConcentrique();
            ma_topo_.InterpolationConcentrique();
            ma_topo_.GetCouchesConcentriques();
            break;
        }

        default:
            break;
    }

    // On met tout à jour pour le viewer (géométrie et topologie)
    cgogn::rendering::update_vbo(ma_topo_.vertex_position_, vbo_pos_);
    volume_drawer_->update_face<Vec3>(ma_topo_.map_, ma_topo_.vertex_position_);
    volume_drawer_->update_edge<Vec3>(ma_topo_.map_, ma_topo_.vertex_position_);
    topo_drawer_->update<Vec3>(ma_topo_.map_, ma_topo_.vertex_position_);
    render_->init_primitives(ma_topo_.map_, cgogn::rendering::POINTS);

    // enable QGLViewer keys
    QOGLViewer::keyPressEvent(ev);
    //update drawing
    update();
}

void Viewer::mousePressEvent(QMouseEvent* e)
{
    QOGLViewer::mousePressEvent(e);
}

void Viewer::draw()
{
    QMatrix4x4 proj;
    QMatrix4x4 view;
    camera()->getProjectionMatrix(proj);
    camera()->getModelViewMatrix(view);

    if (volume_rendering_)
    {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0f, 1.0f);
        volume_drawer_rend_->draw_faces(proj, view, this);
        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    if (vertices_rendering_)
    {
        param_point_sprite_->bind(proj, view);
        render_->draw(cgogn::rendering::POINTS);
        param_point_sprite_->release();
    }

    if (topo_rendering_)
    {
        topo_drawer_->update_color(ma_topo_.dart_to_color_blue_,Vec3(1.0, 0.0, 0.0));
        topo_drawer_->update_color(ma_topo_.dart_to_color_red_,Vec3(0.0, 0.0, 1.0));
        //topo_drawer_->update_color(ma_topo_.dart_to_color_black_,Vec3(0.0, 0.0, 0.0));
        topo_drawer_rend_->draw(proj, view, this);
    }

    if (edge_rendering_)
        volume_drawer_rend_->draw_edges(proj, view, this);

    if (bb_rendering_)
        drawer_rend_->draw(proj, view, this);
}

void Viewer::init()
{
    glClearColor(0.1f,0.1f,0.3f,0.0f);

    vbo_pos_ = new cgogn::rendering::VBO(3);
    cgogn::rendering::update_vbo(ma_topo_.vertex_position_, vbo_pos_);

    render_ = new cgogn::rendering::MapRender();
    render_->init_primitives(ma_topo_.map_, cgogn::rendering::POINTS);

    topo_drawer_ =  new cgogn::rendering::TopoDrawer();
    topo_drawer_rend_ = topo_drawer_->generate_renderer();
    topo_drawer_->set_explode_volume(volume_expl_);
    topo_drawer_->update<Vec3>(ma_topo_.map_, ma_topo_.vertex_position_);

    volume_drawer_ = new cgogn::rendering::VolumeDrawer();
    volume_drawer_->update_face<Vec3>(ma_topo_.map_, ma_topo_.vertex_position_);
    volume_drawer_->update_edge<Vec3>(ma_topo_.map_, ma_topo_.vertex_position_);
    volume_drawer_rend_ = volume_drawer_->generate_renderer();
    volume_drawer_rend_->set_explode_volume(volume_expl_);

    drawer_ = new cgogn::rendering::DisplayListDrawer();
    drawer_rend_ = drawer_->generate_renderer();

    update_bb();

    param_point_sprite_ = cgogn::rendering::ShaderPointSprite::generate_param();
    param_point_sprite_->set_position_vbo(vbo_pos_);
    param_point_sprite_->size_ = bb_.diag_size()/1000.0;
    param_point_sprite_->color_ = QColor(255,0,0);
}

void Viewer::update_bb()
{
    cgogn::geometry::compute_AABB(ma_topo_.vertex_position_, bb_);

    drawer_->new_list();
    drawer_->line_width_aa(2.0);
    drawer_->begin(GL_LINE_LOOP);
        drawer_->color3f(1.0,1.0,1.0);
        drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.min()[2]);
        drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.min()[2]);
        drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.min()[2]);
        drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.min()[2]);
        drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.max()[2]);
        drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.max()[2]);
        drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.max()[2]);
        drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.max()[2]);
    drawer_->end();
    drawer_->begin(GL_LINES);
    drawer_->color3f(1.0,1.0,1.0);
        drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.min()[2]);
        drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.min()[2]);
        drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.max()[2]);
        drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.max()[2]);
        drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.min()[2]);
        drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.max()[2]);
        drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.min()[2]);
        drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.max()[2]);
    drawer_->end();
    drawer_->end_list();
}


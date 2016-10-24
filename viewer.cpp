#include "viewer.h"

Viewer::~Viewer()
{}

void Viewer::closeEvent(QCloseEvent*)
{
    //delete drawer_;
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


void Viewer::rayClick(QMouseEvent* event, qoglviewer::Vec& P, qoglviewer::Vec& Q)
{
    P = camera()->unprojectedCoordinatesOf(qoglviewer::Vec(event->x(), event->y(), 0.0));
    Q = camera()->unprojectedCoordinatesOf(qoglviewer::Vec(event->x(), event->y(), 1.0));
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
        case Qt::Key_X:
            frame_manip_->rotate(cgogn::rendering::FrameManipulator::Xr, 0.1507f);
        break;

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

void Viewer::mousePressEvent(QMouseEvent* event)
{
    qoglviewer::Vec P;
    qoglviewer::Vec Q;
    rayClick(event, P, Q);
    Vec3 A(P[0], P[1], P[2]);
    Vec3 B(Q[0], Q[1], Q[2]);


    if ((event->modifiers() & Qt::ControlModifier) && !(event->modifiers() & Qt::ShiftModifier))
        frame_manip_->pick(event->x(), event->y(),P,Q);

    if ((event->modifiers() & Qt::ShiftModifier) && !(event->modifiers() & Qt::ControlModifier))
    {
        drawer_->new_list();
        std::vector<Map3::Volume> selected;
        cgogn::geometry::picking<Vec3>(ma_topo_.map_, ma_topo_.vertex_position_, A, B, selected);
        cgogn_log_info("Viewer") << "Selected volumes: " << selected.size();
        if (!selected.empty())
        {
            drawer_->line_width(2.0);
            drawer_->begin(GL_LINES);
            // closest vol in red
            drawer_->color3f(1.0, 0.0, 0.0);
            cgogn::rendering::add_to_drawer<Vec3>(ma_topo_.map_, selected[0], ma_topo_.vertex_position_, drawer_.get());
            // others in yellow
            drawer_->color3f(1.0, 1.0, 0.0);
            for (int i = 1u; i < selected.size(); ++i)
                cgogn::rendering::add_to_drawer<Vec3>(ma_topo_.map_, selected[i], ma_topo_.vertex_position_, drawer_.get());
            drawer_->end();
        }
        drawer_->line_width(4.0);
        drawer_->begin(GL_LINES);
        drawer_->color3f(1.0, 0.0, 1.0);
        drawer_->vertex3fv(A);
        drawer_->vertex3fv(B);
        drawer_->end();

        drawer_->end_list();
    }

    if ((event->modifiers() & Qt::ShiftModifier) && (event->modifiers() & Qt::ControlModifier))
    {
        Vec3 position,axis_z;
        frame_manip_->get_position(position);
        frame_manip_->get_axis(cgogn::rendering::FrameManipulator::Zt,axis_z);
        float d = -(position.dot(axis_z));
        QVector4D plane(axis_z[0],axis_z[1],axis_z[2],d);

        cgogn::Dart da = topo_drawer_->pick(A,B,plane);
        if (!da.is_nil())
        {
            topo_drawer_->update_color(da, Vec3(1.0,0.0,0.0));
        }
    }
    QOGLViewer::mousePressEvent(event);
    update();
}

void Viewer::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->modifiers() & Qt::ControlModifier)
        frame_manip_->release();

    QOGLViewer::mouseReleaseEvent(event);
    update();
}

void Viewer::mouseMoveEvent(QMouseEvent* event)
{
    if (event->modifiers() & Qt::ControlModifier)
    {
        bool local_manip = (event->buttons() & Qt::RightButton);
        frame_manip_->drag(local_manip, event->x(), event->y());

        // get/compute Z plane
        Vec3 position;
        Vec3 axis_z;
        frame_manip_->get_position(position);
        frame_manip_->get_axis(cgogn::rendering::FrameManipulator::Zt,axis_z);
        float d = -(position.dot(axis_z));
        // and set clipping
        volume_drawer_rend_->set_clipping_plane(QVector4D(axis_z[0],axis_z[1],axis_z[2],d));
        topo_drawer_rend_->set_clipping_plane(QVector4D(axis_z[0],axis_z[1],axis_z[2],d));
    }


    QOGLViewer::mouseMoveEvent(event);
    update();
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

    frame_manip_->draw(true,true,proj, view, this);

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

    //drawer_ = new cgogn::rendering::DisplayListDrawer();
    drawer_ = cgogn::make_unique<cgogn::rendering::DisplayListDrawer>();
    drawer_rend_ = drawer_->generate_renderer();
    frame_manip_ = cgogn::make_unique<cgogn::rendering::FrameManipulator>();
    frame_manip_->set_size(bb_.diag_size()/4);
    frame_manip_->set_position(bb_.max());
    frame_manip_->z_plane_param(QColor(50,50,50),-1.5f,-1.5f, 2.0f);

    Vec3 position;
    Vec3 axis_z;
    frame_manip_->get_position(position);
    frame_manip_->get_axis(cgogn::rendering::FrameManipulator::Zt,axis_z);
    float d = -(position.dot(axis_z));
    volume_drawer_rend_->set_clipping_plane(QVector4D(axis_z[0],axis_z[1],axis_z[2],d));
    topo_drawer_rend_->set_clipping_plane(QVector4D(axis_z[0],axis_z[1],axis_z[2],d));

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


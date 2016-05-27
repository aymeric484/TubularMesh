/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France       *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Web site: http://cgogn.unistra.fr/                                           *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/


#include <QApplication>

#include <QMatrix4x4>

#include <math.h>

#include <qoglviewer.h>
#include <QKeyEvent>

#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/basic/dart.h>

#include <cgogn/io/map_import.h>

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

#include "branch.h"
#include "window.h"

int counting=0;

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
    void MakeFromBranch(const std::vector<Vec4>& branche, const std::vector<Vec3>& positions, const unsigned int& primitives);
    //void OrientationFromSkel
	virtual ~Viewer();
	virtual void closeEvent(QCloseEvent *e);

private:

    Map3 map_;

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

    unsigned int affinage_;

    float volume_expl_;
};


//
// IMPLEMENTATION
//


void Viewer::MakeFromBranch(const std::vector<Vec4>& branche, const std::vector<Vec3>& positions, const unsigned int& primitives)
{
    MapBuilder mbuild(map_);

    int nb_articulation = branche.size();
    std::vector<Dart> volume_control;
    Volume v1, v2;
    int volume_count=0;
    unsigned int face_count = 0;
    int count = 0;


    //fusionner les deux boucles si possible
    for(int n = 1 ; n < nb_articulation ; n++)
    {
        volume_control.push_back(mbuild.add_prism_topo(primitives));//construit le prisme et renvoi un dart du prisme d'une des faces triangulaires, rendant un parcourt du prisme possible
    }


    for(int m = 1 ; m < nb_articulation-1 ; m++)
    {
        v1.dart = map_.phi2(map_.phi1(map_.phi1(map_.phi2(volume_control[m-1]))));
        v2.dart = volume_control[m];

        mbuild.sew_volumes(v1, v2);
    }

    mbuild.close_map(); //reboucle les volumes en bord de map


    map_.foreach_cell([&] (Volume v){ volume_count++; }); // affichage du nombre de volumes
    std::cout << " Il y a " << volume_count << " Volume(s)" << std::endl;


    //Les vertices vont être indexe automatiquement & creation d'un de leur attribut, position dans l espace 3D
    vertex_position_ = map_.add_attribute<Vec3, Vertex::ORBIT>("position");
    vertex_normal_ = map_.add_attribute<Vec3, Vertex::ORBIT>("normal");



    //On attribut des positions aux sommets des faces du squelette
    for(Dart d : volume_control)
    {
        count = 0;
        Face F(d);

        map_.foreach_incident_vertex(F, [&] (Vertex v)
        {
            vertex_position_[v] = positions[face_count * primitives + count];
            count++;
        });

        face_count++;
    }

    // Ici, on gere la derniere face
    Dart last=volume_control[volume_control.size()-1];
    Face F_end(map_.phi2(map_.phi1(map_.phi1(map_.phi2(map_.phi_1(last))))));
    count =0;
    map_.foreach_incident_vertex(F_end, [&] (Vertex v_end){

            if(count==0)
                vertex_position_[v_end] = { positions[face_count * primitives + count][0], positions[face_count * primitives + count][1], positions[face_count * primitives + count][2] };
            else
                vertex_position_[v_end] = { positions[face_count * primitives + primitives - count][0], positions[face_count * primitives + primitives - count][1], positions[face_count * primitives + primitives - count][2] };

            count++;
    });

    map_.check_map_integrity();

    //bounding boxe et scene parameters
    cgogn::geometry::compute_AABB(vertex_position_, bb_);
    setSceneRadius(bb_.diag_size()/2.0);
    Vec3 center = bb_.center();
    setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
    showEntireScene();
}

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

Viewer::Viewer() :
	map_(),
	vertex_position_(),
    vertex_normal_(),
    cell_cache_prec_(map_),
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
{}

void Viewer::keyPressEvent(QKeyEvent *ev)
{
	switch (ev->key()) {
        case Qt::Key_Minus:
        {
            std::cout<<"minus"<<std::endl;
            break;
        }
        case Qt::Key_Plus:
        {

        /*
            affinage_++;

            Branch branche;
            branche.SubdiBranch( COURBURE_MAX );
            branche.CreateCircleCoordinates( TYPE_PRIMITIVE + affinage_);

            Viewer viewer2;
            viewer2.MakeFromBranch(branche.articulations_, branche.pos_vertices_, TYPE_PRIMITIVE + affinage_ );
            this->map_.swap_attributes(this->vertex_position_, viewer2.vertex_position_);*/

            std::cout<<"plus"<<std::endl;
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
		default:
			break;
	}
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
        topo_drawer_rend_->draw(proj, view, this);

    if (edge_rendering_)
        volume_drawer_rend_->draw_edges(proj, view, this);

	if (bb_rendering_)
        drawer_rend_->draw(proj, view, this);
}

void Viewer::init()
{
	glClearColor(0.1f,0.1f,0.3f,0.0f);

	vbo_pos_ = new cgogn::rendering::VBO(3);
    cgogn::rendering::update_vbo(vertex_position_, vbo_pos_);

    render_ = new cgogn::rendering::MapRender();
    render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS);

    topo_drawer_ =  new cgogn::rendering::TopoDrawer();
    topo_drawer_rend_ = topo_drawer_->generate_renderer();
    topo_drawer_->set_explode_volume(volume_expl_);
    topo_drawer_->update<Vec3>(map_, vertex_position_);

    volume_drawer_ = new cgogn::rendering::VolumeDrawer();
    volume_drawer_->update_face<Vec3>(map_, vertex_position_);
    volume_drawer_->update_edge<Vec3>(map_, vertex_position_);
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
    cgogn::geometry::compute_AABB(vertex_position_, bb_);

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

int main(int argc, char** argv)
{
	QApplication application(argc, argv);
	qoglviewer::init_ogl_context();

    //Branch branche("../../TubularMesh/multibranch");
    Branch branche("../../TubularMesh/multibranch_cave");

    branche.BranchSimplify(DISTANCE_MIN);
    //branche.SubdiBranch( COURBURE_MAX );
    branche.CreateCircleCoordinates(TYPE_PRIMITIVE);

    /*
    Window fenetre;
    fenetre.setWindowTitle("variables");
    fenetre.show();*/

    // Instantiate the viewer.
    Viewer viewer;
	viewer.setWindowTitle("simpleViewer");

    viewer.MakeFromBranch(branche.articulations_, branche.pos_vertices_, TYPE_PRIMITIVE );
    viewer.move(105,68);
    viewer.show();

	// Run main loop.
	return application.exec();
}

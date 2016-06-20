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


    void ComputeNewCoordinates();

    Map3 map_;

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


//
// IMPLEMENTATION
//


void Viewer::MakeFromBranch(const std::vector<Vec3>& positions, const unsigned int& primitives)
{
    MapBuilder mbuild(map_);

    int nb_articulation = positions.size()/(primitives+1);
    int volume_count=0;
    unsigned int count = 0;
    unsigned int arti_count = 0;

    // creation de nos volumes
    for(int n = primitives ; n < nb_articulation*primitives ; n++)
        volume_control_.push_back(mbuild.add_prism_topo(3));//construit le prisme et renvoi un dart du prisme d'une des faces triangulaires, rendant un parcourt du prisme possible


    for(int m = 1 ; m < nb_articulation-1 ; m++)
    {
        for(int k = 0; k < primitives; k++)
        {
            // coudre les faces triangulaires des prismes(3d)
            Dart v1 = map_.phi2(map_.phi1(map_.phi1(map_.phi2(volume_control_[(m-1)*primitives + k]))));
            Dart v2 = volume_control_[m*primitives + k];
            mbuild.sew_volumes(Volume(v1), Volume(v2));

            // coudre les faces rectangulaires des prismes(3d)
            Dart v3 = map_.phi2(map_.phi1(volume_control_[(m-1)*primitives + k]));
            Dart v4;
            if(k+1 != primitives)
                v4 = map_.phi2(volume_control_[(m-1)*primitives + k + 1]);
            else
                v4 = map_.phi2(volume_control_[(m-1)*primitives]);
            mbuild.sew_volumes(Volume(v3), Volume(v4));
        }
    }


    // dernière serie de volumes (bout de branche)
    for(int k = 0; k < primitives; k++)
    {
        Dart v3 = map_.phi2(map_.phi1(volume_control_[(nb_articulation-2)*primitives + k]));
        Dart v4;
        if(k+1 != primitives)
            v4 = map_.phi2(volume_control_[(nb_articulation-2)*primitives + k + 1]);
        else
            v4 = map_.phi2(volume_control_[(nb_articulation-2)*primitives]);
        mbuild.sew_volumes(Volume(v3), Volume(v4));
    }

    mbuild.close_map(); //reboucle les volumes en bord de map


    map_.foreach_cell([&] (Volume v){ volume_count++; }); // affichage du nombre de volumes
    std::cout << " Il y a " << volume_count << " Volume(s)" << std::endl;


    //Les vertices vont être indexe automatiquement & creation d'un de leur attribut, position dans l espace 3D
    vertex_position_ = map_.add_attribute<Vec3, Vertex::ORBIT>("position");
    vertex_normal_ = map_.add_attribute<Vec3, Vertex::ORBIT>("normal");


    // affectation d'un point du prisme triangulaire & du point en commun aux n-prismes (n = primitive)
    for(Dart d : volume_control_)
    {
        // il s'agit des positions des articulations
        if(count == 0 || count == primitives + 1 )
        {
            Vertex v1(map_.phi1(d));
            vertex_position_[v1] = positions[arti_count * (primitives + 1)];
            arti_count++;
            count = 1;
        }

        // ici on a la position des points autour de chaque articulation
        Vertex v2(d);
        vertex_position_[v2] = positions[(arti_count-1) * (primitives + 1) + count]; // le facteur est (arti_count - 1) car on a arti_count++ dans le if
        count++;
    }




    // On gère la dernière articulation

    Dart last_arti = map_.phi1(map_.phi1(map_.phi2(map_.phi1(volume_control_[volume_control_.size() - 1]))));

    vertex_position_[Vertex(last_arti)] = positions[(primitives+1)*(nb_articulation-1)];

    // Ici, on gere la derniere face (les points autour de la dernière articulation)
    for(unsigned int i = 0; i < primitives; i++)
    {
        Dart last_points = map_.phi1(map_.phi2(map_.phi1(map_.phi1(map_.phi2(volume_control_[volume_control_.size()-primitives + i])))));
        Vertex points(last_points);
        vertex_position_[points] = positions[volume_control_.size() + nb_articulation + i];
    }


    map_.check_map_integrity();

    //bounding boxe et scene parameters
    cgogn::geometry::compute_AABB(vertex_position_, bb_);
    setSceneRadius(bb_.diag_size()/2.0);
    Vec3 center = bb_.center();
    setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
    showEntireScene();

}

void Viewer::ComputeNewCoordinates()
{
    if(nb_appuis_ > 0)
    {
        double Terme_repartition = exp(indice_repartition_*0.02) - 1;
        //double Terme_repartition = (1 + tanh(indice_repartition_*0.2))*10 - 1;

        std::cout << "Terme repartition" << Terme_repartition << std::endl;

        for(Dart d : volume_control_)
        {
            Dart d_centre = d;
            Dart d_bord = d;

            for(int k = 0; k < nb_appuis_; k++)
            {
                d_centre = map_.phi<12321>(d_centre);
            }
            d_centre = map_.phi1(d_centre);


            for(int i = 1; i < nb_appuis_ + 1; i++)
            {

                double coeff;
                double m = double(i);
                double n = double(nb_appuis_ + 1);
                //coeff = m/(n + 1/indice_repartition_);
                coeff = m/(n + Terme_repartition);

                d = map_.phi<12321>(d);
                vertex_position_[Vertex(d)] = coeff*vertex_position_[Vertex(d_centre)] + (1-coeff)*vertex_position_[Vertex(d_bord)];

            }
        }

        for(int i = 0; i < TYPE_PRIMITIVE; i++)
        {

            Dart d = map_.phi<211>(volume_control_[volume_control_.size()-TYPE_PRIMITIVE+i]);
            Dart d_bord = d;
            Dart d_centre = d; // le recalculer dans la boucle peut être inutile pour cette dernière face

            //calcul du centre
            for(int k = 0; k < nb_appuis_; k++)
            {
                d_centre = map_.phi<1213121>(d_centre);
            }
            d_centre = map_.phi1(d_centre);

            //calcul de chaque points
            for(int i = 1; i < nb_appuis_ + 1; i++)
            {
                double coeff;
                double m = double(i);
                double n = double(nb_appuis_ + 1);
                //coeff = m/(n + 1/indice_repartition_);
                coeff = m/(n + Terme_repartition);

                d = map_.phi<1213121>(d);
                vertex_position_[Vertex(d)] = coeff*vertex_position_[Vertex(d_centre)] + (1-coeff)*vertex_position_[Vertex(d_bord)];

            }



        }


        cgogn::rendering::update_vbo(vertex_position_, vbo_pos_);
        volume_drawer_->update_face<Vec3>(map_, vertex_position_);
        volume_drawer_->update_edge<Vec3>(map_, vertex_position_);
        topo_drawer_->update<Vec3>(map_, vertex_position_);
        render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS);
    }

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
{
    nb_appuis_ = 0;
    indice_repartition_ = 0;
}

void Viewer::keyPressEvent(QKeyEvent *ev)
{
	switch (ev->key()) {
        case Qt::Key_Minus:
        {
            indice_repartition_--;
            ComputeNewCoordinates();
            std::cout<<"minus"<<std::endl;
            break;
        }
        case Qt::Key_Plus:
        {
            indice_repartition_++;
            ComputeNewCoordinates();
            std::cout<<"plus"<<std::endl;
            /*
            // vérif condition nb_appui <> 0
            if(nb_appuis_ > 0)
            {
                indice_repartition_++;
                double Terme_repartition = exp(indice_repartition_*0.02) - 1;
                //double Terme_repartition = (1 + tanh(indice_repartition_*0.2))*10 - 1;

                std::cout << "Terme repartition" << Terme_repartition << std::endl;

                for(Dart d : volume_control_)
                {
                    Dart d_centre = d;
                    Dart d_bord = d;

                    for(int k = 0; k < nb_appuis_; k++)
                    {
                        d_centre = map_.phi<12321>(d_centre);
                    }
                    d_centre = map_.phi1(d_centre);


                    for(int i = 1; i < nb_appuis_ + 1; i++)
                    {

                        double coeff;
                        double m = double(i);
                        double n = double(nb_appuis_ + 1);
                        //coeff = m/(n + 1/indice_repartition_);
                        coeff = m/(n + Terme_repartition);

                        d = map_.phi<12321>(d);
                        vertex_position_[Vertex(d)] = coeff*vertex_position_[Vertex(d_centre)] + (1-coeff)*vertex_position_[Vertex(d_bord)];

                    }
                }
                std::cout<<"plus"<<std::endl;
                cgogn::rendering::update_vbo(vertex_position_, vbo_pos_);
                volume_drawer_->update_face<Vec3>(map_, vertex_position_);
                volume_drawer_->update_edge<Vec3>(map_, vertex_position_);
                topo_drawer_->update<Vec3>(map_, vertex_position_);
                render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS);
            }*/
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
            unsigned int k = 0;
            unsigned int j = 0;
            std::vector<Dart> nouv_vertex;
            std::vector<Edge> face_limits;


            nb_appuis_++;


            for(Dart d : volume_control_)
            {
                Dart v = map_.cut_edge(Edge(d)).dart;
                nouv_vertex.push_back(v);
                vertex_position_[Vertex(v)] = (vertex_position_[Vertex(map_.phi1(v))]*(1 - MASK_SUBDIV_RAY) + vertex_position_[Vertex(map_.phi_1(v))]*MASK_SUBDIV_RAY);
            }

            // dernière arti
            for(unsigned int i = 0; i < TYPE_PRIMITIVE; i++)
            {
                Dart last_seg = map_.phi2(map_.phi1(map_.phi1(map_.phi2(volume_control_[volume_control_.size() - TYPE_PRIMITIVE + i]))));
                Vertex v = map_.cut_edge(Edge(last_seg));
                Dart dv = v.dart;
                nouv_vertex.push_back(dv);

                vertex_position_[v] = (vertex_position_[Vertex(map_.phi_1(dv))]*(1 - MASK_SUBDIV_RAY) + vertex_position_[Vertex(map_.phi1(dv))]*MASK_SUBDIV_RAY);

            }


            for(Dart d : volume_control_)
            {
                Dart v_int_bottom = map_.phi_1(map_.phi_1(map_.phi2(nouv_vertex[k + j*TYPE_PRIMITIVE])));
                Dart v_int_top = map_.phi1(map_.phi2(nouv_vertex[k + j*TYPE_PRIMITIVE]));
                face_limits.push_back(map_.cut_face(v_int_top, v_int_bottom));

                if(k + 1 == TYPE_PRIMITIVE )
                {
                    Dart v_int_side = map_.phi<2321>(nouv_vertex[j*TYPE_PRIMITIVE]);
                    face_limits.push_back(map_.cut_face(nouv_vertex[k + j*TYPE_PRIMITIVE], v_int_side)); // On coupe la face triangulaire du dernier volume autour de l'articulation
                    k=0;
                    j++;
                }
                else
                {
                    Dart v_int_side = map_.phi<2321>(nouv_vertex[k + 1 + j*TYPE_PRIMITIVE]);
                    face_limits.push_back(map_.cut_face(nouv_vertex[k + j*TYPE_PRIMITIVE], v_int_side)); // On coupe une face triangulaire
                    k++;
                }
            }

            // dernière arti
            for(unsigned int i = 0; i < TYPE_PRIMITIVE; i++)
            {
                if(i + 1 == TYPE_PRIMITIVE )
                {
                    Dart v_int_side = map_.phi<2321>(nouv_vertex[j*TYPE_PRIMITIVE]);
                    face_limits.push_back(map_.cut_face(nouv_vertex[j*TYPE_PRIMITIVE + i], v_int_side)); // on coupe une face triangulaire du dernier volume
                }
                else
                {
                    Dart v_int_side = map_.phi<2321>(nouv_vertex[j*TYPE_PRIMITIVE + i + 1]);
                    face_limits.push_back(map_.cut_face(nouv_vertex[j*TYPE_PRIMITIVE + i], v_int_side)); // on coupe une face triangulaire des volumes autour de la dernière arti
                }
            }




            for(Dart d : volume_control_)
            {
                //
                // version avec les indices (incomplète)
                //
                /*
                std::vector<Edge> face_int;

                face_int.push_back(face_limits[count + 1]);
                face_int.push_back(face_limits[count]);
                Edge eb(map_.phi3(face_limits[2*(TYPE_PRIMITIVE) + 1 + count].dart));
                face_int.push_back(eb);
                if((count % (2*TYPE_PRIMITIVE)) != 0)
                {
                    Edge es(map_.phi3(face_limits[count - 2].dart));
                    face_int.push_back(es);
                }
                else
                {
                    Edge es(map_.phi3(face_limits[count + 2*(TYPE_PRIMITIVE - 1)].dart));
                    face_int.push_back(es);
                }
                */

                //
                // version avec les manipulation phi<>
                //

                std::vector<Dart> face_int;

                Dart dd = map_.phi1(d);
                face_int.push_back(dd);
                dd = map_.phi<121>(dd);
                face_int.push_back(dd);
                dd = map_.phi<121>(dd);
                face_int.push_back(dd);
                dd = map_.phi<121>(dd);
                face_int.push_back(dd);

                Face f = map_.cut_volume(face_int);

            }

            ComputeNewCoordinates();

            map_.check_map_integrity();

            unsigned int volume_count = 0;
            map_.foreach_cell([&] (Volume v){ volume_count++; }); // affichage du nombre de volumes
            std::cout << " Il y a " << volume_count << " Volume(s)" << std::endl;

            break;
        }

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

    Branch branche("../../TubularMesh/multibranch");
    //Branch branche("../../TubularMesh/multibranch_cave");
    //Branch branche;

    branche.BranchSimplify(DISTANCE_MIN);
    //branche.SubdiBranch( COURBURE_MAX );
    branche.CreateCircleCoordinates(TYPE_PRIMITIVE);
    branche.SubdiDirectionT(COURBURE_MAX, TYPE_PRIMITIVE);

    /*
    Window fenetre;
    fenetre.setWindowTitle("variables");
    fenetre.show();*/

    // Instantiate the viewer.
    Viewer viewer;
	viewer.setWindowTitle("simpleViewer");

    viewer.MakeFromBranch(branche.pos_vertices_, TYPE_PRIMITIVE);

    viewer.move(105,68);
    viewer.show();

	// Run main loop.
	return application.exec();
}

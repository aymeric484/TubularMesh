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

using Map3 = cgogn::CMap3<cgogn::DefaultMapTraits>;
using Vertex = typename Map3::Vertex;
using Edge = typename Map3::Edge;
using Face = typename Map3::Face;
using Volume = typename Map3::Volume;

using Dart = cgogn::Dart;

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

//static const unsigned int CHUNK_SIZE = cgogn::DefaultMapTraits::CHUNK_SIZE;

//using ChunkArray = cgogn::ChunkArray<CHUNK_SIZE, T>;
//using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNK_SIZE, unsigned int >;


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
    void import(const std::string& volume_mesh);
    void MakeFromSkel(const std::vector<Vec4>& Squelette);
	virtual ~Viewer();
	virtual void closeEvent(QCloseEvent *e);

private:

    Map3 map_;

    //ChunkArrayContainer vertex_attributes_;
    //ChunkArrayContainer volume_attributes_;

    //std::vector<unsigned int> volumes_vertex_indices_;

	VertexAttribute<Vec3> vertex_position_;
	VertexAttribute<Vec3> vertex_position2_;
    VertexAttribute<Vec3> vertex_normal_;

    cgogn::CellCache<Map3> cell_cache_prec_;

	cgogn::geometry::BoundingBox<Vec3> bb_;

	cgogn::rendering::MapRender* render_;

	cgogn::rendering::VBO* vbo_pos_;
	cgogn::rendering::VBO* vbo_norm_;
	cgogn::rendering::VBO* vbo_color_;
	cgogn::rendering::VBO* vbo_sphere_sz_;

	cgogn::rendering::ShaderBoldLine* shader_edge_;
	cgogn::rendering::ShaderFlat* shader_flat_;
	cgogn::rendering::ShaderVectorPerVertex* shader_normal_;
	cgogn::rendering::ShaderPhong* shader_phong_;
	cgogn::rendering::ShaderPointSprite* shader_point_sprite_;

    cgogn::rendering::ShaderBoldLine::Param* param_edge_;
    cgogn::rendering::ShaderFlat::Param* param_flat_;
    cgogn::rendering::ShaderVectorPerVertex::Param* param_normal_;
    cgogn::rendering::ShaderPhong::Param* param_phong_;
    cgogn::rendering::ShaderPointSprite::Param* param_point_sprite_;

	cgogn::rendering::Drawer* drawer_;

	bool phong_rendering_;
	bool flat_rendering_;
	bool vertices_rendering_;
	bool edge_rendering_;
	bool normal_rendering_;
	bool bb_rendering_;
};



//
// IMPLEMENTATION
//


void Viewer::import(const std::string& volume_mesh)
{
    cgogn::io::import_volume<Vec3>(map_, volume_mesh);

	vertex_position_ = map_.get_attribute<Vec3, Vertex::ORBIT>("position");
	if (!vertex_position_.is_valid())
	{
		cgogn_log_error("Viewer::import") << "Missing attribute position. Aborting.";
		std::exit(EXIT_FAILURE);
	}

	vertex_position2_ = map_.add_attribute<Vec3, Vertex::ORBIT>("position2");
	map_.copy_attribute(vertex_position2_, vertex_position_);

    vertex_normal_ = map_.add_attribute<Vec3, Vertex::ORBIT>("normal");
//	cgogn::geometry::compute_normal_vertices<Vec3>(map_, vertex_position_, vertex_normal_);

    cell_cache_prec_.build<Vertex>();
    cell_cache_prec_.build<Edge>();

	cgogn::geometry::compute_bounding_box(vertex_position_, bb_);


    map_.foreach_cell([&] (Vertex v){ std::cout<< vertex_position_[v] <<std::endl;    });


	setSceneRadius(bb_.diag_size()/2.0);
	Vec3 center = bb_.center();
	setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
	showEntireScene();
}

void Viewer::MakeFromSkel(const std::vector<Vec4>& Squelette)
{

    //
    //  Brouillon
    //

    /*

    vertex_position_[v1] = { Squelette[0](0) + Squelette[0](3) , Squelette[0](1), Squelette[0](2) };
    vertex_position_[v1] = { Squelette[0](0) + Squelette[0](3)*(-0.5) , Squelette[0](1) + Squelette[0](3)*(0.87), Squelette[0](2) };
    vertex_position_[v1] = { Squelette[0](0) + Squelette[0](3)*(-0.5) , Squelette[0](1) + Squelette[0](3)*(-0.87), Squelette[0](2) };
    */

    /*
    //Vertex S = map_.add_vertex(); //sera à tester
    //Vertex v(d);
    //v=map_.add_vertex();


    //parcourt du prisme pour récupérer les sommets du prisme
    const std::array<Dart, 6> vertices_of_prism = {
      d,
      map_.phi1(d),
      map_.phi_1(d),
      map_.phi2(map_.phi1(map_.phi1(map_.phi2(map_.phi_1(d))))),
      map_.phi2(map_.phi1(map_.phi1(map_.phi2(d)))),
      map_.phi2(map_.phi1(map_.phi1(map_.phi2(map_.phi1(d))))),
    };


    */
    /*
        mbuild.template create_embedding<Vertex::ORBIT>();
        //Objectif supposé : associer a chaque Dart un Vertex
        typename Map3::template VertexAttribute<std::vector<Dart>> darts_per_vertex = map_.template add_attribute<std::vector<Dart> , Vertex::ORBIT>("darts_per_vertex");
        typename Map3::DartMarkerStore m(map_);

        mbuild.template swap_chunk_array_container<Vertex::ORBIT>(this->vertex_attributes_);

        unsigned int index = 0u;x

        for (Dart dv : vertices_of_prism)
        {
            const unsigned int emb = this->volumes_vertex_indices_[index];
            index++;
            //mbuild.init_parent_vertex_embedding(dv,emb);


            //ici, on atteint tout ce qui est atteignable à partir de dv, grâce a des changement d arete et de face
            Dart dd = dv;
            do
            {
                m.mark(dd);
                darts_per_vertex[emb].push_back(dd);
                dd = map_.phi1(map_.phi2(dd));
            } while(dd != dv);
        }
    */

    /***************************************/
    //Méthode de test 1
    /*
    int count=0;
    map_.foreach_dart([&] (Dart da){
        count++;
    });
    std::cout<< count<< std::endl;
    count = 0;
    map_.foreach_cell([&](Edge e){
        count++;
        //OK;
    });
    std::cout<< count<< std::endl;
    count = 0;
    map_.foreach_cell([&](Face f){
        count++;
        //OK
    });
    std::cout<< count<< std::endl;
    count = 0;
    map_.foreach_cell([&](Volume w){
        count++;
        //OK
    });
    std::cout<< count<< std::endl;
    count = 0;
    map_.foreach_cell([&](Vertex v){
        count++;
        //FAILED => 1
    });
    std::cout<< count<< std::endl;
    count = 0;
    map_.foreach_cell([&](Vertex v1){
        count++;
        map_.foreach_adjacent_vertex_through_edge(v1, [&] (Vertex v){
            //count++;
            //FAILED => 9
        });
    });
    std::cout<< count<<std::endl;
    */


    /***************************************/
    //Méthode de test 2
    /*
    cgogn::CellCache<Map3> vertices_cache(map_);
    cgogn::CellCache<Map3> edges_cache(map_);
    cgogn::CellCache<Map3> faces_cache(map_);
    cgogn::CellCache<Map3> volumes_cache(map_);

    vertices_cache.build<Vertex>();
    edges_cache.build<Edge>();
    faces_cache.build<Face>();
    volumes_cache.build<Volume>();

    unsigned int nbw = 0u;
    map_.foreach_cell([&nbw] (Volume)
    {
        ++nbw;
    }, volumes_cache);

    unsigned int nbf = 0u;
    map_.foreach_cell([&] (Face f)
    {
        ++nbf;
    }, faces_cache);

    unsigned int nbv = 0;
    map_.foreach_cell([&] (Vertex v)
    {
        ++nbv;
        unsigned int nb_incident = 0;
        map_.foreach_incident_face(v, [&] (Face)
        {
            ++nb_incident;
        });
    }, vertices_cache);

    unsigned int nbe = 0;
    map_.foreach_cell([&nbe] (Edge)
    {
        ++nbe;
    }, edges_cache);

    cgogn_log_info("map3_from_image") << "nb vertices -> " << nbv;
    cgogn_log_info("map3_from_image") << "nb edges -> " << nbe;
    cgogn_log_info("map3_from_image") << "nb faces -> " << nbf;
    cgogn_log_info("map3_from_image") << "nb volumes -> " << nbw;*/


    //iterator & const ref
    /*
    int C1;
     std::vector<Vec4> cpy_Squelette = Squelette;
    for(std::vector<Vec4>::iterator it = cpy_Squelette.begin(); it != cpy_Squelette.end(); ++it) {
        C1++;
        Dart d = mbuild.add_prism_topo(3u);
    }
    std::cout<< C1<< std::endl;
    */



    MapBuilder mbuild(map_);
    int nb_articulation = Squelette.size();


    std::vector<Dart> volume_control;
    Dart it1;
    Dart it2;
    Dart begin;
    Volume v1, v2;
    int volume_count=0;
    //Volume v_prec;
    //bool volume_check=false;
    //Volume v2;


    //fusionner les deux boucles
    for(int n = 1 ; n < nb_articulation ; n++)
    {
        volume_control.push_back(mbuild.add_prism_topo(3u));//construit le prisme et renvoi un dart du prisme d'une des faces triangulaires, rendant un parcourt du prisme possible
    }


/*
    for(int m = 1 ; m < nb_articulation ; m++)
    {
        it1 = volume_control[m-1];
        it2 = volume_control[m];

        begin=it1;

        do
        {
            map_.phi3_sew(it1, it2);
            it1 = map_.phi1(it1);
            it2 = map_.phi_1(it2);
        } while (it1 != begin);


    }*/


    for(int m = 1 ; m < nb_articulation-1 ; m++)
    {

        //v1.dart = map_.phi2(map_.phi1(map_.phi1(map_.phi2(map_.phi_1(volume_control[m-1])))));
        //v1.dart = map_.phi2(map_.phi1(map_.phi1(map_.phi2(map_.phi_1(volume_control[m-1])))));
        //v2.dart = volume_control[m];
        //v1.dart = map_.phi2(map_.phi1(volume_control[m-1]));
        //v2.dart = map_.phi2(map_.phi_1(volume_control[m]));

        //v1.dart = map_.phi_1(map_.phi2(map_.phi1(map_.phi1(volume_control[m-1]))));
        //v1.dart = map_.phi2(map_.phi1(map_.phi1(volume_control[m-1])));
        //v2.dart = map_.phi2(volume_control[m]);


        //v1.dart = map_.phi1(map_.phi2(map_.phi_1(volume_control[m-1])));
        //v2.dart = map_.phi2(map_.phi1(volume_control[m]));

        //v1.dart = volume_control[m-1];
        v1.dart = volume_control[m-1];
        v2.dart = volume_control[m];


        mbuild.sew_volumes(v1, v2);

    }

    mbuild.close_map(); //reboucle les volumes en bord de map

    map_.foreach_cell([&] (Volume v){

        volume_count++;

    });

    std::cout << volume_count << std::endl; //deux volumes => ok





    //Les vertices vont être indexe automatiquement & creation d'un de leur attribut, position dans l espace 3D
    vertex_position_ = map_.add_attribute<Vec3, Vertex::ORBIT>("position");
    vertex_position2_ = map_.add_attribute<Vec3, Vertex::ORBIT>("position2");
    vertex_normal_ = map_.add_attribute<Vec3, Vertex::ORBIT>("normal");



    int count=0;
    double TermeX, TermeY, TermeZ ;
    unsigned int face_count=0;
    //unsigned int face_count_abs=0;
    //bool next_volume= false;

    //map_.foreach_adjacent_volume_through_face;
    //map_.foreach_adjacent_face_through_volume;
    //map_.foreach_incident_face();


    map_.foreach_cell([&](Face f){

        count=0;

        //à refaire en utilisant Cell_Cache + iterator et/ou en rajoutant un filtre au foreach
        map_.foreach_incident_vertex(f,[&](Vertex v){
            count++;
        });
        std::cout << count << std::endl;
        if(count == 3)
        {
            count = 0;
            map_.foreach_incident_vertex(f,[&](Vertex v1){

                TermeX = Squelette[0 + face_count](0) + Squelette[0 + face_count](3) + Squelette[0 + face_count](3)*(-1.5)*count*(2-count) + Squelette[0 + face_count](3)*(-0.75)*count*(count-1);
                TermeY = Squelette[0 + face_count](1) + Squelette[0 + face_count](3)*0.87*count*(2-count) - Squelette[0 + face_count](3)*0.435*count*(count-1);
                TermeZ = Squelette[0 + face_count](2);

                //vertex_position_[count + 3 * face_count_abs] = { TermeX , TermeY, TermeZ };
                vertex_position_[count + 3 * face_count] = { TermeX , TermeY, TermeZ };

                count++;
            });

            /*
            if(!(face_count % 2) || next_volume)
            {
                face_count++;
                next_volume = false;
            }
            else
                next_volume = !next_volume;*/

            //face_count_abs++;
            face_count++;
        }
        else{ count = 0; }



    });

    for (int i= 0; i<(face_count)*3; i++)
        std::cout << vertex_position_[i] << std::endl;

    std::cout << face_count << std::endl;
    //bounding boxe et scene parameters

    cgogn::geometry::compute_bounding_box(vertex_position_, bb_);
    setSceneRadius(bb_.diag_size()/2.0);
    Vec3 center = bb_.center();
    setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
    showEntireScene();


}

Viewer::~Viewer()
{}

void Viewer::closeEvent(QCloseEvent*)
{
	delete render_;
	delete vbo_pos_;
	delete vbo_norm_;
	delete vbo_color_;
	delete vbo_sphere_sz_;
	delete shader_edge_;
	delete shader_flat_;
	delete shader_normal_;
	delete shader_phong_;
	delete shader_point_sprite_;
	delete drawer_;
}

Viewer::Viewer() :
	map_(),
	vertex_position_(),
    vertex_normal_(),
    cell_cache_prec_(map_),
	bb_(),
	render_(nullptr),
	vbo_pos_(nullptr),
	vbo_norm_(nullptr),
	vbo_color_(nullptr),
	vbo_sphere_sz_(nullptr),
	shader_edge_(nullptr),
	shader_flat_(nullptr),
	shader_normal_(nullptr),
	shader_phong_(nullptr),
	shader_point_sprite_(nullptr),
	drawer_(nullptr),
	phong_rendering_(true),
	flat_rendering_(false),
	vertices_rendering_(false),
	edge_rendering_(false),
	normal_rendering_(false),
	bb_rendering_(true)
{}

void Viewer::keyPressEvent(QKeyEvent *ev)
{
	switch (ev->key()) {
		case Qt::Key_P:
			phong_rendering_ = true;
			flat_rendering_ = false;
			break;
		case Qt::Key_F:
			flat_rendering_ = true;
			phong_rendering_ = false;
			break;
		case Qt::Key_N:
			normal_rendering_ = !normal_rendering_;
			break;
		case Qt::Key_E:
			edge_rendering_ = !edge_rendering_;
			break;
		case Qt::Key_V:
			vertices_rendering_ = !vertices_rendering_;
			break;
//        case Qt::Key_B:
//            bb_rendering_ = !bb_rendering_;
//            break;

        case Qt::Key_A: {
            cgogn::geometry::filter_average<Vec3>(map_, cell_cache_prec_, vertex_position_, vertex_position2_);
            map_.swap_attributes(vertex_position_, vertex_position2_);
            cgogn::rendering::update_vbo(vertex_position_, vbo_pos_);
			update_bb();
			setSceneRadius(bb_.diag_size()/2.0);
			break;
        }
        case Qt::Key_B: {

            Vec3 sum;
            uint nbv;

            map_.foreach_cell([&] (Vertex v1){
                sum = Vec3(0.0);
                nbv= 0;

                map_.foreach_adjacent_vertex_through_face(v1, [&] (Vertex v2)
                {
                    sum+=vertex_position_[v2];

                    nbv++;
                });

                vertex_position2_[v1]=sum/nbv;
                //std::cout<< vertex_position2_[v1] << std::endl;
            });


            map_.swap_attributes(vertex_position_, vertex_position2_);
            cgogn::rendering::update_vbo(vertex_position_, vbo_pos_);
            update_bb();
            setSceneRadius(bb_.diag_size()/2);


            break;
        }
        case Qt::Key_R:{

            //map_.add_vertex();
            //map_.add_attribute()
            //map_.Vertex
            //Dart d;
            //auto d = map_.add_dart();

            break;
        }

        case Qt::Key_M: {
            int count=0;
            map_.foreach_cell([&](Vertex v1){
                count++;
            });
            std::cout<< count << std::endl;

            break;
        }

          /*
		case Qt::Key_B:
			cgogn::geometry::filter_bilateral<Vec3>(map_, cell_cache_, vertex_position_, vertex_position2_, vertex_normal_);
			map_.swap_attributes(vertex_position_, vertex_position2_);
			cgogn::geometry::compute_normal_vertices<Vec3>(map_, vertex_position_, vertex_normal_);
			cgogn::rendering::update_vbo(vertex_position_, *vbo_pos_);
			cgogn::rendering::update_vbo(vertex_normal_, *vbo_norm_);
			cgogn::rendering::update_vbo(vertex_normal_, *vbo_color_, [] (const Vec3& n) -> std::array<float,3>
			{
				return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
			});
			update_bb();
			setSceneRadius(bb_.diag_size()/2.0);
            break;*/
            /*
		case Qt::Key_T:
			cgogn::geometry::filter_taubin<Vec3>(map_, cell_cache_, vertex_position_, vertex_position2_);
			cgogn::geometry::compute_normal_vertices<Vec3>(map_, vertex_position_, vertex_normal_);
			cgogn::rendering::update_vbo(vertex_position_, *vbo_pos_);
			cgogn::rendering::update_vbo(vertex_normal_, *vbo_norm_);
			cgogn::rendering::update_vbo(vertex_normal_, *vbo_color_, [] (const Vec3& n) -> std::array<float,3>
			{
				return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
			});
			update_bb();
			setSceneRadius(bb_.diag_size()/2.0);
			break;
            */
        case Qt::Key_C: {
            unsigned int nbv = 0;
            map_.foreach_cell([&] (Vertex v)
            {
                nbv++;
//                std::cout << vertex_position_[v] << std::endl;
                unsigned int nbincident = 0;
                map_.foreach_incident_edge(v, [&] (Edge e)
                {
                    nbincident++;
                });
                std::cout << "vertex " << v << " => " << nbincident << " & " << vertex_position_[v] << std::endl;
            });
            std::cout << "nbv => " << nbv << std::endl;

            unsigned int nbe = 0;
            map_.foreach_cell([&] (Edge e) { nbe++; });
            std::cout << "nbe => " << nbe << std::endl;

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

void Viewer::draw()
{
	QMatrix4x4 proj;
	QMatrix4x4 view;
	camera()->getProjectionMatrix(proj);
	camera()->getModelViewMatrix(view);

	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f, 2.0f);
	if (flat_rendering_)
    {
        param_flat_->bind(proj,view);
        render_->draw(cgogn::rendering::TRIANGLES);
        param_flat_->release();
	}

	if (phong_rendering_)
    {
        param_phong_->bind(proj,view);
        render_->draw(cgogn::rendering::TRIANGLES);
        param_phong_->release();
	}
	glDisable(GL_POLYGON_OFFSET_FILL);

	if (vertices_rendering_)
    {
        param_point_sprite_->bind(proj,view);
        render_->draw(cgogn::rendering::POINTS);
        param_point_sprite_->release();
	}

	if (edge_rendering_)
    {
        param_edge_->bind(proj,view);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        render_->draw(cgogn::rendering::LINES);
        glDisable(GL_BLEND);
        param_edge_->release();
	}

	if (normal_rendering_)
    {
        param_normal_->bind(proj,view);
        render_->draw(cgogn::rendering::POINTS);
        param_normal_->release();
	}

	if (bb_rendering_)
        drawer_->call_list(proj,view,this);
}

void Viewer::init()
{
	glClearColor(0.1f,0.1f,0.3f,0.0f);

	vbo_pos_ = new cgogn::rendering::VBO(3);
    cgogn::rendering::update_vbo(vertex_position_, vbo_pos_);

    //std::cout<< vbo_pos_[0] << std::endl;

	vbo_norm_ = new cgogn::rendering::VBO(3);
    cgogn::rendering::update_vbo(vertex_normal_, vbo_norm_);

	// fill a color vbo with abs of normals
	vbo_color_ = new cgogn::rendering::VBO(3);
    cgogn::rendering::update_vbo(vertex_normal_, vbo_color_, [] (const Vec3& n) -> std::array<float,3>
	{
		return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
	});

	// fill a sphere size vbo
	vbo_sphere_sz_ = new cgogn::rendering::VBO(1);
    cgogn::rendering::update_vbo(vertex_normal_, vbo_sphere_sz_, [&] (const Vec3& n) -> float
	{
		return bb_.diag_size()/1000.0 * (1.0 + 2.0*std::abs(n[2]));
	});

	render_ = new cgogn::rendering::MapRender();

    render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS);
    render_->init_primitives<Vec3>(map_, cgogn::rendering::LINES);
    render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, &vertex_position_);

	shader_point_sprite_ = new cgogn::rendering::ShaderPointSprite(true,true);
    param_point_sprite_ = shader_point_sprite_->generate_param();
    param_point_sprite_->set_vbo(vbo_pos_,vbo_color_, vbo_sphere_sz_);
    param_point_sprite_->size_ = bb_.diag_size()/1000.0;
    param_point_sprite_->color_ = QColor(255,0,0);

	shader_edge_ = new cgogn::rendering::ShaderBoldLine() ;
    param_edge_ = shader_edge_->generate_param();
    param_edge_->set_vbo(vbo_pos_);
    param_edge_->color_ = QColor(255,255,0);
    param_edge_->width_= 2.5f;

    shader_flat_ = new cgogn::rendering::ShaderFlat;
    param_flat_ = shader_flat_->generate_param();
    param_flat_->set_vbo(vbo_pos_);
    param_flat_->front_color_ = QColor(0,200,0);
    param_flat_->back_color_ = QColor(0,0,200);
    param_flat_->ambiant_color_ = QColor(5,5,5);

    shader_normal_ = new cgogn::rendering::ShaderVectorPerVertex;
    param_normal_ = shader_normal_->generate_param();
    param_normal_->set_vbo(vbo_pos_, vbo_norm_);
    param_normal_->color_ = QColor(200,0,200);
    param_normal_->length_ = bb_.diag_size()/50;

    shader_phong_ = new cgogn::rendering::ShaderPhong(true);
    param_phong_ = shader_phong_->generate_param();
    param_phong_->set_vbo(vbo_pos_, vbo_norm_, vbo_color_);

	// drawer for simple old-school g1 rendering
    drawer_ = new cgogn::rendering::Drawer();
	update_bb();
}

void Viewer::update_bb()
{
	cgogn::geometry::compute_bounding_box(vertex_position_, bb_);

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



    std::string volume_mesh;
	if (argc < 2)
	{
		cgogn_log_info("tubularmesh") << "USAGE: " << argv[0] << " [filename]";
		exit(0);
	}
	else
        volume_mesh = std::string(argv[1]);

	QApplication application(argc, argv);
	qoglviewer::init_ogl_context();

	// Instantiate the viewer.
	Viewer viewer;
	viewer.setWindowTitle("simpleViewer");
    std::vector<Vec4> Squelette;
    Vec4 V40(0.0, 0.0, 0.0, 1.0);
    Vec4 V41(0.0, 0.0, 2.0, 1.0);
    Vec4 V42(0.0, 0.0, 3.0, 1.0);
    //Vec4 V43(0.0, 0.0, 4.0, 1.0);
    Squelette.push_back(V40);
    Squelette.push_back(V41);
    Squelette.push_back(V42);
    //Squelette.push_back(V43);

    //viewer.import(volume_mesh);//methode a remplacer, on ne cherchera plus a creer à partir d'un fichier
    viewer.MakeFromSkel(Squelette);

    viewer.show();

	// Run main loop.
	return application.exec();
}

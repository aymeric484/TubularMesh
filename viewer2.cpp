#include "viewer2.h"


//
// IMPLEMENTATION
//



void Viewer2::keyPressEvent(QKeyEvent *ev)
{
    switch (ev->key())
    {
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

void Viewer2::MakeIntersection(std::vector<TriangleGeo> triangles, std::vector<Vec3> sommets_intersection)
{
    // TEST : Affichage d'un tétraèdre
    /*
    std::vector<Dart> tetra_complete; // Nous permettra un accès aux tétraèdres pour l'affectation des coordonnées
    std::vector<Dart> tetra_open;

    MapBuilder2 mbuild2(map_);

    Dart d1 = mbuild2.add_face_topo_parent(3);
    Dart d2 = mbuild2.add_face_topo_parent(3);
    Dart d3 = mbuild2.add_face_topo_parent(3);
    Dart d4 = mbuild2.add_face_topo_parent(3);

    mbuild2.phi2_sew(map_.phi_1(d1), map_.phi1(d2));
    mbuild2.phi2_sew(map_.phi_1(d2), map_.phi1(d3));
    mbuild2.phi2_sew(map_.phi_1(d3), map_.phi1(d1));
    mbuild2.phi2_sew(d1, d4);
    mbuild2.close_map();

    map_.check_map_integrity();

    vertex_position_ = map_.add_attribute<Vec3, Vertex2::ORBIT>("position");
    vertex_normal_ = map_.add_attribute<Vec3, Map2::Vertex::ORBIT>("normal");

    Vec3 a(0.0, 0.0, 0.0);
    Vec3 b(0.0, 2.0, 0.0);
    Vec3 c(2.0, 0.0, 0.0);
    Vec3 d(1.0, 1.0, 1.0);

    vertex_position_[Vertex2(d1)] = a;
    vertex_position_[Vertex2(d2)] = b;
    vertex_position_[Vertex2(d3)] = c;
    vertex_position_[Vertex2(map_.phi_1(d1))] = d;*/

    //
    //
    // Polyèdre map2
    //
    //


    //
    // Init
    //

    MapBuilder2 mbuild2(map_);
    std::vector<Dart> dart_faces;
    std::vector<TriangleGeo> faces_existante;


    //
    // Création et stockage des faces
    //

    for(int i = 0; i < triangles.size(); i++)
    {
        Dart d = mbuild2.add_face_topo_parent(3);
        dart_faces.push_back(d);
    }

    //
    // Se placer sur une face, pour toute les faces
    //
    int nb_courant = 0;
    for(TriangleGeo T_test : triangles)
    {
        Dart d_test = dart_faces[nb_courant];
        int a = T_test.connectivity_[0];
        int b = T_test.connectivity_[1];
        int c = T_test.connectivity_[2];
        std::cout << "triangle de reférence : " << a << "/ " << b << "/ " << c << std::endl;
        int count = 0;
        int cond_valide = 0;
        int Nb_voisin = 0;
        for(TriangleGeo T_courant : triangles)
        {
            //
            // Déterminer si les triangles sont voisins
            bool voisin = false;
            bool sommet1 = (T_courant.connectivity_[0] == a) || (T_courant.connectivity_[1] == a) || (T_courant.connectivity_[2] == a);
            bool sommet2 = (T_courant.connectivity_[0] == b) || (T_courant.connectivity_[1] == b) || (T_courant.connectivity_[2] == b);
            bool sommet3 = (T_courant.connectivity_[0] == c) || (T_courant.connectivity_[1] == c) || (T_courant.connectivity_[2] == c);

            std::cout << "triangle à comparer : " << T_courant.connectivity_[0] << "/ " << T_courant.connectivity_[1] << "/ " << T_courant.connectivity_[2] << std::endl;

            if((sommet1 && sommet2 && !sommet3) || (sommet3 && sommet1 && !sommet2) || (sommet3 && sommet2 && !sommet1))
            {
                voisin = true;
                Nb_voisin++;
            }

            //
            // Si ils sont voisins, alors créer la face et la coller sur la bonne arête
            if(voisin)
            {
                // On accède à la face
                Dart d_courant = dart_faces[count];

                //
                // Puis test pour le sewing : Toute les possibilités orientées y sont représentés
                //

                //
                // test à partir du premier sommet

                // Pour ce sommet équivalent à "a"
                if(T_courant.connectivity_[0] == a)
                {
                    if(T_courant.connectivity_[1] == c)
                    {
                        d_courant = d_courant;
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(map_.phi_1(d_test)) == map_.phi_1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map_.phi_1(d_test));
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[2] == b)
                    {
                        d_courant = map_.phi_1(d_courant);
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(d_test) == d_test)
                        {
                            mbuild2.phi2_sew(d_courant, d_test);
                            cond_valide++;
                        }
                    }
                }

                // Pour ce sommet équivalent à "b"
                if(T_courant.connectivity_[0] == b)
                {
                    if(T_courant.connectivity_[1] == a)
                    {
                        d_courant = d_courant;
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(d_test) == d_test)
                        {
                            mbuild2.phi2_sew(d_courant, d_test);
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[2] == c)
                    {
                        d_courant = map_.phi_1(d_courant);
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(map_.phi1(d_test)) == map_.phi1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map_.phi1(d_test));
                            cond_valide++;
                        }
                    }
                }

                // Pour ce sommet équivalent à "c"
                if(T_courant.connectivity_[0] == c)
                {
                    if(T_courant.connectivity_[1] == b)
                    {
                        d_courant = d_courant;
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(map_.phi1(d_test)) == map_.phi1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map_.phi1(d_test));
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[2] == a)
                    {
                        d_courant = map_.phi_1(d_courant);
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(map_.phi_1(d_test)) == map_.phi_1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map_.phi_1(d_test));
                            cond_valide++;
                        }
                    }
                }

                //
                // test à partir du deuxième sommet

                // Pour ce sommet équivalent à "a"
                if(T_courant.connectivity_[1] == a)
                {
                    if(T_courant.connectivity_[0] == b)
                    {
                        d_courant = d_courant;
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(d_test) == d_test)
                        {
                            mbuild2.phi2_sew(d_courant, d_test);
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[2] == c)
                    {
                        d_courant = map_.phi1(d_courant);
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(map_.phi_1(d_test)) == map_.phi_1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map_.phi_1(d_test));
                            cond_valide++;
                        }
                    }
                }

                // Pour ce sommet équivalent à "b"
                if(T_courant.connectivity_[1] == b)
                {
                    if(T_courant.connectivity_[0] == c)
                    {
                        d_courant = d_courant;
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(map_.phi1(d_test)) == map_.phi1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map_.phi1(d_test));
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[2] == a)
                    {
                        d_courant = map_.phi1(d_courant);
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(d_test) == d_test)
                        {
                            mbuild2.phi2_sew(d_courant, d_test);
                            cond_valide++;
                        }
                    }
                }

                // Pour ce sommet équivalent à "c"
                if(T_courant.connectivity_[1] == c)
                {
                    if(T_courant.connectivity_[0] == a)
                    {
                        d_courant = d_courant;
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(map_.phi_1(d_test)) == map_.phi_1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map_.phi_1(d_test));
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[2] == b)
                    {
                        d_courant = map_.phi1(d_courant);
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(map_.phi1(d_test)) == map_.phi1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map_.phi1(d_test));
                            cond_valide++;
                        }
                    }
                }


                //
                // Test à partir du troisième sommet

                // Pour ce sommet équivalent à "a"
                if(T_courant.connectivity_[2] == a)
                {
                    if(T_courant.connectivity_[0] == c)
                    {
                        d_courant = map_.phi_1(d_courant);
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(map_.phi_1(d_test)) == map_.phi_1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map_.phi_1(d_test));
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[1] == b)
                    {
                        d_courant = map_.phi1(d_courant);
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(d_test) == d_test)
                        {
                            mbuild2.phi2_sew(d_courant, d_test);
                            cond_valide++;

                        }
                    }
                }

                // Pour ce sommet équivalent à "b"
                if(T_courant.connectivity_[2] == b)
                {
                    if(T_courant.connectivity_[0] == a)
                    {
                        d_courant = map_.phi_1(d_courant);
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(d_test) == d_test)
                        {
                            mbuild2.phi2_sew(d_courant, d_test);
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[1] == c)
                    {
                        d_courant = map_.phi1(d_courant);
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(map_.phi1(d_test)) == map_.phi1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map_.phi1(d_test));
                            cond_valide++;

                        }
                    }
                }

                // Pour ce sommet équivalent à "c"
                if(T_courant.connectivity_[2] == c)
                {
                    if(T_courant.connectivity_[0] == b)
                    {
                        d_courant = map_.phi_1(d_courant);
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(map_.phi1(d_test)) == map_.phi1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map_.phi1(d_test));
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[1] == a)
                    {
                        d_courant = map_.phi1(d_courant);
                        if(map_.phi2(d_courant) == d_courant && map_.phi2(map_.phi_1(d_test)) == map_.phi_1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map_.phi_1(d_test));
                            cond_valide++;
                        }
                    }
                }
            }
            count++;
        }
        nb_courant++;
        std::cout << " On effectue les coutures : " << cond_valide << std::endl;
    }

    mbuild2.close_map();

    map_.check_map_integrity();

    vertex_position_ = map_.add_attribute<Vec3, Vertex2::ORBIT>("position");
    vertex_normal_ = map_.add_attribute<Vec3, Vertex2::ORBIT>("normal");

    // TEST : Pour vérifier que l'on est bien sur une sphère

    /*
    int incre = 0;
    for(TriangleGeo T : triangles)
    {
        Dart d = dart_faces[incre];
        vertex_position_[Vertex2(d)] = T.sommets_[0];
        vertex_position_[Vertex2(map_.phi1(d))] = T.sommets_[1];
        vertex_position_[Vertex2(map_.phi_1(d))] = T.sommets_[2];
        incre++;
    }*/


    // Véritables valeurs

    int incre = 0;
    // Ces deux tableaux seront probablement des tableaux de 3 éléments
    // On va y ajouter les Darts correspondants aux points centraux des bouts de branche
    std::vector<Dart> centre_branches;
    std::vector<int> appartenance;

    for(TriangleGeo T : triangles)
    {
        Dart d = dart_faces[incre];

        int cond_s0 = T.connectivity_[0]%(TYPE_PRIMITIVE + 1);
        bool exist0 = false;
        int cond_s1 = T.connectivity_[1]%(TYPE_PRIMITIVE + 1);
        bool exist1 = false;
        int cond_s2 = T.connectivity_[2]%(TYPE_PRIMITIVE + 1);
        bool exist2 = false;

        if(cond_s0 == 0)
        {
            if(!appartenance.empty())
            {
                // Si le dart n'est pas déjà dans le tableau, alors on le rajoute
                for(int j : appartenance)
                {
                    if(j == cond_s0)
                        exist0 = true;
                }
                if(!exist0)
                {
                    centre_branches.push_back(d);
                    appartenance.push_back(cond_s0);
                }
            }
            else
            {
                centre_branches.push_back(d);
                appartenance.push_back(cond_s0);
            }
        }
        if(cond_s1 == 0)
        {
            if(!appartenance.empty())
            {
                // Si le dart n'est pas déjà dans le tableau, alors on le rajoute
                for(int j : appartenance)
                {
                    if(j == cond_s1)
                        exist1 = true;
                }
                if(!exist1)
                {
                    centre_branches.push_back(map_.phi1(d));
                    appartenance.push_back(cond_s1);
                }
            }
            else
            {
                centre_branches.push_back(map_.phi1(d));
                appartenance.push_back(cond_s1);
            }
        }
        if(cond_s2 == 0)
        {
            if(!appartenance.empty())
            {
                // Si le dart n'est pas déjà dans le tableau, alors on le rajoute
                for(int j : appartenance)
                {
                    if(j == cond_s2)
                        exist2 = true;
                }
                if(!exist2)
                {
                    centre_branches.push_back(map_.phi_1(d));
                    appartenance.push_back(cond_s2);
                }
            }
            else
            {
                centre_branches.push_back(map_.phi_1(d));
                appartenance.push_back(cond_s2);
            }
        }


        vertex_position_[Vertex2(d)] = sommets_intersection[T.connectivity_[0]];
        vertex_position_[Vertex2(map_.phi1(d))] = sommets_intersection[T.connectivity_[1]];
        vertex_position_[Vertex2(map_.phi_1(d))] = sommets_intersection[T.connectivity_[2]];
        incre++;
    }

    // Afficher les coordonées avec un foreach ?
    //map_.foreach_cell([&] (Face2 f){ });


    //
    // Subdiviser la map2 (en utilisant les coordonées?)
    //

    std::vector<Dart> faces_a_subdiv;
    int counter_T = 0;
    for(TriangleGeo T : triangles)
    {
        Dart d = dart_faces[counter_T];

        int cond_s0 = T.connectivity_[0]%(TYPE_PRIMITIVE + 1);
        int cond_s1 = T.connectivity_[1]%(TYPE_PRIMITIVE + 1);
        int cond_s2 = T.connectivity_[2]%(TYPE_PRIMITIVE + 1);

        if(0!=cond_s1 && 0!=cond_s0 && 0!=cond_s2)
        {
            faces_a_subdiv.push_back(d);
        }

        counter_T++;
    }

    int nb_of_subdiv = 0;
    std::vector<Dart> edge_to_flip;

    while(nb_of_subdiv != NB_SUBDIV_INTERSECTION)
    {

        for(Dart d1 : faces_a_subdiv)
        {
            edge_to_flip.push_back(d1);
            Dart d2 = map_.phi1(d1);
            edge_to_flip.push_back(d2);
            Dart d3 = map_.phi_1(d1);
            edge_to_flip.push_back(d3);
            Edge2 e1 = map_.cut_face(d1, d2);
            Vertex2 v = map_.cut_edge(e1);
            vertex_position_[v] = (vertex_position_[Vertex2(d1)] + vertex_position_[Vertex2(d3)] + vertex_position_[Vertex2(d2)])/3;
            Dart d_centre = v.dart;
            Edge2 e2 = map_.cut_face(d3,map_.phi<21>(d_centre));
        }
        faces_a_subdiv = edge_to_flip;

        int taille_edge_to_flip = edge_to_flip.size();
        for(int i = 0; i < taille_edge_to_flip; i++)
        {
            for(int j = 0; j < taille_edge_to_flip; j++)
            {
                // condition où deux edges sont les mêmes
                if(i != j && map_.phi2(edge_to_flip[i]) == edge_to_flip[j])
                {
                    map_.flip_edge(Edge2(edge_to_flip[i]));
                    edge_to_flip.erase(edge_to_flip.begin() + j);
                    edge_to_flip.erase(edge_to_flip.begin() + i);
                    taille_edge_to_flip--;
                    taille_edge_to_flip--;
                    i--;
                    if(j > i)
                        i--;
                    break;

                }
            }
        }

        edge_to_flip.clear();
        nb_of_subdiv++;
    }


    cgogn::geometry::compute_normal<Vec3>(map_, vertex_position_, vertex_normal_);
    cgogn::geometry::compute_AABB(vertex_position_, bb_);
    setSceneRadius(bb_.diag_size()/2.0);
    Vec3 center = bb_.center();
    setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
    showEntireScene();
}


void Viewer2::import(const std::string& surface_mesh)
{
    cgogn::io::import_surface<Vec3>(map_, surface_mesh);

    vertex_position_ = map_.get_attribute<Vec3, Map2::Vertex::ORBIT>("position");
    if (!vertex_position_.is_valid())
    {
        cgogn_log_error("Viewer::import") << "Missing attribute position. Aborting.";
        std::exit(EXIT_FAILURE);
    }

    vertex_normal_ = map_.add_attribute<Vec3, Map2::Vertex::ORBIT>("normal");


    cgogn::geometry::compute_normal<Vec3>(map_, vertex_position_, vertex_normal_);
    cgogn::geometry::compute_AABB(vertex_position_, bb_);
    setSceneRadius(bb_.diag_size()/2.0);
    Vec3 center = bb_.center();
    setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
    showEntireScene();
}

Viewer2::~Viewer2()
{}

void Viewer2::closeEvent(QCloseEvent*)
{
    render_.reset();
    vbo_pos_.reset();
    vbo_norm_.reset();
    vbo_color_.reset();
    vbo_sphere_sz_.reset();
    drawer_.reset();
    drawer_rend_.reset();
}

Viewer2::Viewer2() :
    map_(),
    vertex_position_(),
    vertex_normal_(),
    bb_(),
    render_(nullptr),
    vbo_pos_(nullptr),
    vbo_norm_(nullptr),
    vbo_color_(nullptr),
    vbo_sphere_sz_(nullptr),
    drawer_(nullptr),
    drawer_rend_(nullptr),
    phong_rendering_(true),
    flat_rendering_(false),
    vertices_rendering_(false),
    edge_rendering_(false),
    normal_rendering_(false),
    bb_rendering_(true)
{
}

void Viewer2::draw()
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
        drawer_rend_->draw(proj,view,this);
}

void Viewer2::init()
{
    glClearColor(0.1f,0.1f,0.3f,0.0f);

    // create and fill VBO for positions
    vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
    cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());

    // create and fill VBO for normals
    vbo_norm_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
    cgogn::rendering::update_vbo(vertex_normal_, vbo_norm_.get());

    // fill a color vbo with abs of normals
    vbo_color_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
    cgogn::rendering::update_vbo(vertex_normal_, vbo_color_.get(), [] (const Vec3& n) -> std::array<float,3>
    {
        return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
    });

    // fill a sphere size vbo
    vbo_sphere_sz_ = cgogn::make_unique<cgogn::rendering::VBO>(1);
    cgogn::rendering::update_vbo(vertex_normal_, vbo_sphere_sz_.get(), [&] (const Vec3& n) -> float
    {
        return bb_.diag_size()/1000.0 * (1.0 + 2.0*std::abs(n[2]));
    });

    // map rendering object (primitive creation & sending to GPU)
    render_ = cgogn::make_unique<cgogn::rendering::MapRender>();
    render_->init_primitives(map_, cgogn::rendering::POINTS);
    render_->init_primitives(map_, cgogn::rendering::LINES);
    render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, &vertex_position_);

    // generation of one parameter set (for this shader) : vbo + uniforms
    param_point_sprite_ = cgogn::rendering::ShaderPointSpriteColorSize::generate_param();
    // set vbo param (see param::set_vbo signature)
    param_point_sprite_->set_all_vbos(vbo_pos_.get(), vbo_color_.get(), vbo_sphere_sz_.get());
    // set uniforms data

    param_edge_ = cgogn::rendering::ShaderBoldLine::generate_param();
    param_edge_->set_position_vbo(vbo_pos_.get());
    param_edge_->color_ = QColor(255,255,0);
    param_edge_->width_= 2.5f;

    param_flat_ = cgogn::rendering::ShaderFlat::generate_param();
    param_flat_->set_position_vbo(vbo_pos_.get());
    param_flat_->front_color_ = QColor(0,200,0);
    param_flat_->back_color_ = QColor(0,0,200);
    param_flat_->ambiant_color_ = QColor(5,5,5);

    param_normal_ = cgogn::rendering::ShaderVectorPerVertex::generate_param();
    param_normal_->set_all_vbos(vbo_pos_.get(), vbo_norm_.get());
    param_normal_->color_ = QColor(200,0,200);
    param_normal_->length_ = bb_.diag_size()/50;

    param_phong_ = cgogn::rendering::ShaderPhongColor::generate_param();
    param_phong_->set_all_vbos(vbo_pos_.get(), vbo_norm_.get(), vbo_color_.get());

    // drawer for simple old-school g1 rendering
    drawer_ = cgogn::make_unique<cgogn::rendering::DisplayListDrawer>();
    drawer_rend_= drawer_->generate_renderer();
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

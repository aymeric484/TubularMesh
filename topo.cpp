#include "topo.h"

topo::topo(const Squelette& mon_squelette, const unsigned int& primitive) :
    map_(),
    map2_(),
    map3inter_(),
    vertex_position_(),
    vertex_position_ter_(),
    vertex_position_bis_(),
    vertex_normal_(),
    vertex_inter_position_(),
    vertex_appartenance_(),
    cell_cache_prec_(map_)
{
    nb_appuis_ = 0;
    indice_repartition_ = 0;

    //
    // Appel principal :
    //

    MakeFromSkeleton(mon_squelette, primitive);


    //
    // Différents TESTS :
    //

    // Coudre après fusion deux volume simples
    //TestMergeSew();

    // Coudre simplement sans fusion
    //TestSimpleSew();
}

void topo::TestSimpleSew()
{
    //
    // Création tétra_1 et tétra_2

    MapBuilder mbuilding(map_);
    Dart d1 = mbuilding.add_pyramid_topo(3);
    Dart d2 = mbuilding.add_pyramid_topo(3);
    mbuilding.close_map();
    vertex_position_ = map_.add_attribute<Vec3, Vertex::ORBIT>("position");

    //
    // Affectation tétra_1

    Vec3 v11 = {0, 0, 0};
    Vec3 v12 = {0, 1, 0};
    Vec3 v13 = {1, 0, 0};
    Vec3 v14 = {0, 0, 1};

    vertex_position_[Vertex(d1)] = v11;
    vertex_position_[Vertex(map_.phi1(d1))] = v12;
    vertex_position_[Vertex(map_.phi<11>(d1))] = v13;
    vertex_position_[Vertex(map_.phi<211>(d1))] = v14;

    //
    // Affectation tétra_2

    Vec3 v21 = {1, 1, -2};
    Vec3 v22 = {1, 2, -2};
    Vec3 v23 = {2, 1, -2};
    Vec3 v24 = {1, 1, -3};

    vertex_position_[Vertex(d2)] = v21;
    vertex_position_[Vertex(map_.phi1(d2))] = v22;
    vertex_position_[Vertex(map_.phi<11>(d2))] = v23;
    vertex_position_[Vertex(map_.phi<211>(d2))] = v24;

    //
    // Fusion des 2 tétras

    cgogn_assert(map_.check_map_integrity());
    map_.sew_volumes(Face(d1), Face(d2));

}

void topo::TestMergeSew()
{

    //
    // Création prisme 1 et 2

    MapBuilder mbuilding(map_);
    Dart d1 = mbuilding.add_prism_topo(3);
    Dart d2 = mbuilding.add_prism_topo(3);
    mbuilding.sew_volumes(map_.phi2(d2), map_.phi<12>(d1));
    mbuilding.close_map();

    vertex_position_ = map_.add_attribute<Vec3, Vertex::ORBIT>("position");
    vertex_appartenance_ = map_.add_attribute<int, Vertex::ORBIT>("appartenance");

    Vec3 v11 = {0, 0, 0};
    Vec3 v12 = {1, 0, 0};
    Vec3 v13 = {0, 1, 0};
    Vec3 v14 = {1, 1, 0};
    Vec3 v15 = {0, 0, -1};
    Vec3 v16 = {1, 0, -1};
    Vec3 v17 = {0, 1, -1};
    Vec3 v18 = {1, 1, -1};

    vertex_position_[Vertex(d1)] = v11;
    vertex_position_[Vertex(map_.phi1(d1))] = v12;
    vertex_position_[Vertex(map_.phi_1(d1))] = v13;
    vertex_position_[Vertex(map_.phi<21121>(d1))] = v15;
    vertex_position_[Vertex(map_.phi<211211>(d1))] = v17;
    vertex_position_[Vertex(map_.phi<2112111>(d1))] = v16;
    vertex_position_[Vertex(map_.phi_1(d2))] = v14;
    vertex_position_[Vertex(map_.phi<12112>(d2))] = v18;

    cgogn_assert(map_.check_map_integrity());


    //
    // Création tétra 2

    MapBuilder mbuilding2(map3inter_);
    Dart d_bis = mbuilding2.add_pyramid_topo(3);
    mbuilding2.close_map();

    vertex_position_ter_ = map3inter_.add_attribute<Vec3, Vertex::ORBIT>("position");
    vertex_appartenance_bis_ = map3inter_.add_attribute<int, Vertex::ORBIT>("appartenance");

    Vec3 v21 = {0, 0, -1}; // équivaut à v15
    Vec3 v22 = {0, 1, -1}; // équivaut à v17
    Vec3 v23 = {1, 0, -1}; // équivaut à v16
    Vec3 v24 = {0, 0, -2}; // nouveau => équivaut à rien

    vertex_position_ter_[Vertex(d_bis)] = v21;
    vertex_position_ter_[Vertex(map3inter_.phi1(d_bis))] = v23;
    vertex_position_ter_[Vertex(map3inter_.phi<11>(d_bis))] = v22;
    vertex_position_ter_[Vertex(map3inter_.phi<211>(d_bis))] = v24;

    map3inter_.foreach_cell([&] (Vertex v){vertex_appartenance_bis_[v] = 1;});

    //
    // Création prisme 2

    /*
    MapBuilder mbuilding2(map3inter_);
    Dart d_bis = mbuilding2.add_prism_topo(3);
    mbuilding2.close_map();

    vertex_position_ter_ = map3inter_.add_attribute<Vec3, Vertex::ORBIT>("position");
    vertex_appartenance_bis_ = map3inter_.add_attribute<int, Vertex::ORBIT>("appartenance");

    Vec3 v21 = {0, 0, -1};
    Vec3 v22 = {0, 1, -1};
    Vec3 v23 = {1, 0, -1};
    Vec3 v24 = {0, 0, -2};
    Vec3 v25 = {0, 1, -2};
    Vec3 v26 = {1, 0, -2};

    vertex_position_ter_[Vertex(d_bis)] = v21;
    vertex_position_ter_[Vertex(map3inter_.phi1(d_bis))] = v22;
    vertex_position_ter_[Vertex(map3inter_.phi<11>(d_bis))] = v23;
    vertex_position_ter_[Vertex(map3inter_.phi<21121>(d_bis))] = v24;
    vertex_position_ter_[Vertex(map3inter_.phi<211211>(d_bis))] = v26;
    vertex_position_ter_[Vertex(map3inter_.phi<2112111>(d_bis))] = v25;

    map3inter_.foreach_cell([&] (Vertex v){vertex_appartenance_bis_[v] = 1;});*/


    //
    // Fusion des 2 volumes

    cgogn_assert(map3inter_.check_map_integrity());
    Map3::DartMarker dm(map3inter_);
    map_.merge(map3inter_, dm);
    Dart d3,d4;
    d3 = map_.phi<21121>(d1);
    Vertex v_ref;
    double dist_ref = 100;

    //
    // Localisation des Vertices correspondants

    map_.foreach_cell([&] (Vertex v){

        double dist = (vertex_position_[v] - vertex_position_[Vertex(d3)]).norm();
        if(dist < dist_ref)
        {
            dist_ref = dist;
            v_ref = v;
        }

    }, [&] (Vertex v){
        return (vertex_appartenance_[v] == 1);
    });

    //
    // Trouver le bon dart

    int count = 0;

    map_.foreach_dart_of_orbit(v_ref, [&] (Dart d_courant){

        Vec3 pos_test = vertex_position_[Vertex(map_.phi1(d_courant))];
        Vec3 pos_ref = vertex_position_[Vertex(map_.phi<2112>(d1))];
        bool testX = (((pos_test[0] - pos_ref[0])*(pos_test[0] - pos_ref[0])) < 0.0001);
        bool testY = (((pos_test[1] - pos_ref[1])*(pos_test[1] - pos_ref[1])) < 0.0001);
        bool testZ = (((pos_test[2] - pos_ref[2])*(pos_test[2] - pos_ref[2])) < 0.0001);
        if(testX && testY && testZ && map_.is_boundary(map_.phi3(d_courant)))
        {
            count++;
            d4 = d_courant;
        }


    });
    std::cout << "count : " << count << std::endl;

    cgogn_assert(map_.check_map_integrity());

    int incre = 0;
    map_.foreach_cell([&] (Volume v){
        incre++;});
    std::cout << "Nb de volumes : " << incre << std::endl;
    map_.sew_volumes(Face(d4), Face(map_.phi<2112>(d1)));
}

std::unique_ptr<tetgenio> topo::export_tetgen()
{
    std::unique_ptr<tetgenio> output = cgogn::make_unique<tetgenio>();

    // 0-based indexing
    output->firstnumber = 0;

    // input vertices
    output->numberofpoints = map2_.template nb_cells<Vertex2::ORBIT>();
    output->pointlist = new TetgenReal[output->numberofpoints * 3];

    //for each vertex
    map2_.foreach_cell([&] (Vertex2 v)
    {
        const Vec3& vec = vertex2_position_[v];
        const unsigned int emb = map2_.embedding(v);
        output->pointlist[3u*emb + 0u] = vec[0];
        output->pointlist[3u*emb + 1u] = vec[1];
        output->pointlist[3u*emb + 2u] = vec[2];
    });

    output->numberoffacets = map2_.template nb_cells<Face2::ORBIT>();
    output->facetlist = new tetgenio::facet[output->numberoffacets];

    //for each facet
    unsigned int i = 0u;
    map2_.foreach_cell([&] (Face2 face)
    {
        tetgenio::facet* f = &(output->facetlist[i]);// on accede à l'élément i de la liste et on récup le pointeur
        tetgenio::init(f); // initialisation du pointeur
        f->numberofpolygons = 1;
        f->polygonlist = new tetgenio::polygon[1];// nature du polygone codée par le 1?
        tetgenio::polygon* p = &f->polygonlist[0];// On récup le premier et le seul polygone
        tetgenio::init(p);
        p->numberofvertices = map2_.codegree(face);
        p->vertexlist = new int[p->numberofvertices];

        unsigned int j = 0u;
        // construction du polygone avec 3 points de faces
        map2_.foreach_incident_vertex(face, [&] (Vertex2 v)
        {
            p->vertexlist[j++] = map2_.embedding(v);
        });

        ++i;
    });

    return output;
}

void topo::Generate_tetgen(const std::string& tetgen_args, int count)
{
    auto tetgen_input = export_tetgen();

    tetgenio tetgen_output;

    tetgen::tetrahedralize(tetgen_args.c_str(), tetgen_input.get(), &tetgen_output);

    TetgenStructureVolumeImport tetgen_import(&tetgen_output);

    tetgen_import.create_map(map3inter_);

    int count_boundary_vertices = 0;
    map3inter_.foreach_cell([&] (Vertex v){
            if(map3inter_.is_incident_to_boundary(v))
                count_boundary_vertices++;
    });
    std::cout << "Nb de sommets du bord : " << count_boundary_vertices << std::endl;


    Map3::VertexAttribute<int> inter_code = map3inter_.add_attribute<int, Vertex::ORBIT>("appartenance");

    map3inter_.foreach_cell([&] (Vertex v){
        inter_code[v] = count + 1;
    });

    map3inter_.check_map_integrity();
}

// Passer les variables en réferences constantes pourrait être mieux
void topo::MakeIntersection(std::vector<TriangleGeo> triangles, std::vector<Vec3> sommets_intersection)
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

    MapBuilder2 mbuild2(map2_);
    std::vector<Dart> dart_faces;

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
                std::cout << " condition OK " << std::endl;
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
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(map2_.phi_1(d_test)) == map2_.phi_1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map2_.phi_1(d_test));
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[2] == b)
                    {
                        d_courant = map2_.phi_1(d_courant);
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(d_test) == d_test)
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
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(d_test) == d_test)
                        {
                            mbuild2.phi2_sew(d_courant, d_test);
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[2] == c)
                    {
                        d_courant = map2_.phi_1(d_courant);
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(map2_.phi1(d_test)) == map2_.phi1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map2_.phi1(d_test));
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
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(map2_.phi1(d_test)) == map2_.phi1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map2_.phi1(d_test));
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[2] == a)
                    {
                        d_courant = map2_.phi_1(d_courant);
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(map2_.phi_1(d_test)) == map2_.phi_1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, d_test);
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
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(d_test) == d_test)
                        {
                            mbuild2.phi2_sew(d_courant, d_test);
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[2] == c)
                    {
                        d_courant = map2_.phi1(d_courant);
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(map2_.phi_1(d_test)) == map2_.phi_1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map2_.phi_1(d_test));
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
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(map2_.phi1(d_test)) == map2_.phi1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map2_.phi1(d_test));
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[2] == a)
                    {
                        d_courant = map2_.phi1(d_courant);
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(d_test) == d_test)
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
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(map2_.phi_1(d_test)) == map2_.phi_1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map2_.phi_1(d_test));
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[2] == b)
                    {
                        d_courant = map2_.phi1(d_courant);
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(map2_.phi1(d_test)) == map2_.phi1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map2_.phi1(d_test));
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
                        d_courant = map2_.phi_1(d_courant);
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(map2_.phi_1(d_test)) == map2_.phi_1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map2_.phi_1(d_test));
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[1] == b)
                    {
                        d_courant = map2_.phi1(d_courant);
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(d_test) == d_test)
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
                        d_courant = map2_.phi_1(d_courant);
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(d_test) == d_test)
                        {
                            mbuild2.phi2_sew(d_courant, d_test);
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[1] == c)
                    {
                        d_courant = map2_.phi1(d_courant);
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(map2_.phi1(d_test)) == map2_.phi1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map2_.phi1(d_test));
                            cond_valide++;

                        }
                    }
                }

                // Pour ce sommet équivalent à "c"
                if(T_courant.connectivity_[2] == c)
                {
                    if(T_courant.connectivity_[0] == b)
                    {
                        d_courant = map2_.phi_1(d_courant);
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(map2_.phi1(d_test)) == map2_.phi1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map2_.phi1(d_test));
                            cond_valide++;
                        }
                    }
                    if(T_courant.connectivity_[1] == a)
                    {
                        d_courant = map2_.phi1(d_courant);
                        if(map2_.phi2(d_courant) == d_courant && map2_.phi2(map2_.phi_1(d_test)) == map2_.phi_1(d_test))
                        {
                            mbuild2.phi2_sew(d_courant, map2_.phi_1(d_test));
                            cond_valide++;
                        }
                    }
                }
            }
            count++;
        }
        nb_courant++;
        std::cout << " On effectue les coutures : " << cond_valide << std::endl;
        std::cout << "Nombres de voisin qui devrait valoir 3 : " << Nb_voisin << std::endl;
    }


    mbuild2.close_map();

    map2_.check_map_integrity();

    vertex2_position_ = map2_.add_attribute<Vec3, Vertex2::ORBIT>("position");

    // TEST : Pour vérifier que l'on est bien sur une sphère

    /*int incre = 0;
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
                    centre_branches.push_back(map2_.phi1(d));
                    appartenance.push_back(cond_s1);
                }
            }
            else
            {
                centre_branches.push_back(map2_.phi1(d));
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
                    centre_branches.push_back(map2_.phi_1(d));
                    appartenance.push_back(cond_s2);
                }
            }
            else
            {
                centre_branches.push_back(map2_.phi_1(d));
                appartenance.push_back(cond_s2);
            }
        }


        vertex2_position_[Vertex2(d)] = sommets_intersection[T.connectivity_[0]];
        vertex2_position_[Vertex2(map2_.phi1(d))] = sommets_intersection[T.connectivity_[1]];
        vertex2_position_[Vertex2(map2_.phi_1(d))] = sommets_intersection[T.connectivity_[2]];
        incre++;
    }


    //
    // Subdiviser la map2 en utilisant les coordonées
    //

}

void topo::MakeBranch(const std::vector<Vec3>& positions, const unsigned int& primitives)
{
    BranchTopo Bt;
    MapBuilder mbuild(map3branch_);
    int nb_articulation = positions.size()/(primitives+1);
    int volume_count = 0;
    unsigned int count = 0;
    unsigned int arti_count = 0;

    // creation de nos volumes
    for(int n = primitives ; n < nb_articulation*primitives ; n++)
        Bt.volume_control_.push_back(mbuild.add_prism_topo(3));//construit le prisme et renvoi un dart du prisme d'une des faces triangulaires, rendant un parcourt du prisme possible


    for(int m = 1 ; m < nb_articulation-1 ; m++)
    {
        for(int k = 0; k < primitives; k++)
        {
            // coudre les faces triangulaires des prismes(3d)
            Dart v1 = map3branch_.phi2(map3branch_.phi1(map3branch_.phi1(map3branch_.phi2(Bt.volume_control_[(m-1)*primitives + k]))));
            Dart v2 = Bt.volume_control_[m*primitives + k];
            mbuild.sew_volumes(v1, v2);

            // coudre les faces rectangulaires des prismes(3d)
            Dart v3 = map3branch_.phi2(map3branch_.phi1(Bt.volume_control_[(m-1)*primitives + k]));
            Dart v4;
            if(k+1 != primitives)
                v4 = map3branch_.phi2(Bt.volume_control_[(m-1)*primitives + k + 1]);
            else
                v4 = map3branch_.phi2(Bt.volume_control_[(m-1)*primitives]);
            mbuild.sew_volumes(v3,v4);
        }
    }


    // dernière serie de volumes (bout de branche)
    for(int k = 0; k < primitives; k++)
    {
        Dart v3 = map3branch_.phi2(map3branch_.phi1(Bt.volume_control_[(nb_articulation-2)*primitives + k]));
        Dart v4;
        if(k+1 != primitives)
            v4 = map3branch_.phi2(Bt.volume_control_[(nb_articulation-2)*primitives + k + 1]);
        else
            v4 = map3branch_.phi2(Bt.volume_control_[(nb_articulation-2)*primitives]);
        mbuild.sew_volumes(v3,v4);
    }

    mbuild.close_map(); //reboucle les volumes en bord de map


    map3branch_.foreach_cell([&] (Volume v){ volume_count++; }); // affichage du nombre de volumes
    std::cout << " Il y a " << volume_count << " Volume(s)" << std::endl;


    //Les vertices vont être indexe automatiquement & creation d'un de leur attribut, position dans l espace 3D

    vertex_normal_ = map3branch_.add_attribute<Vec3, Vertex::ORBIT>("normal");
    vertex_position_ = map3branch_.add_attribute<Vec3, Vertex::ORBIT>("position");

    // affectation d'un point du prisme triangulaire & du point en commun aux n-prismes (n = primitive)
    for(Dart d : Bt.volume_control_)
    {
        // il s'agit des positions des articulations
        if(count == 0 || count == primitives + 1 )
        {
            Vertex v1(map3branch_.phi1(d));
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

    Dart last_arti = map3branch_.phi1(map3branch_.phi1(map3branch_.phi2(map3branch_.phi1(Bt.volume_control_[Bt.volume_control_.size() - 1]))));

    vertex_position_[Vertex(last_arti)] = positions[(primitives+1)*(nb_articulation-1)];

    // Ici, on gere la derniere face (les points autour de la dernière articulation)
    for(unsigned int i = 0; i < primitives; i++)
    {
        Dart last_points = map3branch_.phi1(map3branch_.phi2(map3branch_.phi1(map3branch_.phi1(map3branch_.phi2(Bt.volume_control_[Bt.volume_control_.size()-primitives + i])))));
        Vertex points(last_points);
        vertex_position_[points] = positions[Bt.volume_control_.size() + nb_articulation + i];
    }


    map3branch_.check_map_integrity();


    /*

    MapBuilder mbuild(map3branch_);

    int nb_articulation = positions.size()/(primitives + 1); // Remplacer position.size() par longueur[i]
    int volume_count = 0;
    unsigned int count = 0;
    unsigned int arti_count = 0;

    // creation de nos volumes
    for(int n = primitives ; n < nb_articulation*primitives ; n++)
        volume_control_.push_back(mbuild.add_prism_topo(3)); // construit le prisme et renvoi un dart du prisme d'une des faces triangulaires, rendant un parcourt du prisme possible

    for(int m = 0 ; m < nb_articulation-2 ; m++)
    {
        for(int k = 0; k < primitives; k++)
        {
            // coudre les faces triangulaires des prismes(3d)
            Dart v1 = map3branch_.phi2(map3branch_.phi1(map3branch_.phi1(map3branch_.phi2(volume_control_[volume_control_.size() - (nb_articulation-1)*primitives + m*primitives + k]))));
            Dart v2 = volume_control_[volume_control_.size() - (nb_articulation-1)*primitives + (m+1)*primitives + k]; // +positions.size des branches précédentes. ou bien toujours partir de la fin! (vu qu'on pushback)
            mbuild.sew_volumes(v1, v2);

            // coudre les faces rectangulaires des prismes(3d)
            Dart v3 = map3branch_.phi2(map3branch_.phi1(volume_control_[volume_control_.size() - (nb_articulation-1)*primitives + m*primitives + k]));
            Dart v4;
            if(k+1 != primitives)
                v4 = map3branch_.phi2(volume_control_[volume_control_.size() - (nb_articulation-1)*primitives + m*primitives + k + 1]);
            else
                v4 = map3branch_.phi2(volume_control_[volume_control_.size() - (nb_articulation-1)*primitives + m*primitives]);

            mbuild.sew_volumes(v3,v4);
        }
    }

    // dernière serie de volumes (bout de branche)
    for(int k = 0; k < primitives; k++)
    {
        Dart v3 = map3branch_.phi2(map3branch_.phi1(volume_control_[volume_control_.size() - primitives + k]));
        Dart v4;
        if(k+1 != primitives)
            v4 = map3branch_.phi2(volume_control_[volume_control_.size() - primitives + k + 1]);
        else
            v4 = map3branch_.phi2(volume_control_[volume_control_.size() - primitives]);
        mbuild.sew_volumes(v3,v4);
    }

    mbuild.close_map(); //reboucle les volumes en bord de map


    map3branch_.foreach_cell([&] (Volume v){ volume_count++; }); // affichage du nombre de volumes
    std::cout << " Il y a " << volume_count << " Volume(s)" << std::endl;


    //Les vertices vont être indexe automatiquement & creation d'un de leur attribut, position dans l espace 3D
    vertex_normal_ = map3branch_.add_attribute<Vec3, Vertex::ORBIT>("normal");
    vertex_position_ = map3branch_.add_attribute<Vec3, Vertex::ORBIT>("position");


    // Affectation d'un point du prisme triangulaire & du point en commun aux n-prismes (n = primitive)
    for(int i = volume_control_.size() - (nb_articulation - 1)*primitives; i < volume_control_.size(); i++)
    {
        Dart d = volume_control_[i];
        // Il s'agit des positions des articulations
        if(count == 0 || count == primitives + 1 )
        {
            Vertex v1(map3branch_.phi1(d));
            vertex_position_[v1] = positions[arti_count * (primitives + 1)];
            arti_count++;
            count = 1;
        }

        // ici on a la position des points autour de chaque articulation
        Vertex v2(d);
        vertex_position_[v2] = positions[(arti_count - 1) * (primitives + 1) + count]; // le facteur est (arti_count - 1) car on a arti_count++ dans le if
        count++;
    }

    // On gère les dernières articulations
    Dart last_arti = map3branch_.phi1(map3branch_.phi1(map3branch_.phi2(map3branch_.phi1(volume_control_[volume_control_.size() - 1]))));

    vertex_position_[Vertex(last_arti)] = positions[(primitives+1)*(nb_articulation-1)];
    // Ici, on gere la derniere face (les points autour de la dernière articulation)
    for(unsigned int i = 0; i < primitives; i++)
    {
        Dart last_points = map3branch_.phi1(map3branch_.phi2(map3branch_.phi1(map3branch_.phi1(map3branch_.phi2(volume_control_[volume_control_.size()-primitives + i])))));
        Vertex points(last_points);
        vertex_position_[points] = positions[positions.size() - primitives + i];
    }
    */

    controls_.push_back(Bt);
}

void topo::MakeFromSkeleton(const Squelette& mon_squelette, const unsigned int& primitives)
{

    MapBuilder mbuild(map_);
    // On parcourt toute les branches et on construit la topologie de chacune via MakeBranch dans une map intermediaire. Puis on merge cette map à notre map3 finale
    for(Branch branche : mon_squelette.branches_)
    {
        BranchTopo Bt;

        int nb_articulation = branche.pos_vertices_.size()/(primitives+1);

        // creation de nos volumes
        for(int n = primitives ; n < nb_articulation*primitives ; n++)
            Bt.volume_control_.push_back(mbuild.add_prism_topo(3));//construit le prisme et renvoi un dart du prisme d'une des faces triangulaires, rendant un parcourt du prisme possible

        for(int m = 1 ; m < nb_articulation-1 ; m++)
        {
            for(int k = 0; k < primitives; k++)
            {
                // coudre les faces triangulaires des prismes(3d)
                Dart v1 = map_.phi2(map_.phi1(map_.phi1(map_.phi2(Bt.volume_control_[(m-1)*primitives + k]))));
                Dart v2 = Bt.volume_control_[m*primitives + k];
                mbuild.sew_volumes(v1, v2);

                // coudre les faces rectangulaires des prismes(3d)
                Dart v3 = map_.phi2(map_.phi1(Bt.volume_control_[(m-1)*primitives + k]));
                Dart v4;
                if(k+1 != primitives)
                    v4 = map_.phi2(Bt.volume_control_[(m-1)*primitives + k + 1]);
                else
                    v4 = map_.phi2(Bt.volume_control_[(m-1)*primitives]);
                mbuild.sew_volumes(v3,v4);
            }
        }


        // dernière serie de volumes (bout de branche)
        for(int k = 0; k < primitives; k++)
        {
            Dart v3 = map_.phi2(map_.phi1(Bt.volume_control_[(nb_articulation-2)*primitives + k]));
            Dart v4;
            if(k+1 != primitives)
                v4 = map_.phi2(Bt.volume_control_[(nb_articulation-2)*primitives + k + 1]);
            else
                v4 = map_.phi2(Bt.volume_control_[(nb_articulation-2)*primitives]);
            mbuild.sew_volumes(v3,v4);
        }

                //save_pos_.push_back((branche.branch_size_ - 1)*primitives);
        controls_.push_back(Bt);
    }

    mbuild.close_map(); //reboucle les volumes en bord de map*/

    vertex_position_ = map_.add_attribute<Vec3, Vertex::ORBIT>("position");
    vertex_appartenance_ = map_.add_attribute<int, Vertex::ORBIT>("appartenance");
    vertex_normal_ = map_.add_attribute<Vec3, Vertex::ORBIT>("normal");

    //map_.foreach_cell([&] (Vertex v){ vertex_appartenance_[v] = 0;});


    // affectation d'un point du prisme triangulaire & du point en commun aux n-prismes (n = primitive)
    int ind_branch = 0;
    for(Branch branche : mon_squelette.branches_)
    {
        int nb_articulation = branche.pos_vertices_.size()/(primitives+1);
        unsigned int count = 0;
        unsigned int arti_count = 0;

        for(Dart d : controls_[ind_branch].volume_control_)
        {
            // il s'agit des positions des articulations
            if(count == 0 || count == primitives + 1 )
            {
                Vertex v1(map_.phi1(d));
                vertex_position_[v1] = branche.pos_vertices_[arti_count * (primitives + 1)];
                arti_count++;
                count = 1;
            }

            // ici on a la position des points autour de chaque articulation
            Vertex v2(d);
            vertex_position_[v2] = branche.pos_vertices_[(arti_count-1) * (primitives + 1) + count]; // le facteur est (arti_count - 1) car on a arti_count++ dans le if
            count++;
        }

        // On gère la dernière articulation

        Dart last_arti = map_.phi1(map_.phi1(map_.phi2(map_.phi1(controls_[ind_branch].volume_control_[controls_[ind_branch].volume_control_.size() - 1]))));

        vertex_position_[Vertex(last_arti)] = branche.pos_vertices_[(primitives+1)*(nb_articulation-1)];

        // Ici, on gere la derniere face (les points autour de la dernière articulation)
        for(unsigned int i = 0; i < primitives; i++)
        {
            Dart last_points = map_.phi1(map_.phi2(map_.phi1(map_.phi1(map_.phi2(controls_[ind_branch].volume_control_[controls_[ind_branch].volume_control_.size()-primitives + i])))));
            Vertex points(last_points);
            vertex_position_[points] = branche.pos_vertices_[controls_[ind_branch].volume_control_.size() + nb_articulation + i];
        }

        ind_branch++;
    }

    cgogn_message_assert(map_.topology_container().fragmentation() == 1.0f, "topo has holes");


    // On parcourt toute les intersection et on construit la topologie 2-Map puis 3-Map de chacune via MakeIntersection dans une map intermediaire. Puis on merge cette map à notre map3 finale
    // VRAI code
    // Fusion intersection avec map_

    int count = 0;
    for(Intersection inter : mon_squelette.intersections_)
    {
        MakeIntersection(inter.faces_, inter.contours_);
        Map3::DartMarker dm(map_);

        // Merge with Map2

        map_.merge(map2_, dm);
        Map3::Attribute<Vec3, Vertex32::ORBIT> pos2 = map_.get_attribute<Vec3, Vertex32::ORBIT>("position");
        map_.foreach_cell([&](Vertex v){
            vertex_position_[v] = pos2[Vertex32(v.dart)];
            vertex_appartenance_[v] = count + 1;
        }, [&] (Vertex v) {return dm.is_marked(v.dart);});


        //Merge with Map3inter
        /*
        Generate_tetgen("pq1.1a0.01Y", count);
        map_.merge(map3inter_, dm);
        map3inter_.clear_and_remove_attributes();*/

        map2_.clear_and_remove_attributes();

        count++;
    }


    //
    // TEST => On va faire le Dart Match making et les tests correspondants sur les deux MAP3 distinctes
    //
    //vertex_inter_position_ = map3inter_.get_attribute<Vec3, Vertex::ORBIT>("position");

    /*
    Intersection inter = mon_squelette.intersections_[0];
    MakeIntersection(inter.faces_, inter.contours_);
    Generate_tetgen("pq1.1a0.01Y", 0);
    vertex_inter_position_ = map3inter_.get_attribute<Vec3, Vertex::ORBIT>("position");//

    for(int i = 0; i < mon_squelette.branches_.size(); i++)
    {
        // Pour savoir si la branche arrive ou quitte l intersection car chaque branche à un départ et une arrivée
        int indic_arrivee = mon_squelette.ind_bout_arrive_[i];
        int indic_depart = mon_squelette.ind_bout_depart_[i];

        int count = 0;



            // Determinera si notre branche courant est liée à l'intersection courante
            bool inter_liee_arrivee = (indic_arrivee == inter.indicateur_);
            bool inter_liee_depart = (indic_depart == inter.indicateur_);

            if(inter_liee_arrivee || inter_liee_depart)
            {
                // On récupèrera un dart de bord de la branche courante.
                Dart d_res, d_branche, d_bord;
                if(inter_liee_arrivee)
                {
                    d_branche = map_.phi<2112>(controls_[i].volume_control_[controls_[i].volume_control_.size() - 1]);
                    d_bord = map_.phi1(d_branche);
                }
                if(inter_liee_depart)
                {
                    d_branche = map_.phi1(controls_[i].volume_control_[0]);
                    d_bord = map_.phi_1(d_branche);
                }

                // Trouver les vertices avec les mêmes coordonées (les plus proches en réalité)
                double diff_bord_ref = 100;
                double diff_centre_ref = 100;
                Vertex v_ref_centre, v_ref_bord;
                map3inter_.foreach_cell( [&](Vertex v){
                    Vec3 a = (vertex_position_[Vertex(d_branche)] - vertex_inter_position_[v]);
                    Vec3 b = (vertex_position_[Vertex(d_bord)] - vertex_inter_position_[v]);

                    if(a.squaredNorm() < diff_centre_ref)
                    {

                        diff_centre_ref = a.squaredNorm();
                        v_ref_centre = v;

                    }
                    if(b.squaredNorm() < diff_bord_ref)
                    {

                        diff_bord_ref = b.squaredNorm();
                        v_ref_bord = v;


                    }
                });

                std::cout << " dist ref : " << diff_bord_ref << std::endl;
                std::cout << " dist centre : " << diff_centre_ref << std::endl;


                map3inter_.foreach_dart_of_orbit(v_ref_bord, [&](Dart d)
                {
                    Dart d_intermediaire;
                    if(inter_liee_arrivee)
                        d_intermediaire = d;
                    else
                        d_intermediaire = map3inter_.phi1(d);
                    bool cond1 = (vertex_inter_position_[Vertex(map3inter_.phi1(d_intermediaire))] - vertex_position_[Vertex(d_branche)]).norm() < 0.0001;
                    bool cond2 = (vertex_inter_position_[Vertex(map3inter_.phi_1(d_intermediaire))] - vertex_position_[Vertex(map_.phi_1(d_branche))]).norm() < 0.0001;
                    bool cond3 = (vertex_inter_position_[Vertex(d_intermediaire)] - vertex_position_[Vertex(map_.phi1(d_branche))]).norm() < 0.0001;

                    if (cond1 && cond2 && cond3)
                    {
                        std::cout << "geometric found / phi3 boundary ? ";
                        std::cout << std::boolalpha << map3inter_.is_boundary(map3inter_.phi3(d_intermediaire)) << std::endl;
                        d_res = d_intermediaire;
                    }

                });

                std::cout << "nb darts map3inter : " << map3inter_.nb_darts() << std::endl;
                //map2_.clear_and_remove_attributes();
                //map_.merge(map3inter_);
                std::cout << "nb dart map : " << map_.nb_darts() << std::endl;
                //map3inter_.clear_and_remove_attributes();


            count++;
        }
    }*/

    //
    // TEST => idem avec MAP3 et la MAP2 si ça fait du sens
    //

    /*
    Intersection inter = mon_squelette.intersections_[0];
    MakeIntersection(inter.faces_, inter.contours_);

    for(int i = 0; i < mon_squelette.branches_.size(); i++)
    {
        // Pour savoir si la branche arrive ou quitte l intersection car chaque branche à un départ et une arrivée
        int indic_arrivee = mon_squelette.ind_bout_arrive_[i];
        int indic_depart = mon_squelette.ind_bout_depart_[i];

        // Determinera si notre branche courant est liée à l'intersection courante
        bool inter_liee_arrivee = (indic_arrivee == inter.indicateur_);
        bool inter_liee_depart = (indic_depart == inter.indicateur_);

        if(inter_liee_arrivee || inter_liee_depart)
        {
            // On récupèrera un dart de bord de la branche courante.
            Dart d_res, d_branche, d_bord;
            if(inter_liee_arrivee)
            {
                d_branche = map_.phi<2112>(controls_[i].volume_control_[controls_[i].volume_control_.size() - 1]);
                d_bord = map_.phi1(d_branche);
            }
            if(inter_liee_depart)
            {
                d_branche = map_.phi1(controls_[i].volume_control_[0]);
                d_bord = map_.phi_1(d_branche);
            }

            // Trouver les vertices avec les mêmes coordonées (les plus proches en réalité)
            double diff_bord_ref = 10000;
            Vertex2 v_ref_bord;
            map2_.foreach_cell( [&](Vertex2 v){
                Vec3 b = (vertex_position_[Vertex(d_bord)] - vertex2_position_[v]);
                if(b.squaredNorm() < diff_bord_ref)
                {
                    diff_bord_ref = b.squaredNorm();
                    v_ref_bord = v;

                }
            });
            std::cout << " dist ref : " << diff_bord_ref << std::endl;

            map2_.foreach_dart_of_orbit(v_ref_bord, [&](Dart d)
            {
                Dart d_intermediaire;
                if(inter_liee_arrivee)
                    d_intermediaire = d;
                else
                    d_intermediaire = map2_.phi1(d);

                bool cond1 = (vertex2_position_[Vertex2(map2_.phi1(d_intermediaire))] - vertex_position_[Vertex(d_branche)]).norm() < 0.0001;
                bool cond2 = (vertex2_position_[Vertex2(map2_.phi_1(d_intermediaire))] - vertex_position_[Vertex(map_.phi_1(d_branche))]).norm() < 0.0001;
                bool cond3 = (vertex2_position_[Vertex2(d_intermediaire)] - vertex_position_[Vertex(map_.phi1(d_branche))]).norm() < 0.0001;

                if(cond1 && cond2 && cond3)
                {
                    std::cout << "geometric found" << std::endl;
                    d_res = d_intermediaire;
                }

            });
        }
    }*/

    map_.check_map_integrity();


    map_.foreach_cell([&] (Vertex v){ std::cout<<vertex_appartenance_[v]<<std::endl;}, [&] (Vertex v){ return false;} );


    /*
    int dart_count = 0;
    int bad_dart = 0;
    map_.foreach_cell([&] (Vertex v){
        map_.foreach_dart_of_orbit(v,[&](Dart d) {
            if(!map_.is_boundary(map_.phi3((d))))
            {
                std::cout<<" le dart n'est pas incident au bord "<<std::endl;
                bad_dart++;
            }
            dart_count++;
        });
    }, [&] (Vertex v){ return (vertex_appartenance_[v] == 1);});

    // Nb de Dart testé/2 == nb de dart mauvais (normalement)
    std::cout << "nb de darts testé (avec redondance)" << dart_count << std::endl;
    std::cout << "nb de darts mauvais (avec redondance)" << bad_dart << std::endl;*/


    //
    // TEST : Coudre deux branches entre elles
    //
    /*
    for(int x = 0; x < TYPE_PRIMITIVE; x++)
    {
        Dart d_end = map_.phi<2112>(controls_[0].volume_control_[controls_[0].volume_control_.size() - TYPE_PRIMITIVE + x]);
        Dart d_start =  controls_[1].volume_control_[x];
        map_.sew_volumes(Face(d_end), Face(d_start));
    }*/


    //
    // TEST : Coudre un unique prisme à un bout de branche
    //
    /*
    // make topo
    MapBuilder mbuildbis(map3inter_);
    Dart d = mbuildbis.add_prism_topo(3);
    mbuildbis.close_map();
    // init attributes
    vertex_position_bis_ = map3inter_.add_attribute<Vec3, Vertex::ORBIT>("position");
    vertex_appartenance_bis_ = map3inter_.add_attribute<int, Vertex::ORBIT>("appartenance");
    vertex_normal_bis_ = map3inter_.add_attribute<Vec3, Vertex::ORBIT>("normal");
    Vec3 a1 = {0, 0, 0};
    Vec3 a2 = {0, 1, 0};
    Vec3 a3 = {1, 1, 0};
    Vec3 b1 = {0, 0, 1};
    Vec3 b2 = {0, 1, 1};
    Vec3 b3 = {1, 0, 1};
    vertex_position_bis_[Vertex(d)] = a1;
    vertex_position_bis_[Vertex(map3inter_.phi1(d))] = a2;
    vertex_position_bis_[Vertex(map3inter_.phi_1(d))] = a3;
    vertex_position_bis_[Vertex(map3inter_.phi<2112>(d))] = b1;
    vertex_position_bis_[Vertex(map3inter_.phi<211211>(d))] = b2;
    vertex_position_bis_[Vertex(map3inter_.phi<21121>(d))] = b3;
    vertex_appartenance_bis_[Vertex(d)] = 1;
    vertex_appartenance_bis_[Vertex(map3inter_.phi1(d))] = 1;
    vertex_appartenance_bis_[Vertex(map3inter_.phi_1(d))] = 1;
    vertex_appartenance_bis_[Vertex(map3inter_.phi<2112>(d))] = 1;
    vertex_appartenance_bis_[Vertex(map3inter_.phi<211211>(d))] = 1;
    vertex_appartenance_bis_[Vertex(map3inter_.phi<21121>(d))] = 1;

    cgogn_assert(map3inter_.check_map_integrity());
    cgogn_assert(map_.check_map_integrity());

    map_.merge(map3inter_);

    map_.check_map_integrity();


    //vertex_appartenance_ = map_.get_attribute<Vec3, Vertex::ORBIT>("position");

    //trouver les darts correspondants à coudre
    double dist_ref = 100000;
    Vertex v_ref;

    Dart d_first = map_.phi<2112>(controls_[0].volume_control_[controls_[0].volume_control_.size() - 1]); // Dart de bout de la dernière branche
    map_.foreach_cell( [&](Vertex v){
        Vec3 dist = vertex_position_[v] - vertex_position_[Vertex(d_first)];
        if(dist.norm() < dist_ref)
        {
            dist_ref = dist.norm();
            v_ref = v;
        }

    }, [&] (Vertex v) {
        if(vertex_appartenance_[v] == 1)
            return true;
        else
            return false;
    });

    Dart d_res;
    map_.foreach_dart_of_orbit(v_ref, [&] (Dart d) {

        if(map_.phi<111>(d) == d)
            d_res = d;

    });

    map_.sew_volumes(Face(d_first), Face(d_res));*/


    //
    // CODE EN COURS
    //
    /*
    int counting = 0;
    // On va tester chaque branche avec chaque intersection
    for(int i = 0; i < mon_squelette.branches_.size(); i++)
    {
        // Pour savoir si la branche arrive ou quitte l intersection car chaque branche à un départ et une arrivée
        int indic_arrivee = mon_squelette.ind_bout_arrive_[i];
        int indic_depart = mon_squelette.ind_bout_depart_[i];

        int increment = 1;
        for(Intersection inter : mon_squelette.intersections_)
        {

            // Determinera si notre branche courant est liée à l'intersection courante
            bool inter_liee_arrivee = (indic_arrivee == inter.indicateur_);
            bool inter_liee_depart = (indic_depart == inter.indicateur_);

            if(inter_liee_arrivee || inter_liee_depart)
            {
                // On récupèrera un dart de bord de la branche courante.
                Dart d_res, d_branche, d_bord;
                if(inter_liee_arrivee)
                {
                    d_branche = map_.phi<2112>(controls_[i].volume_control_[controls_[i].volume_control_.size() - 1]);
                    d_bord = map_.phi1(d_branche);
                }
                if(inter_liee_depart)
                {
                    d_branche = map_.phi1(controls_[i].volume_control_[0]);
                    d_bord = map_.phi_1(d_branche);
                }

                cgogn_message_assert(map_.is_boundary(map_.phi3(d_bord)), "pas boundary");
                cgogn_message_assert(map_.is_boundary(map_.phi3(d_branche)), "pas boundary");

                // Trouver les vertices avec les mêmes coordonées (les plus proches en réalité)
                double diff_bord_ref = 100;
                double diff_centre_ref = 100;
                Vertex v_ref_centre, v_ref_bord;
                map_.foreach_cell( [&](Vertex v){
                    Vec3 a = (vertex_position_[Vertex(d_branche)] - vertex_position_[v]);
                    Vec3 b = (vertex_position_[Vertex(d_bord)] - vertex_position_[v]);
                    if(a.squaredNorm() < diff_centre_ref)
                    {
                        diff_centre_ref = a.squaredNorm();
                        v_ref_centre = v;
                    }
                    if(b.squaredNorm() < diff_bord_ref)
                    {
                        diff_bord_ref = b.squaredNorm();
                        v_ref_bord = v;
                    }
                }, [&](Vertex v) {return (vertex_appartenance_[v] == 1); });

                std::cout << "point du bord trouvé : " << diff_bord_ref << std::endl;
                std::cout << "point du centre trouvé : " << diff_centre_ref << std::endl;
                cgogn_message_assert(!map_.same_orbit(v_ref_centre, Vertex(d_branche)), "v_ref_centre sur la branche");
                cgogn_message_assert(!map_.same_orbit(v_ref_bord, Vertex(d_bord)), "v_ref_bord sur la branche");
                cgogn_message_assert(vertex_appartenance_[v_ref_centre] == 1, "v_ref_centre pas sur intersection");
                cgogn_message_assert(vertex_appartenance_[v_ref_bord] == 1, "v_ref_bord pas sur intersection");

                std::cout << vertex_position_[Vertex(d_branche)][0] << "," << vertex_position_[Vertex(d_branche)][1] << "," << vertex_position_[Vertex(d_branche)][2] << " / ";
                std::cout << vertex_position_[Vertex(map_.phi_1(d_branche))][0] << "," << vertex_position_[Vertex(map_.phi_1(d_branche))][1] << "," << vertex_position_[Vertex(map_.phi_1(d_branche))][2] << " / ";
                std::cout << vertex_position_[Vertex(map_.phi1(d_branche))][0] << "," << vertex_position_[Vertex(map_.phi1(d_branche))][1] << "," << vertex_position_[Vertex(map_.phi1(d_branche))][2] << std::endl;

                map_.foreach_dart_of_orbit(v_ref_bord, [&](Dart d)
                {
                    Dart d_intermediaire;
                    bool cond_dart = false;
                    if(inter_liee_arrivee)
                    {
                        d_intermediaire = d;
                        cond_dart = map_.same_orbit(v_ref_centre, Vertex(map_.phi1(d_intermediaire)));
                    }
                    else
                    {
                        d_intermediaire = map_.phi1(d);
                        cond_dart = map_.same_orbit(v_ref_centre, Vertex(d_intermediaire));
                    }

                    std::cout << vertex_position_[Vertex(d_intermediaire)][0] << "," << vertex_position_[Vertex(d_intermediaire)][1] << "," << vertex_position_[Vertex(d_intermediaire)][2] << " / ";
                    std::cout << vertex_position_[Vertex(map_.phi_1(d_intermediaire))][0] << "," << vertex_position_[Vertex(map_.phi_1(d_intermediaire))][1] << "," << vertex_position_[Vertex(map_.phi_1(d_intermediaire))][2] << " / ";
                    std::cout << vertex_position_[Vertex(map_.phi1(d_intermediaire))][0] << "," << vertex_position_[Vertex(map_.phi1(d_intermediaire))][1] << "," << vertex_position_[Vertex(map_.phi1(d_intermediaire))][2] << std::endl;

                    bool cond1 = (vertex_position_[Vertex(map_.phi1(d_intermediaire))] - vertex_position_[Vertex(d_branche)]).norm() < 0.0001;
                    bool cond2 = (vertex_position_[Vertex(map_.phi_1(d_intermediaire))] - vertex_position_[Vertex(map_.phi_1(d_branche))]).norm() < 0.0001;
                    bool cond3 = (vertex_position_[Vertex(d_intermediaire)] - vertex_position_[Vertex(map_.phi1(d_branche))]).norm() < 0.0001;

                    if (cond1 && cond2 && cond3)
                    {
                        std::cout << "geometric found / phi3 boundary ? ";
                        std::cout << std::boolalpha << map_.is_boundary(map_.phi3(d_intermediaire)) << std::endl;
                        d_res = d_intermediaire;
                    }
                });

                cgogn_message_assert(!d_res.is_nil(), "d_res pas trouvé");




//                for(int w = 0; w < TYPE_PRIMITIVE - 1; w++)
//                {
//                    map_.sew_volumes(Face(d_res), Face(d_branche));

//                    d_res = map_.phi2(d_res);
//                    do
//                    {
//                        d_res = map_.phi<32>(d_res);
//                    }while(!map_.is_boundary(map_.phi3(d_res)));

//                    d_branche = map_.phi<232>(d_branche);
//                    d_res = map_.phi_1(d_res);
//                    d_branche = map_.phi1(d_branche);
//                }

                // IMPORTANT
                map_.sew_volumes(Face(d_branche),Face(d_res));

            }
        }
    }*/


    //
    // TEST : Sew Volume sans tétrahèdriser
    //

    int counting = 0;
    // On va tester chaque branche avec chaque intersection
    for(int i = 0; i < mon_squelette.branches_.size(); i++)
    {
        // Pour savoir si la branche arrive ou quitte l intersection car chaque branche à un départ et une arrivée
        int indic_arrivee = mon_squelette.ind_bout_arrive_[i];
        int indic_depart = mon_squelette.ind_bout_depart_[i];

        int increment = 1;
        for(Intersection inter : mon_squelette.intersections_)
        {

            // Determinera si notre branche courant est liée à l'intersection courante
            bool inter_liee_arrivee = (indic_arrivee == inter.indicateur_);
            bool inter_liee_depart = (indic_depart == inter.indicateur_);

            if(inter_liee_arrivee || inter_liee_depart)
            {
                // On récupèrera un dart de bord de la branche courante.
                Dart d_res, d_branche, d_bord;
                if(inter_liee_arrivee)
                {
                    d_branche = map_.phi<2112>(controls_[i].volume_control_[controls_[i].volume_control_.size() - 1]);
                    d_bord = map_.phi1(d_branche);
                }
                if(inter_liee_depart)
                {
                    d_branche = map_.phi1(controls_[i].volume_control_[0]);
                    d_bord = map_.phi_1(d_branche);
                }

                // Trouver les vertices avec les mêmes coordonées (les plus proches en réalité)
                double diff_bord_ref = 100;
                double diff_centre_ref = 100;
                Vertex v_ref_centre, v_ref_bord;
                map_.foreach_cell( [&](Vertex v){
                    Vec3 a = (vertex_position_[Vertex(d_branche)] - vertex_position_[v]);
                    Vec3 b = (vertex_position_[Vertex(d_bord)] - vertex_position_[v]);
                    if(a.squaredNorm() < diff_centre_ref)
                    {
                        diff_centre_ref = a.squaredNorm();
                        v_ref_centre = v;
                    }
                    if(b.squaredNorm() < diff_bord_ref)
                    {
                        diff_bord_ref = b.squaredNorm();
                        v_ref_bord = v;
                    }
                }, [&](Vertex v) {return (vertex_appartenance_[v] == 1); });

                std::cout << "point du bord trouvé : " << diff_bord_ref << std::endl;
                std::cout << "point du centre trouvé : " << diff_centre_ref << std::endl;

                std::cout << vertex_position_[Vertex(d_branche)][0] << "," << vertex_position_[Vertex(d_branche)][1] << "," << vertex_position_[Vertex(d_branche)][2] << " / ";
                std::cout << vertex_position_[Vertex(map_.phi_1(d_branche))][0] << "," << vertex_position_[Vertex(map_.phi_1(d_branche))][1] << "," << vertex_position_[Vertex(map_.phi_1(d_branche))][2] << " / ";
                std::cout << vertex_position_[Vertex(map_.phi1(d_branche))][0] << "," << vertex_position_[Vertex(map_.phi1(d_branche))][1] << "," << vertex_position_[Vertex(map_.phi1(d_branche))][2] << std::endl;

                map_.foreach_dart_of_orbit(v_ref_bord, [&](Dart d)
                {
                    Dart d_intermediaire;
                    bool cond_dart = false;
                    if(inter_liee_arrivee)
                    {
                        d_intermediaire = d;
                        cond_dart = map_.same_orbit(v_ref_centre, Vertex(map_.phi1(d_intermediaire)));
                    }
                    else
                    {
                        d_intermediaire = map_.phi1(d);
                        cond_dart = map_.same_orbit(v_ref_centre, Vertex(d_intermediaire));
                    }

                    std::cout << vertex_position_[Vertex(d_intermediaire)][0] << "," << vertex_position_[Vertex(d_intermediaire)][1] << "," << vertex_position_[Vertex(d_intermediaire)][2] << " / ";
                    std::cout << vertex_position_[Vertex(map_.phi_1(d_intermediaire))][0] << "," << vertex_position_[Vertex(map_.phi_1(d_intermediaire))][1] << "," << vertex_position_[Vertex(map_.phi_1(d_intermediaire))][2] << " / ";
                    std::cout << vertex_position_[Vertex(map_.phi1(d_intermediaire))][0] << "," << vertex_position_[Vertex(map_.phi1(d_intermediaire))][1] << "," << vertex_position_[Vertex(map_.phi1(d_intermediaire))][2] << std::endl;

                    bool cond1 = (vertex_position_[Vertex(map_.phi1(d_intermediaire))] - vertex_position_[Vertex(d_branche)]).norm() < 0.0001;
                    bool cond2 = (vertex_position_[Vertex(map_.phi_1(d_intermediaire))] - vertex_position_[Vertex(map_.phi_1(d_branche))]).norm() < 0.0001;
                    bool cond3 = (vertex_position_[Vertex(d_intermediaire)] - vertex_position_[Vertex(map_.phi1(d_branche))]).norm() < 0.0001;

                    if (cond1 && cond2 && cond3 && map_.is_boundary(map_.phi3(d_intermediaire)))
                    {
                        std::cout << "geometric found && phi3 boundary " <<std::endl;
                        std::cout << std::boolalpha << map_.is_boundary(map_.phi3(d_intermediaire)) << std::endl;
                        d_res = d_intermediaire;//
                    }
                });

                if(map_.is_boundary(map_.phi3(d_res)))
                    std::cout << "on est sur le bord véritable" << std::endl;
                std::cout << "Dart d'intersection' numero : " << d_res << std::endl;
                std::cout << "Dart de branche numero : " << d_branche << std::endl;

                for(int w = 0; w < TYPE_PRIMITIVE - 1; w++)
                {
                    map_.sew_volumes(Face(d_res), Face(d_branche));
                    d_branche = map_.phi<2321>(d_branche);
                    d_res = map_.phi_1(map_.phi2(d_res));
                }
                map_.sew_volumes(Face(d_res),Face(d_branche));
            }
        }
    }

    vertex_position_ = map_.get_attribute<Vec3, Vertex::ORBIT>("position");

    map_.check_map_integrity();
}

void topo::InterpolationConcentrique()
{

    if(nb_appuis_ > 0)
    {

        for(BranchTopo Bt : controls_)
        {

            // ce terme sert à répartit les points le long d'un segment bord-centre pour chaque volumes
            //double Terme_repartition = exp(indice_repartition_*0.02) - 1;
            //double Terme_repartition = (1 + tanh(indice_repartition_*0.2))*10 - 1;

            //std::cout << "Terme repartition" << Terme_repartition << std::endl;


            //
            //
            // Subdivision concentrique (Ur)
            //
            //

            //
            // Subdivision concentrique sur tout les volumes sauf ceux de la dernière articulation
            //

            for(Dart d : Bt.volume_control_)
            {
                Dart d_centre = d; // d_centre sera modifié
                Dart d_bord = d; // on commence au bord, il s'agit bien de notre Dart de bord final

                //std::vector<Dart> volumes_concentriques; // On va y lister tout les volumes concentrique du prisme à base triangulaire courant (désigné par "d")
                //volumes_concentriques.push_back(d); // le premier volume concentrique partage le même dart que le prisme avant la subdiv concentrique càd "d"

                // Objectif => trouver le centre
                for(int k = 0; k < nb_appuis_; k++)
                {
                    d_centre = map_.phi<12321>(d_centre); // déplacement sur chaque nouveau volume ajouté par subdivision concentrique
                }
                // Le dernier volume que l'on trouve en se propageant vers le centre est un prisme à base triangulaire. Phi1 nous donne le centre
                d_centre = map_.phi1(d_centre);


                // Objectif => affecter les bonnes coordonées aux point du segment bord-centre, pour chaque volume
                for(int i = 1; i < nb_appuis_ + 1; i++)
                {

                    double Terme_repartition = exp(indice_repartition_*0.08) + (nb_appuis_ + 1) - i;


                    // on ne veut pas tronquer coeff (division d'entiers euclidienne pas correcte ici), d'où les casts double
                    double coeff;
                    double m = double(i);
                    double n = double(nb_appuis_ + 1);
                    coeff = m/(n + Terme_repartition);

                    d = map_.phi<12321>(d); // déplacement sur chaque nouveau volume ajouté par subdivision concentrique
                    vertex_position_[Vertex(d)] = coeff*vertex_position_[Vertex(d_centre)] + (1-coeff)*vertex_position_[Vertex(d_bord)];

                }
            }


            //
            // Gestion de la dernière articulation pour la subdivision concentrique
            //

            for(int i = 0; i < TYPE_PRIMITIVE; i++)
            {

                Dart d = map_.phi<211>(Bt.volume_control_[Bt.volume_control_.size() - TYPE_PRIMITIVE + i]); // nous place sur un vertex de bord, de la dernière face
                Dart d_bord = d; // On se situe maintenant sur le bord d'où cette affectation
                Dart d_centre = d; // le recalculer dans la boucle peut être inutile pour cette dernière face vu que ce sera toujours le même Vertex

                // Objectif => trouver le centre
                for(int k = 0; k < nb_appuis_; k++)
                {
                    d_centre = map_.phi<1213121>(d_centre); // déplacement sur chaque nouveau volume ajouté par subdivision concentrique
                }
                d_centre = map_.phi1(d_centre);

                // Objectif => affecter les bonnes coordonées aux point du segment bord-centre, pour chaque volume de la dernière articulation
                for(int i = 1; i < nb_appuis_ + 1; i++)
                {
                    double Terme_repartition = exp(indice_repartition_*0.08) + (nb_appuis_ + 1) - i;


                    // on ne veut pas tronquer coeff (division d'entiers euclidienne pas correcte ici), d'où les casts double
                    double coeff;
                    double m = double(i);
                    double n = double(nb_appuis_ + 1);

                    coeff = m/(n + Terme_repartition); // poid qu'on associera à un bout du segment (point commun à tt les volumes de l'articulation => centre)

                    d = map_.phi<1213121>(d); // déplacement sur chaque nouveau volume ajouté par subdivision concentrique
                    vertex_position_[Vertex(d)] = coeff*vertex_position_[Vertex(d_centre)] + (1-coeff)*vertex_position_[Vertex(d_bord)];

                }

            }
        }

         /*
            // si pas d'appuis sur "C" alors la méthode est inutile car il n'y a pas eu de subdivision
            if(nb_appuis_ > 0)
            {

                // ce terme sert à répartit les points le long d'un segment bord-centre pour chaque volumes
                //double Terme_repartition = exp(indice_repartition_*0.02) - 1;
                //double Terme_repartition = (1 + tanh(indice_repartition_*0.2))*10 - 1;
                //std::cout << "Terme repartition" << Terme_repartition << std::endl;


                //
                //
                // Subdivision concentrique (Ur)
                //
                //

                //
                // Subdivision concentrique sur tout les volumes sauf ceux de la dernière articulation
                //
                int Nb_branch = 0;
                int increC = save_pos_[Nb_branch];
                int counter = 0;

                for(Dart d : Bt.volume_control_)
                {

                    if(counter == increC)
                    {

                        //
                        // Gestion de la dernière articulation pour la subdivision concentrique
                        //

                        for(int i = 0; i < TYPE_PRIMITIVE; i++)
                        {

                            Dart d = map_.phi<211>(volume_control_[increC - TYPE_PRIMITIVE + i]); // nous place sur un vertex de bord, de la dernière face
                            Dart d_bord = d; // On se situe maintenant sur le bord d'où cette affectation
                            Dart d_centre = d; // le recalculer dans la boucle peut être inutile pour cette dernière face vu que ce sera toujours le même Vertex

                            // Objectif => trouver le centre
                            for(int k = 0; k < nb_appuis_; k++)
                            {
                                d_centre = map_.phi<1213121>(d_centre); // déplacement sur chaque nouveau volume ajouté par subdivision concentrique
                            }
                            d_centre = map_.phi1(d_centre);

                            // Objectif => affecter les bonnes coordonées aux point du segment bord-centre, pour chaque volume de la dernière articulation
                            for(int i = 1; i < nb_appuis_ + 1; i++)
                            {
                                double Terme_repartition = exp(indice_repartition_*0.08) + (nb_appuis_ + 1) - i;

                                // on ne veut pas tronquer coeff (division d'entiers euclidienne pas correcte ici), d'où les casts double
                                double coeff;
                                double m = double(i);
                                double n = double(nb_appuis_ + 1);

                                coeff = m/(n + Terme_repartition); // poid qu'on associera à un bout du segment (point commun à tt les volumes de l'articulation => centre)

                                d = map_.phi<1213121>(d); // déplacement sur chaque nouveau volume ajouté par subdivision concentrique
                                vertex_position_[Vertex(d)] = coeff*vertex_position_[Vertex(d_centre)] + (1-coeff)*vertex_position_[Vertex(d_bord)];

                            }

                        }

                        increC += save_pos_[Nb_branch++];

                    }

                    Dart d_centre = d; // d_centre sera modifié
                    Dart d_bord = d; // on commence au bord, il s'agit bien de notre Dart de bord final

                    //std::vector<Dart> volumes_concentriques; // On va y lister tout les volumes concentrique du prisme à base triangulaire courant (désigné par "d")
                    //volumes_concentriques.push_back(d); // le premier volume concentrique partage le même dart que le prisme avant la subdiv concentrique càd "d"

                    // Objectif => trouver le centre
                    for(int k = 0; k < nb_appuis_; k++)
                    {
                        d_centre = map_.phi<12321>(d_centre); // déplacement sur chaque nouveau volume ajouté par subdivision concentrique
                    }
                    // Le dernier volume que l'on trouve en se propageant vers le centre est un prisme à base triangulaire. Phi1 nous donne le centre
                    d_centre = map_.phi1(d_centre);


                    // Objectif => affecter les bonnes coordonées aux point du segment bord-centre, pour chaque volume
                    for(int i = 1; i < nb_appuis_ + 1; i++)
                    {

                        double Terme_repartition = exp(indice_repartition_*0.08) + (nb_appuis_ + 1) - i;

                        // on ne veut pas tronquer coeff (division d'entiers euclidienne pas correcte ici), d'où les casts double
                        double coeff;
                        double m = double(i);
                        double n = double(nb_appuis_ + 1);
                        coeff = m/(n + Terme_repartition);

                        d = map_.phi<12321>(d); // déplacement sur chaque nouveau volume ajouté par subdivision concentrique
                        vertex_position_[Vertex(d)] = coeff*vertex_position_[Vertex(d_centre)] + (1-coeff)*vertex_position_[Vertex(d_bord)];


                    }
                    counter++;
                }
            }*/
    }
}

void topo::GetCouchesConcentriques()
{
    int counter = 0;
    for(BranchTopo Bt : controls_)
    {
        Bt.subdivised_volume_control_.clear();

        for(Dart d : Bt.volume_control_)
        {
            for(int k = 0; k < nb_appuis_ + 1; k++ )
            {
                CoucheConcentrique couche_courante(k,{d});
                Bt.subdivised_volume_control_.push_back(couche_courante);
                d = map_.phi<12321>(d);
            }

        }

        controls_[counter] = Bt;
        counter++;
    }

}

void topo::UpdateCoordinates()
{
    // Cette méthode servira à recalculer les positions des points ajouté par subdivision Utheta
    // On y accèdera facilement grâce aux coucheconcentrique.ind_volumes[]
    // On stockera pas dans chaque couche un attribut d_opp puisqu'on utilisera coucheconcentrique.ind_volumes[0] de la couche suivante

    //
    // Tous les volumes sauf le dernier
    int counter = 0;
    for(BranchTopo Bt : controls_)
    {
        for(unsigned int i = 0; i < Bt.subdivised_volume_control_.size(); i++)
        {
            CoucheConcentrique couche_courante = Bt.subdivised_volume_control_[i];

            Dart extremite_gauche = couche_courante.indic_volumes_[0];

            Dart extremite_droite;
            if(((i)%(TYPE_PRIMITIVE*(nb_appuis_ + 1))) + (nb_appuis_ + 1) > ((nb_appuis_ + 1)*TYPE_PRIMITIVE) - 1) // gros doute pour le "- 1"
                extremite_droite = Bt.subdivised_volume_control_[i + nb_appuis_ + 1 - TYPE_PRIMITIVE*(nb_appuis_ + 1) ].indic_volumes_[0];
            else
                extremite_droite = Bt.subdivised_volume_control_[i + nb_appuis_ + 1].indic_volumes_[0]; // penser à la seg. fault pour le dernier volume


            for(std::vector<Dart>::iterator it = couche_courante.indic_volumes_.begin(); it < couche_courante.indic_volumes_.end(); ++it)
            {
                int index = it - (couche_courante.indic_volumes_.begin());
                if(index > 0)
                {
                    double coeff = double(index)/double(couche_courante.indic_volumes_.size());
                    vertex_position_[Vertex(*it)] = (coeff)*vertex_position_[Vertex(extremite_droite)] + (1-coeff)*vertex_position_[Vertex(extremite_gauche)];

                }
            }
        }


        //
        // gérer le dernier volume

        for(unsigned int i = 0; i < TYPE_PRIMITIVE*(nb_appuis_ + 1); i++)
        {

            CoucheConcentrique couche_courante = Bt.subdivised_volume_control_[Bt.subdivised_volume_control_.size() - TYPE_PRIMITIVE*(nb_appuis_ + 1) + i];

            Dart extremite_gauche = map_.phi<211>(couche_courante.indic_volumes_[0]);

            Dart extremite_droite;
            if((i + (nb_appuis_ + 1)) > ((nb_appuis_ + 1)*TYPE_PRIMITIVE) - 1)
                extremite_droite = map_.phi<211>(Bt.subdivised_volume_control_[Bt.subdivised_volume_control_.size() - 2*(nb_appuis_ + 1)*TYPE_PRIMITIVE + i + nb_appuis_ + 1 ].indic_volumes_[0]);
            else
                extremite_droite = map_.phi<211>(Bt.subdivised_volume_control_[Bt.subdivised_volume_control_.size() - (nb_appuis_ + 1)*TYPE_PRIMITIVE + i + nb_appuis_ + 1].indic_volumes_[0]); // penser à la seg. fault pour le dernier volume


            for(std::vector<Dart>::iterator it = couche_courante.indic_volumes_.begin(); it < couche_courante.indic_volumes_.end(); ++it)
            {

                int index = it - (couche_courante.indic_volumes_.begin());
                if(index > 0)
                {
                    double coeff = double(index)/double(couche_courante.indic_volumes_.size());
                    vertex_position_[Vertex(map_.phi<211>(*it))] = (coeff)*vertex_position_[Vertex(extremite_droite)] + (1-coeff)*vertex_position_[Vertex(extremite_gauche)];

                }
            }
        }
        controls_[counter] = Bt;
        counter++;
    }
}

void topo::SubdivisionCouche(const unsigned int& Nb_subdiv) // mettre un paramètre pourrait nous indiquer de combien subdiviser => boucle for{} pour la subdivision (2^N * Nb_subdiv)
{
    int count = 0;
    for(BranchTopo Bt : controls_)
    {

        //
        //
        // Creation des sommets
        //
        //


        //
        // Pour la face d'entrée de tous les volumes
        //

        unsigned int j = 0;
        for(CoucheConcentrique couche_courante : Bt.subdivised_volume_control_)
        {

            //
            // Calcul du nombre de volume que l'on doit avoir à la fin

            int Volumes_init = (couche_courante.indic_volumes_.size()-1);

            // variable numérotant les couches du centre vers le bord
            double diff = nb_appuis_ - couche_courante.etage_;

            // La fonction en o(2^N) => ici N^2
            double num;

            if(couche_courante.etage_ > 0)
                num = diff + 1.0;
            else
                num = (diff - 1) + 1.0;

            int Nb_vol = pow(2, int(log2(num)) + 1 )*Nb_subdiv - 1;

            if(Nb_vol == 0)
                std::cout<< "num est pas bon car Nb_vol ne doit janais valoir 0 : " << num << std::endl;

            //std::cout << "Nombre courant : " << Nb_vol << std::endl;

    /*
            if( couche_courante.etage_ > 0)
                Nb_vol = pow(2, nb_appuis_ - couche_courante.etage_ + 1 )*Nb_subdiv - 1 ;
            else
                Nb_vol = pow(2,nb_appuis_ )*Nb_subdiv - 1; // en effet, cette arête doit être coupée autant de fois que sa précédente du point de vue concentrique

    */




            //
            // Création de tous les sommets

            Dart d_courant = couche_courante.indic_volumes_[0];
            Dart d_opp = map_.phi_1(d_courant); // à inclure dans une boucle for pour parcourir sizeof(indic_volume) et faire map_.phi_1 ect.. à chaque itération
            // On en profitera pour supprimer des volumes si besoin && pour déplacer les vertices grâce à l'information de Nb_Volume

            for(unsigned int i = 1; i < Nb_vol + 1; i++)
            {
                // Calcul du coeff
                double ind = double(i);
                double max = double(Nb_vol + 1);
                double coeff = ind/max;

                // Creation du point
                Vertex v = map_.cut_edge(Edge(d_opp));

                couche_courante.indic_volumes_.insert(couche_courante.indic_volumes_.begin() + i, v.dart); // l'insertion détruit mon itérateur couche_courante?

                // Calcul des coordonées du point => interpolation linéaire
                vertex_position_[v] = vertex_position_[Vertex(d_courant)]*(1-coeff) + vertex_position_[Vertex(d_opp)]*coeff;

            }
            Bt.subdivised_volume_control_[j] = couche_courante;
            j++;
        }


        //
        // Pour la face de sortie des derniers volumes
        //

        for(unsigned int k = 0; k < TYPE_PRIMITIVE*(nb_appuis_ + 1); k++)
        {
            CoucheConcentrique couche_actu = Bt.subdivised_volume_control_[Bt.subdivised_volume_control_.size() - TYPE_PRIMITIVE*(nb_appuis_ + 1) + k];
            Dart d_bottom = couche_actu.indic_volumes_[0];
            d_bottom = map_.phi<211>(d_bottom);
            Dart d_opp = map_.phi<212>(d_bottom); // devrait se trouver dans une boucle for : phi1 à itérer sizeof(indic_volume) fois

            //
            // Calcul du nombre de volume que l'on doit avoir à la fin

            // variable numérotant les couches du centre vers le bord
            double diff = nb_appuis_ - couche_actu.etage_;

            // La fonction en o(2^N) => ici N^2
            double num = 0.5;

            if(couche_actu.etage_ > 0)
                num = diff + 1.0;
            else
                num = (diff - 1) + 1.0;

            int Nb_vol = pow(2, int(log2(num) + 1))*Nb_subdiv - 1;

            if(Nb_vol == 0)
                std::cout<< "num est pas bon car Nb_vol ne doit janais valoir 0 : " << num << std::endl;

    /*
            //int Nb_vol;
            if( couche_actu.etage_ > 0)
                Nb_vol = pow(2, nb_appuis_ - couche_actu.etage_  + 1)*Nb_subdiv - 1;
            else
                Nb_vol = pow(2,nb_appuis_ )*Nb_subdiv - 1; // en effet, cette arête doit être coupée autant de fois que sa précédente du point de vue concentrique
    */

            //
            // Création de tous les sommets

            for(unsigned int i = 1; i < Nb_vol + 1; i++)
            {
                // Calcul du coeff
                double ind = double(i);
                double max = double(Nb_vol + 1);
                double coeff = ind/max;

                // Creation du point
                Vertex v = map_.cut_edge(Edge(d_opp));

                // Calcul des coordonées du point => interpolation linéaire
                vertex_position_[v] = vertex_position_[Vertex(d_bottom)]*(1-coeff) + vertex_position_[Vertex(d_opp)]*coeff;
            }
        }




        //
        //
        // Creation des arêtes
        //
        //


        //
        // On parcourt toute les couches sauf celles de la dernière articulation
        //

        for(std::vector<CoucheConcentrique>::iterator it = Bt.subdivised_volume_control_.begin(); it < Bt.subdivised_volume_control_.end() - (nb_appuis_ + 1)*TYPE_PRIMITIVE; ++it)
        {

            //
            // indice courant

            // Condition sur l'étage => être au dernier étage ne présente pas d'intérêt
            if((*it).etage_ < nb_appuis_) //il se pourrait que si => revoir stratégie de création
            {
                //
                // Couches voisines

                CoucheConcentrique Couche_suivante_interieure = *(it + 1); //subdivised_volume_control_[index + 1];
                // Condition sur l'articulation => être en bout de branche n'est pas intéressant => cas à traiter à part pour les arêtes de fin => on est sur de la valider => à supprimer
                CoucheConcentrique Couche_suivante_inferieure = *(it + (nb_appuis_ + 1)*TYPE_PRIMITIVE); //subdivised_volume_control_[index + (nb_appuis_ + 1)*TYPE_PRIMITIVE];

                int ratio = (*it).indic_volumes_.size()/Couche_suivante_interieure.indic_volumes_.size();




                //
                //Arête e1

                for(unsigned int i = 1; i < Couche_suivante_interieure.indic_volumes_.size(); i++) // utiliser la taille max du tableau de couche_courante pourrait etre mieux => à revoir
                {
                    Dart d1;

                    // Si l'on se trouve sur l'étage 0, alors l'étage 1 (le suivant interieur) a obligatoirement autant de dart
                    if((*it).etage_ == 0)
                        d1 = (*it).indic_volumes_[i];
                    else
                        d1 = (*it).indic_volumes_[ratio*i]; // *2 à changer dans un future proche => 2 - 1 indique nombre de point qu'il faut sauter (ne pas relier)

                    Dart d2 = map_.phi<2321>(Couche_suivante_interieure.indic_volumes_[i]); // Pas de 2 * i car les dart stocké sur cette couche correspondent à ceux qu'il faut utiliser et pas aux autre de ce même segment

                    Edge e1; // On relie tout les sommets(darts) de couche suivante interieur
                    e1 = map_.cut_face(d1,d2);
                }


                //
                // Arête e2

                for(unsigned int i = 1; i < (*it).indic_volumes_.size(); i++)
                {
                    Dart d4 = map_.phi<32>(Couche_suivante_inferieure.indic_volumes_[i]);
                    Dart d1 = (*it).indic_volumes_[i];
                    Edge e2 = map_.cut_face(map_.phi<21>(d1), d4);
                }

                //
                // On gère l'avant dernière couche pour tracer l'arête de la face interieure de ce volume => Arête e3

                if((*it).etage_ == nb_appuis_ - 1)
                {
                    CoucheConcentrique Couche_int_inf = *(it + (nb_appuis_ + 1)*TYPE_PRIMITIVE + 1); // ajouter + 1?
                    for(unsigned int i = 1; i < Couche_suivante_interieure.indic_volumes_.size(); i++) // utiliser la taille max du tableau de couche suivante inf pourrait etre mieux
                    {

                        Dart d2 = map_.phi<23>(Couche_suivante_interieure.indic_volumes_[i]); // ou peut etre 2*i
                        Dart d3 = map_.phi<3231>(Couche_int_inf.indic_volumes_[i]);
                        Edge e3 = map_.cut_face(d2, d3);
                    }

                }

            }
        }


        //
        // Arêtes de dernier volume
        //

        for(unsigned int k = 0; k < TYPE_PRIMITIVE*(nb_appuis_ + 1); k++)
        {
            // On se place sur le bon volume de subdivision concentrique
            CoucheConcentrique couche_actu = Bt.subdivised_volume_control_[Bt.subdivised_volume_control_.size() - TYPE_PRIMITIVE*(nb_appuis_ + 1) + k];
            CoucheConcentrique couche_suivante = couche_actu; // Juste pour déclarer


            if(couche_actu.etage_ > 0 && couche_actu.etage_ < nb_appuis_)
                couche_suivante = Bt.subdivised_volume_control_[Bt.subdivised_volume_control_.size() - TYPE_PRIMITIVE*(nb_appuis_ + 1) + k + 1]; // Véritable affectation

            if(couche_actu.etage_ == 0)
            {
                Dart d_courant = couche_actu.indic_volumes_[0];
                Dart d2 = map_.phi<12>(d_courant);
                Dart d3 = map_.phi<112>(d2);
                Dart d4 = map_.phi<112>(d3);

                for(unsigned int i = 1; i < couche_actu.indic_volumes_.size(); i++)
                {
                    Dart d1 = couche_actu.indic_volumes_[i];

                    // Gestion arête e1
                    Edge e1 = map_.cut_face(d1,map_.phi<21>(d2));

                    // Gestion arête e2
                    Edge e2 = map_.cut_face(d4, map_.phi<21>(d1));

                    // Gestion arête e4
                    Edge e4 = map_.cut_face(d3, map_.phi<21>(d4));

                    // Déplacement des darts
                    d2 = map_.phi_1(d2);
                    d3 = map_.phi_1(map_.phi2(map_.phi_1(d3)));
                    d4 = map_.phi_1(map_.phi2(map_.phi_1(d4)));

                }
            }
            else if(couche_actu.etage_ > 0 && couche_actu.etage_ < nb_appuis_)
            {
                Dart d_courant = couche_actu.indic_volumes_[0];
                Dart d2 = map_.phi<12>(d_courant);
                Dart d3 = map_.phi<112>(d2);
                Dart d4 = map_.phi<112>(d3);

                int ratio = couche_actu.indic_volumes_.size()/couche_suivante.indic_volumes_.size();


                for(unsigned int i = 1; i < couche_actu.indic_volumes_.size(); i++)
                {
                    Dart d1 = couche_actu.indic_volumes_[i];

                    // Gestion arête e1
                    if(i%ratio == 0)
                        Edge e1 = map_.cut_face(d1,map_.phi<21>(d2));

                    // Gestion arête e2
                    Edge e2 = map_.cut_face(d4, map_.phi<21>(d1));

                    // Gestion arête e4
                    if(i%ratio == 0)
                        Edge e4 = map_.cut_face(d3, map_.phi<21>(d4));

                    // Déplacement des darts
                    if(i%ratio == 0)
                    {
                        d2 = map_.phi_1(d2);
                        d3 = map_.phi_1(map_.phi2(map_.phi_1(d3)));
                    }
                    d4 = map_.phi_1(map_.phi2(map_.phi_1(d4)));
                }
            }
            else
            {
                Dart d_courant = couche_actu.indic_volumes_[0];
                Dart d2 = map_.phi<12>(d_courant);
                Dart d3 = map_.phi<112>(d2);
                Dart d4 = map_.phi<112>(d3);

                for(unsigned int i = 1; i < couche_actu.indic_volumes_.size(); i++)
                {
                    Dart d1 = couche_actu.indic_volumes_[i];

                    // Gestion arête e2
                    Edge e2 = map_.cut_face(d4, map_.phi<21>(d1));

                    // Déplacement des darts
                    d4 = map_.phi_1(map_.phi2(map_.phi_1(d4)));
                }
            }
        }



        //
        //
        // Creation des faces
        //
        //


        for( std::vector<CoucheConcentrique>::iterator it_vol = Bt.subdivised_volume_control_.begin(); it_vol < Bt.subdivised_volume_control_.end(); ++it_vol)
        {
            CoucheConcentrique couche_courante = *it_vol;

            if(couche_courante.etage_ < nb_appuis_)
            {
                CoucheConcentrique couche_suivante = *(it_vol + 1); // Aucun risque d'être sur un nouveau quartier

                // avec iterateur
                for(std::vector<Dart>::iterator it_dart = couche_courante.indic_volumes_.begin() + 1 ; it_dart < couche_courante.indic_volumes_.end(); ++it_dart)
                {
                    bool check_last_binary_dart = true;

                    //en choisir qu'un sur 2
                    Dart d_courant;
                    if(couche_courante.etage_ > 0)
                    {
                        if(couche_suivante.indic_volumes_.size() < couche_courante.indic_volumes_.size())
                        {
                            // lorsqu'on est sur le dernier dart, il ne faut rien faire dans ce cas là =>pas créer de volume

                            it_dart++; // voici le correctif
                            if(it_dart < couche_courante.indic_volumes_.end())
                                d_courant = *it_dart; // Le nb de fois qu'il faudra pour atteindre le bon point => pb, car on refait la même chose à l'itération suivant
                            else
                                check_last_binary_dart = false;

                        }
                        else
                            d_courant = *it_dart; // ici c'est 0 fois car on a autant de point d'une couche à l'autre
                    }
                    else
                        d_courant = *it_dart; // ok

                    if(check_last_binary_dart)
                    {
                        Dart d1 = map_.phi<21>(d_courant);
                        Dart d4 = map_.phi<121>(d1);
                        Dart d3 = map_.phi<121>(d4);
                        Dart d2 = map_.phi<121>(d3);

                        Face f = map_.cut_volume({d4, d3, d2, d1});
                    }
                }
            }
        }
        controls_[count] = Bt;
        count++;
    }
}

void topo::SubdivisionConcentrique()
{
    for(BranchTopo Bt : controls_)
    {

        unsigned int k = 0;
        unsigned int j = 0;
        std::vector<Dart> nouv_vertex;
        std::vector<Edge> face_limits;


        //
        // Ajout des vertices de la subdivision
        //

        for(Dart d : Bt.volume_control_)
        {
            Dart v = map_.cut_edge(Edge(d)).dart;
            nouv_vertex.push_back(v);
            //vertex_position_[Vertex(v)] = (vertex_position_[Vertex(map_.phi1(v))]*(1 - MASK_SUBDIV_RAY) + vertex_position_[Vertex(map_.phi_1(v))]*MASK_SUBDIV_RAY);
        }

        // dernière arti
        for(unsigned int i = 0; i < TYPE_PRIMITIVE; i++)
        {
            Dart last_seg = map_.phi2(map_.phi1(map_.phi1(map_.phi2(Bt.volume_control_[Bt.volume_control_.size() - TYPE_PRIMITIVE + i]))));
            Vertex v = map_.cut_edge(Edge(last_seg));
            Dart dv = v.dart;
            nouv_vertex.push_back(dv);

            //vertex_position_[v] = (vertex_position_[Vertex(map_.phi_1(dv))]*(1 - MASK_SUBDIV_RAY) + vertex_position_[Vertex(map_.phi1(dv))]*MASK_SUBDIV_RAY);

        }


        //
        // Ajout des arêtes de la subdivision à l'aide des vertices rajouté à l'étape précédente
        //


        for(Dart d : Bt.volume_control_)
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



        //
        // Ajout des faces de la subdivision à l'aide des arêtes rajouté à l'étape précédente
        //

        for(Dart d : Bt.volume_control_)
        {

            //
            // version avec les indices (incomplète)
            //



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

    /*
    unsigned int k = 0;
    unsigned int j = 0;
    std::vector<Dart> nouv_vertex;
    std::vector<Edge> face_limits;


    nb_appuis_++;


    //
    // Ajout des vertices de la subdivision
    //

    int counter = 0;
    int nb_branch = 0;
    int increC = save_pos_[nb_branch];

    for(Dart d : volume_control_)
    {
        if(counter == increC)
        {
            // les dernières articulations
            for(unsigned int i = 0; i < TYPE_PRIMITIVE; i++)
            {
                Dart last_seg = map_.phi2(map_.phi1(map_.phi1(map_.phi2(volume_control_[increC - TYPE_PRIMITIVE + i]))));
                Vertex v = map_.cut_edge(Edge(last_seg));
                Dart dv = v.dart;
                nouv_vertex.push_back(dv);

                //vertex_position_[v] = (vertex_position_[Vertex(map_.phi_1(dv))]*(1 - MASK_SUBDIV_RAY) + vertex_position_[Vertex(map_.phi1(dv))]*MASK_SUBDIV_RAY);
            }
            if(nb_branch != save_pos_.size() - 1)
                increC +=  save_pos_[nb_branch++];
            //vertex_position_[Vertex(v)] = (vertex_position_[Vertex(map_.phi1(v))]*(1 - MASK_SUBDIV_RAY) + vertex_position_[Vertex(map_.phi_1(v))]*MASK_SUBDIV_RAY);
        }
        Dart v = map_.cut_edge(Edge(d)).dart;
        nouv_vertex.push_back(v);
        counter++;
    }

    //
    // Ajout des arêtes de la subdivision à l'aide des vertices rajouté à l'étape précédente
    //

    std::cout << " avant les cutfaces" << std::endl;



    for(Dart d : volume_control_)
    {
        std::cout << "combien d'iteration? " << j*TYPE_PRIMITIVE + k << std::endl;

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


    std::cout << " pendant les cutfaces" << std::endl;



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


    std::cout << " apres les cutfaces" << std::endl;

    //
    // Ajout des faces de la subdivision à l'aide des arêtes rajouté à l'étape précédente
    //

    for(Dart d : volume_control_)
    {

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
    unsigned int volume_count = 0;
    map_.foreach_cell([&] (Volume v){ volume_count++; }); // affichage du nombre de volumes
    std::cout << " Il y a " << volume_count << " Volume(s)" << std::endl;
    */

    }
}



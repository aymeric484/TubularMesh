#include "topo.h"

topo::topo(const Squelette& mon_squelette, const unsigned int& primitive) :
    map_(),
    map2_(),
    vertex_position_(),
    vertex_normal_(),
    cell_cache_prec_(map_)
{
    nb_appuis_ = 0;
    indice_repartition_ = 0;
    MakeFromSkeleton(mon_squelette, primitive);
}

std::unique_ptr<tetgenio> topo::export_tetgen()
{
    // Passer une variable en copie

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

void topo::generate_tetgen(const std::string& tetgen_args)
{
    auto tetgen_input = export_tetgen();

    tetgenio tetgen_output;


    tetgen::tetrahedralize(tetgen_args.c_str(), tetgen_input.get(), &tetgen_output);

    TetgenStructureVolumeImport tetgen_import(&tetgen_output);

    tetgen_import.import_file("");

    tetgen_import.create_map(map3inter_);

    map3inter_.check_map_integrity();


    /*
    cgogn::geometry::compute_AABB(vertex_position_, bb_);
    setSceneRadius(bb_.diag_size()/2.0);
    Vec3 center = bb_.center();
    setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
    showEntireScene();*/

    //tetgenio* volume = &tetgen_output;
    //cgogn::io::TetgenVolumeImport<cgogn::CMap3::MapTraits, Eigen::Vector3d> that(tetgen_output);
    //cgogn::io::TetgenVolumeImport<Map3, Vec3>* that;
    //cgogn::io::TetVolumeImport<Map3, Vec3>* that;
    //that->
    //that.create_map(map_);
    //that->set_nb_vertices(tetgen_output->numberofpoints);
    //that->set_nb_volumes(tetgen_output->numberoftetrahedra);

    /*
    if (that.nb_vertices() == 0u || that.nb_volumes()== 0u)
    {
        cgogn_log_warning("TetgenStructureVolumeImport") << "Error while importing data.";
        this->clear();
        return false;
    }

    ChunkArray<Eigen::Vector3d>* position = that.template position_attribute<Eigen::Vector3d>();
    //create vertices
    std::vector<unsigned int> vertices_indices;
    float* p = tetgen_output->pointlist ;

    for(unsigned int i = 0u, end = that.nb_vertices(); i < end; ++i)
    {
        const unsigned id = this->insert_line_vertex_container();
        position->operator[](id) = VEC3(Scalar(p[0]), Scalar(p[1]), Scalar(p[2]));
        vertices_indices.push_back(id);
        p += 3 ;
    }

    //create tetrahedrons
    int* t = volume_->tetrahedronlist ;
    for(uint32 i = 0u, end = this->nb_volumes(); i < end; ++i)
    {
        std::array<uint32,4> ids;
        for(uint32 j = 0u; j < 4u; j++)
            ids[j] = uint32(vertices_indices[t[j] - volume_->firstnumber]);
        this->add_tetra(*position, ids[0], ids[1], ids[2], ids[3], true);
        t += 4 ;
    }*/


    //TetgenV

    //tetgen_output.set
    // On fait une copie? cette étape semble inutile
    // cgogn::io::VolumeImport<Map3::MapTraits> tetgen_import(&tetgen_output);

    /*
    tetgen_output->set_nb_vertices(volume_->numberofpoints);
    this->set_nb_volumes(tetgen_output->numberoftetrahedra);

    if (this->nb_vertices() == 0u || this->nb_volumes()== 0u)
    {
        cgogn_log_warning("TetgenStructureVolumeImport") << "Error while importing data.";
        this->clear();
        return false;
    }

    ChunkArray<VEC3>* position = this->template position_attribute<VEC3>();
    //create vertices
    std::vector<unsigned int> vertices_indices;
    float64* p = volume_->pointlist;

    for(uint32 i = 0u, end = this->nb_vertices(); i < end; ++i)
    {
        const unsigned id = this->insert_line_vertex_container();
        position->operator[](id) = VEC3(Scalar(p[0]), Scalar(p[1]), Scalar(p[2]));
        vertices_indices.push_back(id);
        p += 3 ;
    }*/

    //create tetrahedrons
    //int* t = volume_->tetrahedronlist ; // pointeur sur la liste de tétra => on incrémentera le pointeur


    /*
    for(unsigned int i = 0, end = this->nb_volumes(); i < end; ++i)
    {
        std::array<unsigned int,4> ids; // cool
        for(uint32 j = 0u; j < 4u; j++)
            ids[j] = uint32(vertices_indices[t[j] - volume_->firstnumber]);
        this->add_tetra(*position, ids[0], ids[1], ids[2], ids[3], true); // Pourquoi pas, mais s'adapte à une struct de tetgen, fonctionne avec tetgenio??? => This à remplacer
        t += 4 ;
    }*/

    //tetgen_import.create_map(map3bis_);

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
        int a = T_test.ind1_;
        int b = T_test.ind2_;
        int c = T_test.ind3_;
        int count = 0;

        for(TriangleGeo T_courant : triangles)
        {
            //
            // Déterminer si les triangles sont voisins
            bool voisin = false;
            bool sommet1 = T_courant.ind1_ == a || T_courant.ind1_ == b || T_courant.ind1_ == c;
            bool sommet2 = T_courant.ind2_ == a || T_courant.ind2_ == b || T_courant.ind2_ == c;
            bool sommet3 = T_courant.ind3_ == a || T_courant.ind3_ == b || T_courant.ind3_ == c;

            if((sommet1 && sommet2) || (sommet3 && sommet1) || (sommet3 && sommet2))
                voisin = true;

            //
            // Si ils sont voisins, alors créer la face et la coller sur la bonne arête
            if(voisin)
            {
                // On accède à la face
                Dart d_courant = dart_faces[count];

                // Puis test pour le sewing : Toute les possibilités orientées y sont représentés
                // test à partir du premier sommet
                if(T_courant.ind1_ == a)
                {
                    if(T_courant.ind2_ = c)
                    {
                        d_courant = d_courant;
                        if(map2_.phi2(d_courant) != d_courant && map2_.phi2(map2_.phi_1(d_test)) != map2_.phi_1(d_test))
                            mbuild2.phi2_sew(d_courant, map2_.phi_1(d_test));
                    }
                    if(T_courant.ind3_ == b)
                    {
                        d_courant = map2_.phi_1(d_courant);
                        if(map2_.phi2(d_courant) != d_courant && map2_.phi2(d_test) != d_test)
                            mbuild2.phi2_sew(d_courant, d_test);
                    }
                }
                // test à partir du deuxième sommet
                if(T_courant.ind2_ == b)
                {
                    if(T_courant.ind1_ = c)
                    {
                        d_courant = d_courant;
                        if(map2_.phi2(d_courant) != d_courant && map2_.phi2(map2_.phi1(d_test)) != map2_.phi1(d_test))
                            mbuild2.phi2_sew(d_courant, map2_.phi1(d_test));
                    }
                    if(T_courant.ind3_ == a)
                    {
                        d_courant = map2_.phi1(d_courant);
                        if(map2_.phi2(d_courant) != d_courant && map2_.phi2(d_test) != d_test)
                            mbuild2.phi2_sew(d_courant, d_test);
                    }
                }
                // test à partir du troisième sommet
                if(T_courant.ind3_ == c)
                {
                    if(T_courant.ind1_ = b)
                    {
                        d_courant = map2_.phi_1(d_courant);
                        if(map2_.phi2(d_courant) != d_courant && map2_.phi2(map2_.phi1(d_test)) != map2_.phi1(d_test))
                            mbuild2.phi2_sew(d_courant, map2_.phi1(d_test));
                    }
                    if(T_courant.ind2_ == a)
                    {
                        d_courant = map2_.phi1(d_courant);
                        if(map2_.phi2(d_courant) != d_courant && map2_.phi2(map2_.phi_1(d_test)) != map2_.phi_1(d_test))
                            mbuild2.phi2_sew(d_courant, map2_.phi_1(d_test));
                    }
                }
            }
            count++;
        }
        nb_courant++;
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

    // Afficher les coordonées avec un foreach ?
    // map_.foreach_cell([&] (Face2 f){ });


    //
    // Subdiviser la map2 en utilisant les coordonées
    //

}

void topo::MakeBranch(const std::vector<Vec3>& positions, const unsigned int& primitives)
{
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
        Dart v3 = map3branch_.phi2(map3branch_.phi1(volume_control_[volume_control_.size() - (nb_articulation)*primitives + (nb_articulation-1)*primitives + k]));
        Dart v4;
        if(k+1 != primitives)
            v4 = map3branch_.phi2(volume_control_[volume_control_.size() - (nb_articulation)*primitives + (nb_articulation-1)*primitives + k + 1]);
        else
            v4 = map3branch_.phi2(volume_control_[volume_control_.size() - (nb_articulation)*primitives + (nb_articulation-1)*primitives]);
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
}

void topo::MakeFromSkeleton(const Squelette& mon_squelette, const unsigned int& primitives)
{

    // On parcourt toute les branches et on construit la topologie de chacune via MakeBranch dans une map intermediaire. Puis on merge cette map à notre map3 finale
    for(Branch branche : mon_squelette.branches_)
    {
        MakeBranch(branche.pos_vertices_ , primitives);
        save_pos_.push_back((branche.branch_size_ - 1)*primitives);
        map_.merge(map3branch_);
        map3branch_.clear_and_remove_attributes();

    }

    // On parcourt toute les intersection et on construit la topologie 2-Map puis 3-Map de chacune via MakeIntersection dans une map intermediaire. Puis on merge cette map à notre map3 finale
    for(Intersection inter : mon_squelette.intersections_)
    {
        MakeIntersection(inter.faces_, inter.contours_);
        generate_tetgen("pq1.1Y");
        map2_.clear();
        map_.merge(map3inter_);
        map3inter_.clear_and_remove_attributes();
    }

    Map3::VertexAttribute<Eigen::Vector3d> pos_attribute = map_.template get_attribute<Eigen::Vector3d, Map3::Vertex::ORBIT>("position");
    vertex_position_ = pos_attribute;
    cgogn_assert(pos_attribute.is_valid());
    map_.check_map_integrity();
}

void topo::InterpolationConcentrique()
{

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

        for(Dart d : volume_control_)
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

            Dart d = map_.phi<211>(volume_control_[volume_control_.size() - TYPE_PRIMITIVE + i]); // nous place sur un vertex de bord, de la dernière face
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
}

void topo::GetCouchesConcentriques()
{
    subdivised_volume_control_.clear();

    for(Dart d : volume_control_)
    {
        for(int k = 0; k < nb_appuis_ + 1; k++ )
        {
            CoucheConcentrique couche_courante(k,{d});
            subdivised_volume_control_.push_back(couche_courante);
            d = map_.phi<12321>(d);
        }

    }


}

void topo::UpdateCoordinates()
{
    // Cette méthode servira à recalculer les positions des points ajouté par subdivision Utheta
    // On y accèdera facilement grâce aux coucheconcentrique.ind_volumes[]
    // On stockera pas dans chaque couche un attribut d_opp puisqu'on utilisera coucheconcentrique.ind_volumes[0] de la couche suivante


    //
    // Tous les volumes sauf le dernier

    for(unsigned int i = 0; i < subdivised_volume_control_.size(); i++)
    {
        CoucheConcentrique couche_courante = subdivised_volume_control_[i];

        Dart extremite_gauche = couche_courante.indic_volumes_[0];

        Dart extremite_droite;
        if(((i)%(TYPE_PRIMITIVE*(nb_appuis_ + 1))) + (nb_appuis_ + 1) > ((nb_appuis_ + 1)*TYPE_PRIMITIVE) - 1) // gros doute pour le "- 1"
            extremite_droite = subdivised_volume_control_[i + nb_appuis_ + 1 - TYPE_PRIMITIVE*(nb_appuis_ + 1) ].indic_volumes_[0];
        else
            extremite_droite = subdivised_volume_control_[i + nb_appuis_ + 1].indic_volumes_[0]; // penser à la seg. fault pour le dernier volume


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

        CoucheConcentrique couche_courante = subdivised_volume_control_[subdivised_volume_control_.size() - TYPE_PRIMITIVE*(nb_appuis_ + 1) + i];

        Dart extremite_gauche = map_.phi<211>(couche_courante.indic_volumes_[0]);

        Dart extremite_droite;
        if((i + (nb_appuis_ + 1)) > ((nb_appuis_ + 1)*TYPE_PRIMITIVE) - 1)
            extremite_droite = map_.phi<211>(subdivised_volume_control_[subdivised_volume_control_.size() - 2*(nb_appuis_ + 1)*TYPE_PRIMITIVE + i + nb_appuis_ + 1 ].indic_volumes_[0]);
        else
            extremite_droite = map_.phi<211>(subdivised_volume_control_[subdivised_volume_control_.size() - (nb_appuis_ + 1)*TYPE_PRIMITIVE + i + nb_appuis_ + 1].indic_volumes_[0]); // penser à la seg. fault pour le dernier volume


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
}

void topo::SubdivisionCouche(const unsigned int& Nb_subdiv) // mettre un paramètre pourrait nous indiquer de combien subdiviser => boucle for{} pour la subdivision (2^N * Nb_subdiv)
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
    for(CoucheConcentrique couche_courante : subdivised_volume_control_)
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
        subdivised_volume_control_[j] = couche_courante;
        j++;
    }


    //
    // Pour la face de sortie des derniers volumes
    //

    for(unsigned int k = 0; k < TYPE_PRIMITIVE*(nb_appuis_ + 1); k++)
    {
        CoucheConcentrique couche_actu = subdivised_volume_control_[subdivised_volume_control_.size() - TYPE_PRIMITIVE*(nb_appuis_ + 1) + k];
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

    for(std::vector<CoucheConcentrique>::iterator it = subdivised_volume_control_.begin(); it < subdivised_volume_control_.end() - (nb_appuis_ + 1)*TYPE_PRIMITIVE; ++it)
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
        CoucheConcentrique couche_actu = subdivised_volume_control_[subdivised_volume_control_.size() - TYPE_PRIMITIVE*(nb_appuis_ + 1) + k];
        CoucheConcentrique couche_suivante = couche_actu; // Juste pour déclarer


        if(couche_actu.etage_ > 0 && couche_actu.etage_ < nb_appuis_)
            couche_suivante = subdivised_volume_control_[subdivised_volume_control_.size() - TYPE_PRIMITIVE*(nb_appuis_ + 1) + k + 1]; // Véritable affectation

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


    for( std::vector<CoucheConcentrique>::iterator it_vol = subdivised_volume_control_.begin(); it_vol < subdivised_volume_control_.end(); ++it_vol)
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
}

void topo::SubdivisionConcentrique()
{
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


    /*
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
    GetCouchesConcentriques();//
    InterpolationConcentrique(); // Calcul des coordonées des vertices ajouté
}



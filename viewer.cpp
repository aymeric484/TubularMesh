#include "viewer.h"

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


// Code subdivision Utheta améliorée chaotique

/*
void Viewer::Junk()
{

    //
    //
    // Subdivision Utheta améliorée
    //
    //

    //
    // Remplissage du tableau représentant le nb de subdiv à faire par quartier et des autres tableaux de stockage de darts correspondants aux volumes concentriques
    //

    std::vector<int> Stockage_Nb_Volume;
    std::vector<Dart> Stockage_Dart_Concentrique;
    std::vector<Dart> Stockage_Dart_Oppose;

    for(Dart d : volume_control_)//parcourt des quartiers
    {
        // On stocke le dart de volume control
        Stockage_Dart_Concentrique.push_back(d);

        // Décalage pour l'itération suivante, si il y en a une
        Dart d_prev = d;
        d = map_.phi<12321>(d); // pose problème si ce n'est pas le premier appui

        // On se place sur chaque volume ajouté par subdivision concentrique
        for(int i = 1; i < nb_appuis_ + 1; i++)
        {
            // Calcul de la distance du coté interne => epaisseur d'une couche concentrique
            Vec3 li = vertex_position_[Vertex(d)] - vertex_position_[Vertex(d_prev)];
            double Dli = li.norm();

            // Calcul de la distance de bordure supérieure
            Dart d_next = map_.phi_1(d_prev);
            Vec3 cote = vertex_position_[Vertex(d_next)] - vertex_position_[Vertex(d_prev)];
            double Dcote = cote.norm();

            // Condition de subdivision
            int Nb_ajout_volume = 0;
            if(2*Dcote > 3*Dli)
            {
                 Nb_ajout_volume= int(Dcote/Dli - 1/2);
            }
            std::cout << "Nb de volume " << Nb_ajout_volume << std::endl;

            // Remplisage du tableau indicateur du nombre de volume à ajouter sur chaque couche concentrique de chaque quartier
            Stockage_Nb_Volume.push_back(Nb_ajout_volume); // à parcourir après par un itérateur
            // Remplissage du Dart de volume correspondant à ce nombre
            Stockage_Dart_Concentrique.push_back(d);
            // Remplissage du Dart opposée au précedent
            Stockage_Dart_Oppose.push_back(map_.phi_1(d));

            // Décalage pour l'itération suivante si il y en a une
            d_prev = d;
            d = map_.phi<12321>(d);

        }
    }


    //
    // Création des nouveaux volumes sur chaque volume de subdivision concentrique
    //

    for(std::vector<int>::iterator it = Stockage_Nb_Volume.begin(); it<Stockage_Nb_Volume.end(); ++it) // Parcourt des volumes concentriques
    {
        // Cas ou il faut subdiviser dans la couche concentrique
        if(*it > 0)
        {
            //
            // Init

            // Extraction de l'indice de l'élément "Nb de volume à ajouter" => s'utilisera pour les tableau de stockage de même indexation
            unsigned int index = it - Stockage_Nb_Volume.begin();

            // Utile pour le test de condition 1 : partie supérieure & extérieure => arète 1
            double pas_cour1; // Pas du volume courant sur l'arète 1
            double pas_up1; // Pas du voisin du dessus sur l'arète 1
            double pas_out1; // Pas du voisin exterieur au point de vu subdivision concentrique sur l'arète 1
            unsigned int incre_down1 = 1; // indique de combien de fois pas_down on c'est déplacé sur l'arète 1
            unsigned int incre_up1 = 1; // idem pour pas_up
            unsigned int incre_out1 = 1; // idem pour pas_out

            // Utile pour le test de condition 2 : partie supérieure & intérieure => arète 2
            double pas_cour2; // Pas du volume courant sur l'arète 2
            double pas_up2; // Pas du voisin du dessus sur l'arète 2
            unsigned int incre_down2 = 1; // indique de combien de fois pas_down on c'est déplacé sur l'arète 2
            unsigned int incre_up2 = 1; // idem pour pas_up

            // Utile pour le test de condition 4 : partie inférieure & extérieure => arète 4
            double pas_cour4; // Pas du volume courant sur l'arète 4
            double pas_out4; // Pas du voisin exterieur au point de vu subdivision concentrique sur l'arète 4
            unsigned int incre_down4 = 1; // indique de combien de fois pas_down on c'est déplacé sur l'arète 4
            unsigned int incre_out4 = 1; // idem pour pas_out

            // d_courant et d_opp servent au calcul des coordonées de chaque point ajouté
            Dart d_courant = Stockage_Dart_Concentrique[index];
            Dart d_opp = Stockage_Dart_Oppose[index];

            // On se positionne de façon à pouvoir subdiviser avec cut_edge(di)
            Dart d1 = map_.phi_1(d_courant);
            Dart d2 = map_.phi<12>(d_courant);
            Dart d3 = map_.phi<112>(d2);
            Dart d4 = map_.phi<112>(d3);


            //
            // Calcul des pas

            pas_cour1 = (vertex_position_[Vertex(d_courant)] - vertex_position_[Vertex(d_opp)])/(Stockage_Nb_Volume[index] + 1);
            if(index - TYPE_PRIMITIVE*(nb_appuis_ + 1)> 0)
                pas_up1 = (vertex_position_[Vertex(d_courant)] - vertex_position_[Vertex(d_opp)])/(Stockage_Nb_Volume[index - TYPE_PRIMITIVE*(nb_appuis_ + 1)] + 1);
            else
            {
                pas_up1 = bb_.diag_size(); // pas_up1 = infinite
            }
            if((index)%(TYPE_PRIMITIVE*(nb_appuis_ + 1)) > 0)
                pas_out1 = (vertex_position_[Vertex(d_courant)] - vertex_position_[Vertex(d_opp)])/(Stockage_Nb_Volume[index - 1] + 1);
            else
            {
                pas_out1 = bb_.diag_size(); // pas_out1 = infinite
            }

            pas_cour2 = (vertex_position_[Vertex(map_.phi1(Stockage_Dart_Concentrique[index]))] - vertex_position_[Vertex(map_.phi_1(Stockage_Dart_Oppose[index]))])/(Stockage_Nb_Volume[index] + 1);
            if(index - TYPE_PRIMITIVE*(nb_appuis_ + 1) > 0)
                pas_up2 = (vertex_position_[Vertex(map_.phi1(Stockage_Dart_Concentrique[index]))] - vertex_position_[Vertex(map_.phi_1(Stockage_Dart_Oppose[index]))])/(Stockage_Nb_Volume[index - TYPE_PRIMITIVE*(nb_appuis_ + 1)] + 1);
            else
            {
                pas_up2 = bb_.diag_size(); // pas_up2 = infinite
            }

            pas_cour4 = (vertex_position_[Vertex(map_.phi<211>(Stockage_Dart_Concentrique[index]))] - vertex_position_[Vertex(map_.phi<211>(Stockage_Dart_Oppose[index]))])/(Stockage_Nb_Volume[index] + 1);
            if((index)%(TYPE_PRIMITIVE*(nb_appuis_ + 1)) > 0)
                pas_out4 = (vertex_position_[Vertex(map_.phi<211>(Stockage_Dart_Concentrique[index]))] - vertex_position_[Vertex(map_.phi<211>(Stockage_Dart_Oppose[index]))])/(Stockage_Nb_Volume[index - TYPE_PRIMITIVE*(nb_appuis_ + 1)] + 1);;
            else
            {
                pas_out4 = bb_.diag_size(); // pas_out4 = infinite
            }


            //
            // Pour chaque volume à ajouter, on va créer {vertices, edges, face, volume}

            for( int l = 1; l < (*it + 1); l++)
            {
                double coeff2 = double(l)/double(*it + 1); // à chaque volume son coefficient
                std::cout << "coeff2 " <<  coeff2 << std::endl;

                std::vector<Vertex> face_vertices;
                std::vector<Dart> path;

                bool check = false;

                // Premier sommet à créer => pas à revoir => Les pas sont propre à chaque sommet

                while(check != true)
                {
                    // On gère les 4 possibilitées
                    if(pas_down1*incre_down1 < pas_up1*incre_up1 && pas_down1*incre_down1 < pas_out1*incre_out1)
                    {
                        incre_down1++;
                        face_vertices.push_back(map_.cut_edge(d1)); // On créer enfin le sommet puis on le stocke
                        check = true;
                    }
                    else if(pas_down1*incre_down1 > pas_up1*incre_up1 && pas_down1*incre_down1 < pas_out1*incre_out1)
                    {
                        incre_up1++;
                        d1 = map_.phi2(map_.phi_1(map_.phi_1(d1)));
                    }
                    else if(pas_down1*incre_down1 < pas_up1*incre_up1 && pas_down1*incre_down1 > pas_out1*incre_out1)
                    {
                        incre_out1++;
                        d1 = map_.phi_1(d1);
                    }
                    else
                    {
                        if(pas_up*incre_up1 < pas_out*incre_out1)
                        {
                            d1 = map_.phi2(map_.phi_1(map_.phi_1(d1)));
                            incre_up1++;
                        }
                        else
                        {
                            d1 = map_.phi_1(d1);
                            incre_out1++;
                        }
                    }
                }
                check = false;

                // Deuxième sommet à créer => calculer un nouveau pas pour pas_up et down (il n'y a pas de pas_out pour cette partie) => en gros ajouter +1 aux indices
                // Possibilité d'utiliser le résultat précédent car le nb de volume est le meme et les pas proportionnels
                while(check != true)
                {
                    // On gère les 2 possibilitées
                    if(pas_down2*incre_down2 < pas_up2*incre_up2) // proportionnel à pas_down*incre_down < pas_up*increup
                    {
                        incre_down2++;
                        face_vertices.push_back(map_.cut_edge(d2)); // On créer enfin le sommet puis on le stocke
                        check = true;
                    }
                    else
                    {
                        incre_up2++;
                        d2 = map_.phi_1(d2);
                    }
                }
                check = false;

                // Troisième sommet à créer => pas besoin des pas pour celui là

                face_vertices.push_back(map_.cut_edge(d3));
                incre_down3++;


                // Quatrième sommet à créer => uniquement besoin de pas_out qu'il faut recalculer

                while(check != true)
                {
                    // On gère les 2 possibilitées
                    if(pas_down4*incre_down4 < pas_out4*incre_out4) // proportionnel à pas_down*incre_down < pas_up*increup
                    {
                        incre_down4++;
                        face_vertices.push_back(map_.cut_edge(d4)); // On créer enfin le sommet puis on le stocke
                        check = true;
                    }
                    else
                    {
                        incre_out4++;
                        d4 = map_.phi2(map_.phi_1(map_.phi_1(d4)));
                    }
                }


                // Ici on calcul les coordonées des 4 sommets

                for(int v = 0; v < 4; v++)
                {
                    vertex_position_[face_vertices[v]] = coeff2 * vertex_position_[Vertex(d_opp)] + (1-coeff2)*vertex_position_[Vertex(cpy_d_courant)];
                    d_opp = map_.phi2(map_.phi_1(map_.phi_1(d_opp)));
                    cpy_d_courant = map_.phi<121>(cpy_d_courant);
                }


                // Puis on place les 4 arètes

                for(int e = 0; e < 4; e++)
                {
                    if(e + 1 > 3)
                        map_.cut_face(face_vertices[3].dart, map_.phi<21>(face_vertices[0].dart));
                    else
                        map_.cut_face(face_vertices[e].dart, map_.phi<21>(face_vertices[e + 1].dart));
                }


                // Après calcul du chemin, on coupe le volume

                for(int v = 0; v < 4; v++)
                    path.push_back(face_vertices[v].dart);

                Face f = map_.cut_volume(path);
            }

        }
    }

    // Prévoir un nettoyage à chaque appel => on enleve tout les volumes et on recalcul?
    // Condition de simplification (à faire plus tard) ou peut etre pas si on reinitialise tt à chaque fois
    //if(2*Dli > 3*Dcote)
        //int Nb_suppression_volume = int(Dli/Dcote - 1/2);


}*/

void Viewer::InterpolationConcentrique()
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

            Dart d = map_.phi<211>(volume_control_[volume_control_.size()-TYPE_PRIMITIVE+i]); // nous place sur un vertex de bord, de la dernière face
            Dart d_bord = d; // On se situe maintenant sur le bord d'où cette affectation
            Dart d_centre = d; // le recalculer dans la boucle peut être inutile pour cette dernière face vu que ce serq toujours le même Vertex

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

void Viewer::GetCouchesConcentriques()
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

void Viewer::UpdateCoordinates()
{
    // Cette méthode servira à recalculer les positions des points ajouté par subdivision Utheta
    // On y accèdera facilement grâce aux coucheconcentrique.ind_volumes[]
    // On stockera pas dans chaque couche un attribut d_opp puisqu'on utilisera coucheconcentrique.ind_volumes[0] de la couche suivante
}

void Viewer::SubdivisionCouche(const unsigned int& Nb_subdiv) // mettre un paramètre pourrait nous indiquer de combien subdiviser => boucle for{} pour la subdivision (2^N * Nb_subdiv)
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
        /*
        if(couche_courante.etage_ == 0 && Nb_subdiv > 0) // mettre condition && (déjà une subdiv)
        {
            for(unsigned int i = 0; i < Nb_subdiv; i++)
            {

                //
                // Localisation sur la couche => faire le tri des cas exterieur et inferieur

                Dart d_courant1, d_courant2, d_courant3, d_courant4, d1, d2, d3, d4;
                d_courant1 = couche_courante.indic_volumes_[i];
                d_courant2 = map_.phi<121>(d_courant1);
                d_courant3 = map_.phi<121>(d_courant2);
                d_courant4 = map_.phi<121>(d_courant3);
                d1 = map_.phi_1(d_courant1);
                d2 = map_.phi<12>(d_courant1);
                d3 = map_.phi<112>(d2);
                d4 = map_.phi<112>(d3);


                //
                // Ajout des points sur l'arête à subdiviser

                Vertex v1, v2, v3, v4;
                v1 = map_.cut_edge(Edge(d1)); // v1 à ajouter ensuite à notre indic_volume
                v2 = map_.cut_edge(Edge(d2));
                v3 = map_.cut_edge(Edge(d3));
                v4 = map_.cut_edge(Edge(d4));

                //
                // Calcul des coordonées de ces points (pas nécessaire si l'on recycle ceux d'une couche supérieure)

                vertex_position_[v1] = vertex_position_[Vertex(d_courant1)]*1/2 + vertex_position_[Vertex(d1)]*1/2;
                vertex_position_[v2] = vertex_position_[Vertex(d_courant2)]*1/2 + vertex_position_[Vertex(d2)]*1/2;
                vertex_position_[v3] = vertex_position_[Vertex(d_courant3)]*1/2 + vertex_position_[Vertex(d3)]*1/2;
                vertex_position_[v4] = vertex_position_[Vertex(d_courant4)]*1/2 + vertex_position_[Vertex(d4)]*1/2;

                //
                // Construction des arêtes


                Edge e1, e2, e3, e4;
                e1 = map_.cut_face(v1.dart, map_.phi<21>(v2.dart)); // pour que les deux darts soient d'une même face
                e2 = map_.cut_face(v2.dart, map_.phi<21>(v3.dart));
                e3 = map_.cut_face(v3.dart, map_.phi<21>(v4.dart));
                e4 = map_.cut_face(v4.dart, map_.phi<21>(v1.dart));

                //
                // Construction de la face

                Face f = map_.cut_volume({map_.phi_1(v1.dart),map_.phi_1(v2.dart),map_.phi_1(v3.dart),map_.phi_1(v4.dart)});

                //
                // Stockage du dart ajouté (il va représenter le volume ajouté)

                couche_courante.indic_volumes_.insert(couche_courante.indic_volumes_.begin() + i + 1, map_.phi2(map_.phi_1(v1.dart)));
                break;
            }
        }
        break;*/

        //
        // Calcul du nombre de volume que l'on doit avoir à la fin

        int Volumes_init = (couche_courante.indic_volumes_.size()-1);

        int Nb_vol;
        if( couche_courante.etage_ > 0)
            Nb_vol = pow(2, nb_appuis_ - couche_courante.etage_ + 1 )*Nb_subdiv - 1 ;
        else
            Nb_vol = pow(2,nb_appuis_ )*Nb_subdiv - 1; // en effet, cette arête doit être coupée autant de fois que sa précédente du point de vue concentrique

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
            //std::cout << "le coeff " << coeff << std::endl;

            // Creation du point
            Vertex v = map_.cut_edge(Edge(d_opp));

            couche_courante.indic_volumes_.insert(couche_courante.indic_volumes_.begin() + i, v.dart); // l'insertion détruit mon itérateur couche_courante?

            // Calcul des coordonées du point => interpolation linéaire
            vertex_position_[v] = vertex_position_[Vertex(d_courant)]*(1-coeff) + vertex_position_[Vertex(d_opp)]*coeff;

            //couche_courante.indic_volumes_.insert(couche_courante.indic_volumes_.begin() + i + 1, map_.phi2(map_.phi_1(v1.dart)));
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

        int Nb_vol;
        if( couche_actu.etage_ > 0)
            Nb_vol = pow(2, nb_appuis_ - couche_actu.etage_  + 1)*Nb_subdiv - 1;
        else
            Nb_vol = pow(2,nb_appuis_ )*Nb_subdiv - 1; // en effet, cette arête doit être coupée autant de fois que sa précédente du point de vue concentrique

        //
        // Création de tous les sommets



        for(unsigned int i = 1; i < Nb_vol + 1; i++)
        {
            // Calcul du coeff
            double ind = double(i);
            double max = double(Nb_vol + 1);
            double coeff = ind/max;
            //std::cout << "le coeff " << coeff << std::endl;

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

            //
            //Arête e1

            for(unsigned int i = 1; i < Couche_suivante_interieure.indic_volumes_.size(); i++) // utiliser la taille max du tableau de couche_courante pourrait etre mieux => à revoir
            {
                Dart d1;

                // Si l'on se trouve sur l'étage 0, alors l'étage 1 (le suivant interieur) a obligatoirement autant de dart
                if((*it).etage_ == 0)
                    d1 = (*it).indic_volumes_[i];
                else
                    d1 = (*it).indic_volumes_[2*i]; // *2 à changer dans un future proche => 2 - 1 indique nombre de point qu'il faut sauter (ne pas relier)

                Dart d2 = map_.phi<2321>(Couche_suivante_interieure.indic_volumes_[i]); // Pas de 2 * i car les dart stocké sur cette couche correspondent à ceux qu'il faut utiliser et pas aux autre de ce même segment

                Edge e1; // On relie tout les sommets(darts) de couche suivante interieur
                e1 = map_.cut_face(d1,d2);

                /*
                if((*it).etage_ == 0)
                    e1 = map_.cut_face(d1,d2);// Couche 0 et couche 1 ont le même Nb de sommets
                else
                {
                    // Il y a deux fois moins de sommets d'une couche à l'autre => pas toujours vrai et le modulo sera à changer
                    if( (i%2) == 0)
                        e1 = map_.cut_face(d1,d2);
                }*/

                /*
                //
                // On va gérer les derniers volumes concentriques

                if((*it).etage_ == nb_appuis_)
                {
                    e3 = map_.cut_face(d2, map_.phi<21>(d3));
                    d2 = map_.phi_1(map_.phi2(map_.phi_1(d2)));
                }
                else
                {
                    d2 = map_.phi_1(d2); // car les arêtes sur cette face ne sont pas encore tracé
                }*/
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



        std::cout << "taille des derniers volumes " << couche_actu.indic_volumes_.size() << std::endl;


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

            for(unsigned int i = 1; i < couche_actu.indic_volumes_.size(); i++)
            {
                Dart d1 = couche_actu.indic_volumes_[i];

                // Gestion arête e1
                if(i%2 == 0)
                    Edge e1 = map_.cut_face(d1,map_.phi<21>(d2));

                // Gestion arête e2
                Edge e2 = map_.cut_face(d4, map_.phi<21>(d1));

                // Gestion arête e4
                if(i%2 == 0)
                    Edge e4 = map_.cut_face(d3, map_.phi<21>(d4));

                // Déplacement des darts
                if(i%2 == 0)
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


    for(CoucheConcentrique couche_courante : subdivised_volume_control_)
    {
        if(couche_courante.etage_ < nb_appuis_)
        {
            std::cout << "la taille est " << couche_courante.indic_volumes_.size() << std::endl;
            // avec iterateur
            for(std::vector<Dart>::iterator it = couche_courante.indic_volumes_.begin() + 1 ; it < couche_courante.indic_volumes_.end(); ++it)
            {
                //en choisir qu'un sur 2
                Dart d_courant;
                if(couche_courante.etage_ > 0)
                    d_courant = map_.phi_1(*it);// Le nb de fois qu'il faudra pour atteindre le bon point
                else
                    d_courant = *it;

                Dart d1 = map_.phi<21>(d_courant);
                Dart d4 = map_.phi<121>(d1);
                Dart d3 = map_.phi<121>(d4);
                Dart d2 = map_.phi<121>(d3);

                Face f = map_.cut_volume({d4, d3, d2, d1});
            }
        }
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
        case Qt::Key_L:
        {
            SubdivisionCouche(1);
            break;

        }
        case Qt::Key_Minus:
        {
            indice_repartition_--;
            InterpolationConcentrique(); // Calcul des coordonées de chaque points ajouté par "C"
            std::cout<<"minus"<<std::endl;
            break;
        }
        case Qt::Key_Plus:
        {
            indice_repartition_++;
            InterpolationConcentrique(); // Calcul des coordonées de chaque points ajouté par "C"
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
        case Qt::Key_C:
        {
            unsigned int k = 0;
            unsigned int j = 0;
            std::vector<Dart> nouv_vertex;
            std::vector<Edge> face_limits;


            nb_appuis_++;


            //
            // Ajout des vertices de la subdivision
            //

            for(Dart d : volume_control_)
            {
                Dart v = map_.cut_edge(Edge(d)).dart;
                nouv_vertex.push_back(v);
                //vertex_position_[Vertex(v)] = (vertex_position_[Vertex(map_.phi1(v))]*(1 - MASK_SUBDIV_RAY) + vertex_position_[Vertex(map_.phi_1(v))]*MASK_SUBDIV_RAY);
            }

            // dernière arti
            for(unsigned int i = 0; i < TYPE_PRIMITIVE; i++)
            {
                Dart last_seg = map_.phi2(map_.phi1(map_.phi1(map_.phi2(volume_control_[volume_control_.size() - TYPE_PRIMITIVE + i]))));
                Vertex v = map_.cut_edge(Edge(last_seg));
                Dart dv = v.dart;
                nouv_vertex.push_back(dv);

                //vertex_position_[v] = (vertex_position_[Vertex(map_.phi_1(dv))]*(1 - MASK_SUBDIV_RAY) + vertex_position_[Vertex(map_.phi1(dv))]*MASK_SUBDIV_RAY);

            }


            //
            // Ajout des arêtes de la subdivision à l'aide des vertices rajouté à l'étape précédente
            //

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



            //
            // Ajout des faces de la subdivision à l'aide des arêtes rajouté à l'étape précédente
            //

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

            InterpolationConcentrique(); // Calcul des coordonées des vertices ajouté
            GetCouchesConcentriques();
            //SubdivisionCouche(1);



            unsigned int volume_count = 0;
            map_.foreach_cell([&] (Volume v){ volume_count++; }); // affichage du nombre de volumes
            std::cout << " Il y a " << volume_count << " Volume(s)" << std::endl;

            break;
        }

        default:
            break;
    }

    // On met tout à jour pour le viewer (géométrie et topologie)
    map_.check_map_integrity();
    cgogn::rendering::update_vbo(vertex_position_, vbo_pos_);
    volume_drawer_->update_face<Vec3>(map_, vertex_position_);
    volume_drawer_->update_edge<Vec3>(map_, vertex_position_);
    topo_drawer_->update<Vec3>(map_, vertex_position_);
    render_->init_primitives(map_, cgogn::rendering::POINTS);

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
    render_->init_primitives(map_, cgogn::rendering::POINTS);

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


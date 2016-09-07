#include "squelette.h"

Squelette::Squelette(const std::string& filename)
{
    //articulations_.clear();

    std::ifstream fp(filename.c_str(), std::ios::in); // 1er argu => getpath && 2eme argu => read()
    std::string line;

    Vec4 arti_inter;

    unsigned int word_pos = 0;

    line.reserve(512);

    std::getline(fp, line);

    bool check = !fp.eof();
    int count = 0; // comptera le nombre de mots lu pour toute une branche

    Branch branche_courante(count);

    // tant qu'on est pas à la fin du fichier
    while (check == true)
    {
        fp >> line; // extrait uniquement jusqu'a un espace

        // On verifie si l'on commence une nouvelle branche
        std::size_t pos = line.find("--") ;



        if(pos != std::string::npos)
        {

            // Si on avait une branche ... alors libérer la variable
            if(branche_courante.branch_size_ > 0)
            {
                // On termine les affectations pour la branche_courante
                branche_courante.articulation_externe_end_ = branche_courante.articulations_.back();
                branche_courante.articulations_.pop_back();
                branche_courante.branch_size_ = branche_courante.articulations_.size();
                branches_.push_back(branche_courante);
                branche_courante.articulations_.clear(); // on peut recommencer une nouvelle branche
                branche_courante.branch_size_ = 0;

            }


            // On prépare une nouvelle branche
            char MemberLeft[line.size()];
            char MemberRight[line.size()];


            std::size_t length1 = line.copy(MemberLeft, pos, 0);
            std::size_t length2 = line.copy(MemberRight, line.size() - (pos + 2), pos + 2);

            ind_bout_depart_.push_back(std::stoi(std::string(MemberLeft)));
            ind_bout_arrive_.push_back(std::stoi(std::string(MemberRight)));

            count = 0;

            fp.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // decalage à la ligne suivante
            //std::cout<< "hidden " <<std::endl;

        }
        else
        {
            arti_inter[word_pos] = std::stod(line); // transformation en double du mot lu
            word_pos++; // position du mot courant sur la ligne


            // Le prochain mot sera sur la ligne suivante
            if(word_pos > 3)
            {
                if(count == 0)
                {
                    branche_courante.articulation_externe_begin_ = arti_inter; // on gère le point externe => premier point qui n'a pas de face associé
                }
                else
                {
                    branche_courante.articulations_.push_back(arti_inter);
                    //std::cout<< " x =  " << arti_inter[0] << " y =  " << arti_inter[1] << " z =  " << arti_inter[2] << " r =  " << arti_inter[3] << std::endl;
                    branche_courante.branch_size_ = branche_courante.articulations_.size();
                }

                count++;
                word_pos = 0;
                fp.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // decalage à la ligne suivante
            }
            else{fp.ignore(std::numeric_limits<std::streamsize>::max(), ' ');} // decalage au mot suivant

            check = !fp.eof(); // verifie si on a terminé avec le fichier

            if(!check)
            {
                // On termine les affectations pour la branche_courante
                branche_courante.articulation_externe_end_ = branche_courante.articulations_.back();
                branche_courante.articulations_.pop_back();
                branche_courante.branch_size_ = branche_courante.articulations_.size();
                branches_.push_back(branche_courante);
                branche_courante.articulations_.clear(); // on ne recommencera pas de nouvelle branche

            }
        }


    }

    // On gère le point externe de la fin de la branche => derrnier point qui n'a pas de face associé puis on le retire de articulation


    //mise à jour de la taille du tableau
    //branch_size_= articulations_.size();

    //
    // Création des coordonées de chaque branche et stockage dans le tableau pos_vertices (attribut de branche)
    //

    for(int k = 0; k < branches_.size(); k++)
    {
        Branch branche = branches_[k];
        branche.BranchSimplify(DISTANCE_MIN);
        branche.CreateCircleCoordinates(TYPE_PRIMITIVE);
        branche.SubdiDirectionT(COURBURE_MAX, TYPE_PRIMITIVE);
        for(Vec3 v : branche.pos_vertices_)
            positions_.push_back(v);
        longueurs_.push_back(branche.pos_vertices_.size());
        branches_[k] = branche;
    }


    //
    // Détection des intersections et création de leurs coordonées
    //

    int counter_end = 0;
    bool change = false;

    for(int i : ind_bout_arrive_)
    {
        int counter_begin = 0;
        bool lock = true;
        Intersection inter(i);

        for(int j : ind_bout_depart_)
        {

            if(i == j)
            {
                if(lock == true)
                {
                    Branch branche_arrivee = branches_[counter_end]; // On récupère la branche du même indice de branche
                    inter.centre_ = branche_arrivee.articulation_externe_end_; // Correspond au centre de l'intersection
                    int taille_max = branche_arrivee.pos_vertices_.size();

                    // On stocke le bout de la branche
                    for(int n = 0; n < TYPE_PRIMITIVE + 1; n++)
                        inter.contours_.push_back(branche_arrivee.pos_vertices_[taille_max - TYPE_PRIMITIVE - 1 + n]);

                    // Mettre un autre pushback *3 pour y stocker les indices de branches dans l'ordre
                    inter.branches_incidentes_.push_back(counter_end);

                    lock = false;
                }

                Branch branche_depart = branches_[counter_begin];
                for(int m = 0; m < TYPE_PRIMITIVE + 1; m++)
                    inter.contours_.push_back(branche_depart.pos_vertices_[m]);

                // Mettre un autre pushback *3 pour y stocker les indices de branches dans l'ordre
                inter.branches_incidentes_.push_back(counter_begin);

                change = true;

            }

            counter_begin++;

        }

        if(change == true)
        {
            intersections_.push_back(inter);
            change = false;
        }

        counter_end++;
    }


    for(int j = 0; j < intersections_.size(); j++)
    {
        Intersection inter = intersections_[j];
        inter.ComputeConnectivity9();
        intersections_[j] = inter;
    }

    //
    // TEST : affichage des coordonées de l'intersection
    //

    /*
    for(Intersection intersection : intersections_)
    {
        for(Vec3 coord : intersection.contours_)
           std::cout << "x = " << coord[0] << "  y = " << coord[1] << "  z = " << coord[2] << std::endl;
    }*/



    //
    // Obtenir la connectivité => se code pourrait-il être fait ailleurs?
    //

    /*
    Intersection inter = intersections_[0];
    inter.ComputeConnectivity3();*/
}




















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

            // On termine les affectations pour la branche_courante
            branche_courante.articulation_externe_end_ = branche_courante.articulations_.back();
            branche_courante.articulations_.pop_back();
            branche_courante.branch_size_ = branche_courante.articulations_.size();
            branches_.push_back(branche_courante);
            branche_courante.articulations_.clear(); // on peut recommencer une nouvelle branche

            // On prépare une nouvelle branche
            char* MemberLeft;
            char* MemberRight;

            std::size_t length1 = line.copy(MemberLeft, pos, 0);
            std::size_t length2 = line.copy(MemberRight, line.size() - (pos + 2), pos + 2);

            ind_bout_depart_.push_back(std::stoi(std::string(MemberLeft)));
            ind_bout_arrive_.push_back(std::stoi(std::string(MemberRight)));

            count = 0;

            fp.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // decalage à la ligne suivante
            std::cout<< "hidden " <<std::endl;

        }
        else
        {
            arti_inter[word_pos] = std::stod(line); // transformation en double du mot lu
            word_pos++; // position du mot courant sur la ligne


            // Le prochain mot sera sur la ligne suivante
            if(word_pos > 3)
            {
                if(count == 0)
                    branche_courante.articulation_externe_begin_ = arti_inter; // on gère le point externe => premier point qui n'a pas de face associé
                else
                    branche_courante.articulations_.push_back(arti_inter);

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

}

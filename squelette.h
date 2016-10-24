#ifndef SQUELETTE_H
#define SQUELETTE_H

#include "branch.h"
#include "intersection.h"
#include <string>

class Squelette
{

private:


public:

    // Ces deux variables codent la connectivité du squelette
    // On obtiendra les extremité begin et end à l'indice correspondant à l'indice de la branche
    // la valeur à ces extrimités seront l'indice d'une intersection du tableau intersections_
    // Il y a toujours 2 intersections par branche => deux extrémités
    std::vector<int> ind_bout_depart_;
    std::vector<int> ind_bout_arrive_;

    std::vector<Branch> branches_;
    std::vector<Intersection> intersections_;

    // Le constructeur est un parseur
    Squelette(const std::string&);


    // Dans un deuxième temps, on pourrait faire un constructeur qui extrait le squelette d'une image 3D


};

#endif // SQUELETTE_H

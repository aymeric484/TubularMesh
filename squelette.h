#ifndef SQUELETTE_H
#define SQUELETTE_H

#include "branch.h"
#include "intersection.h"
#include <string>

class Squelette
{

private:


    std::vector<Intersection> intersections_;

    // Ces deux variables codent la connectivité du squelette
    // On obtiendra les extremité begin et end de ces deux variables
    // la position de l'indice est celle d'une branche et la valeur à cet indice sera l'indice d'une intersection
    // Dans le cas où il n'y a pas de bout de départ ou de bout d'arrivé, on pourra rediriger vers autre chose (pour commencer on aura toujours 2 intersection par branche)

    std::vector<int> ind_bout_depart_;
    std::vector<int> ind_bout_arrive_;

public:

    std::vector<Branch> branches_;

    // Le constructeur sera un parseur
    Squelette(const std::string&);


};

#endif // SQUELETTE_H

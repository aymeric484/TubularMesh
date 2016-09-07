#ifndef INTERSECTION_H
#define INTERSECTION_H

#include "trianglegeo.h"



using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

class Intersection
{
private :

    //
    // Il faut une méthode dans branche qui nous renvoi le bout sous forme de vector
    // Ne devrait pas être trop dur car CreateCircleCoordinates le fait


    //
    // Il nous faut une méthode qui calcul l'enveloppe convexe avec un tableau de vec3 en entrée
    // void ComputeConvexhull(std::vector<Vec3>)
    // Il faudrait récupérer une connectivité à partir de cette enveloppe convexe

    unsigned int indicateur_;


public:

    //
    // Vec4 en attribut est obligatoire => centre théorique de l'intersection => ce même centre pourrait être le point externe des branches
    Vec4 centre_;
    std::vector<TriangleGeo> faces_;

    // Cette variables stocke les branches incidente, sachant que le premier terme est un bout de branche arrivant sur l'intersection
    // Alors que tout les autres indices représenteront des début de branches quittant l'intersection
    std::vector<int> branches_incidentes_;

    Intersection(int&);

    //
    // Il nous faut les derniers sommets des branches voisines une fois qu'elles sont tracés, sans le centre!
    std::vector<Vec3> contours_;

    // Faire une methode projection des coordinates sur la sphère, qui prend en compte le calcul de la sphère optimale
    // Stocker la connectivité dans un tableau stockant les indices des sommets 3 par 3 car notre volume ne comporte que des triangles
    // Le volume courant se stocke dans ce tableau et se fait modifier ainsi
    // Les faces doivent être orientées => cross product de 2 vecteurs coplanaires, puis pour le sens de la normale, faire le test point du volume-normale
    // si < 0, alors inverser le vecteur
    // sinon, on a la bonne normale orienté vers l'intérieure
    // Créer une classe triangle pour le calcul de la normale, la structure {3*Vec3}, et la structure {1,2,8} => des indices qui codent les points de notre nuage
    // Modifier l'ordre des Vec3 dans la structure selon le sens de la normale
    // la connectivité stocke un tableau de triangles
    // une fois qu'on a le sens et la direction de la normale, effectuer le test dot(normale,AM) avec M un point quelconque de notre nuage de point
    // Si dot > 0, alors on ignore le point et on passe au suivant
    // Sinon, on supprime notre triangle te on en créer 3 nouveaux reliant à chaque fois 2 points du triangle au point externe
    // 1ere tentative fausse
    void ComputeConnectivity();
    // Algo classique faux
    void ComputeConnectivity2();
    // Algo classique d'enveloppe convexe (fonctionnel)
    void ComputeConnectivity3();
    // Tentative emballage avec contrainte (raté)
    void ComputeConnectivity4();
    // Tentative avec scrutation des résulats possibles (en cours)
    void ComputeConnectivity5();
    // Tentative ComputeConnectivity3 avec réparation à la fin
    void ComputeConnectivity6();
    // Tentative ComputeConnectivity3 sans le pb rencontré de la face manquante => ajout du centre de branche
    void ComputeConnectivity7();
    // Tentative emballage sans contrainte
    void ComputeConnectivity8();
    // Tentative emballage sans priorité OK
    void ComputeConnectivity9();


};

#endif // INTERSECTION_H

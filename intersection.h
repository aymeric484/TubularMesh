#ifndef INTERSECTION_H
#define INTERSECTION_H

#include "trianglegeo.h"



using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

class Intersection
{
private :



public:

    //
    // indice que l'on associe à l'intersection
    unsigned int indicateur_;

    //
    // Vec4 en attribut est obligatoire => centre théorique de l'intersection => ce même centre est un point externe des branches
    Vec4 centre_;

    //
    // Représente les faces de notre volume d'intersection
    std::vector<TriangleGeo> faces_;

    //
    // Il nous faut les derniers sommets des branches voisines une fois qu'elles sont tracés, sans le centre!
    std::vector<Vec3> contours_;

    // Cette variables stocke les branches incidente, sachant que le premier terme est un bout de branche arrivant sur l'intersection
    // Alors que tout les autres indices représenteront des début de branches quittant l'intersection
    std::vector<int> branches_incidentes_;

    //
    // Constructeur
    Intersection(int&);

    //
    // Algo d'emballage sans imposer des faces initialement
    void ComputeConnectivity9();


};

#endif // INTERSECTION_H

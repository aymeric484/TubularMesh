#ifndef INTERSECTION_H
#define INTERSECTION_H


#include <cgogn/io/map_import.h>


using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

class Intersection
{
private :

    //
    // Il faut une méthode dans branche qui nous renvoi le bout sous forme de vector
    // Ne devrait pas être trop dur car CreateCircleCoordinates le fait

    //
    // Vec4 en attribut est obligatoire => centre théorique de l'intersection => ce même centre pourrait être le point externe des branches
    Vec4 centre_;

    //
    // Il nous faut les derniers sommets des branches voisines une fois qu'elles sont tracés, sans le centre!
    std::vector<Vec3> contours_;

    //
    // Il nous faut une méthode qui calcul l'enveloppe convexe avec un tableau de vec3 en entrée
    //void ComputeConvexhull(std::vector<Vec3>)
    // Il faudrait récupérer une connectivité à partir de cette enveloppe convexe




public:


    Intersection();



};

#endif // INTERSECTION_H

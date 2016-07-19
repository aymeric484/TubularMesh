#ifndef TRIANGLEGEO_H
#define TRIANGLEGEO_H

#include <cgogn/io/map_import.h>


using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

class TriangleGeo
{


private:





public:

    TriangleGeo(Vec3,Vec3,Vec3,int,int,int);

    // L'objectif ici est d'orienter les normales vers le centre du polyèdre pour le calcul de l'enveloppe convexe => inverser deux des indices du triangle
    void OrientedNormal(const Vec3&);

    Vec3 sommets_[3];
    int connectivity_[3];
    Vec3 normal_;



};

#endif // TRIANGLEGEO_H

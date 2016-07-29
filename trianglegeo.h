#ifndef TRIANGLEGEO_H
#define TRIANGLEGEO_H

#include <cgogn/io/map_import.h>
#include "config.h"


using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

class TriangleGeo
{


private:





public:

    TriangleGeo(Vec3, Vec3, Vec3, int, int, int, int);

    // L'objectif ici est d'orienter les normales vers le centre du polyèdre pour le calcul de l'enveloppe convexe => inverser deux des indices du triangle
    void OrientedNormal(const Vec3&);

    // Coordonées des sommets et leur position dans le tableau des points du contour de l'intersection
    Vec3 sommets_[3];
    int connectivity_[3];

    // Nous sert à orienter l'ordre de lecture des sommets du triangle
    Vec3 normal_;

    // Test d'appartenance de chaque face à une branche ... prend le numéro de la branche si ces 3 sommets sont ceux de la branche..
    int num_branch_;

    // Indice des triangles voisins dans le tableau de triangle
    int ind1_, ind2_, ind3_;

    void SetNeighboursData(int, int, int);
    std::vector<int> GetNeighboursData();
    int GetIncidentBranch();

    // A partir d'un buffer de points, on veut récupérer le plus loin de notre face
    int ComputefurthestPoint(const std::vector<Vec3>&);




};

#endif // TRIANGLEGEO_H

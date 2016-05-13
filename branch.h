#ifndef BRANCH_H
#define BRANCH_H

#endif // BRANCH_H

#include <math.h>
//#include <vector>

#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/basic/dart.h>

#include <cgogn/io/map_import.h>
#include <time.h>

#define Pi  3.14159265359

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

class Branch
{

public:

    Branch();   //constructor
    // Branch(squelette S, std::vector<int> ind); devrait nous permettre l'extraction des branches d'un squelette grace aux indices des articulations concernees

    void CreateTrianglesCoordinates(const unsigned int&);

    void SubdiBranch(const double&); // Subdivise la branche selon le seuil de courbure en entrée

    std::vector<Vec4> articulations_;
    std::vector<Vec3> pos_vertices_; // point des primitives

    std::vector<float> courbure_;
    std::vector<float> torsion_;

    Vec4 articulation_externe_begin_;
    Vec4 articulation_externe_end_; // articulation n'appartenant pas a la branche, au bout de la branche




private:

    void ComputeMatrixFromBranch(); // Nous donnes les 3 axes T,N,B pour chaque articulation et la matrice de changement de repère

    //calcul de la torsion. A faire apres l'inversion préventive du repere et avant l affectation des coordonées
    void CalculateTorsion();

    std::vector<Eigen::Matrix4d> NBT_to_xyz_;

    std::vector<Vec3> T_axis_;
    std::vector<Vec3> N_axis_;
    std::vector<Vec3> B_axis_;

    unsigned int branch_size_;
};




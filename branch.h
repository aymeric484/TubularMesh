#ifndef BRANCH_H
#define BRANCH_H

#endif // BRANCH_H

#include <math.h>
//#include <vector>

#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/basic/dart.h>

#include <cgogn/io/map_import.h>
#include <time.h>

#include "config.h"


using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

class Branch
{

public:

    Branch();   //constructor
    // Branch(squelette S, std::vector<int> ind); devrait nous permettre l'extraction des branches d'un squelette grace aux indices des articulations concernees

    void CreateTrianglesCoordinates(const unsigned int&);

    void SubdiBranch(const double&); // Subdivise la branche selon le seuil de courbure en argument

    std::vector<Vec4> articulations_; // noeuds du squelette
    std::vector<Vec3> pos_vertices_; // points des primitives en chaque noeuds

    Vec4 articulation_externe_begin_; // articulations n'appartenant pas a la branche, au bout de la branche
    Vec4 articulation_externe_end_;



private:

    void ComputeMatrixFromBranch(); // Nous donnes les 3 axes T,N,B pour chaque articulation et la matrice de changement de repère et la torsion

    std::vector<Eigen::Matrix4d> NBT_to_xyz_;

    std::vector<Vec3> T_axis_;
    std::vector<Vec3> N_axis_;
    std::vector<Vec3> B_axis_;

    std::vector<double> theta_;
    std::vector<double> courbure_;

    unsigned int branch_size_;

};




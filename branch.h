#ifndef BRANCH_H
#define BRANCH_H


#include <math.h>

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
    Branch(const std::string&);

    void BranchSimplify(const double&);
    void CreateCircleCoordinates(const unsigned int&);
    void SubdiBranch(const double&); // Subdivise la branche selon le seuil de courbure en argument
    void SubdiDirectionT(const double&, const unsigned int&); // Subdivise la branche selon le seuil de courbure en argument, dans la direction de propagation, s'appelle après CreateCircleCoordinates

    std::vector<Vec4> articulations_; // noeuds du squelette à passer en privé ?
    std::vector<Vec3> pos_vertices_; // points des primitives en chaque noeuds

    Vec4 articulation_externe_begin_; // articulations n'appartenant pas a la branche, au bout de la branche
    Vec4 articulation_externe_end_;

private:

    void ComputeMatrixFromBranch(); // Nous donnes les 3 axes T,N,B pour chaque articulation et la matrice de changement de repère

    void ComputeCourbureMax(const unsigned int&);

    unsigned int FindGreatestDistance(const double&, const unsigned int&, const unsigned int& );

    std::vector<Eigen::Matrix4d> NBT_to_xyz_;

    std::vector<Vec3> T_axis_;
    std::vector<Vec3> N_axis_;
    std::vector<Vec3> B_axis_;

    std::vector<double> theta_;
    std::vector<double> courbure_; // pourra être éventuelement supprimée => à faire aussi dans ComputeMatrixFromBranch et subdibranch
    std::vector<double> courbure_max_;

    unsigned int branch_size_;

};


#endif // BRANCH_H






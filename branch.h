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

    // Constructor
    Branch(int&);

    // Constructor non utilisé, servirait à afficher une branche à partir du main directement
    Branch(const std::string&);

    // Simplifie les noeuds du squelette de la branche
    void BranchSimplify(const double&);

    // Créer des points autour des noeuds, dans un plan calculé par ComputeMatrixFromBranch()
    void CreateCircleCoordinates(const unsigned int&);

    // Subdivision du squelette non utilisé
    void SubdiBranch(const double&); // Subdivise la branche selon le seuil de courbure en argument

    // Subdivision interpolante sur spline cubique de chaque ligne délimitant notre branche
    void SubdiDirectionT(const double&, const unsigned int&); // Subdivise la branche selon le seuil de courbure en argument, dans la direction de propagation, s'appelle après CreateCircleCoordinates

    std::vector<Vec4> articulations_; // noeuds du squelette
    std::vector<Vec3> pos_vertices_; // points des primitives en chaque noeuds

    Vec4 articulation_externe_begin_; // articulations n'appartenant pas a la branche, au bout de la branche
    Vec4 articulation_externe_end_;

    unsigned int branch_size_;

private:

    // Pour trouver le plan où l'on place les points autour de chaque noeud
    void ComputeMatrixFromBranch(); // Nous donnes les 3 axes T,N,B pour chaque articulation et la matrice de changement de repère

    // Méthode utilisée dans SubdidirectionT pour trouver la ligne avec le plus de courbure, en un noeud.
    void ComputeCourbureMax(const unsigned int&);

    // Méthode utilisée par BranchSimplify pour l'algo Douglas-Pleuckler
    unsigned int FindGreatestDistance(const double&, const unsigned int&, const unsigned int& );

    std::vector<Eigen::Matrix4d> NBT_to_xyz_; // Matrice de changement de repère pour créer des points dans un plan particulier

    std::vector<Vec3> T_axis_; // Tangente, utile pour la matrice de changement de repère
    std::vector<Vec3> N_axis_; // Normale, utile pour la matrice de changement de repère
    std::vector<Vec3> B_axis_; // Bitangente, utile pour la matrice de changement de repère

    std::vector<double> theta_; // Angle de torsion
    std::vector<double> courbure_; // Cette variable pourra être éventuelement supprimée => à supprimer aussi dans ComputeMatrixFromBranch et subdibranch
    std::vector<double> courbure_max_; //



};


#endif // BRANCH_H






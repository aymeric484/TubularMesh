#ifndef BRANCH_H
#define BRANCH_H

#endif // BRANCH_H

#include <math.h>
#include <vector>

#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/basic/dart.h>

#include <cgogn/io/map_import.h>


using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

class Branch
{

public:

    Branch();   //constructor
    // Branch(squelette S, std::vector<int> ind); devrait nous permettre l'extraction des branches d'un squelette grace aux indices des articulations concernees

    void GetAxisFromBranch(); //Nous donnes les 3 axes T,N,B pour chaque articulation


    std::vector<Vec4> articulations_;

    std::vector<Vec3> T_axis_;
    std::vector<Vec3> N_axis_;
    std::vector<Vec3> B_axis_;

    std::vector<float> courbure_;
    std::vector<float> torsion_; // pas vraiment necessaire il semblerait

    Vec4 articulation_externe_begin_;
    Vec4 articulation_externe_end_; // articulation n'appartenant pas a la branche, au bout de la branche
    //Il faudrait en faire un deuxieme groupe ici pour avoir les derivees externes
    //vec4 articulation_externe2_begin;
    //vec4 articulation_externe2_end;

    unsigned int branch_size_;

private:

    // ces 3 méthodes peuvent être implémenté en utilisant une définition différente de la dérivation
    void ComputeT(); // determine les tangentes en chaque articulation et normalise
    void ComputeN(); // determine les normales en chaque articulation et normalise
    void ComputeB(); // idem pour B

};




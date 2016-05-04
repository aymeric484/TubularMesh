#include "branch.h"


Branch::Branch()
{
    Vec4 V40(0.1, 2.0, 1.0, 1.0);
    Vec4 V41(0.0, 0.3, 2.0, 2.0);
    Vec4 V42(0.3, 0.0, 3.0, 1.0);
    Vec4 V43(0.0, 0.0, 4.0, 1.0);
    Vec4 V44(0.5, 0.5, 5.0, 1.0);
    Vec4 V45(0.0, 0.5, 7.0, 1.0);

    Vec4 V_externe_begin(0.0, 0.0, 0.0, 1.0);
    Vec4 V_externe_end(0.5, 0.5, 8.0, 1.0);

    articulations_.push_back(V40);
    articulations_.push_back(V41);
    articulations_.push_back(V42);
    articulations_.push_back(V43);
    articulations_.push_back(V44);
    articulations_.push_back(V45);

    articulation_externe_begin_ = V_externe_begin;
    articulation_externe_end_ = V_externe_end;

    branch_size_= articulations_.size();

}



void Branch::GetAxisFromBranch()
{
    ComputeT();
    ComputeN();
    ComputeB();

    /*
    for(int i = 0; i < branch_size_; i++)
    {
        std::cout << " iteration " << i << std::endl;
        std::cout << " tangente " << T_axis_[i][0] << " ; " << T_axis_[i][1] << " ; " << T_axis_[i][2] << std::endl;
        std::cout << " normale " << N_axis_[i][0] << " ; " << N_axis_[i][1] << " ; " << N_axis_[i][2] << std::endl;
        std::cout << " B " << B_axis_[i][0] << " ; " << B_axis_[i][1] << " ; " << B_axis_[i][2] << std::endl;
        std::cout << " courbure " << courbure_[i] << std::endl;
    }*/


}

void Branch::ComputeT()
{
    Vec3 normalized_tan;

    normalized_tan = (articulations_[1].head<3>() - articulation_externe_begin_.head<3>())/2;
    normalized_tan = normalized_tan/sqrt(normalized_tan[0]*normalized_tan[0] + normalized_tan[1]*normalized_tan[1] + normalized_tan[2]*normalized_tan[2]);
    T_axis_.push_back(normalized_tan);

    for( int i = 1; i < branch_size_ - 1; i++ )
    {
        normalized_tan = (articulations_[i+1].head<3>() - articulations_[i-1].head<3>())/2;
        normalized_tan = normalized_tan/sqrt(normalized_tan[0]*normalized_tan[0] + normalized_tan[1]*normalized_tan[1] + normalized_tan[2]*normalized_tan[2]);
        T_axis_.push_back(normalized_tan);
    }

    normalized_tan = (articulation_externe_end_.head<3>() - articulations_[branch_size_-2].head<3>())/2;
    normalized_tan = normalized_tan/sqrt(normalized_tan[0]*normalized_tan[0] + normalized_tan[1]*normalized_tan[1] + normalized_tan[2]*normalized_tan[2]);
    T_axis_.push_back(normalized_tan);
}

void Branch::ComputeN()
{
    Vec3 normalized_nor;
    Vec3 normalized_tan_prec;
    Vec3 normalized_tan_suiv;

    //normalisation de la tangente T(-1)
    normalized_tan_prec = articulations_[0].head<3>() - articulation_externe_begin_.head<3>();
    normalized_tan_prec = normalized_tan_prec/sqrt(normalized_tan_prec[0] * normalized_tan_prec[0] + normalized_tan_prec[1] * normalized_tan_prec[1] + normalized_tan_prec[2] * normalized_tan_prec[2]);
    //normalisation de la tangente T(0)
    normalized_tan_suiv = articulations_[1].head<3>() - articulations_[0].head<3>();
    normalized_tan_suiv = normalized_tan_suiv/sqrt(normalized_tan_suiv[0] * normalized_tan_suiv[0] + normalized_tan_suiv[1] * normalized_tan_suiv[1] + normalized_tan_suiv[2] * normalized_tan_suiv[2]);
    //Calcul de la normale N(0) + normalisation et calcul de la courbure
    normalized_nor = normalized_tan_suiv - normalized_tan_prec;
    courbure_.push_back(sqrt(normalized_nor[0]*normalized_nor[0] + normalized_nor[1]*normalized_nor[1] + normalized_nor[2]*normalized_nor[2]));
    normalized_nor = normalized_nor/courbure_[0];
    N_axis_.push_back(normalized_nor);

    for( int i = 1; i < branch_size_ - 1; i++ )
    {
        //normalisation de la tangente T(i-1)
        normalized_tan_prec = articulations_[i-1].head<3>() - articulations_[i].head<3>();
        normalized_tan_prec = normalized_tan_prec/sqrt(normalized_tan_prec[0] * normalized_tan_prec[0] + normalized_tan_prec[1] * normalized_tan_prec[1] + normalized_tan_prec[2] * normalized_tan_prec[2]);
        //normalisation de la tangente T(i)
        normalized_tan_suiv = articulations_[i+1].head<3>() - articulations_[i].head<3>();
        normalized_tan_suiv = normalized_tan_suiv/sqrt(normalized_tan_suiv[0] * normalized_tan_suiv[0] + normalized_tan_suiv[1] * normalized_tan_suiv[1] + normalized_tan_suiv[2] * normalized_tan_suiv[2]);
        //Calcul de la normale + normalisation et calcul de la courbure
        normalized_nor = normalized_tan_suiv - normalized_tan_prec;
        courbure_.push_back(sqrt(normalized_nor[0]*normalized_nor[0] + normalized_nor[1]*normalized_nor[1] + normalized_nor[2]*normalized_nor[2]));
        normalized_nor = normalized_nor/courbure_[i];
        N_axis_.push_back(normalized_nor);
    }

    //normalisation de la tangente T(size -2)
    normalized_tan_prec = articulations_[branch_size_ - 1].head<3>() - articulations_[branch_size_ - 2].head<3>();
    normalized_tan_prec = normalized_tan_prec/sqrt(normalized_tan_prec[0] * normalized_tan_prec[0] + normalized_tan_prec[1] * normalized_tan_prec[1] + normalized_tan_prec[2] * normalized_tan_prec[2]);
    //normalisation de la tangente T(size -1)
    normalized_tan_suiv = articulation_externe_end_.head<3>() - articulations_[branch_size_ - 1].head<3>();
    normalized_tan_suiv = normalized_tan_suiv/sqrt(normalized_tan_suiv[0] * normalized_tan_suiv[0] + normalized_tan_suiv[1] * normalized_tan_suiv[1] + normalized_tan_suiv[2] * normalized_tan_suiv[2]);
    //Calcul de la normale N(size-1) + normalisation et calcul de la courbure
    normalized_nor = normalized_tan_suiv - normalized_tan_prec;
    courbure_.push_back(sqrt(normalized_nor[0]*normalized_nor[0] + normalized_nor[1]*normalized_nor[1] + normalized_nor[2]*normalized_nor[2]));
    normalized_nor = normalized_nor/courbure_[branch_size_ -1];
    N_axis_.push_back(normalized_nor);

}

void Branch::ComputeB()
{
    /* Pour calculer la torsion, utiliser l egalité B' = -torsion_ *N
      Donc, avec B' = (B(i+1)-B(i-1))/2 => dérivée approximée
      */

    Vec3 normalized_vec3;

    for( int i = 0 ; i < T_axis_.size() ; i++ )
    {

        normalized_vec3 = T_axis_[i].cross(N_axis_[i]);
        normalized_vec3 = normalized_vec3/sqrt(normalized_vec3[0]*normalized_vec3[0] + normalized_vec3[1]*normalized_vec3[1] + normalized_vec3[2]*normalized_vec3[2]);
        B_axis_.push_back(normalized_vec3);
    }

}

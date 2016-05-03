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

/*    for(int i = 0; i < branch_size_; i++)
        std::cout << courbure_[i] << std::endl;*/

}

void Branch::ComputeT()
{
    /*
    for(std::vector<Vec4>::iterator it = articulations_.begin(); it != articulations_.end(); ++it)
    {
       if(it != articulations_.end())
           T_axis_.push_back( (*(std::next(it, 1))).head<3>() - (*it).head<3>());
       else{
           T_axis_.push_back( articulation_externe_.head<3>() - (*it).head<3>());
           std::cout << "here" << std::endl;
       }
    }*/

    Vec3 normalized_vec3;

    normalized_vec3 = (articulations_[1].head<3>() - articulation_externe_begin_.head<3>())/2;
    normalized_vec3 = normalized_vec3/sqrt(normalized_vec3[0]*normalized_vec3[0] + normalized_vec3[1]*normalized_vec3[1] + normalized_vec3[2]*normalized_vec3[2]);
    T_axis_.push_back(normalized_vec3);

    for( int i = 1; i < branch_size_ - 1; i++ )
    {
        normalized_vec3 = (articulations_[i+1].head<3>() - articulations_[i-1].head<3>())/2;
        normalized_vec3 = normalized_vec3/sqrt(normalized_vec3[0]*normalized_vec3[0] + normalized_vec3[1]*normalized_vec3[1] + normalized_vec3[2]*normalized_vec3[2]);
        T_axis_.push_back(normalized_vec3);
         //T_axis_.push_back( (articulations_[i+1].head<3>() - articulations_[i-1].head<3>())/2 );
    }

    normalized_vec3 = (articulation_externe_end_.head<3>() - articulations_[branch_size_-2].head<3>())/2;
    normalized_vec3 = normalized_vec3/sqrt(normalized_vec3[0]*normalized_vec3[0] + normalized_vec3[1]*normalized_vec3[1] + normalized_vec3[2]*normalized_vec3[2]);
    T_axis_.push_back(normalized_vec3);
    //T_axis_.push_back( (articulation_externe_end_.head<3>() - articulations_[branch_size_-2].head<3>())/2 );
}

void Branch::ComputeN()
{
    /*
    for(std::vector<Vec3>::iterator it = T_axis_.begin(); it != T_axis_.end(); ++it)
    {
       if(it != T_axis_.end())
           N_axis_.push_back( *(std::next(it, 1)) - *it);
       else{
           N_axis_.push_back( *it - *(std::prev(it, 1)));
           std::cout << "here" << std::endl;
       }

    }
    */

    /*
    N_axis_.push_back( T_axis_[1] - T_axis_[0]);

    for( int i = 1; i < branch_size_ - 1; i++ )
        N_axis_.push_back( (T_axis_[i+1].head<3>() - T_axis_[i-1].head<3>())/2);

    N_axis_.push_back( T_axis_[branch_size_ - 1] - T_axis_[branch_size_ -2]);
    */

    Vec3 normalized_vec3;

    normalized_vec3 = articulations_[1].head<3>() + articulation_externe_begin_.head<3>() - 2*articulations_[0].head<3>();
    courbure_.push_back(sqrt(normalized_vec3[0]*normalized_vec3[0] + normalized_vec3[1]*normalized_vec3[1] + normalized_vec3[2]*normalized_vec3[2]));
    normalized_vec3 = normalized_vec3/courbure_[0];
    N_axis_.push_back(normalized_vec3);
    //N_axis_.push_back( articulations_[1].head<3>() + articulation_externe_begin_.head<3>() - 2*articulations_[0].head<3>());

    for( int i = 1; i < branch_size_ - 1; i++ )
    {
        normalized_vec3 = articulations_[i+1].head<3>() + articulations_[i-1].head<3>() - 2*(articulations_[i].head<3>());
        courbure_.push_back(sqrt(normalized_vec3[0]*normalized_vec3[0] + normalized_vec3[1]*normalized_vec3[1] + normalized_vec3[2]*normalized_vec3[2]));
        normalized_vec3 = normalized_vec3/courbure_[i];
        N_axis_.push_back(normalized_vec3);
        //N_axis_.push_back( articulations_[i+1].head<3>() + articulations_[i-1].head<3>() - 2*(articulations_[i].head<3>()));
    }

    normalized_vec3 = articulations_[branch_size_ - 2].head<3>() + articulation_externe_end_.head<3>() - 2*(articulations_[branch_size_ - 1].head<3>());
    courbure_.push_back(sqrt(normalized_vec3[0]*normalized_vec3[0] + normalized_vec3[1]*normalized_vec3[1] + normalized_vec3[2]*normalized_vec3[2]));
    normalized_vec3 = normalized_vec3/courbure_[branch_size_ -1];
    N_axis_.push_back(normalized_vec3);
    //N_axis_.push_back( articulations_[branch_size_ - 2].head<3>() + articulation_externe_end_.head<3>() - 2*(articulations_[branch_size_ - 1].head<3>()));

}

void Branch::ComputeB()
{
    Vec3 normalized_vec3;

    for( int i = 0 ; i < T_axis_.size() ; i++ )
    {
        normalized_vec3 = T_axis_[i].cross(N_axis_[i]);
        normalized_vec3 = normalized_vec3/sqrt(normalized_vec3[0]*normalized_vec3[0] + normalized_vec3[1]*normalized_vec3[1] + normalized_vec3[2]*normalized_vec3[2]);
        B_axis_.push_back(normalized_vec3);
        //B_axis_.push_back(T_axis_[i].cross(N_axis_[i]));
    }

}

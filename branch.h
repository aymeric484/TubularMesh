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

    Branch();

    std::vector<Vec4> articulations_;

    std::vector<Vec3> T_axis_;
    std::vector<Vec3> N_axis_;
    std::vector<Vec3> B_axis_;

    std::vector<float> courbure_;
    std::vector<float> torsion_;

    //branch();


private:


};




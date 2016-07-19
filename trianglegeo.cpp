#include "trianglegeo.h"

TriangleGeo::TriangleGeo(Vec3 p1, Vec3 p2, Vec3 p3, int i1, int i2, int i3)
{
    sommets_[0]=p1;
    sommets_[1]=p2;
    sommets_[2]=p3;
    connectivity_[0]=i1;
    connectivity_[1]=i2;
    connectivity_[2]=i3; // Les In seront les indices dans contours de intersection

    Vec3 v1 = p2 - p1;
    Vec3 v2 = p1 - p3;

    normal_ = v1.cross(v2);
}

void TriangleGeo::OrientedNormal(const Vec3& point_volume_courant)
{
    Vec3 res = point_volume_courant - sommets_[0];
    double sca = res.dot(normal_);

    assert(sca*sca > 0.01);

    if(sca < 0)
    {
        int i4 = connectivity_[0];
        Vec3 p4 = sommets_[0];
        connectivity_[0] = connectivity_[2];
        sommets_[0] = sommets_[2];
        connectivity_[2] = i4;
        sommets_[2] = p4;

        Vec3 v1 = sommets_[1] - sommets_[0];
        Vec3 v2 = sommets_[0] - sommets_[2];

        normal_ = v1.cross(v2);


    }

    if(res.dot(normal_) < 0)
        std::cout<<"pas bon" << std::endl;
}

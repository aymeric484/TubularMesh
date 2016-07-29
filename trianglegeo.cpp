#include "trianglegeo.h"

TriangleGeo::TriangleGeo(Vec3 p1, Vec3 p2, Vec3 p3, int i1, int i2, int i3, int incident_branch)
{
    num_branch_ = incident_branch;

    sommets_[0]=p1;
    sommets_[1]=p2;
    sommets_[2]=p3;
    connectivity_[0]=i1;
    connectivity_[1]=i2;
    connectivity_[2]=i3; // Les In seront les indices dans contours de intersection

    Vec3 v1 = p2 - p1;
    Vec3 v2 = p1 - p3;

    normal_ = v1.cross(v2);
    normal_.normalize();


}

void TriangleGeo::OrientedNormal(const Vec3& point_volume_courant)
{
    Vec3 res = point_volume_courant - sommets_[0];
    double sca = res.dot(normal_);

    //assert(sca*sca > 0.001);

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
        normal_.normalize();


    }

    if(res.dot(normal_) < 0)
        std::cout<<"pas bon" << std::endl;
}

int TriangleGeo::ComputefurthestPoint(const std::vector<Vec3>& point_buffer)
{

    // On veut juste récupérer un indice
    double dist_ref = 0;
    int indice_ref;
    int indice = 0;

    for(Vec3 point_courant : point_buffer)
    {
        Vec3 res = point_courant - sommets_[0];
        double dist = res.dot(normal_);

        if(dist < dist_ref)
        {
            dist_ref = dist;
            indice_ref = indice;

        }

        indice++;
    }

    return indice_ref;
}

void TriangleGeo::SetNeighboursData(int a, int b, int c)
{
    ind1_ = a;
    ind2_ = b;
    ind3_ = c;
}

std::vector<int> TriangleGeo::GetNeighboursData()
{
    return {ind1_, ind2_, ind3_};
}

int TriangleGeo::GetIncidentBranch()
{
    return num_branch_;
}

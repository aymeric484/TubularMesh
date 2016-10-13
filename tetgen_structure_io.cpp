/*******************************************************************************
* SCHNApps                                                                     *
* Copyright (C) 2016, IGG Group, ICube, University of Strasbourg, France       *
* Plugin Volume Mesh From Surface                                              *
* Author Etienne Schmitt (etienne.schmitt@inria.fr) Inria/Mimesis              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Web site: http://cgogn.unistra.fr/                                           *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/

#include <tetgen_structure_io.h>
#include <tetgen/tetgen.h>


TetgenStructureVolumeImport::TetgenStructureVolumeImport(tetgenio* tetgen_output)
{
	volume_ = tetgen_output;
    import_tetgen_structure();
}

bool TetgenStructureVolumeImport::import_tetgen_structure()
{
    using cgogn::uint32;
    using cgogn::float64;

    const uint32 nb_vertices = volume_->numberofpoints;
    const uint32 nb_volumes = volume_->numberoftetrahedra;
    this->reserve(nb_volumes);

    if (nb_vertices == 0u || nb_volumes== 0u)
    {
        cgogn_log_warning("TetgenStructureVolumeImport") << "Error while importing data.";
        this->clear();
        return false;
    }

    ChunkArray<Eigen::Vector3d>* position = this->add_position_attribute();
    //create vertices
    std::vector<uint32> vertices_indices;
    float64* p = volume_->pointlist ;

    for(uint32 i = 0u; i < nb_vertices; ++i)
    {
        const unsigned id = this->insert_line_vertex_container();
        position->operator[](id) = Eigen::Vector3d(Scalar(p[0]), Scalar(p[1]), Scalar(p[2]));
        vertices_indices.push_back(id);
        p += 3 ;
    }

    //create tetrahedrons
    int* t = volume_->tetrahedronlist ;
    for(uint32 i = 0u; i < nb_volumes; ++i)
    {
        std::array<uint32,4> ids;
        for(uint32 j = 0u; j < 4u; j++)
            ids[j] = uint32(vertices_indices[t[j] - volume_->firstnumber]);
        this->add_tetra(ids[0], ids[2], ids[1], ids[3], false);
        t += 4 ;
    }

    return true;
}


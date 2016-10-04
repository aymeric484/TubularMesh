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

#ifndef TETGEN_STRUCTURE_IO_H
#define TETGEN_STRUCTURE_IO_H

#include <cgogn/io/volume_import.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/io/tetgen_io.h>
#include "tetgen.h"


class TetgenStructureVolumeImport : public cgogn::io::VolumeImport<cgogn::CMap3<cgogn::DefaultMapTraits>::MapTraits, Eigen::Vector3d>
{
public:
    using Inherit = cgogn::io::VolumeImport<cgogn::CMap3<cgogn::DefaultMapTraits>::MapTraits, Eigen::Vector3d>;
    using Self = TetgenStructureVolumeImport;
    using Scalar = cgogn::geometry::vector_traits<Eigen::Vector3d>::Scalar;
    template <typename T>
    using ChunkArray = typename Inherit::template ChunkArray<T>;
    using tetgenio = tetgen::tetgenio;

    explicit TetgenStructureVolumeImport(tetgenio * tetgen_output);
    CGOGN_NOT_COPYABLE_NOR_MOVABLE(TetgenStructureVolumeImport);

    bool import_tetgen_structure();

private:
    tetgenio* volume_;
};

//class TetgenStructureVolumeImport : public cgogn::io::VolumeImport<cgogn::CMap3<cgogn::DefaultMapTraits>::MapTraits>
//{
//public:
//    using Inherit = cgogn::io::VolumeImport<cgogn::CMap3<cgogn::DefaultMapTraits>::MapTraits>;
//	using Self = TetgenStructureVolumeImport;
//    using Scalar = cgogn::geometry::vector_traits<Eigen::Vector3d>::Scalar;
//	template <typename T>
//	using ChunkArray = typename Inherit::template ChunkArray<T>;
//	using tetgenio = tetgen::tetgenio;

//	explicit TetgenStructureVolumeImport(tetgenio * tetgen_output);
//	CGOGN_NOT_COPYABLE_NOR_MOVABLE(TetgenStructureVolumeImport);

//    // Variable qui vient d'être ajouté pour récupérer la position
//    ChunkArray<Eigen::Vector3d>* cpy_pos_ = NULL;


//protected:
//    virtual bool import_file_impl(const std::string& /*filename*/) override;

//private:
//	tetgenio* volume_;
//};

#endif

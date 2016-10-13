/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France       *
*                                                                              *
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


//#include "branch.h"
//#include "window.h"
//#include "coucheconcentrique.h"
#include "viewer.h"
#include "viewer2.h"

int main(int argc, char** argv)
{

    /*
    QApplication application(argc, argv);
    qoglviewer::init_ogl_context();

    //Branch branche("../../TubularMesh/multibranch");
    Squelette squelette("../../TubularMesh/intersect3");


    Branch branche = squelette.branches_[0];
    Intersection inter = squelette.intersections_[0]; // compute connectivity devrait déjà avoir été fait dans le constructeur Squelette


    //inter.ComputeConnectivity();
    //Branch branche("../../TubularMesh/multibranch_cave");
    //Branch branche;

    //branche.BranchSimplify(DISTANCE_MIN);
    //branche.SubdiBranch( COURBURE_MAX );
    //branche.CreateCircleCoordinates(TYPE_PRIMITIVE);
    //branche.SubdiDirectionT(COURBURE_MAX, TYPE_PRIMITIVE);


    Window fenetre;
    fenetre.setWindowTitle("variables");
    fenetre.show();

    // Instantiate the viewer.
    Viewer viewer;
    viewer.setWindowTitle("simpleViewer");

    viewer.MakeFromSkeleton(branche.pos_vertices_, TYPE_PRIMITIVE);
    //viewer.MakeIntersection(inter.faces_, inter.centre_);

    viewer.move(105,68);
    viewer.show();
    */



    //std::string surface_mesh =  "../../TubularMesh/old/Vessels/dot/porte_t1.ply";

    QApplication application(argc, argv);
    qoglviewer::init_ogl_context();


    Squelette squelette("../../TubularMesh/intersect4");


    //
    // Affichage de l'ensemble intersections + branches
    Viewer viewer(squelette);


    //
    // Affichage de la Map2 uniquement
    /*
    Viewer2 viewer;
    Intersection inter = squelette.intersections_[0];
    std::vector<int> code = inter.ComputeConnectivity9();
    viewer.MakeIntersection(inter.faces_, inter.contours_);*/


    viewer.move(105,68);//
    viewer.show();




    // Run main loop.
    return application.exec();
}

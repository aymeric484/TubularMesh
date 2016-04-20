/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* version 0.1                                                                  *
* Copyright (C) 2009-2012, IGG Team, LSIIT, University of Strasbourg           *
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

#include "viewer.h"
#include <iostream>

#include "Algo/Tiling/Volume/cubic.h"
#include "Algo/Modelisation/polyhedron.h"
#include "Algo/Import/import.h"
#include "Algo/Geometry/volume.h"
#include "Algo/Geometry/area.h"


void Viewer::initGUI()
{
    setDock(&dock) ;

    setCallBack( dock.checkBox_volumes, SIGNAL(toggled(bool)), SLOT(volumes_onoff(bool)) );
    setCallBack( dock.checkBox_edges, SIGNAL(toggled(bool)), SLOT(edges_onoff(bool)) );
    setCallBack( dock.checkBox_topo, SIGNAL(toggled(bool)), SLOT(topo_onoff(bool)) );

    setCallBack( dock.checkBox_hide, SIGNAL(toggled(bool)), SLOT(hide_onoff(bool)) );
    setCallBack( dock.checkBox_plane, SIGNAL(toggled(bool)), SLOT(clipping_onoff(bool)) );
    setCallBack( dock.slider_explode, SIGNAL(valueChanged(int)), SLOT(slider_explode(int)) );

    setCallBack( dock.slider_explode, SIGNAL(sliderPressed()), SLOT(slider_pressed()) );
    setCallBack( dock.slider_explode, SIGNAL(sliderReleased()), SLOT(slider_released()) );

    setCallBack( dock.slider_explode_face, SIGNAL(valueChanged(int)), SLOT(slider_explodeF(int)) );

    setCallBack( dock.slider_explode_face, SIGNAL(sliderPressed()), SLOT(slider_pressed()) );
    setCallBack( dock.slider_explode_face, SIGNAL(sliderReleased()), SLOT(slider_released()) );


    dock.slider_explode->setValue(80);
    dock.slider_explode_face->setValue(100);
    clipping_onoff(true);
}

void Viewer::volumes_onoff(bool /*x*/)
{
    render_volumes = !render_volumes;
    updateGL();
}

void Viewer::edges_onoff(bool /*x*/)
{
    render_edges = !render_edges;
    updateGL();
}

void Viewer::topo_onoff(bool /*x*/)
{
    render_topo = !render_topo;
    if (render_topo)
	{
        m_topo_render->updateData(myMap, position, 0.8f, m_explode_factorf-0.05f, m_explode_factor);
    }

    updateGL();
}

void Viewer::clipping_onoff(bool x)
{
    clip_volume = x;

    if (clip_volume)
    {
        Geom::Vec3f pos = m_PlanePick->getPosition();
        float pipo;
        Geom::Vec3f normal = m_PlanePick->getAxisScale(2, pipo); // 2 = Z axis = plane normal
        float d = -(pos*normal);
        m_explode_render->setClippingPlane(Geom::Vec4f(normal[0],normal[1],normal[2],d));
        m_topo_render->setClippingPlane(Geom::Vec4f(normal[0],normal[1],normal[2],d));
    }
    else
    {
        m_explode_render->setNoClippingPlane();
        m_topo_render->setNoClippingPlane();
    }
    updateGL();
}

void Viewer::hide_onoff(bool /*x*/)
{
    hide_clipping = !hide_clipping;
    updateGL();
}

void Viewer::slider_explode(int x)
{
    m_explode_factor = 0.01f*(x+1)-0.0001f;
    m_explode_render->setExplodeVolumes(m_explode_factor);
    updateGL();
}

void Viewer::slider_explodeF(int x)
{
    m_explode_factorf = 0.01f*(x+1);
    m_explode_render->setExplodeFaces(m_explode_factorf);
    updateGL();
}

void Viewer::slider_pressed()
{
    render_topoTemp = render_topo;
    render_topo = false;
    updateGL();
}

void Viewer::slider_released()
{
    render_topo = render_topoTemp;
    if (render_topo)
    {
//		SelectorDartNoBoundary<MAP> nb(myMap);
        //TODO MapBrowser
        m_topo_render->updateData(myMap, position, 0.8f, m_explode_factorf-0.05f, m_explode_factor );
    }
    updateGL();
}

void Viewer::cb_Open()
{
    std::string filters("all (*.*)") ;
    std::string filename = selectFile("Open Mesh", "", filters) ;
    if (filename.empty())
        return ;

    myMap.clear(true);

    std::vector<std::string> attrNames ;

    size_t pos = filename.rfind(".");    // position of "." in filename
    std::string extension = filename.substr(pos);

//	if(extension == std::string(".off"))
//	{
//		if(!Algo::Volume::Import::importMeshToExtrude<PFP>(myMap, filename, attrNames))
//		{
//			std::cerr << "could not import " << filename << std::endl ;
//			return ;
//		}
//		else
//		{
//			position = myMap.getAttribute<VEC3, VERTEX>(attrNames[0]) ;
//			myMap.closeMap();
//		}
//	}
//	else
//	{
        if(!Algo::Volume::Import::importMesh<PFP>(myMap, filename, attrNames))
        {
            std::cerr << "could not import " << filename << std::endl ;
            return ;
        }
        else
            position = myMap.getAttribute<VEC3, VERTEX, MAP>(attrNames[0]) ;
    //}

    color = myMap.addAttribute<VEC3, VOLUME, MAP>("color");

    TraversorCell<MAP, VOLUME> tra(myMap);
    for (Dart d = tra.begin(); d != tra.end(); d = tra.next())
    {
//		float v = Algo::Geometry::tetrahedronVolume<PFP>(myMap, d, position);
        color[d] = PFP::VEC3(1.0,0,0);
//		color[d] = VEC3(v,0,0);

//		if (v>maxV)
//			maxV=v;

        if(myMap.isVolumeIncidentToBoundary(d))
            color[d] = VEC3(1.0f,0.41f,0.706f);
    }
//	for (unsigned int i = color.begin(); i != color.end(); color.next(i))
//	{
//		color[i][0] /= maxV;
//		color[i][2] = 1.0f - color[i][0];
//	}

//	SelectorDartNoBoundary<MAP> nb(myMap);
    m_topo_render->updateData(myMap, position,  0.8f, 0.8f, 0.8f);
    m_explode_render->updateData<PFP>(myMap, position, color);

    updateGL() ;
}

void Viewer::cb_Save()
{
    std::string filters("all (*.*);; tetmesh (*.tetmesh);; tet (*.tet);; node (*.node);; msh (*.msh);; vtu (*.vtu);; nas (*.nas);; vbgz (*.vbgz)") ;
    std::string filename = selectFileSave("Save Mesh", "", filters) ;
    if (filename.empty())
        return ;

    Algo::Volume::Export::exportMesh<PFP>(myMap, position, filename);
}

void Viewer::cb_initGL()
{
    // create the renders
    m_topo_render = new Algo::Render::GL2::Topo3RenderMap<PFP>();	
    m_explode_render = new Algo::Render::GL2::ExplodeVolumeRender(true,true,true);

    m_topo_render->updateData(myMap, position,  0.8f, 0.8f, 0.8f);
    m_explode_render->updateData<PFP>(myMap, position, color);
    m_explode_render->setExplodeVolumes(0.8f);
    m_explode_render->setExplodeFaces(0.9f);
    m_explode_render->setAmbiant(Geom::Vec4f(0.2f,0.2f,0.2f,1.0f));
    m_explode_render->setBackColor(Geom::Vec4f(0.9f,0.9f,0.9f,1.0f));
    m_explode_render->setColorLine(Geom::Vec4f(0.8f,0.2f,0.2f,1.0f));


    registerShader(m_explode_render->shaderFaces());
    registerShader(m_explode_render->shaderLines());

    m_PlanePick = new Utils::Pickable(Utils::Pickable::GRID,1);
    m_frame = new Utils::FrameManipulator();
    m_frame->setSize(m_WidthObj/2.0f);

    m_explode_render->setClippingPlane(Geom::Vec4f(0,0,1,m_PosObj*Geom::Vec3f(0,0,-1)));
}



void Viewer::cb_redraw()
{
	glClearColor(1.0,1.0,1.0,1.0);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_LIGHTING);

    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);

    if (render_topo)
        m_topo_render->drawTopo();


	if (render_edges)
    {
        glLineWidth(2.0f);
        m_explode_render->drawEdges();
    }

    glDisable(GL_POLYGON_OFFSET_FILL);

    if (render_volumes)
    {
        m_explode_render->drawFaces();
    }

    if (clip_volume && !hide_clipping)
    {
        m_frame->draw();
        m_PlanePick->draw();
    }

    m_nbFrames++;
    if (m_nbFrames >=40)
    {
        std::cout << 40000.0/m_frame_ch.elapsed()<< " fps"<<std::endl;
        m_nbFrames = 0;
        m_frame_ch.start();
    }

//    md->callList();
}


void  Viewer::cb_mousePress(int /*button*/, int x, int y)
{
    if (!Shift())
        return;

    if (hide_clipping || !clip_volume)
        return;

    m_begX = x;
    m_begY = y;

    // get ray of selection
    Geom::Vec3f rayA,rayB;
    float dist = getOrthoScreenRay(x,y,rayA,rayB);
    Geom::Vec3f AB = rayB-rayA;

    unsigned int fr_picked =0;
    // picking the frame -> axis
    fr_picked = m_frame->pick(rayA,AB,dist);

    if (fr_picked != 0)
    {
        m_pickedAxis=fr_picked;
        m_frame->highlight(m_pickedAxis);
        m_frame->storeProjection(m_pickedAxis);
        updateGL();
    }
}

void  Viewer::cb_mouseRelease(int /*button*/, int /*x*/, int /*y*/)
{

    if (hide_clipping || !clip_volume)
        return;

    m_pickedAxis=0;
    m_frame->highlight(m_pickedAxis);
    updateGL();

}

void  Viewer::cb_mouseMove(int buttons, int x, int y)
{
    if (!Shift())
        return;

    if (hide_clipping || !clip_volume)
        return;

    // rotation selected ?
    if (Utils::FrameManipulator::rotationAxis(m_pickedAxis))
    {
        if (buttons&1)
        {
            float angle = m_frame->angleFromMouse(x,y,x-m_begX, y-m_begY);
            m_frame->rotate(m_pickedAxis, angle);
        }
        else if (buttons&2)
            m_frame->rotateInScreen(x-m_begX, y-m_begY);

        m_PlanePick->transfo() = m_frame->transfo();
    }
    // translation selected
    else if (Utils::FrameManipulator::translationAxis(m_pickedAxis))
    {
        if (buttons&1)
        {
            float dist =  m_frame->distanceFromMouse(x-m_begX, y-m_begY);
            m_frame->translate(m_pickedAxis, dist);
        }
        else if (buttons&2)
            m_frame->translateInScreen(x-m_begX, y-m_begY);

        m_PlanePick->transfo() = m_frame->transfo();
    }
    // scale selected
    else if (Utils::FrameManipulator::scaleAxis(m_pickedAxis) )
    {
        float scale = m_frame->scaleFromMouse(x-m_begX, y-m_begY);
        m_frame->scale(m_pickedAxis, scale );
        m_PlanePick->transfo() = m_frame->transfo();
    }

    Geom::Vec3f pos = m_PlanePick->getPosition();
    float pipo;
    Geom::Vec3f normal = m_PlanePick->getAxisScale(2, pipo); // 2 = Z axis = plane normal
    float d = -(pos*normal);
    m_explode_render->setClippingPlane(Geom::Vec4f(normal[0],normal[1],normal[2],d));
    m_topo_render->setClippingPlane(Geom::Vec4f(normal[0],normal[1],normal[2],d));


    m_begX = x;
    m_begY = y;
    updateGL();
}


void Viewer::sampleMesh()
{
    position = myMap.addAttribute<VEC3, VERTEX, MAP>("position");

    int nb = 16;
    Algo::Volume::Tilings::Cubic::Grid<PFP> cubic(myMap, nb, nb, nb);
    cubic.embedIntoGrid(position, 1.0f, 1.0f, 1.0f);

    for (unsigned int i = position.begin(); i != position.end(); position.next(i))
    {
        VEC3 pert(float(double(rand())/RAND_MAX/200.0),float(double(rand())/RAND_MAX/200.0),float(double(rand())/RAND_MAX/200.0));
        position[i]+= pert;
    }

    color = myMap.addAttribute<VEC3, VOLUME, MAP>("color");
    TraversorW<MAP> tra(myMap);
    for (Dart d = tra.begin(); d != tra.end(); d = tra.next())
        color[d] = position[d] + VEC3(0.5,0.5,0.5);

    Geom::Vec3f pos = m_PlanePick->getPosition();
    float pipo;
    Geom::Vec3f normal = m_PlanePick->getAxisScale(2, pipo); // 2 = Z axis = plane normal
    float d = -(pos*normal);
    m_explode_render->setClippingPlane(Geom::Vec4f(normal[0],normal[1],normal[2],d));
    m_topo_render->setClippingPlane(Geom::Vec4f(normal[0],normal[1],normal[2],d));

    m_topo_render->updateData(myMap, position,  0.8f, 0.8f, 0.8f);
    m_explode_render->updateData<PFP>(myMap, position, color);

    //  bounding box
    Geom::BoundingBox<VEC3> bb = Algo::Geometry::computeBoundingBox<PFP>(myMap, position);
    m_WidthObj = std::max<REAL>(std::max<REAL>(bb.size(0), bb.size(1)), bb.size(2));
    m_PosObj = (bb.min() +  bb.max()) / REAL(2);

    // envoit info BB a l'interface
    setParamObject(m_WidthObj, m_PosObj.data());
    updateGL();
}

void Viewer::importMesh(std::string& filename)
{
    myMap.clear(true);

    std::vector<std::string> attrNames ;

//    if(!Algo::Volume::Import::importMesh<PFP>(myMap, filename, attrNames))
//    {
//        std::cerr << "could not import " << filename << std::endl ;
//        return ;
//    }
//    else
//		position = myMap.getAttribute<VEC3, VERTEX, MAP>(attrNames[0]) ;

	position = myMap.addAttribute<VEC3, VERTEX, MAP>("position");

	load = new Loader<PFP>(myMap, position, filename.c_str());

	load->computeDecalOfVessels();

	load->extrudeVessels();

	// construct the intersections
//	load->constructIntersection(); // TODO: use ellipsoid instead of sphere

	// merge the vessels and the intersections together
//	load->mergeVessels();

	myMap.check();

/*
	std::cout << "enables MR object..." << std::flush;
	map3MR_PR = new Algo::Volume::MR::Primal::Regular::Map3MR<PFP>(myMap);
	std::cout << "...done" << std::endl;

	map3MR_PR->clearSynthesisFilters();
	map3MR_PR->clearAnalysisFilters();

	LerpQuadOddSynthesisFilter = new Algo::Volume::MR::Primal::Filters::LerpQuadOddSynthesisFilter<PFP>(myMap, position);
	map3MR_PR->addSynthesisFilter(LerpQuadOddSynthesisFilter);

//	ber02EvenSynthesisFilter1 = new Algo::Volume::MR::Primal::Filters::Ber02EvenSynthesisFilter<PFP>(myMap, position,-3.0/8.0);
//	Ber02OddSynthesisFilter = new Algo::Volume::MR::Primal::Filters::Ber02OddSynthesisFilter<PFP>(myMap, position, 0.5);
//	Ber02EvenSynthesisFilter = new Algo::Volume::MR::Primal::Filters::Ber02EvenSynthesisFilter<PFP>(myMap, position, 0.5);
//	Ber02ScaleSynthesisFilter = new Algo::Volume::MR::Primal::Filters::Ber02ScaleSynthesisFilter<PFP>(myMap, position, 0.5);

//	Ber02OddAnalysisFilter = new Algo::Volume::MR::Primal::Filters::Ber02OddAnalysisFilter<PFP>(myMap, position, 0.5);
//	Ber02EvenAnalysisFilter = new Algo::Volume::MR::Primal::Filters::Ber02EvenAnalysisFilter<PFP>(myMap, position, 0.5);
//	Ber02ScaleAnalysisFilter = new Algo::Volume::MR::Primal::Filters::Ber02ScaleAnalysisFilter<PFP>(myMap, position, 0.5);
//	ber02EvenAnalysisFilter1 = new Algo::Volume::MR::Primal::Filters::Ber02EvenAnalysisFilter<PFP>(myMap, position,-3.0/8.0);

//	map3MR_PR->addSynthesisFilter(ber02EvenSynthesisFilter1);
//	map3MR_PR->addSynthesisFilter(Ber02OddSynthesisFilter);
//	map3MR_PR->addSynthesisFilter(Ber02EvenSynthesisFilter);
//	map3MR_PR->addSynthesisFilter(Ber02ScaleSynthesisFilter);

//	map3MR_PR->addAnalysisFilter(Ber02ScaleAnalysisFilter);
//	map3MR_PR->addAnalysisFilter(Ber02EvenAnalysisFilter);
//	map3MR_PR->addAnalysisFilter(Ber02OddAnalysisFilter);
//	map3MR_PR->addAnalysisFilter(ber02EvenAnalysisFilter1);

	std::cout << "add new level (hexa).." << std::flush ;
	map3MR_PR->addNewLevelHexa();
	std::cout << "..done" << std::endl ;

	std::cout << "synthesis..." << std::flush ;
	map3MR_PR->synthesis();
	std::cout << "..done" << std::endl ;
*/

//	md = new Utils::Drawer();
//	md->newList(GL_COMPILE);

//	for(unsigned int i = 0 ; i < load->listVessels.size() ; ++i)
//	{
//		Vessel<VEC3> v = load->listVessels[i];

//		md->begin(GL_POINTS);
//		md->pointSize(8.0f);
//		md->color3f(125.0,0.0,0.0);

//		// Geom::Vec3f color = Utils::color_map_BCGYR(float(v.averageRadius));

//		// md->color(color);
//		// for(unsigned int i = 0 ; i < v.lstCoordinates.size() ; i++)
//		// {
//		// 	md->vertex(v.lstCoordinates[i]);
//		// 	//bb.addPoint(v.lstCoordinates[i]);
//		// }
//		md->end();

////		md->begin(GL_LINES);
////		TraversorE<MAP> me(myMap);
////		for(Dart e = me.begin() ; e != me.end() ; e = me.next())
////		{
////			Dart e1 = v->m_graph->m_map.phi1(e);

////			md->vertex(v->m_positions[e]);
////			md->vertex(v->m_positions[e1]);
////		}
////		md->end();
//	}

//	md->endList();


    color = myMap.addAttribute<VEC3, VOLUME, MAP>("colorVol");


    TraversorCell<MAP, VOLUME> tra(myMap);
    float maxV = 0.0f;
    for (Dart d = tra.begin(); d != tra.end(); d = tra.next())
    {
		float v = Algo::Geometry::convexPolyhedronVolume<PFP>(myMap, d, position);
		color[d] = VEC3(1.0,0,0);
		if (v>maxV)
			maxV=v;
    }
    for (unsigned int i = color.begin(); i != color.end(); color.next(i))
    {
        color[i][0] /= maxV;
        color[i][2] = 1.0f - color[i][0];
    }

	m_topo_render->updateData(myMap, position,  0.8f, 0.8f, 0.8f);
	m_explode_render->updateData<PFP>(myMap, position, color);

    //  bounding box
	Geom::BoundingBox<VEC3> bb = Algo::Geometry::computeBoundingBox<PFP>(myMap, position);

    m_WidthObj = std::max<REAL>(std::max<REAL>(bb.size(0), bb.size(1)), bb.size(2));
    m_PosObj = (bb.min() +  bb.max()) / REAL(2);

    // envoit info BB a l'interface
    setParamObject(m_WidthObj, m_PosObj.data());
    updateGL();
}

int main(int argc, char **argv)
{
    // un peu d'interface
    QApplication app(argc, argv);
    Viewer sqt;

    sqt.show();

    if(argc == 2)
    {
        std::string filename(argv[1]);
        sqt.importMesh(filename);
    }
    else
    {
        sqt.sampleMesh();
    }

    sqt.initGUI();

    // et on attend la fin.
    return app.exec();
}

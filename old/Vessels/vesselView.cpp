/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* version 0.1                                                                  *
* Copyright (C) 2009, IGG Team, LSIIT, University of Strasbourg                *
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
* Web site: https://iggservis.u-strasbg.fr/CGoGN/                              *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/

/******************************************************************************/
/*************************		 AUTHORS       ********************************/
/******************************************************************************/
/*
							  Younis Hijazi
							  Cyril Kern
							  Sylvain Thery
*/


/******************************************************************************/
/*************************		 ABSTRACT       *******************************/
/******************************************************************************/
/*
 
 Fully-automatic branching reconstruction algorithm: application to vascular trees
 ---------------------------------------------------------------------------------
 
 Y. Hijazi, D. Bechmann, D. Cazier, C. Kern and S. Thery
 
 
 Reconstructing tubular structures with high-order branching is a difficult task 
 to perform automatically. Medical applications in particular demand accurate 
 models of such objects that fulfill specific topological and geometric criteria. 
 Indeed, the reconstructed object should be a 2-manifold surface with compact, 
 adaptive geometry. We present a generic algorithm for automatically reconstructing 
 n-furcated tubular surfaces. Our approach relies on a strong underlying topological 
 structure and a novel n-furcation reconstruction algorithm using convex entities.

*/


/******************************************************************************/
/*************************      INCLUDES		*******************************/
/******************************************************************************/

#include <iostream>
#include "os_spec.h"
#include "GLSLShader.h"
#include "glutwin.h"

#include "map_dual/emapd2.h"
#include "import/export.h"
#include "Render/map_glRender.h"
#include "point3d.h"
#include "Modelisation/primitives.h"
#include "Modelisation/extrusion.h"
#include "Modelisation/subdivision.h"

#include <gmtl/Output.h>
#include <gmtl/Matrix.h>
#include <gmtl/Quat.h>
#include <gmtl/Vec.h>
#include <gmtl/Generate.h>
#include <gmtl/Math.h>
#include <gmtl/Intersection.h>

#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <stdlib.h>

#include "Headers/StringTokenizer.hpp"
#include "Headers/Vessel.h"
#include "Headers/MathTools.h"
#include "Headers/Loader.h"
#include "Headers/myMap.h"


/******************************************************************************/
/*************************      TYPEDEFS		*******************************/
/******************************************************************************/
using namespace CGoGN;
using namespace std;

PFP::MAP myMap;



/******************************************************************************/
/*************************      GLUT CLASS		*******************************/
/******************************************************************************/
class myGlutWin: public Utils::SimpleGlutWin
{
public:
	gmtl::Vec4f colDif;
	gmtl::Vec4f colSpec;
	gmtl::Vec4f colClear;
	gmtl::Vec4f colNormal;
	float shininess;
	
	/**
	 * position of object
	 */
	gmtl::Vec3f gPosObj;
	
	/**
	 * width of object
	 */
	float gWidthObj;
	
	/**
	 * factor to apply to normal drawing
	 */
	float normalScaleFactor;
	
	/**
	 * mesh normals
	 */
	std::vector<gmtl::Vec3f> vnormals;
	
	/**
	 * redraw CB
	 */
	void myRedraw();
	
	/**
	 * keyboard CB
	 */
	void myKeyboard(unsigned char keycode, int x, int y);
	
	/**
	 * Display list init
	 */
	void initDL();
	
	/**
	 * Display list init only for normal drawing
	 */
	void initDLNormals(void);
	
	/**
	 * Display list init only for lines drawing
	 */
	void initDLLines(void);
	
	/**
	 * GL initialization
	 */
	void myInitGL();
	
	/**
	 * GL (old school) rendering function (called by init DL)
	 */
	void render(int renderMode);
	
	/**
	 * table of shaders
	 */
	Utils::GLSLShader shaders[8];
	
	/**
	 * inverse the normal when computing normal
	 */
	bool invertedNormals;
	
	/**
	 * inverse object for culling
	 */
	bool invertedObject;
	
	/**
	 * rendering normals ?
	 */
	bool renderNormal;
	
	/**
	 * rendering lines ?
	 */
	bool renderLines;
	
	/**
	 * style of rendering
	 */
	int renderStyle;
	
	/**
	 * Display List for object
	 */
	GLuint dl_obj;
	
	/**
	 * Display List for object (second)
	 */
	GLuint dl_obj2;
	
	/**
	 * Display List for normal
	 */
	GLuint dl_norm;
	
	/**
	 * render mode enum
	 */
	enum { CLEAR=1, LINE, FLAT, GOURAUD, PHONG, NORMAL };
	
	myGlutWin(	int* argc, char **argv, int winX, int winY) :
				SimpleGlutWin(argc,argv,winX,winY),
	invertedNormals(false), invertedObject(false),
	renderNormal(false), renderLines(true), renderStyle(FLAT),
	dl_obj(-1), dl_obj2(-1), dl_norm(-1) {}
};


/******************************************************************************/
/*************************      GLUT CLASS FUNCTIONS		*******************/
/******************************************************************************/
void myGlutWin::myInitGL()
{
	//glClearColor(0.2,0.2,0.2,0.0);
	glClearColor(1.0,1.0,1.0,0.0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	
	// shader init
	if (! shaders[0].loadShaders("phong_vs.txt","phong_ps.txt")) {
		std::cerr<< "No shader !!"<< std::endl;
	}
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
	glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition);
	glEnable(GL_LIGHT0);
	
	SelectorTrue<PFP::MAP::Dart> allDarts;
	Algo::Render::computeNormalVertices<PFP>(myMap, vnormals, allDarts, invertedNormals);
}


void myGlutWin::render(int renderMode)
{
	SelectorTrue<PFP::MAP::Dart> allDartsOri;
	SelectorTrue<PFP::MAP::Dart> allDarts;
	
	switch (renderMode) {
		case PHONG:
			Algo::Render::computeNormalVertices<PFP>(myMap, vnormals, allDarts, invertedNormals);
			glEnable(GL_LIGHTING);
			glEnable(GL_CULL_FACE);
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
			glShadeModel(GL_SMOOTH);
			
			if (invertedObject)
				glFrontFace(GL_CW);
			else
				glFrontFace(GL_CCW);
			
			shaders[0].bind();
			Algo::Render::Direct::renderTriQuadPoly<PFP>(myMap, Algo::Render::Direct::SMOOTH, 1.0f, allDartsOri, vnormals, invertedNormals);
			shaders[0].unbind();
			break;
			
		case GOURAUD:
			Algo::Render::computeNormalVertices<PFP>(myMap, vnormals, allDarts, invertedNormals);
			glEnable(GL_LIGHTING);
			glEnable(GL_CULL_FACE);
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
			glShadeModel(GL_SMOOTH);
			
			if (invertedObject)
				glFrontFace(GL_CW);
			else
				glFrontFace(GL_CCW);
			
			Algo::Render::Direct::renderTriQuadPoly<PFP>(myMap, Algo::Render::Direct::SMOOTH, 1.0f, allDarts, vnormals, invertedNormals);
			break;
			
		case FLAT:
			glEnable(GL_LIGHTING);
			glEnable(GL_CULL_FACE);
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
			glShadeModel(GL_FLAT);
			Algo::Render::computeNormalVertices<PFP>(myMap, vnormals, allDarts, invertedNormals);
			
			if (invertedObject)
				glFrontFace(GL_CW);
			else
				glFrontFace(GL_CCW);
			
			Algo::Render::Direct::renderTriQuadPoly<PFP>(myMap, Algo::Render::Direct::FLAT, 1.0f, allDartsOri, vnormals, invertedNormals);
			break;
			
		case CLEAR:
			glDisable(GL_LIGHTING);
			glEnable(GL_CULL_FACE);
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
			glShadeModel(GL_SMOOTH);
			
			if (invertedObject)
				glFrontFace(GL_CW);
			else
				glFrontFace(GL_CCW);
			
			glColor4fv(colClear.getData());
			
			Algo::Render::Direct::renderTriQuadPoly<PFP>(myMap, Algo::Render::Direct::FLAT, 1.0f, allDartsOri, vnormals, invertedNormals);
			break;
			
			
		case LINE:
			glDisable(GL_LIGHTING);
			glEnable(GL_CULL_FACE);
			glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
			
			if (invertedObject)
				glFrontFace(GL_CW);
			else
				glFrontFace(GL_CCW);
			
			if (renderStyle!=CLEAR)
				glColor3f(0.0f,0.0f,0.0f);
			else
				glColor4fv(colDif.getData());
			
			Algo::Render::Direct::renderTriQuadPoly<PFP>(myMap, Algo::Render::Direct::LINE, 1.0f, allDartsOri, vnormals, invertedNormals);
			break;
			
		case NORMAL:
			glDisable(GL_LIGHTING);
			glColor3fv(colNormal.getData());
			
			Algo::Render::Direct::renderNormalVertices<PFP>(myMap, allDarts, vnormals, normalScaleFactor);
			break;
			
		default:
			break;
	}
}


void myGlutWin::initDL(void)
{
	GLint t1 = glutGet(GLUT_ELAPSED_TIME);
	
	if (glIsList(dl_obj)) glDeleteLists(dl_obj,1);
	dl_obj = glGenLists(1);
	glNewList(dl_obj,GL_COMPILE);
	render(renderStyle);
	glEndList();
	
	GLint t2 = glutGet(GLUT_ELAPSED_TIME);
	GLfloat seconds = (t2 - t1) / 1000.0f;
	std::cout << "GListisation: "<< seconds << "sec" << std::endl;
}


void myGlutWin::initDLNormals(void)
{
	std::vector<gmtl::Vec3f> normals;
	SelectorTrue<PFP::MAP::Dart> allDarts;
	
	if (glIsList(dl_norm)) glDeleteLists(dl_norm,1);
	
	Algo::Render::computeNormalVertices<PFP>(myMap, normals, allDarts, invertedNormals);
	
	if (renderNormal) {
		dl_norm = glGenLists(1);
		glNewList(dl_norm,GL_COMPILE);
		render(NORMAL);
		glEndList();
	}
}


void myGlutWin::initDLLines(void)
{
	SelectorTrue<PFP::MAP> allDarts;
	std::vector<gmtl::Vec3f> normals;
	
	if (glIsList(dl_obj2)) glDeleteLists(dl_obj2,1);
	
	dl_obj2 = glGenLists(1);
	glNewList(dl_obj2,GL_COMPILE);
	render(LINE);
	glEndList();
}


void myGlutWin::myRedraw(void)
{
	GLfloat black[4]= {0.0f,0.0f,0.0f,1.0f};
	//GLfloat amb[4]= {0.2f,0.1f,0.1f,1.0f};
	GLfloat amb[4]= {1.0f,0.1f,0.6f,1.0f};
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	
	float sc = 50./gWidthObj;
	glScalef(sc,sc,sc);
	glTranslatef(-gPosObj[0],-gPosObj[1],-gPosObj[2]);
	
	if (glIsList(dl_obj2)) glCallList(dl_obj2);
	if (glIsList(dl_norm)) glCallList(dl_norm);
	
	glEnable( GL_POLYGON_OFFSET_FILL );
	glPolygonOffset( 1.0f, 1.0f );
	
	if (glIsList(dl_obj)) {
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, colDif.getData());
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
		
		if (renderStyle!=PHONG) {
			glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,black);
			glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, 1.0f );
		}
		else {
			glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,colSpec.getData());
			glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, shininess );
		}
		glCallList(dl_obj);
	}
	glDisable( GL_POLYGON_OFFSET_FILL );
	glPopMatrix();
}


void myGlutWin::myKeyboard(unsigned char keycode, int x, int y)
{
	SelectorTrue<PFP::MAP::Dart> allDarts;
	GLint t1 = glutGet(GLUT_ELAPSED_TIME);
	GLint t2;
	GLfloat seconds;
	
	switch(keycode) {
		case 'c':
			Algo::Modelisation::CatmullClark<PFP>(myMap,allDarts);
			initDL();
			if (renderLines) initDLLines();
			else {
				if (glIsList(dl_obj2))
					glDeleteLists(dl_obj2,1);
			}
			glutPostRedisplay();
			break;
		case 'L':
			Algo::Modelisation::LoopSubdivision<PFP>(myMap,allDarts);
			initDL();
			if (renderLines) initDLLines();
			else {
				if (glIsList(dl_obj2))
					glDeleteLists(dl_obj2,1);
			}
			glutPostRedisplay();
			break;
		case 'S':
			normalScaleFactor *= 1.1f;
			initDLNormals();
			glutPostRedisplay();
			break;
		case 's':
			normalScaleFactor /= 1.1f;
			initDLNormals();
			glutPostRedisplay();
			break;
		case 'v':
			for(int i=0; i<20; ++i) {
				myRedraw();
				glutSwapBuffers();
			}
			t2 = glutGet(GLUT_ELAPSED_TIME);
			seconds = (t2 - t1) / 1000.0f;
			std::cout << "fps: "<< 20.0/seconds << "sec" << std::endl;
			break;
		case 'u':
			myMap.reverseOrientation();
			std::cout << "reverseOrientation" << std::endl;		
			glutPostRedisplay();
			break;
		case 'd':
			setFoc(getFoc()/1.05f);
			reshape(-1,-1);
			glutPostRedisplay();
			break;
		case 'D':
			setFoc(getFoc()*1.05f);
			reshape(-1,-1);
			glutPostRedisplay();
			break;
		case 'i':
			invertedNormals = !invertedNormals;
			initDL();
			glutPostRedisplay();
			break;
		case 'o':
			invertedObject = !invertedObject;
			initDL();
			glutPostRedisplay();
			break;
		case 'O':
			invertedObject = !invertedObject;
			invertedNormals = !invertedNormals;
			initDL();
			glutPostRedisplay();
			break;
		case 'n':
			renderNormal = ! renderNormal;
			if (renderNormal) initDLNormals();
			else {
				if (glIsList(dl_norm)) glDeleteLists(dl_norm,1);
			}
			glutPostRedisplay();
			break;
		case 'f':
			renderStyle = FLAT;
			initDL();
			glutPostRedisplay();
			break;
		case 'g':
			renderStyle = GOURAUD;
			initDL();
			glutPostRedisplay();
			break;
		case 'p':
			renderStyle = PHONG;
			initDL();
			glutPostRedisplay();
			break;
		case 'b':
			renderStyle = CLEAR;
			initDL();
			glutPostRedisplay();
			break;
		case 'l':
			renderLines = !renderLines;
			if (renderLines) initDLLines();
			else {
				if (glIsList(dl_obj2)) glDeleteLists(dl_obj2,1);
			}
			glutPostRedisplay();
			break;
		case 'z':
			shininess /= 1.1f;
			glutPostRedisplay();
			break;
		case 'Z':
			shininess *= 1.1f;
			glutPostRedisplay();
			break;
	}
}


/******************************************************************************/
/*************************              MAIN		***************************/
/******************************************************************************/
int main(int argc, char **argv)
{
	myGlutWin mgw(&argc,argv,800,800);
	mgw.init();
	gmtl::Vec3f gMax;
	gmtl::Vec3f gMin;

	// load the vessel network
	string defaultPath="./dot/";
	std::string filename=defaultPath+std::string("porte_t1.dot");
	Loader * load = new Loader(filename.c_str());
	
	cout << "Number of vessels = " << load->listVessels.size() << endl;
	
	// preprocessing: cut the vessels in the intersection areas
	load->computeDecalOfVessels(); // TODO: tune this function for less neighborhood cut
	
	// extrude the vessels
	load->extrudeVessels();

	// construct the intersections
	load->constructIntersection(); // TODO: use ellipsoid instead of sphere
	
	// merge the vessels and the intersections together
	load->mergeVessels();
	
	// compute the bounding box
	Algo::Render::computeBoundingBox<PFP>(myMap,gMin,gMax);
	std::cout <<"BB:"<<gMin<<" / "<<gMax<<std::endl;

	mgw.gPosObj = (gMax+gMin)/2.0f;
	float tailleX = gMax[0] - gMin[0];
	float tailleY = gMax[1] - gMin[1];
	float tailleZ = gMax[2] - gMin[2];

	mgw.gWidthObj = std::max( std::max(tailleX,tailleY),tailleZ);
	mgw.normalScaleFactor = std::min( std::min(tailleX,tailleY),tailleZ) / 50.0f;
	mgw.colClear = gmtl::Vec4f(0.2f,0.2f,0.2f,0.1);
	mgw.colDif = gmtl::Vec4f(0.8f,0.9f,0.7f,1.0f);
	mgw.colSpec = gmtl::Vec4f(0.9f,0.9f,0.9f,1.0f);
	mgw.colNormal = gmtl::Vec4f(1.0f,0.0f,0.0f,1.0f);
	mgw.shininess=80.0f;

	std::cout << "Nb sommets = " << myMap.getNbVertices() << std::endl;
	std::cout << "Nb aretes  = " << myMap.getNbEdges() << std::endl;
	std::cout << "Nb faces   = " << myMap.getNbFaces() << std::endl;

	std::cout << "closing map ... ";
	Marker m = myMap.closeMap(true);
	myMap.setBoundaryMarkerValue(m);
	std::cout << "done" << std::endl;

	std::cout << "Nb marked darts = " << myMap.getNbMarked(m) << std::endl;
	std::cout << "Nb sommets = " << myMap.getNbVertices() << std::endl;
	std::cout << "Nb aretes  = " << myMap.getNbEdges() << std::endl;
	std::cout << "Nb faces   = " << myMap.getNbFaces() << std::endl;
	
	mgw.myInitGL();
	mgw.initDL();
	mgw.initDLLines();
	mgw.mainLoop();
	return 0;
}

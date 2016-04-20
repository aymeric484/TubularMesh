cmake_minimum_required(VERSION 2.8)

project(ConvexHull)


SET(CGoGN_ROOT_DIR ${CMAKE_SOURCE_DIR}/../../cgogn CACHE STRING "CGoGN root dir")
include(${CGoGN_ROOT_DIR}/apps_cmake.txt)

include_directories( 
	${CMAKE_CURRENT_SOURCE_DIR} 
	${CMAKE_CURRENT_BINARY_DIR}
)

set(QT_USE_QTXML true)

file(   GLOB
        includes
        *.hpp
        *.h
)


QT4_WRAP_UI( source_ui viewer.ui )
QT4_WRAP_CPP( source_moc viewer.h )

add_executable( viewer viewer.cpp ${includes} ${source_moc} ${source_ui} )
target_link_libraries( viewer ${CGoGN_LIBS} ${NUMERICAL_LIBS} ${CGoGN_EXT_LIBS})
qt4_use_modules(viewer Core Gui OpenGL Xml Svg)

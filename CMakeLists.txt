cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(tubularmesh
	LANGUAGES CXX
)

#### Default build type
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()

#### CGOGN_PATH / CGOGN_SOURCE_DIR
set(CGOGN_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../CGoGN_2")
set(CGOGN_SOURCE_DIR ${CGOGN_PATH})

set(CGOGN_THIRDPARTY_QOGLVIEWER_INCLUDE_DIR "${CGOGN_PATH}/thirdparty/libQGLViewer/QOGLViewer" CACHE PATH "QOGLViewer include directory")
set(CGOGN_THIRDPARTY_EIGEN3_INCLUDE_DIR "${CGOGN_PATH}/thirdparty/eigen-3.2.8" CACHE PATH "Include directory for find_package(Eigen3)")
set(CGOGN_THIRDPARTY_TINYXML2_INCLUDE_DIR "${CGOGN_PATH}/thirdparty/TinyXml2" CACHE PATH "TinyXml2 include directory")
set(CGOGN_THIRDPARTY_LM6_INCLUDE_DIR "${CGOGN_PATH}/thirdparty/lm6" CACHE PATH "LM6 include directory")
set(CGOGN_THIRDPARTY_PLY_INCLUDE_DIR "${CGOGN_PATH}/thirdparty/ply" CACHE PATH "Ply include directory")

#### Build configuration
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)


find_package(Qt5Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)


set(HEADER_FILES
        branch.h
        config.h
        window.h
        coucheconcentrique.h
        viewer.h
        intersection.h
        squelette.h
)

set(SOURCE_FILES
	main.cpp
        branch.cpp
        window.cpp
        coucheconcentrique.cpp
        viewer.cpp
        intersection.cpp
        squelette.cpp
)

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})

# use of target_compile_options to have a transitive c++11 flag
if(NOT MSVC)
	target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11")
endif()
if(MSVC)
	target_compile_options(${PROJECT_NAME} PUBLIC "-D_USE_MATH_DEFINES")
endif()

link_directories(${CGOGN_PATH}/../CGoGN_2-build/lib)

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_QOGLVIEWER_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_EIGEN3_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_TINYXML2_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_LM6_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_THIRDPARTY_PLY_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CGOGN_SOURCE_DIR}>
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(${PROJECT_NAME}
        ${CGOGN_PATH}/../CGoGN_2-build/lib/libcgogn_core.so
        ${CGOGN_PATH}/../CGoGN_2-build/lib/libcgogn_rendering.so
        ${CGOGN_PATH}/../CGoGN_2-build/lib/libcgogn_io.so
	${CGOGN_PATH}/../CGoGN_2-build/lib/libQOGLViewer.so
	Qt5::Widgets
)

cmake_minimum_required( VERSION 3.0 )

## Use the variable PROJECT_NAME for changing the target name
set( PROJECT_NAME "5552_Code" )

## Set our project name
project( ${PROJECT_NAME} )
if(WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 /W4")
else()
	set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -g -Wall -Wextra -Wpedantic" )
endif()

#include(clang-format.cmake)

find_package( PythonLibs 3 REQUIRED )
include_directories( SYSTEM ${PYTHON_INCLUDE_DIRS} )

if (WIN32)
	SET(EIGEN3_INCLUDE_DIR "" CACHE PATH "eigen3 include directory")
elseif (UNIX)
	find_package( Eigen3 REQUIRED )
endif()
include_directories( SYSTEM ${EIGEN3_INCLUDE_DIR} )

find_package( Threads REQUIRED )

if (WIN32)
	find_package( OpenGL REQUIRED )
	SET(GLM_DIR "" CACHE PATH "glm directory")
	SET(GLEW_DIR "" CACHE PATH "GLEW directory")
	SET(SDL2_DIR "" CACHE PATH "SDL2 directory")
	#SET(GLM_LIBRARY "" CACHE FILEPATH "glm library")
	
	SET(GLEW_LIBRARY )
	SET(SDL2_LIBRARY )
	SET(SDL2main_LIBRARY )
	message( ${GLEW_DIR}/include )
	include_directories( SYSTEM ${GLM_DIR} "${GLEW_DIR}/include" "${SDL2_DIR}/include")
elseif (UNIX)
	find_package( OpenGL REQUIRED )
	find_package( glm REQUIRED )
	find_package( GLEW REQUIRED )
	find_package( SDL2 REQUIRED )
endif()

include_directories( src )
add_subdirectory( src )

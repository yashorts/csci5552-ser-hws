cmake_minimum_required( VERSION 3.0 )

## Use the variable PROJECT_NAME for changing the target name
set( PROJECT_NAME "5552_Code" )

## Set our project name
project( ${PROJECT_NAME} )

# Build GLFW
# Also disable building some of the extra things GLFW has (examples, tests, docs)
set(GLFW_BUILD_EXAMPLES  OFF CACHE BOOL " " FORCE)
set(GLFW_BUILD_TESTS     OFF CACHE BOOL " " FORCE)
set(GLFW_BUILD_DOCS      OFF CACHE BOOL " " FORCE)

# Now actually run cmake on the CMakeLists.txt file found inside of the GLFW directory
add_subdirectory(ext/glfw)

if(WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 /W4")
else()
	set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -g -Wall -Wextra -Wpedantic" )
endif()

#include(clang-format.cmake)

find_package( PythonLibs 3 REQUIRED )
include_directories( SYSTEM ${PYTHON_INCLUDE_DIRS} )

include_directories( SYSTEM ext/eigen )
include_directories( SYSTEM ext/glm )
include_directories( SYSTEM ext/glad/include )
include_directories( SYSTEM ext/glfw/include )

find_package( Threads REQUIRED )

include_directories( src )

add_subdirectory( ext )
add_subdirectory( src )

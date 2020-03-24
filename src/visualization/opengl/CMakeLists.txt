add_definitions( -DGL_SHADER_PATH="${CMAKE_SOURCE_DIR}/src/visualization/opengl/shaders/" )

set(OpenGL_GL_PREFERENCE "GLVND")
find_package( OpenGL REQUIRED )
find_package( glm REQUIRED )
find_package( GLEW REQUIRED )
find_package( SDL2 REQUIRED )

set( GL_VIS_HEADERS glvis.h opengl_utils.h )
set( GL_VIS_SOURCE glvis.cpp opengl_util.cpp )

add_library( gl_visualization ${GL_VIS_SOURCE} ${GL_VIS_HEADERS} )
target_link_libraries( gl_visualization GL glm GLEW SDL2 ${CMAKE_THREAD_LIBS_INIT} )
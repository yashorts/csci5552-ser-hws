add_definitions( -DGL_SHADER_PATH="${CMAKE_SOURCE_DIR}/src/visualization/opengl/shaders/" )

set( GL_VIS_HEADERS glvis.h opengl_utils.h )
set( GL_VIS_SOURCE glvis.cpp opengl_util.cpp )

add_library( gl_visualization ${GL_VIS_SOURCE} ${GL_VIS_HEADERS} )	
if (UNIX)
	target_link_libraries( gl_visualization GL glm GLEW SDL2 ${CMAKE_THREAD_LIBS_INIT} )
elseif (WIN32)
	target_link_libraries(gl_visualization ${OPENGL_LIBRARIES}
	                                       ${GLEW_DIR}/lib/Release/x64/glew32s.lib
										   ${SDL2_DIR}/lib/x64/SDL2.lib
										   ${SDL2_DIR}/lib/x64/SDL2main.lib
										   ${CMAKE_THREAD_LIBS_INIT} )
endif()
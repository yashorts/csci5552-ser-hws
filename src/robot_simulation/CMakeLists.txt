add_definitions( -DSCENE_PATH="${CMAKE_SOURCE_DIR}/src/robot_simulation/scenes/" )

set( SIM_HEADERS simulator.h line_segment.h )
set( SIM_SOURCE simulator.cpp line_segment.cpp )

add_library( simulator ${SIM_SOURCE} ${SIM_HEADERS} )
target_link_libraries( simulator ${CMAKE_THREAD_LIBS_INIT} )

add_executable( test_sim test_simulator.cpp )
target_link_libraries( test_sim simulator visualization )
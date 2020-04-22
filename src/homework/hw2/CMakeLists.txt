set( HW2_HEADERS hw2.h )
set( HW2_SOURCE hw2.cpp hw2_test.cpp )

add_executable( hw2_test ${HW2_HEADERS} ${HW2_SOURCE} )
target_link_libraries( hw2_test visualization gl_visualization )

set( HW2_SOLN_HEADERS hw2.h )
set( HW2_SOLN_SOURCE hw2_soln.cpp hw2_test.cpp )

add_executable( hw2_soln_test ${HW2_SOLN_HEADERS} ${HW2_SOLN_SOURCE} )
target_link_libraries( hw2_soln_test visualization gl_visualization )
set( HW1_HEADERS hw1.h )
set( HW1_SOURCE hw1.cpp hw1_test.cpp )

add_executable( hw1_test ${HW1_HEADERS} ${HW1_SOURCE} )
target_link_libraries( hw1_test visualization )

set( HW1_SOLN_HEADERS hw1.h )
set( HW1_SOLN_SOURCE hw1_soln.cpp hw1_test.cpp )

add_executable( hw1_soln_test ${HW1_SOLN_HEADERS} ${HW1_SOLN_SOURCE} )
target_link_libraries( hw1_soln_test visualization )
set( HW1_HEADERS hw1.h )
set( HW1_SOURCE hw1.cpp hw1_test.cpp )

add_executable( hw1_test ${HW1_HEADERS} ${HW1_SOURCE} )
target_link_libraries( hw1_test visualization )
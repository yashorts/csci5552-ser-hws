set( HW0_HEADERS hw0.h )
set( HW0_SOURCE hw0.cpp hw0_test.cpp )

add_executable( hw0_test ${HW0_HEADERS} ${HW0_SOURCE} )
target_link_libraries( hw0_test visualization )

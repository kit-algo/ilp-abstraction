find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
find_package( Threads )

add_executable (run_tests main.cpp)

set_target_properties(run_tests
                      PROPERTIES COMPILE_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage -Wall -Wextra -pedantic -Wfloat-equal -Wundef -Wshadow -Wpointer-arith -Wcast-align -Wstrict-overflow=5 -Wwrite-strings -Wcast-qual -Wswitch-default -Wswitch-enum -Wconversion -Wunreachable-code -Wfloat-equal -Wswitch-default"
                      )
set_target_properties(run_tests
                      PROPERTIES LINK_FLAGS "-fprofile-arcs -ftest-coverage"
                      )

target_link_libraries (run_tests LINK_PUBLIC ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT}
        ${LIBS})

add_custom_target(clion_test_dummy SOURCES basic.hpp main.cpp ../src/common.hpp ../src/common.cpp
        ../src/ilpa_cplex.cpp ../src/ilpa_cplex.hpp ../src/ilpa_gurobi.cpp ../src/ilpa_gurobi.hpp)
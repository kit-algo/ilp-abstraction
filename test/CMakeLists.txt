add_subdirectory(gtest)

include_directories(SYSTEM ${GTEST_INCLUDE_DIRS})
find_package( Threads )

add_executable (run_tests main.cpp)
add_dependencies(run_tests googletest)

set_target_properties(run_tests
                      PROPERTIES COMPILE_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage -Wall -Wextra -pedantic -Wfloat-equal -Wundef -Wshadow -Wpointer-arith -Wcast-align -Wstrict-overflow=5 -Wwrite-strings -Wcast-qual -Wswitch-default -Wswitch-enum -Wconversion -Wunreachable-code -Wfloat-equal -Wswitch-default"
                      )
set_target_properties(run_tests
                      PROPERTIES LINK_FLAGS "-fprofile-arcs -ftest-coverage"
                      )

target_link_libraries (run_tests LINK_PUBLIC  ${GTEST_LIBS_DIR}/libgtest.a ${GTEST_LIBS_DIR}/libgtest_main.a ${CMAKE_THREAD_LIBS_INIT} ${LIBS} ${CMAKE_DL_LIBS})

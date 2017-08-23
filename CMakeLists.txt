cmake_minimum_required(VERSION 3.5)
project(ilp_abstraction)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

# Find Gurobi
find_package(GUROBI)
if (GUROBI_FOUND)
    include_directories(${GUROBI_INCLUDE_DIRS})
    set(LIBS ${LIBS} ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARIES} ${GUROBI_CXX_LIBRARY})
endif(GUROBI_FOUND)

set(SOURCE_FILES main.cpp)
add_executable(ilp_abstraction ${SOURCE_FILES})
target_link_libraries(ilp_abstraction ${LIBS})

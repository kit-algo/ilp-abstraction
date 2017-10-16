cmake_minimum_required(VERSION 3.5)
project(ilp_abstraction)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

# Find Gurobi
find_package(GUROBI REQUIRED)
include_directories(SYSTEM ${GUROBI_INCLUDE_DIRS})
set(LIBS ${LIBS} ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARIES} ${GUROBI_CXX_LIBRARY})

# Find CPLEX
find_package(CPLEX REQUIRED)
include_directories(SYSTEM ${CPLEX_INCLUDE_DIRS})
set(LIBS ${LIBS} ${CPLEX_LIBRARIES})

add_subdirectory(test)
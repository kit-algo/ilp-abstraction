#include <gtest/gtest.h>

#include "../src/ilpa_gurobi.hpp"
#include "../src/ilpa_cplex.hpp"

#include "basic.hpp"
#include "param_passthrough.hpp"

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

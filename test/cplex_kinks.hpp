//
// Created by lukas on 07.01.18.
//

#ifndef ILP_ABSTRACTION_CPLEX_KINKS_HPP
#define ILP_ABSTRACTION_CPLEX_KINKS_HPP

namespace ilpabstraction {
namespace testing {

TEST(CPLEXKinks, test_large_seeds) {
CPLEXInterface solver(true);
auto m = solver.create_model();
m.set_param(ParamType::SEED, std::numeric_limits<unsigned long>::max());
}

}
}

#endif //ILP_ABSTRACTION_CPLEX_KINKS_HPP

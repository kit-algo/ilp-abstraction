//
// Created by lukas on 05.01.18.
//

#ifndef ILP_ABSTRACTION_PARAM_PASSTHROUGH_HPP
#define ILP_ABSTRACTION_PARAM_PASSTHROUGH_HPP


namespace ilpabstraction {
namespace testing {

TEST(ParamPassthrough, test_gurobi) {
	GurobiInterface solver(true);
	auto m = solver.create_model();
	m.set_param_passthrough(GRB_DoubleParam_TimeLimit, 1);
}

TEST(ParamPassthrough, test_cplex) {
	CPLEXInterface solver(true);
	auto m = solver.create_model();
	m.set_param_passthrough(IloCplex::Param::MIP::Display, 0);
}

} // namespace testing
} // namespace ilpabstraction

#endif //ILP_ABSTRACTION_PARAM_PASSTHROUGH_HPP

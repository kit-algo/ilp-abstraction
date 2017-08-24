#include <iostream>

#include "src/ilpa_gurobi.hpp"

using namespace ilpabstraction;

int
main()
{
	GurobiInterface grbi(true);
	GurobiInterface::Model m = grbi.create_model();

	auto var = m.add_var(VariableType::INTEGER, GurobiInterface::NEGATIVE_INFTY,
	          GurobiInterface::INFTY);
	m.add_constraint(GurobiInterface::NEGATIVE_INFTY, var, 10);

	m.solve();
}
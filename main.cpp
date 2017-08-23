#include <iostream>

#include "src/ilpa_gurobi.hpp"

using namespace ilpabstraction;

int
main()
{
	GurobiInterface grbi(true);
	GurobiInterface::Model m = grbi.create_model();

	auto var = m.add_var(VariableType::INTEGER, GurobiInterface::NEGATIVE_INFINITY,
	          GurobiInterface::INFINITY);
	m.add_constraint(GurobiInterface::NEGATIVE_INFINITY, var, 10);

	m.solve();
}
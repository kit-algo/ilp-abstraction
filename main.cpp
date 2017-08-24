#include <iostream>

#include "src/ilpa_gurobi.hpp"
#include "src/ilpa_cplex.hpp"

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


	CPLEXInterface cplex(true);
	CPLEXInterface::Model cm = cplex.create_model();

	auto cvar = cm.add_var(VariableType::INTEGER, CPLEXInterface::NEGATIVE_INFTY,
	                       CPLEXInterface::INFTY);
	cm.add_constraint(CPLEXInterface::NEGATIVE_INFTY, cvar, 10);

	cm.solve();
}
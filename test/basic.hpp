//
// Created by lukas on 12.10.17.
//

#ifndef ILP_ABSTRACTION_BASIC_HPP
#define ILP_ABSTRACTION_BASIC_HPP

namespace ilpabstraction {
namespace testing {

template<class Solver>
class BasicTest {
public:
	using Variable = typename Solver::Variable;
	using Expression = typename Solver::Expression;
	using Model = typename Solver::Model;

	void test_minimal() {
		auto var = this->m.add_var(VariableType::INTEGER, Solver::NEGATIVE_INFTY,
		                           Solver::INFTY);
		m.add_constraint(Solver::NEGATIVE_INFTY, var, 10);

		auto obj = this->s.create_expression();
		obj += var;
		m.set_objective(obj, ObjectiveType::MAXIMIZE);

		m.solve();

		ASSERT_EQ(m.get_objective_value(), 10);
	}

	void test_math_ops() {
		auto var = this->m.add_var(VariableType::INTEGER, Solver::NEGATIVE_INFTY,
		                           Solver::INFTY);
		auto var2 = this->m.add_var(VariableType::INTEGER, Solver::NEGATIVE_INFTY,
		                           Solver::INFTY);
		auto expr = this->s.create_expression();
		expr += var;

		m.add_constraint(0, var, 10);  // variable vs integer
		m.add_constraint(0, expr, 20); // expression vs integer;
		m.add_constraint(0u, expr, 20u); // expression vs unsigned integer;
		m.add_constraint(0.5, expr, 20.5); // expression vs double;
		m.add_constraint(expr, var2, Solver::INFTY); // expression vs variable

		auto obj = this->s.create_expression();
		obj += var;
		m.set_objective(obj, ObjectiveType::MAXIMIZE);

		m.solve();

		ASSERT_EQ(m.get_objective_value(), 10);
	}

	void test_sos1() {
		auto var1 = this->m.add_var(VariableType::INTEGER, Solver::NEGATIVE_INFTY,
		                           10);
		auto var2 = this->m.add_var(VariableType::INTEGER, Solver::NEGATIVE_INFTY,
		                            20);

		auto obj = this->s.create_expression();
		obj += var1;
		obj += var2;
		m.set_objective(obj, ObjectiveType::MAXIMIZE);

		std::vector<Variable> vs;
		vs.push_back(var1);
		vs.push_back(var2);

		std::vector<double> weights;
		weights.push_back(1.0);
		weights.push_back(1.1);

		m.add_sos1_constraint(vs, weights);

		m.solve();

		ASSERT_EQ(m.get_objective_value(), 20);
	}

	BasicTest() : s(true), m(s.create_model()) {}
	virtual ~BasicTest() {}

private:

	std::vector<Variable> vars;
	std::vector<Expression> exprs;
	Solver s;
	Model m;
};

TEST(BasicTest, test_minimal) {
	BasicTest<GurobiInterface> test_grb;
	test_grb.test_minimal();
	BasicTest<CPLEXInterface> test_cplex;
	test_cplex.test_minimal();
}

TEST(BasicTest, test_sos1) {
BasicTest<GurobiInterface> test_grb;
test_grb.test_sos1();
BasicTest<CPLEXInterface> test_cplex;
test_cplex.test_sos1();
}

TEST(BasicTest, test_math_ops) {
BasicTest<GurobiInterface> test_grb;
test_grb.test_math_ops();
BasicTest<CPLEXInterface> test_cplex;
test_cplex.test_math_ops();
}

}
}

#endif //ILP_ABSTRACTION_BASIC_HPP

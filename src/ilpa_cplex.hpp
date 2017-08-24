//
// Created by lukas on 24.08.17.
//

#ifndef ILP_ABSTRACTION_CPLEX_HPP
#define ILP_ABSTRACTION_CPLEX_HPP

#include "common.hpp"

#define IL_STD 1
#include <ilcplex/ilocplex.h>

namespace ilpabstraction {

class CPLEXExpression : public IloExpr {
public:
	~CPLEXExpression();
	using IloExpr::IloExpr;
};

class CPLEXVariable : public IloNumVar {
public:
	~CPLEXVariable();
	using IloNumVar::IloNumVar;
	operator CPLEXExpression() const;
};

class CPLEXInterface : public Interface<CPLEXVariable, CPLEXExpression>
{
public:
	using Base = Interface<CPLEXVariable, CPLEXExpression>;

	static constexpr const auto INFTY = +IloInfinity;
	static constexpr const auto NEGATIVE_INFTY = -IloInfinity;

	class Model {
	public:
		template <class LowerValType, class UpperValType>
		inline void add_constraint(LowerValType lower_bound, Expression expr, UpperValType upper_bound,
		                           std::string name = "");

		template <class LowerValType, class UpperValType>
		inline Variable add_var(VariableType type, LowerValType lower_bound,
		                        UpperValType upper_bound, std::string name = "");

		inline void commit_variables();

		inline void set_objective(Expression expr, ObjectiveType type);

		inline void solve();

		inline double get_variable_assignment(const Variable &var) const;
		inline double get_objective_value() const;
		inline double get_bound() const;

		inline unsigned int get_variable_count();
		inline unsigned int get_constraint_count();
		inline unsigned int get_nonzero_count();

		inline ModelStatus get_status() const;
		inline bool has_feasible() const;

		template <class T>
		void set_param(ParamType type, T val);

		inline ~Model();
	protected:
		inline Model(CPLEXInterface *interface);

		CPLEXInterface *interface;

		friend class CPLEXInterface;

		IloCplex cplex;
		IloModel m;

		ModelStatus status;

		bool cplex_up_to_date;

		void extract();
	};

	inline CPLEXInterface(bool auto_commit_variables);
	inline ~CPLEXInterface();

	inline Model create_model();

private:
	IloEnv env;
};

} // namespace ilpabstraction

#include "ilpa_cplex.cpp"

#endif //ILP_ABSTRACTION_CPLEX_HPP

//
// Created by lukas on 24.08.17.
//

#ifndef ILP_ABSTRACTION_CPLEX_HPP
#define ILP_ABSTRACTION_CPLEX_HPP

#include "common.hpp"

#define IL_STD 1
#include <ilcplex/ilocplex.h>

namespace ilpabstraction {

class CPLEXVariable : public IloNumVar {
public:
	inline ~CPLEXVariable();
	using IloNumVar::IloNumVar;
	//operator CPLEXExpression() const;
};

class CPLEXExpression : public IloExpr {
public:
	inline CPLEXExpression() : initialized(false) {};
	inline CPLEXExpression(IloEnv env) : IloExpr(env), initialized(true) {};
	inline CPLEXExpression(CPLEXVariable var) : IloExpr(var), initialized(true) {};

	template <class T>
	CPLEXExpression(T param) : IloExpr(param), initialized(true) {};

	inline ~CPLEXExpression();
private:
	bool initialized;
};

class CPLEXInterface : public Interface<CPLEXVariable, CPLEXExpression>
{
public:
	using Base = Interface<CPLEXVariable, CPLEXExpression>;

	static constexpr const char * NAME = "CPLEX";

	static constexpr const auto INFTY = +IloInfinity;
	static constexpr const auto NEGATIVE_INFTY = -IloInfinity;

	class Model : public Base::Model
	{
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

		inline void extract();
	};

	inline CPLEXInterface(bool auto_commit_variables);
	inline ~CPLEXInterface();

	inline Model create_model();
	inline Expression create_expression();
	inline Variable create_variable();

private:
	IloEnv env;
};

} // namespace ilpabstraction

inline auto operator* (ilpabstraction::CPLEXVariable & var, unsigned int i) {
	assert(i <= std::numeric_limits<int>::max());
	return (IloNumVar)var * (int)i;
}
inline auto operator* (unsigned int i, ilpabstraction::CPLEXVariable & var) {
	assert(i <= std::numeric_limits<int>::max());
	return (IloNumVar)var * (int)i;
}
inline auto operator* (ilpabstraction::CPLEXVariable & var, int i) {
	assert(i <= std::numeric_limits<int>::max());
	return (IloNumVar)var * i;
}
inline auto operator* (int i, ilpabstraction::CPLEXVariable & var) {
	return (IloNumVar)var * i;
}
inline auto operator* (ilpabstraction::CPLEXVariable & var, double d) {
	return (IloNumVar)var * d;
}
inline auto operator* (double d, ilpabstraction::CPLEXVariable & var) {
	return (IloNumVar)var * d;
}

inline auto operator* (unsigned int i, ilpabstraction::CPLEXExpression & expr) {
	assert(i <= std::numeric_limits<int>::max());
	return (IloExpr)expr * (int)i;
}
inline auto operator* (ilpabstraction::CPLEXExpression & expr, unsigned int i) {
	assert(i <= std::numeric_limits<int>::max());
	return (IloExpr)expr * (int)i;
}
inline auto operator* (double d, ilpabstraction::CPLEXExpression & expr) {
	return (IloExpr)expr * d;
}
inline auto operator* (ilpabstraction::CPLEXExpression & expr, double d) {
	return (IloExpr)expr * d;
}

inline auto operator* (unsigned int i, const IloNumLinExprTerm & expr) {
	assert(i <= std::numeric_limits<int>::max());
	return expr * (int)i;
}
inline auto  operator* (const IloNumLinExprTerm & expr, unsigned int i) {
	assert(i <= std::numeric_limits<int>::max());
	return expr * (int)i;
}

#include "ilpa_cplex.cpp"

#endif //ILP_ABSTRACTION_CPLEX_HPP

//
// Created by lukas on 24.08.17.
//

#ifndef ILP_ABSTRACTION_CPLEX_HPP
#define ILP_ABSTRACTION_CPLEX_HPP

#include "common.hpp"

#define IL_STD 1
#include <ilcplex/ilocplex.h>
#include <unordered_map>

namespace ilpabstraction {

namespace cplex_internal {
	class CallbackContext {
	public:
		inline CallbackContext(IloCplex::MIPInfoCallbackI * cplex_cb);
		inline double get_objective_value() const;
		inline double get_bound() const;
		inline double get_gap() const;
		inline double get_time() const;
		inline int get_processed_nodes() const;
		inline int get_open_nodes() const;

	private:
		IloCplex::MIPInfoCallbackI * cplex_cb;
	};
} // namespace cplex_internal

class CPLEXVariable : public IloNumVar {
public:
	using IloNumVar::IloNumVar;

	bool operator==(const CPLEXVariable & other) const {
		return other.getId() == this->getId();
	}
};

class CPLEXExpression : public IloExpr {
public:
	using IloExpr::IloExpr;
};

using CPLEXConstraint = std::pair<IloRange, IloRange>;

class CPLEXInterface : public Interface<CPLEXVariable, CPLEXExpression, CPLEXConstraint,
                                        cplex_internal::CallbackContext>
{
public:
	using Base = Interface<CPLEXVariable, CPLEXExpression, CPLEXConstraint,
	                       cplex_internal::CallbackContext>;
	using Callback = Base::Callback;
	using CallbackContext = Base::CallbackContext;

	static constexpr const char * NAME = "CPLEX";

	static constexpr const auto INFTY = +IloInfinity;
	static constexpr const auto NEGATIVE_INFTY = -IloInfinity;

	class Model : public Base::Model
	{
	public:
		template <class LowerValType, class UpperValType>
		inline Constraint add_constraint(LowerValType lower_bound, Expression expr,
		                                 UpperValType upper_bound, std::string name = "");

		template <class LowerValType, class UpperValType>
		inline Variable add_var(VariableType type, LowerValType lower_bound,
		                        UpperValType upper_bound, std::string name = "");

		inline void add_sos1_constraint(const std::vector<Variable> & vars,
		                                const std::vector<double> & weights, std::string name = "");

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

		template <class T>
		void set_start(Variable & var, T val);

		template <class LowerValType, class UpperValType>
		void change_var_bounds(Variable & var, LowerValType lower_bound,
		                       UpperValType upper_bound);
		template <class UpperValType>
		void change_constraint_ub(Constraint & constr, UpperValType upper_bound);
		template <class LowerValType>
		void change_constraint_lb(Constraint & constr, LowerValType lower_bound);
		
		inline void write(const std::string & filename);
		inline void write_solution(const std::string & filename);

		inline ~Model();
	protected:

		class CallbackAdapter : public IloCplex::MIPInfoCallbackI {
		public:
			inline static IloCplex::Callback create(IloEnv env, Model * model);
			inline virtual IloCplex::CallbackI * duplicateCallback() const override;
		protected:
			inline virtual void main() override;
		private:
			inline CallbackAdapter(IloEnv env, Model * model);

			Model * model;
		};

		inline Model(CPLEXInterface *interface);

		CPLEXInterface *interface;
		IloCplex::Callback cba;

		friend class CPLEXInterface;

		IloCplex cplex;
		IloModel m;

		ModelStatus status;

		bool cplex_up_to_date;

		inline void extract();
		inline void apply_start_solution();

		std::unordered_map<long, IloNum> start_values;
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

inline auto operator<= (unsigned int i, const IloNumExprArg & expr) {
	assert(i <= std::numeric_limits<int>::max());
	return (int)i <= expr;
}
inline auto operator<= (const IloNumExprArg & expr, unsigned int i) {
	assert(i <= std::numeric_limits<int>::max());
	return expr <= (int)i ;
}

#include "ilpa_cplex.cpp"

#endif //ILP_ABSTRACTION_CPLEX_HPP

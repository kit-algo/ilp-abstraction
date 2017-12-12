//
// Created by lukas on 23.08.17.
//

#ifndef ILP_ABSTRACTION_GUROBI_HPP
#define ILP_ABSTRACTION_GUROBI_HPP

#include "common.hpp"

#include "gurobi_c++.h"

#include <limits>
#include <cassert>

namespace ilpabstraction {

namespace grb_internal {
	class CallbackContext; // forward

	class GRBCallbackFriendshipProxy : public GRBCallback {
	protected:
		friend class CallbackContext;
	};

	class CallbackContext
	{
	public:
		inline CallbackContext(GRBCallbackFriendshipProxy * grb_cb);

		inline double get_objective_value() const;
		inline double get_bound() const;
		inline double get_gap() const;
		inline double get_time() const;
		inline int get_processed_nodes() const;
		inline int get_open_nodes() const;
		inline int get_solution_count() const;
	private:
		GRBCallbackFriendshipProxy * grb_cb;
	};

	using ConstraintPair = std::pair<GRBConstr, GRBConstr>;

} // namespace grb_internal

class GurobiInterface : public Interface<GRBVar, GRBLinExpr, grb_internal::ConstraintPair,
                                         grb_internal::CallbackContext>
{
public:
	using Base = Interface<GRBVar, GRBLinExpr, grb_internal::ConstraintPair,
	                       grb_internal::CallbackContext>;
	using Callback = Base::Callback;
	using CallbackContext = Base::CallbackContext ;
	using Constraint = Base::Constraint;

	static constexpr const char * NAME = "Gurobi";

	class Model : public Base::Model
	{
	public:
		template <class LowerValType, class UpperValType>
		inline Constraint add_constraint(LowerValType lower_bound, Expression expr, UpperValType upper_bound,
		                                 std::string name = "");

		template <class LowerValType, class UpperValType>
		inline Variable add_var(VariableType type, LowerValType lower_bound,
		                        UpperValType upper_bound, std::string name = "");

		inline void add_sos1_constraint(const std::vector<Variable> & vars,
		                                const std::vector<double> & weights, std::string name = "");

		inline void commit_variables();

		inline void set_objective(Expression expr, ObjectiveType type);

		inline void solve();

		inline ~Model();

		inline double get_variable_assignment(const Variable &var) const;
		inline double get_objective_value() const;
		inline double get_bound() const;

		inline unsigned int get_variable_count() const;
		inline unsigned int get_constraint_count() const;
		inline unsigned int get_nonzero_count() const;

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

	protected:
		inline Model(GurobiInterface *interface);

		GurobiInterface *interface;
		GRBModel *m;

		friend class GurobiInterface;

	private:
		// Arithmetic types
		template <class LowerValType>
		GRBConstr add_lower_constraint(std::enable_if_t<std::is_arithmetic<LowerValType>::value> lower_bound,
		                               Expression expr, std::string name);

		// Exrpessions
		inline GRBConstr add_lower_constraint(Expression lower_bound, Expression expr, std::string name);

		// Disable
		inline GRBConstr add_lower_constraint(DummyValType lower_bound, Expression expr, std::string name);

		// Arithmetic types
		template <class UpperValType>
		GRBConstr add_upper_constraint(std::enable_if_t<std::is_arithmetic<UpperValType>::value> upper_bound,
		                          Expression expr, std::string name);

		// Exrpessions
		inline GRBConstr add_upper_constraint(Expression upper_bound, Expression expr, std::string name);

		// Disable
		inline GRBConstr add_upper_constraint(DummyValType upper_bound, Expression expr, std::string name);

		ModelStatus status;

		class CallbackAdapter : public grb_internal::GRBCallbackFriendshipProxy
		{
		public:
			inline CallbackAdapter(Model *model);
			inline void callback();

		private:
			Model *model;
		};

		CallbackAdapter cba;
	};

	inline GurobiInterface(bool auto_commit_variables);

	template <class T>
	void set_param(ParamType type, T val);

	inline Model create_model();

protected:
	GRBEnv *env;
};

template <class T>
bool
operator==(const GurobiInterface::DummyValType &lhs, const T &rhs);

template <class T>
bool
operator!=(const GurobiInterface::DummyValType &lhs, const T &rhs);

//constexpr GurobiInterface::DummyValType GurobiInterface::INFTY;
//constexpr GurobiInterface::DummyValType GurobiInterface::NEGATIVE_INFTY;

} // namespace ilpabstraction

#include "ilpa_gurobi.cpp"

#endif //ILP_ABSTRACTION_GUROBI_HPP

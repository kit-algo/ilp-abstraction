//
// Created by lukas on 23.08.17.
//

#ifndef ILP_ABSTRACTION_GUROBI_HPP
#define ILP_ABSTRACTION_GUROBI_HPP

#include "common.hpp"

#include "gurobi_c++.h"

#include <limits>
#include <cassert>

class GurobiInterface {
public:
	class DummyValType {
	public:
		int tag;

		template<class T>
		friend bool operator==(const GurobiInterface::DummyValType & lhs, const T & rhs);
	};
public:

	static constexpr DummyValType INFINITY { 1 };
	static constexpr DummyValType NEGATIVE_INFINITY { 2 };

	using Expression = GRBLinExpr;
	using Variable = GRBVar;

	class Model {
	public:
		template <class LowerValType, class UpperValType>
		void add_constraint(LowerValType lower_bound, Expression expr, UpperValType upper_bound,
		                    std::string * name);
		template <class LowerValType, class UpperValType>
		Variable add_var(VariableType type, std::string * name, LowerValType lower_bound,
		                 UpperValType upper_bound);
		void commit_variables();
		void set_objective(Expression expr, ObjectiveType type);
		void solve();

		~Model();

	protected:
		Model(GurobiInterface * interface);

		GurobiInterface * interface;
		GRBModel * m;

		friend class GurobiInterface;

	private:
		// Arithmetic types
		template <class LowerValType>
		void add_lower_constraint(std::enable_if_t<std::is_arithmetic<LowerValType>::value> lower_bound,
		                          Expression expr, std::string * name);
		// Exrpessions
		void add_lower_constraint(Expression lower_bound, Expression expr, std::string * name);
		// Disable
		void add_lower_constraint(DummyValType lower_bound, Expression expr, std::string * name);

		// Arithmetic types
		template <class UpperValType>
		void add_upper_constraint(std::enable_if_t<std::is_arithmetic<UpperValType>::value> upper_bound,
		                          Expression expr, std::string * name);
		// Exrpessions
		void add_upper_constraint(Expression upper_bound, Expression expr, std::string * name);
		// Disable
		void add_upper_constraint(DummyValType upper_bound, Expression expr, std::string * name);

	};

	GurobiInterface(bool auto_commit_variables);

	template <class T>
	void set_param(ParamType type, T val);

	Model create_model();
protected:
	GRBEnv *env;

	bool auto_commit_variables;
};

template<class T>
bool operator==(const GurobiInterface::DummyValType & lhs, const T & rhs);
template<class T>
bool operator!=(const GurobiInterface::DummyValType & lhs, const T & rhs);

constexpr GurobiInterface::DummyValType GurobiInterface::INFINITY;
constexpr GurobiInterface::DummyValType GurobiInterface::NEGATIVE_INFINITY;

#include "ilpa_gurobi.cpp"

#endif //ILP_ABSTRACTION_GUROBI_HPP

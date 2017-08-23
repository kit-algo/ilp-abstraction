//
// Created by lukas on 23.08.17.
//

// This is actually not needed because this file will never be compiled as a compilation unit.
// However, otherwise some "smart" IDEs don't see the declarations…
#include "ilpa_gurobi.hpp"

namespace grb_internal {

	static constexpr std::pair<ParamType, int> param_map_data[] {
								{ParamType::LOG_TO_CONSOLE, GRB_IntParam_LogToConsole},
								{ParamType::SEED, GRB_IntParam_Seed},
								{ParamType::TIME_LIMIT, GRB_DoubleParam_TimeLimit}
	};

	static constexpr int param_map(ParamType type)
	{
		for (auto pair : param_map_data)
			if (pair.first == type)
				return pair.second;

		return -1;
	}

	static constexpr std::pair<VariableType , int> vtype_map_data[] {
					{VariableType::CONTINUOUS, GRB_CONTINUOUS},
					{VariableType::BINARY, GRB_BINARY},
					{VariableType::INTEGER, GRB_INTEGER},
	};

	static constexpr int vtype_map(VariableType type)
	{
		for (auto pair : vtype_map_data)
			if (pair.first == type)
				return pair.second;

		return -1;
	}

	template<class ValType>
	double get_value(ValType val) {
		return val;
	};

	template<>
	double get_value<GurobiInterface::DummyValType>(GurobiInterface::DummyValType val) {
		if (val == GurobiInterface::INFINITY) {
			return std::numeric_limits<double>::max();
		} else if (val == GurobiInterface::NEGATIVE_INFINITY) {
			return std::numeric_limits<double>::lowest();
		} else {
			assert(false);
		}
	};

	template <class T>
	class DVComparator {
	public:
		static bool compare(const GurobiInterface::DummyValType & lhs, const T & rhs) {
			return false;
		}
	};

	template <>
	class DVComparator<GurobiInterface::DummyValType> {
	public:
		static bool compare(const GurobiInterface::DummyValType & lhs,
		                    const GurobiInterface::DummyValType & rhs) {
			return lhs.tag == rhs.tag;
		}
	};

}; // namespace grb_internal

template<class T>
bool operator==(const GurobiInterface::DummyValType & lhs, const T & rhs)
{
	return grb_internal::DVComparator<T>::compare(lhs, rhs);
}

template<class T>
bool operator!=(const GurobiInterface::DummyValType & lhs, const T & rhs)
{
	return ! (lhs == rhs);
}

template <class T>
void
GurobiInterface::set_param(ParamType type, T val)
{
	this->env->set(grb_internal::param_map(type), val);
}

template <class LowerValType, class UpperValType>
void
GurobiInterface::Model::add_constraint(LowerValType lower_bound, Expression expr,
                                       UpperValType upper_bound, std::string * name)
{
	std::string lower_name = "";
	std::string upper_name = "";
	if (name != nullptr) {
		lower_name = *name + std::string("_lower");
		upper_name = *name + std::string("_upper");
	}

	add_lower_constraint(lower_bound, expr, &lower_name);
	add_upper_constraint(upper_bound, expr, &upper_name);
}

template <class LowerValType, class UpperValType>
GurobiInterface::Variable
GurobiInterface::Model::add_var(VariableType type, std::string * name, LowerValType lower_bound,
                               UpperValType upper_bound)
{
	double lower = grb_internal::get_value(lower_bound);
	double upper = grb_internal::get_value(upper_bound);

	std::string name_str = "";
	if (name != nullptr) {
		name_str = *name;
	}

	Variable var = this->m->addVar(lower, upper, 0, grb_internal::vtype_map(type), name_str);

	if (this->interface->auto_commit_variables) {
		this->commit_variables();
	}

	return var;
}

void
GurobiInterface::Model::commit_variables()
{
	this->m->update();
}

void
GurobiInterface::Model::set_objective(Expression expr, ObjectiveType type) {
	auto obj_type = (type == ObjectiveType::MAXIMIZE) ? GRB_MAXIMIZE : GRB_MINIMIZE;
	this->m->setObjective(expr, obj_type);
}

void
GurobiInterface::Model::solve()
{
	this->m->optimize();
}

GurobiInterface::Model::Model(GurobiInterface * interface_in)
  : interface(interface_in), m(new GRBModel(*interface_in->env))
{
}

GurobiInterface::Model::~Model()
{
	delete this->m;
}

GurobiInterface::GurobiInterface(bool auto_commit_variables_in)
  : env(new GRBEnv()), auto_commit_variables(auto_commit_variables_in)
{
}

GurobiInterface::Model
GurobiInterface::create_model()
{
	return GurobiInterface::Model(this);
}

template <class UpperValType>
void
GurobiInterface::Model::add_upper_constraint(std::enable_if_t<std::is_arithmetic<UpperValType>::value> upper_bound,
                                             Expression expr, std::string * name)
{
	this->m->addConstr(upper_bound, GRB_GREATER_EQUAL, expr, *name);
}

void
GurobiInterface::Model::add_upper_constraint(Expression upper_bound, Expression expr,
                                             std::string * name)
{
	this->m->addConstr(upper_bound, GRB_GREATER_EQUAL, expr, *name);
}

void
GurobiInterface::Model::add_upper_constraint(DummyValType upper_bound, Expression expr,
                                             std::string * name)
{
	assert(upper_bound == GurobiInterface::INFINITY);
}

template <class LowerValType>
void
GurobiInterface::Model::add_lower_constraint(std::enable_if_t<std::is_arithmetic<LowerValType>::value> lower_bound,
                                             Expression expr, std::string * name)
{
	this->m->addConstr(lower_bound, GRB_LESS_EQUAL, expr, *name);
}

void
GurobiInterface::Model::add_lower_constraint(Expression lower_bound, Expression expr,
                                             std::string * name)
{
	this->m->addConstr(lower_bound, GRB_LESS_EQUAL, expr, *name);
}

void
GurobiInterface::Model::add_lower_constraint(DummyValType lower_bound, Expression expr,
                                             std::string * name)
{
	assert(lower_bound == GurobiInterface::NEGATIVE_INFINITY);
}
//
// Created by lukas on 23.08.17.
//

// This is actually not needed because this file will never be compiled as a compilation unit.
// However, otherwise some "smart" IDEs don't see the declarations…
#include "ilpa_gurobi.hpp"

namespace grb_internal {

	static constexpr std::pair<VariableType , int> vtype_map_data[] {
					{VariableType::CONTINUOUS, GRB_CONTINUOUS},
					{VariableType::BINARY, GRB_BINARY},
					{VariableType::INTEGER, GRB_INTEGER},
	};

	inline static constexpr int vtype_map(VariableType type)
	{
		for (auto pair : vtype_map_data)
			if (pair.first == type)
				return pair.second;

		return -1;
	}

	template<class ValType>
	inline double get_value(ValType val) {
		return val;
	};

	template<>
	inline double get_value<GurobiInterface::DummyValType>(GurobiInterface::DummyValType val) {
		if (val == GurobiInterface::INFTY) {
			return std::numeric_limits<double>::max();
		} else if (val == GurobiInterface::NEGATIVE_INFTY) {
			return std::numeric_limits<double>::lowest();
		} else {
			assert(false);
		}
	};

	template <class T>
	class DVComparator {
	public:
		inline static bool compare(const GurobiInterface::DummyValType & lhs, const T & rhs) {
			return false;
		}
	};

	template <>
	class DVComparator<GurobiInterface::DummyValType> {
	public:
		inline static bool compare(const GurobiInterface::DummyValType & lhs,
		                    const GurobiInterface::DummyValType & rhs) {
			return lhs == rhs;
		}
	};

	template <class T>
	void set_param_on_env(GRBEnv env, ParamType type, T val) {
		switch(type) {
			case ParamType::LOG_TO_CONSOLE:
				env.set(GRB_IntParam_LogToConsole, val);
				break;
			case ParamType::TIME_LIMIT:
				env.set(GRB_DoubleParam_TimeLimit, val);
				break;
			case ParamType::SEED:
				env.set(GRB_IntParam_Seed, val);
				break;
			default:
				assert(false);
		}
	}

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
	grb_internal::set_param_on_env(*this->env, type, val);
}

template <class T>
void
GurobiInterface::Model::set_param(ParamType type, T val)
{
	grb_internal::set_param_on_env(this->m->getEnv(), type, val);
}


template <class LowerValType, class UpperValType>
void
GurobiInterface::Model::add_constraint(LowerValType lower_bound, Expression expr,
                                       UpperValType upper_bound, std::string name)
{
	std::string lower_name = "";
	std::string upper_name = "";
	if (name != "") {
		lower_name = name + std::string("_lower");
		upper_name = name + std::string("_upper");
	}

	add_lower_constraint(lower_bound, expr, lower_name);
	add_upper_constraint(upper_bound, expr, upper_name);
}

template <class LowerValType, class UpperValType>
GurobiInterface::Variable
GurobiInterface::Model::add_var(VariableType type, LowerValType lower_bound,
                               UpperValType upper_bound, std::string name)
{
	double lower = grb_internal::get_value(lower_bound);
	double upper = grb_internal::get_value(upper_bound);

	Variable var = this->m->addVar(lower, upper, 0, grb_internal::vtype_map(type), name);

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
	this->status = ModelStatus::SOLVING;
	this->m->optimize();
	switch (this->m->get(GRB_IntAttr_Status)) {
		case GRB_OPTIMAL:
			this->status = ModelStatus::OPTIMAL;
			break;
		case GRB_INFEASIBLE:
			this->status = ModelStatus::INFEASIBLE;
			break;
		case GRB_UNBOUNDED:
			this->status = ModelStatus::UNBOUNDED;
			break;
		case GRB_TIME_LIMIT:
		case GRB_ITERATION_LIMIT:
		case GRB_NODE_LIMIT:
		default:
			this->status = ModelStatus::STOPPED;
			break;
	}
}

GurobiInterface::Model::Model(GurobiInterface * interface_in)
  : interface(interface_in), m(new GRBModel(*interface_in->env)), status(ModelStatus::READY),
    cba(this)
{
	this->m->setCallback(&this->cba);
}

GurobiInterface::Model::~Model()
{
	delete this->m;
}

GurobiInterface::GurobiInterface(bool auto_commit_variables_in)
  : env(new GRBEnv()), Interface(auto_commit_variables_in)
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
                                             Expression expr, std::string name)
{
	this->m->addConstr(upper_bound, GRB_GREATER_EQUAL, expr, name);
}

void
GurobiInterface::Model::add_upper_constraint(Expression upper_bound, Expression expr,
                                             std::string name)
{
	this->m->addConstr(upper_bound, GRB_GREATER_EQUAL, expr, name);
}

void
GurobiInterface::Model::add_upper_constraint(DummyValType upper_bound, Expression expr,
                                             std::string name)
{
	assert(upper_bound == Interface::INFTY);
}

template <class LowerValType>
void
GurobiInterface::Model::add_lower_constraint(std::enable_if_t<std::is_arithmetic<LowerValType>::value> lower_bound,
                                             Expression expr, std::string name)
{
	this->m->addConstr(lower_bound, GRB_LESS_EQUAL, expr, name);
}

void
GurobiInterface::Model::add_lower_constraint(Expression lower_bound, Expression expr,
                                             std::string name)
{
	this->m->addConstr(lower_bound, GRB_LESS_EQUAL, expr, name);
}

void
GurobiInterface::Model::add_lower_constraint(DummyValType lower_bound, Expression expr,
                                             std::string name)
{
	assert(lower_bound == Base::NEGATIVE_INFTY);
}

double
GurobiInterface::Model::get_variable_assignment(const Variable &var) const
{
	return var.get(GRB_DoubleAttr_X);
}

double
GurobiInterface::Model::get_objective_value() const
{
	return this->m->get(GRB_DoubleAttr_ObjVal);
}

double
GurobiInterface::Model::get_bound() const
{
	return this->m->get(GRB_DoubleAttr_ObjBound);
}

ModelStatus
GurobiInterface::Model::get_status() const
{
	return this->status;
}

GurobiInterface::Model::CallbackAdapter::CallbackAdapter(Model *model_in)
  : model(model_in)
{}

void
GurobiInterface::Model::CallbackAdapter::callback()
{
	switch (this->where) {
		case GRB_CB_POLLING: // Polling
			for (auto & cb : this->model->cbs) {
				cb.on_poll();
			}
			break;
		case GRB_CB_PRESOLVE: // Presolve
		case GRB_CB_SIMPLEX: // Simplex
			break;
		case GRB_CB_MIP: // MIP
		{
			double obj_val = this->getDoubleInfo(GRB_CB_MIP_OBJBST);
			double obj_bound = this->getDoubleInfo(GRB_CB_MIP_OBJBND);

			for (auto &cb : this->model->cbs) {
				cb.on_mip(obj_val, obj_bound);
			}

			break;
		}
		case GRB_CB_MIPSOL: // MIPsol
		case GRB_CB_MIPNODE: // MIPnode
			break;
		case GRB_CB_MESSAGE: // Message
		{
			std::string msg = this->getStringInfo(GRB_CB_MSG_STRING);
			for (auto &cb : this->model->cbs) {
				cb.on_message(msg);
			}
			break;
		}
		case GRB_CB_BARRIER: // Barrier
			break;
		default:
			assert(false);
	}
}

unsigned int
GurobiInterface::Model::get_variable_count() const
{
	return (unsigned int)this->m->get(GRB_IntAttr_NumVars);
}

unsigned int
GurobiInterface::Model::get_constraint_count() const
{
	return (unsigned int)this->m->get(GRB_IntAttr_NumConstrs);
}

unsigned int
GurobiInterface::Model::get_nonzero_count() const
{
	return (unsigned int)this->m->get(GRB_IntAttr_NumNZs);
}

bool
GurobiInterface::Model::has_feasible() const
{
	return this->m->get(GRB_IntAttr_SolCount) > 0;
}
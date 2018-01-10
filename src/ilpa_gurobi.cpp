//
// Created by lukas on 23.08.17.
//

// This is actually not needed because this file will never be compiled as a compilation unit.
// However, otherwise some "smart" IDEs don't see the declarations…
#include "ilpa_gurobi.hpp"

namespace ilpabstraction {

namespace grb_internal {

static constexpr std::pair<VariableType, char> vtype_map_data[]{
				{VariableType::CONTINUOUS, GRB_CONTINUOUS},
				{VariableType::BINARY,     GRB_BINARY},
				{VariableType::INTEGER,    GRB_INTEGER},
};

inline static constexpr char
vtype_map(VariableType type)
{
	for (auto pair : vtype_map_data)
		if (pair.first == type)
			return pair.second;

	return -1;
}

template <class ValType>
inline double
get_value(ValType val)
{
	return val;
}

template <>
inline double
get_value<GurobiInterface::DummyValType>(GurobiInterface::DummyValType val)
{
	if (val == GurobiInterface::INFTY) {
		return std::numeric_limits<double>::max();
	} else if (val == GurobiInterface::NEGATIVE_INFTY) {
		return std::numeric_limits<double>::lowest();
	} else {
		assert(false);
	}
}

template <class T>
class DVComparator
{
public:
	inline static bool compare(const GurobiInterface::DummyValType &lhs, const T &rhs)
	{
		// This is only used to compare 'other' types to a DummyValType
		(void)lhs;
		(void)rhs;
		return false;
	}
};

template <>
class DVComparator<GurobiInterface::DummyValType>
{
public:
	inline static bool compare(const GurobiInterface::DummyValType &lhs,
	                           const GurobiInterface::DummyValType &rhs)
	{
		return lhs == rhs;
	}
};

template <class T>
void
set_param_on_env(GRBEnv env, ParamType type, T val)
{
	switch (type) {
		case ParamType::LOG_TO_CONSOLE:
			env.set(GRB_IntParam_LogToConsole, val);
			break;
		case ParamType::TIME_LIMIT:
			env.set(GRB_DoubleParam_TimeLimit, val);
			break;
		case ParamType::SEED:
			env.set(GRB_IntParam_Seed, val);
			break;
		case ParamType::THREADS:
			env.set(GRB_IntParam_Threads, val);
			break;
		case ParamType::MIP_FOCUS:
			assert(false);
		default:
			assert(false);
	}
}

template <>
inline void
set_param_on_env<ParamMIPFocus>(GRBEnv env, ParamType type, ParamMIPFocus val)
{
	assert(type == ParamType::MIP_FOCUS);

	switch (val) {
		case ParamMIPFocus::BALANCED:
			env.set(GRB_IntParam_MIPFocus, 0);
			break;
		case ParamMIPFocus::OPTIMALITY:
			env.set(GRB_IntParam_MIPFocus, 2);
			break;
		case ParamMIPFocus::QUALITY:
			env.set(GRB_IntParam_MIPFocus, 1);
			break;
		case ParamMIPFocus::BOUND:
			env.set(GRB_IntParam_MIPFocus, 3);
			break;
		default:
			assert(false);
	}
}

} // namespace grb_internal

template <class T>
bool
operator==(const GurobiInterface::DummyValType &lhs, const T &rhs)
{
	return grb_internal::DVComparator<T>::compare(lhs, rhs);
}

template <class T>
bool
operator!=(const GurobiInterface::DummyValType &lhs, const T &rhs)
{
	return !(lhs == rhs);
}

template <class SolverParamType, class T>
void
GurobiInterface::Model::set_param_passthrough(SolverParamType type, T val)
{
	this->m->getEnv().set(type, val);
}

template <class T>
void
GurobiInterface::Model::set_param(ParamType type, T val)
{
	grb_internal::set_param_on_env(this->m->getEnv(), type, val);
}

template <class T>
void
GurobiInterface::Model::set_start(Variable & var, T val)
{
	var.set(GRB_DoubleAttr_Start, val);
}

template <class LowerValType, class UpperValType>
GurobiInterface::Constraint
GurobiInterface::Model::add_constraint(LowerValType lower_bound, Expression expr,
                                       UpperValType upper_bound, std::string name)
{
	std::string lower_name = "";
	std::string upper_name = "";
	if (name != "") {
		lower_name = name + std::string("_lower");
		upper_name = name + std::string("_upper");
	}

	GRBConstr lower_constr = add_lower_constraint(lower_bound, expr, lower_name);
	GRBConstr upper_constr = add_upper_constraint(upper_bound, expr, upper_name);
	return std::make_pair<GRBConstr, GRBConstr>(std::move(lower_constr), std::move(upper_constr));
}

void
GurobiInterface::Model::add_sos1_constraint(const std::vector<Variable> & vars,
                                            const std::vector<double> & weights, std::string name)
{
	(void)name;

	assert(vars.size() <= std::numeric_limits<int>::max());
	assert(vars.size() <= std::numeric_limits<double>::max());

	if (weights.size() > 0) {
		this->m->addSOS(vars.data(), weights.data(), (int)vars.size(), GRB_SOS_TYPE1);
	} else {
		std::vector<double> dummy_weights;
		for (size_t i = 0 ; i < vars.size() ; ++i) {
			dummy_weights.push_back((int)i);
		}

		this->m->addSOS(vars.data(), dummy_weights.data(), (int)vars.size(), GRB_SOS_TYPE1);
	}
}

template <class UpperValType>
void
GurobiInterface::Model::change_constraint_ub(Constraint & constr, UpperValType upper_bound)
{
	GRBConstr grbconstr = constr.second;

	auto sense = grbconstr.get(GRB_CharAttr_Sense);
	int factor = 1;
	switch (sense) {
		case '<':
			factor = 1;
			break;
		case '>':
			factor = -1;
			break;
		default:
			assert(false);
	}
	grbconstr.set(GRB_DoubleAttr_RHS, factor * grb_internal::get_value(upper_bound));
}

void
GurobiInterface::Model::change_objective_coefficient(Variable &var, double coefficient)
{
	var.set(GRB_DoubleAttr_Obj, coefficient);
}

template <class LowerValType>
void
GurobiInterface::Model::change_constraint_lb(Constraint & constr, LowerValType lower_bound)
{
	GRBConstr grbconstr = constr.first;

	auto sense = grbconstr.get(GRB_CharAttr_Sense);
	int factor = 1;
	switch (sense) {
		case '<':
			factor = -1;
			break;
		case '>':
			factor = 1;
			break;
		default:
			assert(false);
	}

	grbconstr.set(GRB_DoubleAttr_RHS, grb_internal::get_value(lower_bound));
}

template <class LowerValType, class UpperValType>
void
GurobiInterface::Model::change_var_bounds(Variable & var, LowerValType lower_bound,
                                          UpperValType upper_bound)
{
	var.set(GRB_DoubleAttr_LB, grb_internal::get_value(lower_bound));
	var.set(GRB_DoubleAttr_UB, grb_internal::get_value(upper_bound));
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
GurobiInterface::Model::write(const std::string & filename)
{
	this->m->write(filename);
}

void
GurobiInterface::Model::write_solution(const std::string & filename)
{
	this->m->write(filename + std::string(".sol"));
}


void
GurobiInterface::Model::commit_variables()
{
	this->m->update();
}

void
GurobiInterface::Model::set_objective(Expression expr, ObjectiveType type)
{
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

GurobiInterface::Model::Model(GurobiInterface *interface_in)
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
				: Interface(auto_commit_variables_in), env(new GRBEnv())
{
}

GurobiInterface::Model
GurobiInterface::create_model()
{
	return GurobiInterface::Model(this);
}

template <class UpperValType>
GRBConstr
GurobiInterface::Model::add_upper_constraint(
				std::enable_if_t<std::is_arithmetic<UpperValType>::value> upper_bound,
				Expression expr, std::string name)
{
	return this->m->addConstr(upper_bound, GRB_GREATER_EQUAL, expr, name);
}

GRBConstr
GurobiInterface::Model::add_upper_constraint(Expression upper_bound, Expression expr,
                                             std::string name)
{
	return this->m->addConstr(upper_bound, GRB_GREATER_EQUAL, expr, name);
}

GRBConstr
GurobiInterface::Model::add_upper_constraint(DummyValType upper_bound, Expression expr,
                                             std::string name)
{
	(void)upper_bound;
	(void)expr;
	(void)name;
	assert(upper_bound == Interface::INFTY);
	return GRBConstr();
}

template <class LowerValType>
GRBConstr
GurobiInterface::Model::add_lower_constraint(
				std::enable_if_t<std::is_arithmetic<LowerValType>::value> lower_bound,
				Expression expr, std::string name)
{
	return this->m->addConstr(lower_bound, GRB_LESS_EQUAL, expr, name);
}

GRBConstr
GurobiInterface::Model::add_lower_constraint(Expression lower_bound, Expression expr,
                                             std::string name)
{
	return this->m->addConstr(lower_bound, GRB_LESS_EQUAL, expr, name);
}

GRBConstr
GurobiInterface::Model::add_lower_constraint(DummyValType lower_bound, Expression expr,
                                             std::string name)
{
	(void)lower_bound;
	(void)expr;
	(void)name;
	assert(lower_bound == Base::NEGATIVE_INFTY);
	return GRBConstr();
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
	grb_internal::CallbackContext ctx(this);

	switch (this->where) {
		case GRB_CB_POLLING: // Polling
			for (auto cb : this->model->cbs) {
				cb->on_poll(ctx);
			}
			break;
		case GRB_CB_PRESOLVE: // Presolve
		case GRB_CB_SIMPLEX: // Simplex
			break;
		case GRB_CB_MIP: // MIP
		{
			for (auto cb : this->model->cbs) {
				cb->on_poll(ctx);
			}
			/*
			double obj_val = this->getDoubleInfo(GRB_CB_MIP_OBJBST);
			double obj_bound = this->getDoubleInfo(GRB_CB_MIP_OBJBND);

			for (auto cb : this->model->cbs) {
				cb->on_mip(obj_val, obj_bound);
			}
			*/
			break;
		}
		case GRB_CB_MIPSOL: // MIPsol
		case GRB_CB_MIPNODE: // MIPnode
			break;
		case GRB_CB_MESSAGE: // Message
		{
			std::string msg = this->getStringInfo(GRB_CB_MSG_STRING);
			for (auto cb : this->model->cbs) {
				cb->on_message(ctx, msg);
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

namespace grb_internal {

CallbackContext::CallbackContext(GRBCallbackFriendshipProxy *grb_cb_in)
				: grb_cb(grb_cb_in)
{}

double
CallbackContext::get_objective_value() const
{
	if (this->get_solution_count() < 1) {
		return -1;
	}

	if (this->grb_cb->where == GRB_CB_MIP) {
		return this->grb_cb->getDoubleInfo(GRB_CB_MIP_OBJBST);
	} else if (this->grb_cb->where == GRB_CB_MIPNODE) {
		return this->grb_cb->getDoubleInfo(GRB_CB_MIPNODE_OBJBST);
	} else {
		return -1;
	}
}

double
CallbackContext::get_bound() const
{
	if (this->grb_cb->where == GRB_CB_MIP) {
		return this->grb_cb->getDoubleInfo(GRB_CB_MIP_OBJBND);
	} else if (this->grb_cb->where == GRB_CB_MIPNODE) {
		return this->grb_cb->getDoubleInfo(GRB_CB_MIPNODE_OBJBND);
	} else {
		return -1;
	}
}

double
CallbackContext::get_gap() const
{
	if (this->get_solution_count() < 1) {
		return 1;
	}

	if ((this->grb_cb->where == GRB_CB_MIP) || (this->grb_cb->where == GRB_CB_MIPNODE)) {
		double obj_val = this->get_objective_value();
		double bound = this->get_bound();

		if (obj_val > bound) {
			return 1 - (bound / obj_val);
		} else {
			return obj_val / bound;
		}
	} else {
		return -1;
	}
}

int
CallbackContext::get_processed_nodes() const
{
	if (this->grb_cb->where == GRB_CB_MIP) {
		return (int)this->grb_cb->getDoubleInfo(GRB_CB_MIP_NODCNT);
	} else {
		return -1;
	}
}

int
CallbackContext::get_open_nodes() const
{
	if (this->grb_cb->where == GRB_CB_MIP) {
		return (int)this->grb_cb->getDoubleInfo(GRB_CB_MIP_NODLFT);
	} else {
		return -1;
	}
}

int
CallbackContext::get_solution_count() const
{
	if (this->grb_cb->where == GRB_CB_MIP) {
		return (int)this->grb_cb->getIntInfo(GRB_CB_MIP_SOLCNT);
	} else if (this->grb_cb->where == GRB_CB_MIPNODE) {
		return (int)this->grb_cb->getIntInfo(GRB_CB_MIPNODE_SOLCNT);
	} else {
		return -1;
	}
}

} // namespace grb_internal

} // namespace ilpabstraction
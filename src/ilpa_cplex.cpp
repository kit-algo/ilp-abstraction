//
// Created by lukas on 24.08.17.
//

#include "ilpa_cplex.hpp"

namespace ilpabstraction {

namespace cplex_internal {
	template <class T>
	void set_param_on_cplex(IloCplex & cplex, ParamType type, T val) {
		switch(type) {
			case ParamType::LOG_TO_CONSOLE:
				if (val) {
					cplex.setParam(IloCplex::Param::MIP::Display, 2);
					cplex.setParam(IloCplex::Param::Simplex::Display, 1);
				} else {
					cplex.setParam(IloCplex::Param::MIP::Display, 0);
					cplex.setParam(IloCplex::Param::Simplex::Display, 0);
				}
				break;
			case ParamType::TIME_LIMIT:
				cplex.setParam(IloCplex::Param::TimeLimit, val);
				break;
			case ParamType::SEED:
				cplex.setParam(IloCplex::Param::RandomSeed, val);
				break;
			default:
				assert(false);
		}
	}
} // namespace cplex_internal

/*CPLEXExpression::~CPLEXExpression()
{
	if (this->initialized) {
		//this->end();
	}
}*/

/*CPLEXVariable::~CPLEXVariable(){
	//this->end();
}*/

/*CPLEXVariable::operator CPLEXExpression() const
{
	CPLEXExpression expr(this->getEnv());
	expr += *this;
	return expr;
}*/

CPLEXInterface::~CPLEXInterface()
{
	this->env.end();
}

CPLEXInterface::CPLEXInterface(bool auto_commit_variables_in)
  : Interface(auto_commit_variables_in)
{}

CPLEXInterface::Model
CPLEXInterface::create_model()
{
	return Model(this);
}

CPLEXInterface::Expression
CPLEXInterface::create_expression()
{
	return Expression(this->env);
}

CPLEXInterface::Variable
CPLEXInterface::create_variable()
{
	return Variable(this->env);
}

CPLEXInterface::Model::Model(CPLEXInterface *interface_in)
  : interface(interface_in), cplex(interface_in->env), m(interface_in->env)
{
	this->status = ModelStatus::READY;
	this->cba = CallbackAdapter::create(interface_in->env, this);
}

CPLEXInterface::Model::~Model()
{
	this->cplex.end();
	this->m.end();
}

template<class T>
void
CPLEXInterface::Model::set_param(ParamType type, T val)
{
	cplex_internal::set_param_on_cplex(this->cplex, type, val);
}

bool
CPLEXInterface::Model::has_feasible() const
{
	return (this->cplex.getStatus() == IloAlgorithm::Status::Feasible) ||
					(this->cplex.getStatus() == IloAlgorithm::Status::Optimal);
}

ModelStatus
CPLEXInterface::Model::get_status() const
{
	return this->status;
}

void
CPLEXInterface::Model::extract()
{
	this->cplex.clearModel();
	this->cplex.extract(this->m);
	this->cplex_up_to_date = true;
}

unsigned int
CPLEXInterface::Model::get_nonzero_count()
{
	if (!this->cplex_up_to_date) {
		this->extract();
	}

	return (unsigned int)this->cplex.getNNZs();
}

void
CPLEXInterface::Model::write(const std::string & filename)
{
	if (!this->cplex_up_to_date) {
		this->extract();
	}

	this->cplex.exportModel(filename.c_str());
}

void
CPLEXInterface::Model::write_solution(const std::string & filename)
{
	if (!this->cplex_up_to_date) {
		this->extract();
	}

	std::string extended_fname = filename + std::string(".sol");
	this->cplex.writeSolution(extended_fname.c_str());
}

unsigned int
CPLEXInterface::Model::get_constraint_count()
{
	if (!this->cplex_up_to_date) {
		this->extract();
	}

	return (unsigned int)this->cplex.getNrows();
}

unsigned int
CPLEXInterface::Model::get_variable_count()
{
	if (!this->cplex_up_to_date) {
		this->extract();
	}

	return (unsigned int)this->cplex.getNcols();
}

double
CPLEXInterface::Model::get_bound() const
{
	return this->cplex.getBestObjValue();
}

double
CPLEXInterface::Model::get_objective_value() const
{
	return this->cplex.getObjValue();
}

double
CPLEXInterface::Model::get_variable_assignment(const Variable &var) const
{
	return this->cplex.getValue(var);
}

void
CPLEXInterface::Model::solve()
{
	if (!this->cplex_up_to_date) {
		this->extract();
	}

	this->status = ModelStatus::SOLVING;

	this->cplex.use(this->cba);

	this->cplex.solve();

	switch (this->cplex.getStatus()) {
		case IloAlgorithm::Status::Feasible:
			this->status = ModelStatus::STOPPED;
			break;
		case IloAlgorithm::Status::Optimal:
			this->status = ModelStatus::OPTIMAL;
			break;
		case IloAlgorithm::Status::Infeasible:
			this->status = ModelStatus::INFEASIBLE;
			break;
		case IloAlgorithm::Status::Unbounded:
		case IloAlgorithm::Status::InfeasibleOrUnbounded:
			this->status = ModelStatus::UNBOUNDED;
		default:
		case IloAlgorithm::Status::Unknown:
		case IloAlgorithm::Status::Error:
			assert(false);
	}
}

void
CPLEXInterface::Model::set_objective(Expression expr, ObjectiveType type)
{
	auto sense = (type == ObjectiveType::MAXIMIZE) ? IloObjective::Maximize : IloObjective::Minimize;
	IloObjective obj(this->interface->env, expr, sense);
	this->m.add(obj);

	this->cplex_up_to_date = false;
}

void
CPLEXInterface::Model::commit_variables()
{} // nothing to do

template <class LowerValType, class UpperValType>
CPLEXInterface::Variable
CPLEXInterface::Model::add_var(VariableType type, LowerValType lower_bound,
                               UpperValType upper_bound, std::string name)
{
	(void) name;

	IloNumVar::Type vartype;
	switch (type) {
		case VariableType::CONTINUOUS:
			vartype = IloNumVar::Float;
			break;
		case VariableType::BINARY:
			vartype = IloNumVar::Bool;
			break;
		case VariableType::INTEGER:
			vartype = IloNumVar::Int;
			break;
		default:
			assert(false);
	}
	Variable var(this->interface->env, lower_bound, upper_bound, vartype);
	this->m.add(var);
	this->cplex_up_to_date = false;

	return var;
}

template <class LowerValType, class UpperValType>
void
CPLEXInterface::Model::add_constraint(LowerValType lower_bound, Expression expr,
                                      UpperValType upper_bound, std::string name)
{
	(void) name;
	//IloRange constr(this->interface->env, lower_bound, expr, upper_bound);
	//this->m.add(constr);

	this->m.add(lower_bound <= expr);
	this->m.add(expr <= upper_bound);

	this->cplex_up_to_date = false;
}

void
CPLEXInterface::Model::add_sos1_constraint(const std::vector<Variable> & vars,
                                           const std::vector<double> & weights, std::string name)
{
	IloNumVarArray array(this->interface->env);
	for (const auto & var : vars) {
		array.add(var);
	}

	if (weights.size() == 0) {
		IloSOS1 sos(this->interface->env, array, name.c_str());
		this->m.add(sos);
	} else {
		IloNumArray w_array(this->interface->env);
		for (const auto & w : weights) {
			w_array.add(w);
		}
		IloSOS1 sos(this->interface->env, array, w_array, name.c_str());
		this->m.add(sos);
	}

	this->cplex_up_to_date = false;
}


CPLEXInterface::Model::CallbackAdapter::CallbackAdapter(IloEnv env_in, Model * model_in)
	: IloCplex::MIPInfoCallbackI(env_in), model(model_in)
{}

IloCplex::CallbackI *
CPLEXInterface::Model::CallbackAdapter::duplicateCallback() const
{
	return (new (this->getEnv()) CallbackAdapter(*this));
}

IloCplex::Callback
CPLEXInterface::Model::CallbackAdapter::create(IloEnv env, Model * model)
{
	return (new (env) CallbackAdapter(env, model));
}

void
CPLEXInterface::Model::CallbackAdapter::main()
{
	cplex_internal::CallbackContext ctx(this);
	for (auto cb : this->model->cbs) {
		cb->on_poll(ctx);
	}
}

namespace cplex_internal {

CallbackContext::CallbackContext(IloCplex::MIPInfoCallbackI * cplex_cb_in)
				: cplex_cb(cplex_cb_in)
{}

double
CallbackContext::get_objective_value() const
{
	if (this->cplex_cb->hasIncumbent()) {
		return this->cplex_cb->getIncumbentObjValue();
	} else {
		return -1;
	}
}

double
CallbackContext::get_bound() const
{
	if (this->cplex_cb->hasIncumbent()) {
		return this->cplex_cb->getBestObjValue();
	} else {
		return -1;
	}
}

double
CallbackContext::get_gap() const
{
	return this->cplex_cb->getMIPRelativeGap();
}

int
CallbackContext::get_processed_nodes() const
{
	return (int)this->cplex_cb->getNnodes();

}

int
CallbackContext::get_open_nodes() const
{
	return (int)this->cplex_cb->getNremainingNodes();
}

} // namespace cplex_internal

} // namespace ilpabstraction
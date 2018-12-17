//
// Created by lukas on 24.08.17.
//

#include "ilpa_cplex.hpp"

namespace ilpabstraction {

namespace cplex_internal {
	template <class T>
	void
	set_param_on_cplex(IloCplex & cplex, ParamType type, T val) {
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
				if (((double)val < 0.0) || ((double)val > 1e+75)) {
					cplex.setParam(IloCplex::Param::TimeLimit, 1e+75);
				} else {
					cplex.setParam(IloCplex::Param::TimeLimit, (double)val);
				}
				break;
			case ParamType::SEED:
				cplex.setParam(IloCplex::Param::RandomSeed, (int)(val % CPX_BIGINT));
				break;
			case ParamType::THREADS:
				cplex.setParam(IloCplex::Param::Threads, (int)val);
				break;
			case ParamType::MIP_FOCUS:
				assert(false); // handled by specialization
				break;
			case ParamType::NODE_FILE_DIR:
				assert(false);
				break;
			case ParamType::NODE_FILE_START:
				cplex.setParam(IloCplex::Param::MIP::Strategy::File, 3); // write node files on disk and compressed
				cplex.setParam(IloCplex::Param::WorkMem, 1024 * (double) val); // conversion to megabytes
				break;
			default:
				assert(false);
		}
	}

	template<>
	inline void
	set_param_on_cplex<const char*>(IloCplex & cplex, ParamType type, const char* val) {
		switch(type) {
			case ParamType::NODE_FILE_DIR:
				cplex.setParam(IloCplex::Param::WorkDir, val);
				break;
			default:
				assert(false);
		}
	}

	template<>
	inline void
	set_param_on_cplex<ParamMIPFocus>(IloCplex & cplex, ParamType type, ParamMIPFocus val) {
		assert(type == ParamType::MIP_FOCUS);

		switch (val) {
			case ParamMIPFocus::BALANCED:
				cplex.setParam(IloCplex::Param::Emphasis::MIP, CPX_MIPEMPHASIS_BALANCED);
				break;
			case ParamMIPFocus::OPTIMALITY:
				cplex.setParam(IloCplex::Param::Emphasis::MIP, CPX_MIPEMPHASIS_OPTIMALITY);
				break;
			case ParamMIPFocus::QUALITY:
				cplex.setParam(IloCplex::Param::Emphasis::MIP, CPX_MIPEMPHASIS_FEASIBILITY);
				break;
			case ParamMIPFocus::BOUND:
				cplex.setParam(IloCplex::Param::Emphasis::MIP, CPX_MIPEMPHASIS_BESTBOUND);
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

/* Begin Kappa Stats */
void
CPLEXInterface::Model::enable_kappa_statistics()
{
	this->cplex.setParam(IloCplex::Param::MIP::Strategy::KappaStats, 1); // TODO make 1 or 2 configurable
}

CPLEXInterface::Model::KappaStats
CPLEXInterface::Model::kappa_stats()
{
	double stable = this->cplex.getQuality(IloCplex::Quality::KappaStable, -1);
	double suspicious = this->cplex.getQuality(IloCplex::Quality::KappaSuspicious, -1);
	double unstable = this->cplex.getQuality(IloCplex::Quality::KappaUnstable, -1);
	double illposed = this->cplex.getQuality(IloCplex::Quality::KappaIllposed, -1);
	double kappamax = this->cplex.getQuality(IloCplex::Quality::KappaMax, -1);
	double attention = this->cplex.getQuality(IloCplex::Quality::KappaAttention, -1);

	return {stable, suspicious, unstable, illposed, kappamax, attention};
}

/* End Kappa Stats */

template<class T>
void
CPLEXInterface::Model::set_param(ParamType type, T val)
{
	cplex_internal::set_param_on_cplex(this->cplex, type, val);
}

template <class SolverParamType, class T>
void
CPLEXInterface::Model::set_param_passthrough(SolverParamType type, T val)
{
	this->cplex.setParam(type, val);
}

template<class T>
void
CPLEXInterface::Model::set_start(Variable & var, T val)
{
	this->start_values.insert(std::make_pair<long, IloNum>(var.getId(), IloNum(val)));
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
CPLEXInterface::Model::apply_start_solution()
{
	if (this->start_values.empty()) {
		// TODO remove a set start solution?
		return;
	}

	IloNumVarArray startVar(this->interface->env);
	IloNumArray startVal(this->interface->env);
	for (auto & entry : this->start_values) {
		IloNumVar var((IloNumVarI*)this->interface->env.getExtractable(entry.first));
		startVar.add(var);
		startVal.add(entry.second);
	}

	this->cplex.addMIPStart(startVar, startVal);

	startVar.end();
	startVal.end();
}

void
CPLEXInterface::Model::solve()
{
	if (!this->cplex_up_to_date) {
		this->extract();
	}

	this->apply_start_solution();

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
			break;
		default:
			assert(false);
		case IloAlgorithm::Status::Unknown:
			assert(false);
		case IloAlgorithm::Status::Error:
			assert(false);
	}
}

void
CPLEXInterface::Model::set_objective(Expression expr, ObjectiveType type)
{
	if (this->objective.getImpl() != nullptr) {
		this->objective.removeFromAll();
	}

	auto sense = (type == ObjectiveType::MAXIMIZE) ? IloObjective::Maximize : IloObjective::Minimize;
	this->objective = IloObjective(this->interface->env, expr, sense);
	this->m.add(this->objective);

	this->cplex_up_to_date = false;
}

void
CPLEXInterface::Model::commit_variables()
{} // nothing to do

template <class UpperValType>
void
CPLEXInterface::Model::change_constraint_ub(Constraint & constr, UpperValType upper_bound)
{
	IloRange range = constr.second;
	range.setUB(upper_bound);
	this->cplex_up_to_date = false;
}

template <class LowerValType>
void
CPLEXInterface::Model::change_constraint_lb(Constraint & constr, LowerValType lower_bound)
{
	IloRange range = constr.first;
	range.setLB(lower_bound);
	this->cplex_up_to_date = false;
}

template <class LowerValType, class UpperValType>
void
CPLEXInterface::Model::change_var_bounds(Variable & var, LowerValType lower_bound,
                                         UpperValType upper_bound)
{
	var.setLB(lower_bound);
	var.setUB(upper_bound);

	this->cplex_up_to_date = false;
}

void
CPLEXInterface::Model::change_objective_coefficient(Variable &var, double coefficient)
{
	this->objective.setLinearCoef(var, coefficient);

	this->cplex_up_to_date = false;
}

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
CPLEXInterface::Constraint
CPLEXInterface::Model::add_constraint(LowerValType lower_bound, Expression expr,
                                      UpperValType upper_bound, std::string name)
{
	(void) name;

	IloRange lower_constr = IloRange(this->interface->env, 0, expr - lower_bound, IloInfinity);
	this->m.add(lower_constr);

	IloRange upper_constr(this->interface->env, -1 * IloInfinity, expr - upper_bound, 0);
	this->m.add(upper_constr);

	this->cplex_up_to_date = false;

	return {lower_constr, upper_constr};
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

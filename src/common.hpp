#ifndef ILP_ABSTRACTION_COMMON_HPP
#define ILP_ABSTRACTION_COMMON_HPP

#include <string>
#include <vector>
#include <optional>

namespace ilpabstraction {

namespace util {
template <typename T, typename... Args>
struct has_type;

template <typename T>
struct has_type<T> : std::false_type
{
};

template <typename T, typename U, typename... Ts>
struct has_type<T, U, Ts...> : has_type<T, Ts...>
{
};

template <typename T, typename... Ts>
struct has_type<T, T, Ts...> : std::true_type
{
};
} // namespace util

class Features {
public:
	class KAPPA_STATS {
	};
	class IIS {
	};

	template <class... Flags>
	class FeatureList {
	public:
		template <class Flag>
		constexpr bool
		has_feature()
		{
			return util::has_type<Flag, Flags...>{};
		}
	};
};

/* Kappa Statistics */
struct KappaStats
{
	std::optional<double> stable;
	std::optional<double> suspicious;
	std::optional<double> unstable;
	std::optional<double> illposed;
	std::optional<double> kappamax;
	std::optional<double> attention;
	std::optional<double> root_kappa;
	std::optional<double> presolved_kappa;
};

enum class ParamMIPFocus
{
	BALANCED,
	QUALITY,
	BOUND,
	OPTIMALITY
};

enum class ParamType
{
	LOG_TO_CONSOLE,
	SEED,
	TIME_LIMIT,
	THREADS,
	MIP_FOCUS,
	// write compressed node files to disk
	NODE_FILE_DIR,  // specify directory for node files
	NODE_FILE_START // specify limit to in-set memory for node files (in GB)
};

enum class AttributeType
{
	LOG_TO_CONSOLE,
	SEED,
	TIME_LIMIT
};

enum class VariableType
{
	CONTINUOUS,
	INTEGER,
	BINARY
};

enum class ObjectiveType
{
	MAXIMIZE,
	MINIMIZE
};

enum class ModelStatus
{
	READY,
	SOLVING,
	OPTIMAL,
	INFEASIBLE,
	UNBOUNDED,
	STOPPED
};

enum class QualityMetrics
{
	KAPPA
};

template <class ContextT>
class CallbackBase {
public:
	using Context = ContextT;

	virtual void
	on_message(Context & ctx, std::string & message)
	{
		(void)ctx;
		(void)message;
	};

	virtual void
	on_poll(Context & ctx)
	{
		(void)ctx;
	};

protected:
	// TODO move to implementation
	double
	get_bound(Context & ctx) const
	{
		return ctx.get_bound();
	};

	double
	get_objective_value(Context & ctx) const
	{
		return ctx.get_objective_value();
	};

	double
	get_gap(Context & ctx) const
	{
		return ctx.get_gap();
	};

	double
	get_time(Context & ctx) const
	{
		return ctx.get_time();
	};

	int
	get_processed_nodes(Context & ctx) const
	{
		return ctx.get_processed_nodes();
	};

	int
	get_open_nodes(Context & ctx) const
	{
		return ctx.get_open_nodes();
	};

	int
	get_solution_count(Context & ctx) const
	{
		return ctx.get_solution_count();
	}
};

template <class VariableT, class ExpressionT, class ConstraintT,
          class CContextT>
class Interface {
public:
	enum class DummyValType
	{
		V_INFTY,
		V_NEGATIVE_INFTY
	};

public:
	using Expression = ExpressionT;
	using Variable = VariableT;
	using Constraint = ConstraintT;
	using CallbackContext = CContextT;
	using Callback = CallbackBase<CallbackContext>;

	static constexpr const DummyValType INFTY = DummyValType::V_INFTY;
	static constexpr const DummyValType NEGATIVE_INFTY =
	    DummyValType::V_NEGATIVE_INFTY;

	static constexpr const char * NAME = "BASE";

	class Model {
	public:
		template <class LowerValType, class UpperValType>
		Constraint add_constraint(LowerValType lower_bound, Expression expr,
		                          UpperValType upper_bound,
		                          std::string name = "") = delete;

		template <class LowerValType, class UpperValType>
		Variable add_var(VariableType type, LowerValType lower_bound,
		                 UpperValType upper_bound, std::string name = "") = delete;

		inline void add_sos1_constraint(const std::vector<Variable> & vars,
		                                const std::vector<double> & weights,
		                                std::string name = "") = delete;

		void commit_variables() = delete;

		void set_objective(Expression expr, ObjectiveType type) = delete;

		void solve() = delete;

		double get_variable_assignment(const Variable & var) const = delete;
		double get_objective_value() const = delete;
		double get_bound() const = delete;

		unsigned int get_variable_count() const = delete;
		unsigned int get_constraint_count() const = delete;
		unsigned int get_nonzero_count() const = delete;

		ModelStatus get_status() const = delete;
		bool has_feasible() const = delete;

		void
		add_callback(CallbackBase<CallbackContext> * cb)
		{
			this->cbs.push_back(cb);
		};

		template <class T>
		void set_param(ParamType type, T val) = delete;

		template <class SolverParamType, class T>
		void set_param_passthrough(SolverParamType type, T val) = delete;

		template <class T>
		void set_start(Variable & var, T val) = delete;

		template <class LowerValType, class UpperValType>
		void change_var_bounds(Variable & var, LowerValType lower_bound,
		                       UpperValType upper_bound) = delete;

		template <class UpperValType>
		void change_constraint_ub(Constraint & constr,
		                          UpperValType upper_bound) = delete;
		template <class LowerValType>
		void change_constraint_lb(Constraint & constr,
		                          LowerValType lower_bound) = delete;

		void change_objective_coefficient(Variable & var,
		                                  double coefficient) = delete;

	protected:
		std::vector<CallbackBase<CallbackContext> *> cbs;
	};

	Interface(bool auto_commit_variables_in)
	    : auto_commit_variables(auto_commit_variables_in)
	{}

	Model create_model() = delete;

	Expression
	create_expression()
	{
		return Expression();
	};

	Variable
	create_variable()
	{
		return Variable();
	};

protected:
	bool auto_commit_variables;
};

} // namespace ilpabstraction

#include "common.cpp"

#endif // ILP_ABSTRACTION_COMMON_HPP

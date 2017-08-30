//
// Created by lukas on 23.08.17.
//

#ifndef ILP_ABSTRACTION_COMMON_HPP
#define ILP_ABSTRACTION_COMMON_HPP

#include <vector>

namespace ilpabstraction {

enum class ParamType
{
	LOG_TO_CONSOLE,
	SEED,
	TIME_LIMIT
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

template<class ContextT>
class CallbackBase
{
public:
	using Context = ContextT;

	virtual void on_message(Context & ctx, std::string &message)
	{
		(void) ctx;
		(void) message ;
	};

	virtual void on_poll(Context & ctx)
	{
		(void) ctx;
	};

protected:
	// TODO move to implementation
	double get_bound(Context & ctx) const {
		return ctx.get_bound() ;
	};

	double get_objective_value(Context & ctx) const
	{
		return ctx.get_objective_value() ;
	};

	double get_gap(Context & ctx) const
	{
		return ctx.get_gap() ;
	};

	double get_time(Context & ctx) const
	{
		return ctx.get_time() ;
	};

	int get_processed_nodes(Context & ctx) const
	{
		return ctx.get_processed_nodes() ;
	};

	int get_open_nodes(Context & ctx) const
	{
		return ctx.get_open_nodes();
	};

	int get_solution_count(Context & ctx) const
	{
		return ctx.get_solution_count();
	}
};

template<class VariableT, class ExpressionT, class CContextT>
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
	using CallbackContext = CContextT;
	using Callback = CallbackBase<CallbackContext>;

	static constexpr const DummyValType INFTY = DummyValType::V_INFTY;
	static constexpr const DummyValType NEGATIVE_INFTY = DummyValType::V_NEGATIVE_INFTY;

	static constexpr const char * NAME = "BASE";

	class Model
	{
	public:
		template <class LowerValType, class UpperValType>
		void add_constraint(LowerValType lower_bound, Expression expr, UpperValType upper_bound,
		                    std::string name = "") = delete;

		template <class LowerValType, class UpperValType>
		Variable add_var(VariableType type, LowerValType lower_bound,
		                 UpperValType upper_bound, std::string name = "") = delete;

		void commit_variables() = delete;

		void set_objective(Expression expr, ObjectiveType type) = delete;

		void solve() = delete;

		double get_variable_assignment(const Variable &var) const = delete;
		double get_objective_value() const = delete;
		double get_bound() const = delete;

		unsigned int get_variable_count() const = delete;
		unsigned int get_constraint_count() const = delete;
		unsigned int get_nonzero_count() const = delete;

		ModelStatus get_status() const = delete;
		bool has_feasible() const = delete;

		void add_callback(CallbackBase<CallbackContext> * cb) {
			this->cbs.push_back(cb);
		};

		template <class T>
		void set_param(ParamType type, T val) = delete;

	protected:
		std::vector<CallbackBase<CallbackContext> *> cbs;
	};

	Interface(bool auto_commit_variables_in)
	  : auto_commit_variables(auto_commit_variables_in)
	{}

	template <class T>
	void set_param(ParamType type, T val) = delete;

	Model create_model() = delete;

	Expression create_expression() {
		return Expression();
	};

	Variable create_variable() {
		return Variable();
	};

protected:
	bool auto_commit_variables;
};

} // namespace ilpabstraction

#include "common.cpp"

#endif //ILP_ABSTRACTION_COMMON_HPP

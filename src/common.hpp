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

class Callback
{
public:
	virtual void on_message(std::string &message)
	{ (void) message ;};

	virtual void on_poll()
	{};

	virtual void on_mip(double incumbent, double bound)
	{
		(void) incumbent;
		(void) bound;
	};
};

template<class VariableT, class ExpressionT>
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

		void add_callback(Callback cb) {
			this->cbs.push_back(cb);
		};

		template <class T>
		void set_param(ParamType type, T val) = delete;

	private:
		std::vector<Callback> cbs;
	};

	Interface(bool auto_commit_variables_in)
	  : auto_commit_variables(auto_commit_variables_in)
	{}

	template <class T>
	void set_param(ParamType type, T val) = delete;

	Model create_model() = delete;

protected:
	bool auto_commit_variables;
};

} // namespace ilpabstraction

#endif //ILP_ABSTRACTION_COMMON_HPP

//
// Created by lukas on 23.08.17.
//

#ifndef ILP_ABSTRACTION_COMMON_HPP
#define ILP_ABSTRACTION_COMMON_HPP

enum class ParamType {
	LOG_TO_CONSOLE,
	SEED,
	TIME_LIMIT
};


enum class AttributeType {
	LOG_TO_CONSOLE,
	SEED,
	TIME_LIMIT
};

enum class VariableType {
	CONTINUOUS,
	INTEGER,
	BINARY
};

enum class ObjectiveType {
	MAXIMIZE,
	MINIMIZE
};

enum class ModelStatus {
	READY,
	SOLVING,
	OPTIMAL,
	INFEASIBLE,
	UNBOUNDED,
	STOPPED
};

class Callback {
public:
	void on_message(std::string & message) {};
	void on_poll() {};
	void on_mip(double incumbent, double bound) {};
};

#endif //ILP_ABSTRACTION_COMMON_HPP

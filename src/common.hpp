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

enum class VariableType {
	CONTINUOUS,
	INTEGER,
	BINARY
};

enum class ObjectiveType {
	MAXIMIZE,
	MINIMIZE
};

/*
class ParamVal {
public:
	ParamVal(unsigned long l);
	ParamVal(std::string str);
	ParamVal(bool b);
private:
	union v {
		unsigned long l;
		std::string str;
		bool b;
	};
};
 */

#endif //ILP_ABSTRACTION_COMMON_HPP

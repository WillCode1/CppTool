#pragma once
#include <string>

namespace Exception {
	using namespace std;

	enum ErrorCode
	{
		UserNotExistence,
		PasswordError,
		InvaildOperator
	};

	struct ExceptionBase
	{
		ExceptionBase(ErrorCode _errorCode, const std::string& _msg) : errorCode(_errorCode), msg(_msg) {}
		ExceptionBase(ErrorCode _errorCode, std::string&& _msg) : errorCode(_errorCode), msg(move(_msg)) {}
		virtual ~ExceptionBase() = default;

		ErrorCode errorCode;
		std::string msg;
	};

	struct ExceptionUserNotExistence : ExceptionBase
	{
		ExceptionUserNotExistence(int user_id) : ExceptionBase(UserNotExistence, "User Not Existence."), errorUserId(user_id) {}

		int errorUserId;
	};

	struct ExceptionOperator : ExceptionBase
	{
		// 完美转发
		template<typename T>
		ExceptionOperator(T&& msg) : ExceptionBase(InvaildOperator, std::forward<T>(msg)) {}
	};
};

int testException();

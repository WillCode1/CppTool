#pragma once
#include <memory>

// https://www.cnblogs.com/tekkaman/p/3501122.html

namespace Template {
	template<typename T1, typename T2>
	auto Sum(T1& t1, T2& t2) -> decltype(t1 + t2)
	{
		return t1 + t2;
	}

	class TestClass
	{
	public:
		TestClass(bool _a, int _b, double _c) : a(_a), b(_b), c(_c) {}

	private:
		bool a;
		int b;
		double c;
	};

	class VariableLengthArg
	{
	public:
		template<class T, typename ...Types>
		static std::shared_ptr<T> CreateObject(Types&&... args)
		{
			return std::make_shared<T>(std::forward<Types>(args)...);
		}
	};
};

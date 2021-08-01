#pragma once
#include <tuple>
#include <string>
#include <iostream>


namespace cpp11 {
	using namespace std;

	class Tuple
	{
	public:
		void TestTuple()
		{
			int size;
			//创建一个 tuple 对象存储 10 和 'x'
			std::tuple<int, char> mytuple(10, 'x');

			//计算 mytuple 存储元素的个数
			size = std::tuple_size<decltype(mytuple)>::value;

			//输出 mytuple 中存储的元素
			std::cout << std::get<0>(mytuple) << " " << std::get<1>(mytuple) << std::endl;

			//修改指定的元素
			std::get<0>(mytuple) = 100;
			std::cout << std::get<0>(mytuple) << std::endl;

			//使用 makde_tuple() 创建一个 tuple 对象
			auto bar = std::make_tuple("test", 3.1, 14);

			//拆解 bar 对象，分别赋值给 mystr、mydou、myint
			const char* mystr = nullptr;
			double mydou;
			int myint;
			//使用 tie() 时，如果不想接受某个元素的值，实参可以用 std::ignore 代替
			std::tie(mystr, mydou, myint) = bar;
			//std::tie(std::ignore, std::ignore, myint) = bar;  //只接收第 3 个整形值

			//将 mytuple 和 bar 中的元素整合到 1 个 tuple 对象中
			auto mycat = std::tuple_cat(mytuple, bar);
			size = std::tuple_size<decltype(mycat)>::value;
			std::cout << size << std::endl;

			// 还有一个创建右值的引用元组方法：forward_as_tuple
			std::map<int, std::string> m;
			m.emplace(std::piecewise_construct, std::forward_as_tuple(10), std::forward_as_tuple(20, 'a'));
		}
	};
}

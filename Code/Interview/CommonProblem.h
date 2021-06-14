#pragma once
#include <vector>
#include <algorithm>
#include <memory>
using namespace std;

class CommonProblem
{
public:
	void VectorUnique()
	{
		vector<int> ivec = { 4,2,3,3,3,2,4,1,2,3,4 };
		std::sort(ivec.begin(), ivec.end());
		auto unique_end = std::unique(ivec.begin(), ivec.end());
		ivec.erase(unique_end, ivec.end());
	}

	void UseWeakPtr()
	{
		auto sp = make_shared<int>(42);
		weak_ptr<int> wp(sp);
		if (auto spt = wp.lock()) {
			// 只有lock()之后，才有后续操作
		}
	}

	void TestLamdba(int InValue)
	{
		int Value = 0;

		auto a1 = [](int x) {/*仅能访问全局外部变量*/};
		auto a2 = [Value](int x) {/*值传递局部变量Value*/};
		auto a3 = [this](int x) {/*值传递this指针*/};
		auto a4 = [&Value](int x) {/*引用传递局部变量Value*/};
		auto a5 = [=](int x) {/*值传递所有可访问的外部变量*/};
		auto a6 = [&](int x) {/*引用传递所有可访问的外部变量*/};
		auto a7 = [=, &Value](int x) {/*引用传递局部变量Value，值传递所有其他可访问的外部变量*/};
		auto a8 = [&, Value](int x) {/*值传递局部变量Value，引用传递所有其他可访问的外部变量*/};
	}
};


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
};


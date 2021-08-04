#pragma once
#include <algorithm>
#include <string>
#include <iostream>
#include <vector>
#include <list>
#include <forward_list>
#include <map>           // 基于红黑树的有序映射
#include <unordered_map> // 基于散列表的无序映射
using namespace std;

// https://blog.csdn.net/u014183456/article/details/82835281
// https://zh.cppreference.com/w/cpp/algorithm

namespace cpp11 {
	class Algorithm
	{
	public:
		void AlgoFind()
		{
			vector<int> vec = { 4,3,2,1,6,7,5 };
			vector<int> s = { 3,2,1 };
			auto res = std::search(vec.begin(), vec.end(), s.begin(), s.end());
			res = std::search_n(vec.begin(), vec.end(), 1, 1);
		}
	};
}

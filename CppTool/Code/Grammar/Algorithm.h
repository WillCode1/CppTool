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

// https://zh.cppreference.com/w/cpp/algorithm

// STL中的所有算法(70个)
// https://www.cnblogs.com/lsgxeva/p/7791288.html
/*
	常用算法汇总：

	常用的查找算法：
	adjacent_find()（adjacent 是邻近的意思）,binary_search(),count(), count_if(),equal_range(),find(),find_if()。
	adjacent_find()：https://blog.csdn.net/weixin_44566320/article/details/93921307

	常用的排序算法：
	merge(),sort(),random_shuffle()（shuffle是洗牌的意思） ,reverse()。
	
	常用的拷贝和替换算法：
	copy(), replace(), replace_if(),swap()
	
	常用的算术和生成算法：
	accumulate()（accumulate 是求和的意思）,fill(),。
	
	常用的集合算法：
	set_union(),set_intersection(), set_difference()。
	
	常用的遍历算法：
	for_each(), transform()（transform 是变换的意思）
	for_each()：https://www.cnblogs.com/zhangkele/p/9373063.html
	transform()：https://www.cnblogs.com/KeenLeung/archive/2013/03/17/2965192.html
 */

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

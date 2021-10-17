// Algorithm.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "Sort.h"
#include "Search.h"


int main()
{
	Sort sort;
	std::vector<int> arr = { 5,1,1,2,0,0 };
	sort.sortArray(arr);
	sort.printArray(arr);

	Search search;
	std::cout << search.BinarySearch(arr, 2, 0, arr.size() - 1) << std::endl;

	return 0;
}

// AssociativeContainer（关联容器）
#include "pch.h"
#include "AssociativeContainer.h"
using namespace std;

void Test()
{
	std::map<string, size_t> word_count;
	std::set<string> exclude = { "The", "But", "And", "Or", "An", "A",
								 "the", "but", "and", "or", "an", "a" };

	string word;
	while (cin >> word)
		if (exclude.find(word) == exclude.end())
			++word_count[word];
	for (const auto& w : word_count)
		cout << w.first << " occurs " << w.second << ((w.second > 1) ? " times" : " time") << endl;
	cout << "----------Test---------" << endl;
}

void Test_Map()
{
	// 列表初始化
	std::map<string, int> number = { {"Zero", 0},{"One",1},{"Two",2},{"Three",3},{"Four",4} };

	cout << "----------Test_Map---------" << endl;
}

void Test_Set()
{
	// 列表初始化
	std::set<string> exclude = { "The", "But", "And", "Or", "An", "A" };

	std::set<int> number = { 5,4,3,2,1 };
	cout << number;
	std::unordered_set<int> number2 = { 5,4,3,2,1 };
	cout << number2;

	cout << "----------Test_Set---------" << endl;

}
#pragma once
#include "pch.h"
using namespace std;

class A
{
public:
//	A() = delete;
	A() = default;
	A(int a) :_a(a) {}

	void init() { _a = 0; }

public:
	int _a;
};

// ²Ù×÷·ûÖØÔØ
ostream& operator<<(ostream& os, const A& a);

template<typename T>
ostream& operator<<(ostream& os, const vector<T>& va) {
	for (auto& i : va)
		os << i << ' ';
	os << endl;
	return os;
}

template<typename T>
ostream& operator<<(ostream& os, const list<T>& va) {
	for (auto& i : va)
		os << i << ' ';
	os << endl;
	return os;
}

template<typename T>
ostream& operator<<(ostream& os, const forward_list<T>& va) {
	for (auto& i : va)
		os << i << ' ';
	os << endl;
	return os;
}

template<typename T>
ostream& operator<<(ostream& os, const array<T, 4>& va) {
	for (auto& i : va)
		os << i << ' ';
	os << endl;
	return os;
}

template<typename T, typename M>
ostream& operator<<(ostream& os, const map<T, M>& va) {
	for (auto& i : va)
		os << i << ' ';
	os << endl;
	return os;
}

template<typename T, typename M>
ostream& operator<<(ostream& os, const unordered_map<T, M>& va) {
	for (auto& i : va)
		os << i << ' ';
	os << endl;
	return os;
}

template<typename T>
ostream& operator<<(ostream& os, const set<T>& va) {
	for (auto& i : va)
		os << i << ' ';
	os << endl;
	return os;
}

template<typename T>
ostream& operator<<(ostream& os, const unordered_set<T>& va) {
	for (auto& i : va)
		os << i << ' ';
	os << endl;
	return os;
}
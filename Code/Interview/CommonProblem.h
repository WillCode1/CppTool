#pragma once
#include <vector>
#include <algorithm>
#include <memory>
using namespace std;

class VectorMoveCopy
{
public:
	VectorMoveCopy(const std::vector<int>& vec)
	{
		int_vec = vec;
	}
	VectorMoveCopy(const VectorMoveCopy& other) = default;
	VectorMoveCopy(VectorMoveCopy&& other)
	{
		if (&other == this)
		{
			return;
		}

		int_vec = std::move(other.int_vec);
		std::cout << "Move Copy" << std::endl;
	}
private:
	std::vector<int> int_vec;
};

struct TPose2D
{
	double  x;
	double  y;
	double  phi;

	TPose2D(double px = 0, double py = 0, double pp = 0) {
		x = px;
		y = py;
		phi = pp;
	}

	TPose2D& operator= (const double& value) {
		x = y = phi = value;
		return *this;
	}
	bool operator== (const TPose2D& p3f) {
		return x == p3f.x && y == p3f.y && phi == p3f.phi;
	}
	TPose2D operator+ (const TPose2D& p3f) {
		return TPose2D(x + p3f.x, y + p3f.y, phi + p3f.phi);
	}
	TPose2D operator- (const TPose2D& p3f) {
		return TPose2D(x - p3f.x, y - p3f.y, phi - p3f.phi);
	}
	TPose2D operator* (const double& scale) {
		return TPose2D(x * scale, y * scale, phi * scale);
	}
};

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


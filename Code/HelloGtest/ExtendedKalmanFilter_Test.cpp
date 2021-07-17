#include "ExtendedKalmanFilter_Test.h"
#include "ExtendedKalmanFilter.h"
#include "ExtendedKalmanFilter_Mock.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "gtest/stubext.h"

using namespace testing;
using namespace stub_ext;

// ASSERT_EQ，断言条件不成立，立即跳出当前的用例
// EXPECT_EQ，断言条件不成立，会继续执行当前的用例


 // Code
bool test_Function(ExtendedKalmanFilter& ekf, int& add, int& NoChange)
{
	add = ekf.Add(1, 2);
	add += ekf.Add(1, 2);
	add += ekf.Add(1, 2);
	add += ekf.Add(1, 2);

	NoChange = ekf.NoChange(10);

	if (ekf.Pass())
	{
		return true;
	}
	return false;
}


TEST_F(ExtendedKalmanFilter_Test, stub1)
{
	ASSERT_TRUE(false);
	//ASSERT_EQ(4, 4);
	//EXPECT_TRUE(true);
	//EXPECT_EQ(4, 4);
	std::cout << "执行结束......" << std::endl;
}

TEST_F(ExtendedKalmanFilter_Test, stub2)
{
	ExtendedKalmanFilter ekf;
	StubExt st;
	st.set_lamda(&test_Function, [](ExtendedKalmanFilter& filter, int& a, int& b) { return 100; });
	auto func = &ExtendedKalmanFilter::Add;
	st.set_lamda(&func, [](int a, int b) { return 100; });
	//st.set(ADDR(ExtendedKalmanFilter, NoChange), [](ExtendedKalmanFilter* filter, int a) { return 100; });

	int add, noChange;
	EXPECT_EQ(true, test_Function(ekf, add, noChange));
	EXPECT_EQ(211, add);
	EXPECT_EQ(100, noChange);
}

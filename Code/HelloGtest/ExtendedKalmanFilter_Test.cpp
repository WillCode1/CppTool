#include "ExtendedKalmanFilter_Test.h"
#include "ExtendedKalmanFilter_Mock.h"
#include "Stub_Test.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "gtest/stubext.h"

using namespace testing;
using namespace stub_ext;

// ASSERT_EQ，断言条件不成立，立即跳出当前的用例
// EXPECT_EQ，断言条件不成立，会继续执行当前的用例


int func()
{
	return 1;
}


TEST_F(ExtendedKalmanFilter_Test, stub1)
{
	ASSERT_TRUE(false);
	//ASSERT_EQ(4, 4);
	//EXPECT_TRUE(true);
	//EXPECT_EQ(4, 4);
	std::cout << "执行结束......" << std::endl;
}

// 使用VADDR()便捷地获取虚函数地址。由于功能实现采用的gcc/g++的转换方法，目前应该只支持gcc/g++编译器
TEST_F(ExtendedKalmanFilter_Test, stub2)
{
	StubExt st;
	st.set_lamda(&func, []() { return 100; });
	//st.set_lamda(VADDR(StubTest, Add), [](StubTest* filter, int a) { return 100; });
	st.set_lamda(ADDR(StubTest, NoChange), [](StubTest* filter, int a) { return 100; });
	st.set_lamda(ADDR(StubTest, Pass), [](StubTest* filter) { return true; });

	StubTest sb;
	EXPECT_EQ(100, func());
	EXPECT_EQ(true, sb.Pass());
	EXPECT_EQ(100, sb.NoChange(1));
	EXPECT_EQ(3, sb.Add(1, 2));
}

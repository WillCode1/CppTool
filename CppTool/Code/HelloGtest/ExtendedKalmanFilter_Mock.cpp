#include "ExtendedKalmanFilter_Test.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "ExtendedKalmanFilter_Mock.h"

using namespace testing;
// https://rdc.hundsun.com/portal/article/781.html

// ASSERT_EQ，断言条件不成立，立即跳出当前的用例
// EXPECT_EQ，断言条件不成立，会继续执行当前的用例
/*
	注：在实际使用时，应当将基类中的所有纯虚函数进行打桩，否则会编译报错。
		gmock 依赖C++多态机制进行工作，只有虚函数才能被mock

	▲几个宏的说明：

	MOCK_METHOD#1(#2, #3(#4))
	#1表示你要mock的方法共有几个参数
	#2是你要mock的方法名称
	#3表示这个方法的返回值类型
	#4是这个方法具体的参数

	ON_CALL(#1, #2(#3)).WillByDefault(Return(#4));
	#1表示mock对象
	#2表示想定义的那个方法名称。
	#3表示定义方法的参数，只有当传入的参数=#3的值时，才会生效，也可用_，表示匹配任何参数输入都生效
	#4表示调用要的返回值。

	EXPECT_CALL(#1, #2(#3))，参数含义同ON_CALL，还可以有额外的功能，比如:
	EXPECT_CALL(#1, #2(#3))

	// 期望被调用4 次，如果调用次数不是4 次则会报错。
	.Times(4)

	// 表示第一次返回100，第二次调用返回150，后面全部返回200，以此类推。
	.WillOnce(Return(100)).WillOnce(Return(150)).WillRepeatedly(Return(200));

	// 使用helperAcc 对象中的monitorAccount() 函数替换，但前提是此函数与原函数的返回值和参数一致
	.WillRepeatedly(testing::Invoke(&helperAcc, &HelperThreadRCInvester::monitorAccount));

	// 至少一次
	EXPECT_CALL(#1, #2(#3)).Times(AtLeast(1));
 */

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


TEST_F(ExtendedKalmanFilter_Test, mock2)
{
	ExtendedKalmanFilter_Mock ekf;

	//ON_CALL(ekf, NoChange(testing::_)).WillByDefault(testing::Return(100));
	EXPECT_CALL(ekf, NoChange(testing::_)).WillRepeatedly(testing::Return(100));

	EXPECT_CALL(ekf, Pass()).WillRepeatedly(testing::Invoke(&ekf, &ExtendedKalmanFilter_Mock::stub_Pass));

	EXPECT_CALL(ekf, Add(testing::_, testing::_))
		.Times(4)
		.WillOnce(testing::Return(1))
		.WillOnce(testing::Return(10))
		.WillRepeatedly(testing::Return(100));

	int add, noChange;
	EXPECT_EQ(true, test_Function(ekf, add, noChange));
	EXPECT_EQ(211, add);
	EXPECT_EQ(100, noChange);
}

#pragma once
#include "gtest/gtest.h"
#include <iostream>
#include "ExtendedKalmanFilter.h"


class ExtendedKalmanFilter_Test: public testing::Test
{
protected:
	// 整组测试用例调用前后执行
	static void SetUpTestCase()
	{
		std::cout << "SetUpTestCase" << std::endl;
	}
	static void TearDownTestCase()
	{
		std::cout << "TearDownTestCase" << std::endl;
	}

	// 每个测试用例调用前后执行
	virtual void SetUp()
	{
		std::cout << "SetUp" << std::endl;
	}
	virtual void TearDown()
	{
		std::cout << "TearDown" << std::endl;
	}

private:

};

bool test_Function(ExtendedKalmanFilter& ekf, int& add, int& NoChange);

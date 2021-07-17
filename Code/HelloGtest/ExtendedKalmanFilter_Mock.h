#pragma once
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "ExtendedKalmanFilter.h"


class ExtendedKalmanFilter_Mock : public ExtendedKalmanFilter
{
public:
	MOCK_METHOD2(Add, int(int, int));
	MOCK_METHOD1(NoChange, int(int));
	MOCK_METHOD0(Pass, bool());
	MOCK_METHOD0(Print, void());

	bool stub_Pass()
	{
		return true;
	}
};


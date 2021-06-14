#pragma once
#include <random>
#include <ctime>


class Random
{
public:
	static int RandInt(int max = 100)
	{
		static std::default_random_engine engine(time(nullptr));
		static std::uniform_int_distribution<int> uniform(1, max);
		return uniform(engine);
	}

	static int RandInt(int min, int max)
	{
		//return rand() % (max - min + 1) + min;

		static std::default_random_engine engine(time(nullptr));
		static std::uniform_int_distribution<int> uniform(min, max);
		return uniform(engine);
	}

	static double RandDouble(double min = 0.0, double max = 1.0)
	{
		static std::default_random_engine engine(time(nullptr));
		static std::uniform_real_distribution<double> uniform(min, max);
		return uniform(engine);
	}
};

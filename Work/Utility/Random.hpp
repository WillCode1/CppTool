#pragma once
#include <random>
#include <ctime>


class Random
{
public:
	static int RandInt(const int& max = 100)
	{
		static std::default_random_engine engine(time(nullptr));
		std::uniform_int_distribution<int> uniform(1, max);
		return uniform(engine);
	}

	static int RandInt(const int& min, const int& max)
	{
		//return rand() % (max - min + 1) + min;

		static std::default_random_engine engine(time(nullptr));
		std::uniform_int_distribution<int> uniform(min, max);
		return uniform(engine);
	}

	static double RandDouble(const double& min = 0.0, const double& max = 1.0)
	{
		static std::default_random_engine engine(time(nullptr));
		std::uniform_real_distribution<double> uniform(min, max);
		return uniform(engine);
	}

	static double RandNormal(const double& mu = 0.0, const double& sigma = 1.0)
	{
		static std::default_random_engine engine(time(nullptr));
		std::normal_distribution<double> uniform(mu, sigma);
		return uniform(engine);
	}
};

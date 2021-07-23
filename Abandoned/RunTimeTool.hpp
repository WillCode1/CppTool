#pragma once
#include <ctime>
#include <iostream>

class RunTimeTool
{
public:
	void StartTimer()
	{
		startTime = clock();
		lastTime = clock();
	}

	double TotalCostTime() const
	{
		double total_time = static_cast<double>(clock() - startTime) / CLOCKS_PER_SEC;
		std::cout << "总计耗时：" << total_time << std::endl;
		return total_time;
	}

	double ElapsedTime()
	{
		double total_time = static_cast<double>(clock() - lastTime) / CLOCKS_PER_SEC;
		std::cout << "间隔耗时：" << total_time << std::endl;
		lastTime = clock();
		return total_time;
	}

private:
	clock_t startTime = 0;
	clock_t lastTime = 0;
};

#pragma once
#include <vector>
#include <algorithm>


class Search
{
public:
	int BinarySearch(std::vector<int>& array, int target, int left, int right)
	{
		//std::sort(array.begin(), array.end());

		while (left < right)
		{
			int mid = (left + right) / 2;
			if (array[mid] == target)
			{
				return mid;
			}
			else if (array[mid] < target)
			{
				left = mid + 1;
			}
			else
			{
				right = mid - 1;
			}
		}

		return -1;
	}
};


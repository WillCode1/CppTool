#pragma once
#include <iostream>
#include <vector>
using namespace std;

class Sort
{
public:
	void sortArray(vector<int>& nums)
	{
		//bubbleSort(nums);
		//insertSort(nums);
		//selectSort(nums);
		//quickSort(nums, 0, nums.size() - 1);
		//mergeSort(nums, 0, nums.size() - 1);
		heapSort(nums, nums.size());
	}

	void printArray(const vector<int>& nums)
	{
		for (auto& i: nums)
		{
			cout << i << ',';
		}
		cout << endl;
	}

private:
	void bubbleSort(vector<int>& array);
	void selectSort(vector<int>& nums);
	void insertSort(vector<int>& nums);
	void shellSort(vector<int>& nums, int n);
	
	void quickSort(vector<int>& array, int left, int right);
	//void quick1(vector<int>& array, int left, int right);

	void merge(vector<int>& arr, int left, int mid, int right);
	void mergeSort(vector<int>& arr, int left, int right);

	void adjust(vector<int>& nums, int len, int index);
	void heapSort(vector<int>& nums, int size);

	//’€∞Î≤Â»Î
	void HInsertSort(vector<int>& nums, int n) {
		int i, j, low, high, mid;
		for (i = 0; i < n; i++) {
			int tmp = nums[i];
			low = 0;
			high = i - 1;
			while (low <= high) {
				mid = low + (high - low) / 2;
				if (nums[mid] > tmp) {
					high = mid - 1;
				}
				else {
					low = mid + 1;
				}
			}
			for (j = i - 1; j >= high + 1; j--) {
				nums[j + 1] = nums[j];
			}
			nums[high + 1] = tmp;
		}
	}
};

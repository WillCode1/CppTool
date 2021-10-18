#include "Sort.h"


void Sort::bubbleSort(vector<int>& array)
{
	for (int j = array.size() - 1; j >= 0; j--)
	{
		bool find = false;
		for (int i = 0; i < j; i++)
			if (array[i] > array[i + 1])
			{
				swap(array[i], array[i + 1]);
				find = true;
			}
		if (!find)
			return;
	}
}

//简单选择
void Sort::selectSort(vector<int>& nums)
{
	int n = nums.size();
	for (int i = 0; i < n - 1; i++) {
		int min = i;
		for (int j = i + 1; j < n; j++) {
			if (nums[j] < nums[min])
				min = j;

		}
		if (min != i)
			swap(nums[i], nums[min]);
	}
}

//插入
void Sort::insertSort(vector<int>& nums)
{
	for (int i = 1; i < nums.size(); i++)
	{
		int temp = nums[i];
		int j = i - 1;
		while (j >= 0 && nums[j] > temp)
		{
			nums[j + 1] = nums[j];
			j--;
		}
		nums[j + 1] = temp;
	}
}

//希尔
void Sort::shellSort(vector<int>& nums, int n)
{
	for (int dk = n / 2; dk >= 1; dk = dk / 2) {
		for (int i = dk; i < n; ++i) {
			if (nums[i] < nums[i - dk]) {
				int tmp = nums[i], j;
				for (j = i - dk; j >= 0 && tmp < nums[j]; j -= dk) {
					nums[j + dk] = nums[j];
				}
				nums[j + dk] = tmp;
			}
		}
	}
}

// https://www.cnblogs.com/sky20080101/articles/8693343.html
/*
（二）挖坑法:
1.备份轴记录
2.取两个指针low和high，初始值就是序列的两端下标，保证low<=high
3.移动两个指针
*从high向左找到第一个小于轴的元素, 放在low的位置
*从low向右找到第一个大于轴的元素，放在high的位置
4.重复，直到low=high，
5.把轴放在low所指的位置
6.分别对low所指的位置的左边和右边进行上述的递归
*/
void Sort::quickSort(vector<int>& array, int left, int right)
{
	if (left < right)
	{
		int low = left;									// 左边第一个，因为第一个已经用pivot保存了
		int high = right;								// 右边
		int pivot = array[left];						// 第一个，已被保存
		while (low != high)								// 当左小于右,当相等的时候会跳出循环
		{
			while (low < high && array[high] >= pivot)  // 从右向左找第一个小于x的数
				high--;
			if (low < high)
				array[low++] = array[high];

			while (low < high && array[low] < pivot)	// 从左向右找第一个大于等于x的数
				low++;
			if (low < high)
				array[high--] = array[low];
		}
		array[low] = pivot;
		//printArray(array);

		quickSort(array, left, low - 1);
		quickSort(array, low + 1, right);
	}
}

//void Sort::quick1(vector<int>& array, int left, int right)
//{
//	if (left < right)
//	{
//		int low = left;
//		int high = right;
//		int pivot = array[(low + high) / 2];
//		while (low != high)								// 当左小于右,当相等的时候会跳出循环
//		{
//			while (low < high && array[high] > pivot)  // 从右向左找第一个小于x的数
//				high--;
//			while (low < high && array[low] < pivot)	// 从左向右找第一个大于等于x的数
//				low++;
//			if (low < high)
//				swap(array[high], array[low]);
//			printArray(array);
//		}
//
//		quick(array, left, low - 1);
//		quick(array, low + 1, right);
//	}
//}

// 归并
void Sort::merge(vector<int>& arr, int left, int mid, int right)
{
	vector<int> L_arr(arr.begin() + left, arr.begin() + mid);
	vector<int> R_arr(arr.begin() + mid, arr.begin() + right + 1);

	int i = 0, j = 0, k = left;
	while (i < L_arr.size() && j < R_arr.size()) {
		if (L_arr[i] < R_arr[j])
			arr[k++] = L_arr[i++];
		else
			arr[k++] = R_arr[j++];
	}

	while (i < L_arr.size())
		arr[k++] = L_arr[i++];
	while (j < R_arr.size())
		arr[k++] = R_arr[j++];
}

void Sort::mergeSort(vector<int>& arr, int left, int right)
{
	if (left == right)
		return;
	else {
		int mid = (left + right) / 2;
		mergeSort(arr, left, mid);
		mergeSort(arr, mid + 1, right);
		merge(arr, left, mid + 1, right);
	}
}

// https://blog.csdn.net/alzzw/article/details/98087519
// 堆排序
void Sort::adjust(vector<int>& nums, int len, int index)
{
	int left = 2 * index + 1;	// index的左子节点
	int right = 2 * index + 2;	// index的右子节点

	int maxIdx = index;
	if (left < len && nums[left] > nums[maxIdx])
		maxIdx = left;
	if (right < len && nums[right] > nums[maxIdx])
		maxIdx = right;

	if (maxIdx != index)
	{
		swap(nums[maxIdx], nums[index]);
		adjust(nums, len, maxIdx);
	}
}

void Sort::heapSort(vector<int>& nums, int size)
{
	for (int root = size / 2 - 1; root >= 0; root--) {
		adjust(nums, size, root);
	}
	for (int i = size - 1; i >= 1; i--) {
		swap(nums[0], nums[i]);
		adjust(nums, i, 0);
	}
}


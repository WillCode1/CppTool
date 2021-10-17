#include "Sort.h"

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
void Sort::quick(vector<int>& array, int left, int right)
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

		quick(array, left, low - 1);
		quick(array, low + 1, right);
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

void Sort::bubble(vector<int>& array)
{
	for (int j = array.size() - 1; j >= 0; j--)
		for (int i = 0; i < j; i++)
			if (array[i] > array[i + 1])
				swap(array[i], array[i + 1]);
}

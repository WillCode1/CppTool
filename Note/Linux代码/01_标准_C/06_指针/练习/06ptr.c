/*
   指针练习,找到数组中的最大值(的地址)
   */
#include <stdio.h>
int *max(int *p_num, int size) {
	int *p_tmp = NULL, *p_max = NULL;
	for (p_tmp = p_num;p_tmp <= p_num + size - 1;p_tmp++) {
		if (!p_max || *p_max < *p_tmp) {
			p_max = p_tmp;
		}
	}
	return p_max;
}
int main() {
	int arr[] = {13, 7, 25, 16, 57, 33, 42};
	int *p_num = max(arr, 7);
	printf("最大数字是%d\n", *p_num);
	return 0;
}




